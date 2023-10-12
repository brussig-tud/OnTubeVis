
//////
//
// Includes
//

// OptiX SDK
#include <optix.h>

// OptiX SDK examples common helpers
#include <cuda/helpers.h>
#include <sutil/vec_math.h>

// Host interface
#include "optix_interface.h"

// Local includes
#include "optix_tools.cuh"
#if defined(OTV_PRIMITIVE_RUSSIG) || defined(OTV_PRIMITIVE_PHANTOM)
	#ifdef OTV_PRIMITIVE_RUSSIG
		#include "optix_isect_russig.cuh"
	#else
		#include "optix_isect_phantom.cuh"
	#endif
	#define SET_RESULT_PAYLOAD_0 optixSetPayload_4
	#define SET_RESULT_PAYLOAD_1 optixSetPayload_5
	#define SET_RESULT_PAYLOAD_2 optixSetPayload_6
	#define SET_RESULT_PAYLOAD_3 optixSetPayload_7
	#define SET_RESULT_PAYLOAD_4 optixSetPayload_8
	#define SET_RESULT_PAYLOAD_5 optixSetPayload_9
	#define SET_RESULT_PAYLOAD_6 optixSetPayload_10
	#define SET_RESULT_PAYLOAD_7 optixSetPayload_11
	#define SET_RESULT_PAYLOAD_8 optixSetPayload_12
	#define SET_RESULT_PAYLOAD_9 optixSetPayload_13
	#define SET_RESULT_PAYLOAD_10 optixSetPayload_14
	#define SET_RESULT_PAYLOAD_11 optixSetPayload_15
	#define SET_RESULT_PAYLOAD_12 optixSetPayload_16
	#define SET_RESULT_PAYLOAD_13 optixSetPayload_17
#else
	#define SET_RESULT_PAYLOAD_0 optixSetPayload_2
	#define SET_RESULT_PAYLOAD_1 optixSetPayload_3
	#define SET_RESULT_PAYLOAD_2 optixSetPayload_4
	#define SET_RESULT_PAYLOAD_3 optixSetPayload_5
	#define SET_RESULT_PAYLOAD_4 optixSetPayload_6
	#define SET_RESULT_PAYLOAD_5 optixSetPayload_7
	#define SET_RESULT_PAYLOAD_6 optixSetPayload_8
	#define SET_RESULT_PAYLOAD_7 optixSetPayload_9
	#define SET_RESULT_PAYLOAD_8 optixSetPayload_10
	#define SET_RESULT_PAYLOAD_9 optixSetPayload_11
	#define SET_RESULT_PAYLOAD_10 optixSetPayload_12
	#define SET_RESULT_PAYLOAD_11 optixSetPayload_13
	#define SET_RESULT_PAYLOAD_12 optixSetPayload_14
	#define SET_RESULT_PAYLOAD_13 optixSetPayload_15
#endif



//////
//
// Global constant memory
//

extern "C" {
	__constant__ curve_rt_params params;
}

/*__constant__ const uint3 mono_mult{1, 1, 1};
__constant__ const uint3 holo_mult{3, 1, 1};*/



///////
//
// Structs
//

/// struct representing a ray
struct Ray
{
	/// the ray origin point
	float3 orig;

	/// the ray direction vector
	float3 dir;
};

/// struct storing state related to holographic raycasting
struct HoloState
{
	mat4 P, invP, MV, invMV, N;
	float3 cyclops_eyespace;	// position of the cyclopic eye relative to current view eye space
	unsigned to_fullpxl_divisor;
	float subpxl_width;
	unsigned subpxl;
	uint2 pxl;
	float2 uv;
	float view;
};



/////
//
// Functions
//

static __forceinline__ __device__ mat4 compute_frustum (
	const float l, const float r, const float b, const float t, const float n, const float f
)
{
	mat4 ret(.0f);
	ret.m[0][0] = 2.f * n / (r - l);
	ret.m[2][0] = (r + l) / (r - l);
	ret.m[1][1] = 2.f * n / (t - b);
	ret.m[2][1] = (t + b) / (t - b);
	ret.m[2][2] = (n + f) / (n - f);
	ret.m[3][2] = 2.f * n * f / (n - f);
	ret.m[2][3] = -1.f;
	return ret;
}

constexpr float eye_separation = .3f;

// eye from left to right / -1 to 1
static __forceinline__ __device__ mat4 compute_stereo_frustum_screen (const float eye)
{
	const float znear = params.cam_clip.x;
	const float aspect = params.screen_size.x / params.screen_size.y;
	const float top = .5f*params.screen_size.y*znear / params.parallax_zero_depth;
	const float bottom = -top;
	const float delta = .5*params.holo_eyes_dist*eye*params.screen_size.x*znear / params.parallax_zero_depth;
	const float left = bottom * aspect - delta;
	const float right = top * aspect - delta;
	return compute_frustum(left, right, bottom, top, znear, params.cam_clip.y/*<-- z_far*/);/*
	const float *P = params.cam_P;
	return {P[ 0], P[ 1], P[ 2], P[ 3], P[ 4], P[ 5], P[ 6], P[ 7],
	        P[ 8], P[ 9], P[10], P[11], P[12], P[13], P[14], P[15]};*/
}

// eye from left to right / -1 to 1
static __forceinline__ __device__ mat4 stereo_translate_modelview_matrix (const float eye)
{
	const float *MV = params.cam_MV;
	return {MV[0], MV[1], MV[2], MV[3], MV[4], MV[5], MV[6], MV[7], MV[8], MV[9], MV[10], MV[11],
	        MV[12] - .5f*params.holo_eyes_dist*eye*params.screen_size.x,  MV[13], MV[14], MV[15]};
}

static __forceinline__ __device__ float get_view (const HoloState &hs)
{
	// looking Glass calibration values
	constexpr float pitch  =  673.46088569750157f;
	constexpr float slope  = -0.074780801514116493f;
	constexpr float center =  0.076352536678314209f;

	// compute view according to calibration info
	float z = (hs.uv.x + hs.subpxl_width*float(hs.subpxl) + slope*hs.uv.y) * pitch - center;
	z = 1.f - fmodf(z + ceilf(fabsf(z)), 1.f);
	return z+z - 1.f;
}

static __forceinline__ __device__ Ray compute_ray (HoloState &hs)
{
	// for reconstructing stereo eye position
	static const float4 stereo_eye_local {.0f, .0f, .0f, 1.f};

	// determine sub-pixel jitter for TAA
	float2 jitter_offset = obtain_jittered_subpxl_offset(params.taa_subframe_id, params.taa_jitter_scale);

	// compute exact point on the screen
	/*const float2 d = 2.f * make_float2(
		(static_cast<float>(idx.x)+subpxl_offset.x) / static_cast<float>(dim.x),
		(static_cast<float>(idx.y)+subpxl_offset.y) / static_cast<float>(dim.y)
	) - 1.f;*/

	// find clip coordinates of the fragment corresponding to the OptiX launch ID
	const float2 fragment {
		static_cast<float>(hs.pxl.x)+jitter_offset.x/hs.to_fullpxl_divisor + float(hs.subpxl)/hs.to_fullpxl_divisor,
		static_cast<float>(hs.pxl.y)+jitter_offset.y
	};
	const float4 frag_clip = params.unproject_mode_dbg ? float4 {
			 (hs.uv.x+hs.uv.x) - 1.f,
			 (hs.uv.y+hs.uv.y) - 1.f,
			 0.f,
			 1.f
		} : float4 {
			(fragment.x+fragment.x)/float(params.viewport_dims.x) - 1.f,
			(fragment.y+fragment.y)/float(params.viewport_dims.y) - 1.f,
			0.f, 1.f
		};

	// determine camera parameters of current view
	const float stereo = params.holo==Holo::OFF ? params.holo_eye : hs.view;
	hs.P =
		stereo==0 ? params.cam_P : compute_stereo_frustum_screen(stereo);
		hs.invP = hs.P.inverse();
	hs.MV = stereo_translate_modelview_matrix(stereo); hs.invMV = hs.MV.inverse();
	hs.N = hs.invMV.transposed();
	hs.cyclops_eyespace = mul3_mat_pos(hs.MV, params.cam_eye);

	// transform fragment coordinates from clip to world space
	const float4 frag_world = mul_mat_vec(hs.invMV, mul_mat_vec(hs.invP, frag_clip));

	// calculate ray origin
	const float3 ray_orig = w_clip(mul_mat_vec(hs.invMV, stereo_eye_local));

	// output resulting ray
	return {
		ray_orig,
		normalize(w_clip(frag_world) - ray_orig)
	}; // { params.cam_eye, normalize(d.x*params.cam_u + d.y*params.cam_v + params.cam_w) }
}

static __forceinline__ __device__ float3 calc_hit_point (void)
{
	// query ray params from OptiX
    const float  l     = optixGetRayTmax();
    const float3 ray_o = optixGetWorldRayOrigin();
    const float3 ray_d = optixGetWorldRayDirection();

	// compute point
    return ray_o + l*ray_d;
}

#ifndef OTV_PRIMITIVE_RUSSIG
	template <class dcurve_type>
	static __forceinline__ __device__ float3 calc_sweptdisc_surface_normal (
		const float3 &p_surface, const float3 &p_curve, const dcurve_type &dcurve, const float ts
	)
	{
		// special handling for endcaps
		if (ts <= 0.f)
			return normalize(-dcurve.eval(0));
		else if (ts >= 1.f)
			return normalize(dcurve.eval(1));
		else
			return normalize(p_surface - p_curve);
	}
#endif

static __forceinline__ __device__ void set_payload (
	const float4 &color, const float u, const float v, const unsigned seg_id, const float3 &position,
	const float3 &normal, const float3 &tangent, const float depth
)
{
	// albedo
	SET_RESULT_PAYLOAD_0(pack_unorm_4x8(color));  // surface color as 8bit (per channel) RGBA
	SET_RESULT_PAYLOAD_1(__float_as_int(u));
	SET_RESULT_PAYLOAD_2(__float_as_int(v));
	SET_RESULT_PAYLOAD_3(seg_id);
	// position
	SET_RESULT_PAYLOAD_4(__float_as_int(position.x));
	SET_RESULT_PAYLOAD_5(__float_as_int(position.y));
	SET_RESULT_PAYLOAD_6(__float_as_int(position.z));
	// normal
	SET_RESULT_PAYLOAD_7(__float_as_int(normal.x));
	SET_RESULT_PAYLOAD_8(__float_as_int(normal.y));
	SET_RESULT_PAYLOAD_9(__float_as_int(normal.z));
	// tangent
	SET_RESULT_PAYLOAD_10(__float_as_int(tangent.x));
	SET_RESULT_PAYLOAD_11(__float_as_int(tangent.y));
	SET_RESULT_PAYLOAD_12(__float_as_int(tangent.z));
	// depth
	SET_RESULT_PAYLOAD_13(__float_as_int(depth));
}

static __forceinline__ __device__ float eval_alen (const cuda_arclen &param, const float t)
{
	const float t4 = t + t + t + t;
	const int seg = max(int(fminf(t4, 3.f)), 0);
	const float t_inner = t4 - seg;
	return eval_cubic_bezier(param.span[seg], t_inner);
}


extern "C" __global__ void __raygen__basic (void)
{
	// Lookup our location within the launch grid
	const uint3 dim = optixGetLaunchDimensions(),
	            idx = optixGetLaunchIndex();

	// Initialize holography parameters
	HoloState hs;
	hs.to_fullpxl_divisor = params.holo==Holo::OFF ? 1 : 3;
	hs.subpxl_width = 1.f / params.framebuf_dims.x;
	hs.subpxl = idx.x % hs.to_fullpxl_divisor;
	hs.pxl = {idx.x/hs.to_fullpxl_divisor, idx.y};
	hs.uv = {
		float(hs.pxl.x)/float(params.viewport_dims.x),
		float(hs.pxl.y)/float(params.viewport_dims.y)
	};
	hs.view = get_view(hs);

	// Map our launch idx to a subpixel (equivalent to full pixel when holography is disabled) and create a ray
	// from the camera location through the subpixel
	const Ray ray = compute_ray(hs);

	// Pre-create ray-centric coordinate system for custom intersectors
	// ToDo: investigate unifying the RCC used across custom intersectors
#if defined(OTV_PRIMITIVE_RUSSIG) || defined(OTV_PRIMITIVE_PHANTOM)
	#ifdef OTV_PRIMITIVE_RUSSIG
		// Russig et al. use x-aligned RCC
		const mat4 rcc = RCC::calc_system_transform(ray.orig, ray.dir, RCC::x_axis);
	#else // ifdef OTV_PRIMITIVE_PHANTOM
		// Reshetov and Luebke use z-aligned RCC
		const mat4 rcc = RCC::calc_system_transform(ray.orig, ray.dir, RCC::z_axis);
	#endif
	// encode pointer into two 4-byte ints for passing into the payload registers
	#pragma nv_diag_suppress 69
		const unsigned rcc_msb = (unsigned)(((size_t)&rcc)>>32), rcc_lsb = (unsigned)(size_t)&rcc;
	#pragma nv_diag_default 69
#endif

	// Encode pointer to the HoloState struct into two 4-byte ints for passing into the payload registers
	#pragma nv_diag_suppress 69
		const unsigned hs_msb = (unsigned)(((size_t)&hs)>>32), hs_lsb = (unsigned)(size_t)&hs;
	#pragma nv_diag_default 69

	// Trace the ray against our traversable
	// - create payload storage
	unsigned int
		pl_color, pl_u, pl_v, pl_seg_id,
		pl_position_x, pl_position_y, pl_position_z,
		pl_normal_x, pl_normal_y, pl_normal_z,
		pl_tangent_x, pl_tangent_y, pl_tangent_z,
		pl_depth;
	// - launch ray
	optixTrace(
#if defined(OTV_PRIMITIVE_RUSSIG) || defined(OTV_PRIMITIVE_PHANTOM)
		OPTIX_PAYLOAD_TYPE_ID_0, // ensure correct payload usage
	#endif
		params.accelds,
		ray.orig, ray.dir,
		0.f,					  // Min intersection distance
		1e16f,					  // Max intersection distance
		0.f,					  // rayTime -- used for motion blur
		OptixVisibilityMask(255), // Specify always visible
		(unsigned)OPTIX_RAY_FLAG_NONE,
		0U, // SBT offset   -- See SBT discussion
		1U, // SBT stride   -- See SBT discussion
		0U, // missSBTIndex -- See SBT discussion
		// payloads:
		// - pass on pointer to struct containing current MV/P matrices
		const_cast<unsigned&>(hs_msb), const_cast<unsigned&>(hs_lsb),
	#if defined(OTV_PRIMITIVE_RUSSIG) || defined(OTV_PRIMITIVE_PHANTOM)
		// - pass on ray-centric coordinate system to custom intersectors
		const_cast<unsigned&>(rcc_msb), const_cast<unsigned&>(rcc_lsb),
	#endif
		// - output slots
		pl_color, pl_u, pl_v, pl_seg_id,
		pl_position_x, pl_position_y, pl_position_z,
		pl_normal_x, pl_normal_y, pl_normal_z,
		pl_tangent_x, pl_tangent_y, pl_tangent_z,
		pl_depth);
	// - process payload
	const float r = fmodf(.5f*(hs.view+1.f), 1.f), g = fmodf(r+0.333f, 1.f), b = fmodf(r+0.667f, 1.f);
	float4 albedo;
		//if (params.holo == Holo::OFF)
			albedo.x = __int_as_float(pl_color);
		/*else {
			albedo.x = __int_as_float(//pack_unorm_4x8(float4{r, g, b, 1})
				hs.subpxl==0 ?
					pack_unorm_4x8(float4{1, 0, 0, 1}) :
					(hs.subpxl==1 ?
						pack_unorm_4x8(float4{0, 1, 0, 1}) :
						pack_unorm_4x8(float4{0, 0, 1, 1}))
			);
		}*/
		albedo.y = __int_as_float(pl_u);
		albedo.z = __int_as_float(pl_v);
		albedo.w = __int_as_float(pl_seg_id);
	float3 position;
		position.x = __int_as_float(pl_position_x);
		position.y = __int_as_float(pl_position_y);
		position.z = __int_as_float(pl_position_z);
	float3 normal;
		normal.x = __int_as_float(pl_normal_x);
		normal.y = __int_as_float(pl_normal_y);
		normal.z = __int_as_float(pl_normal_z);
	float3 tangent;
		tangent.x = __int_as_float(pl_tangent_x);
		tangent.y = __int_as_float(pl_tangent_y);
		tangent.z = __int_as_float(pl_tangent_z);
	float1 depth;
		depth.x  = __int_as_float(pl_depth);

	// Record results in our output raster
	const unsigned pxl = idx.y*params.framebuf_dims.x + idx.x;
	params.albedo[pxl] = albedo;
	params.position[pxl] = position;
	params.normal[pxl] = normal;
	params.tangent[pxl] = tangent;
	params.depth[pxl] = depth;
	/*//----------------------------
	// BEGIN: DEBUG OUTPUT
	const unsigned mid = (params.framebuf_dims.y/2)*params.framebuf_dims.x + (params.framebuf_dims.x/2);
	if (pxl == mid)
		printf("segid: %d   u=%f  v=%f\n",
		       __float_as_int(params.albedo[pxl].w), albedo.y, albedo.z);
	// END:   DEBUG OUTPUT
	//---------------------------*/
}

#ifdef OTV_PRIMITIVE_RUSSIG
extern "C" __global__ void __intersection__russig (void)
{
	// ensure correct payload usage
	optixSetPayloadTypes(OPTIX_PAYLOAD_TYPE_ID_0);

	// determine which segment we're testing and its time frame
	const unsigned pid = optixGetPrimitiveIndex(), subseg = pid%2;
	const uint2 node_ids = params.node_ids[pid/2];
	const float t0 = params.nodes[node_ids.x].frame_normal_t.w,
				t_mid = (t0 + params.nodes[node_ids.y].frame_normal_t.w) / 2;
	if ((subseg == 0 && params.max_t <= t0) || (subseg == 1 && params.max_t <= t_mid))
		return;

	// fetch quadratic node data
	const unsigned nid = pid+pid+pid;
	const float3 nodes[3] = {
		params.positions[nid], params.positions[nid+1], params.positions[nid+2]
	};
	const float radii[3] = {
		params.radii[nid].x, params.radii[nid+1].x, params.radii[nid+2].x
	};

	// perform intersection
	const Hit hit = EvalSplineISect(
		*(mat4*)((((size_t)optixGetPayload_2())<<32) | optixGetPayload_3()),  // fetch ray-centric coordinate system
		nodes[0], nodes[1], nodes[2], radii[0], radii[1], radii[2]
	);
	if (hit.l < pos_inf)
		// report our intersection
		optixReportIntersection(
			hit.l, 0u/* hit kind, unused*/, __float_as_int(hit.t),
			__float_as_int(nodes[0].x), __float_as_int(nodes[0].y), __float_as_int(nodes[0].z),
			__float_as_int(nodes[1].x), __float_as_int(nodes[1].y), __float_as_int(nodes[1].z)
		);
	else if (params.show_bvol)
		optixReportIntersection(
			.1f, 0u/* hit kind, unused*/, __float_as_int(1.f),
			__float_as_int(nodes[0].x), __float_as_int(nodes[0].y), __float_as_int(nodes[0].z),
			__float_as_int(nodes[1].x), __float_as_int(nodes[1].y), __float_as_int(nodes[1].z)
		);
}
#elif defined(OTV_PRIMITIVE_PHANTOM)
extern "C" __global__ void __intersection__phantom (void)
{
	// ensure correct payload usage
	optixSetPayloadTypes(OPTIX_PAYLOAD_TYPE_ID_0);

	// determine which segment we're testing and its time frame
	const unsigned pid = optixGetPrimitiveIndex(), subseg = pid%2;
	const uint2 node_ids = params.node_ids[pid/2];
	const float t0 = params.nodes[node_ids.x].frame_normal_t.x,
				t_mid = (t0 + params.nodes[node_ids.y].frame_normal_t.x) / 2;
	if ((subseg == 0 && params.max_t <= t0) || (subseg == 1 && params.max_t <= t_mid))
		return;

	// fetch quadratic node data
	const unsigned nid = pid+pid+pid;
	const float3 nodes[3] = {
		params.positions[nid], params.positions[nid+1], params.positions[nid+2]
	};
	const float radii[3] = {
		params.radii[nid].x, params.radii[nid+1].x, params.radii[nid+2].x
	};

	// perform intersection
	const Hit hit = intersect_spline_tube(
		*(mat4*)((((size_t)optixGetPayload_2())<<32) | optixGetPayload_3()),  // fetch ray-centric coordinate system
		optixGetWorldRayOrigin(), optixGetWorldRayDirection(),
		nodes[0], nodes[1], nodes[2], radii[0], radii[1], radii[2]
	);
	/*//----------------------------
	// BEGIN: DEBUG OUTPUT
	const uint3 idx = optixGetLaunchIndex();
	const unsigned pxl = idx.y*params.framebuf_dims.x + idx.x,
	               mid = (params.framebuf_dims.y/2)*params.framebuf_dims.x + (params.framebuf_dims.x/2);
	if (pxl == mid)
		printf("t=: %f,  l=%f\n", hit.t, hit.l);
	// END:   DEBUG OUTPUT
	//---------------------------*/

	if (hit.l < pos_inf)
		// report our intersection
		optixReportIntersection(
			hit.l, 0u/* hit kind, unused*/, __float_as_int(hit.t),
			__float_as_int(nodes[0].x), __float_as_int(nodes[0].y), __float_as_int(nodes[0].z),
			__float_as_int(nodes[1].x), __float_as_int(nodes[1].y), __float_as_int(nodes[1].z)
		);
	else if (params.show_bvol)
		optixReportIntersection(
			.1f, 0u/* hit kind, unused*/, __float_as_int(1.f),
			__float_as_int(nodes[0].x), __float_as_int(nodes[0].y), __float_as_int(nodes[0].z),
			__float_as_int(nodes[1].x), __float_as_int(nodes[1].y), __float_as_int(nodes[1].z)
		);
}
#endif

extern "C" __global__ void __miss__ms (void)
{
	#if defined(OTV_PRIMITIVE_RUSSIG) || defined(OTV_PRIMITIVE_PHANTOM)
		optixSetPayloadTypes(OPTIX_PAYLOAD_TYPE_ID_0); // ensure correct payload usage
	#endif
	data_miss *data  = reinterpret_cast<data_miss*>(optixGetSbtDataPointer());
	set_payload(data->bgcolor, -1.f, 0.f, 0, nullvec3, nullvec3, nullvec3, 1.f);
}


extern "C" __global__ void __closesthit__ch (void)
{
	// retrieve holo state
	auto hs = *(HoloState*)((((size_t)optixGetPayload_0())<<32) | optixGetPayload_1());

	// retrieve curve parameter, hitpoint and segment index
	#ifdef OTV_PRIMITIVE_BUILTIN
		const unsigned pid = optixGetPrimitiveIndex(), seg_id = pid/2, subseg = pid%2;
		const float ts = optixGetCurveParameter(), t = .5f*(ts + float(subseg));
		// retrieve actual node data
		float4 nodes[3]; // w-component contains radius
		optixGetQuadraticBSplineVertexData(
			optixGetGASTraversableHandle(), pid, optixGetSbtGASIndex(), 0.f, nodes
		);
		quadr_interpolator_vec3 curve;
		curve.from_bspline(nodes);
		const linear_interpolator_vec3 dcurve = curve.derive();
	#elif defined(OTV_PRIMITIVE_BUILTIN_CUBIC)
		const float ts = optixGetCurveParameter(), t = ts;
		const unsigned seg_id = optixGetPrimitiveIndex();
		// retrieve actual node data
		float4 nodes[4]; // w-component contains radius
		optixGetCatmullRomVertexData(
			optixGetGASTraversableHandle(), seg_id, optixGetSbtGASIndex(), 0.f, nodes
		);
		cubic_interpolator_vec3 curve;
		curve.from_catmullrom(nodes);
		const quadr_interpolator_vec3 dcurve = curve.derive();
	#else
		// ensure correct payload usage
		optixSetPayloadTypes(OPTIX_PAYLOAD_TYPE_ID_0);
		// retrieve segment index
		const unsigned pid=optixGetPrimitiveIndex(), seg_id=pid/2, subseg=pid%2, nid=pid*3;
		// retrieve curve parameter at hit
		const float ts = optixGetCurveParameter(), t  = .5f*(ts + float(subseg));
		// retrieve node data
		quadr_interpolator_vec3 curve; // ToDo: test performance with less attribute registers in exchange for more global mem access
		curve.b[0] = make_float3(__int_as_float(optixGetAttribute_1()), __int_as_float(optixGetAttribute_2()), __int_as_float(optixGetAttribute_3()));
		curve.b[1] = make_float3(__int_as_float(optixGetAttribute_4()), __int_as_float(optixGetAttribute_5()), __int_as_float(optixGetAttribute_6()));
		curve.b[2] = params.positions[nid+2];
		const linear_interpolator_vec3 dcurve = curve.derive();
	#endif
	const uint2 node_ids = params.node_ids[seg_id];

	// compute hit position (world space)
	const float3 pos = calc_hit_point();
		// in the general setting, we would first call optixTransformPointFromWorldToObjectSpace() before
		// doing anything with the hitpoint obtained from a ray (e.g. use it to compute surface normals),
		// since the segment could be in a bottom-level acceleration structure and subject to an additional
		// model or instance transform (we don't use those though so we're fine)

	// compute hit normal in eye-space
	const float3 pos_curve = curve.eval(ts);
	const float3 normal = normalize(mul3_mat_vec(hs.N, // <-- we use eye-dependent lighting in holo mode,
#if defined(OTV_PRIMITIVE_RUSSIG)                      //     change to params.cam_N for cyclopic lighting
		pos - pos_curve  // swept-sphere makes this especially easy
#else
		// swept-disc tube normal calculation with special handling for end caps
		calc_sweptdisc_surface_normal(pos, pos_curve, dcurve, ts)
#endif
	));	// in the general setting, we would first call optixTransformNormalFromObjectToWorldSpace()
		// before doing anything with the normal, since the segment could be in a bottom-level acceleration
		// structure and subject to an additional model or instance transform (we don't use those though so
		// we're fine)

	// compute hit tangent in eye-space
	const cuda_node &n0 = params.nodes[node_ids.x], &n1 = params.nodes[node_ids.y];
#ifndef OTV_PRIMITIVE_BUILTIN_CUBIC
	float3 tangent;
	if (params.cubic_tangents)
	{
		cubic_interpolator_vec3 curve_orig;
		curve_orig.from_hermite(n0.pos_rad, n0.tangent, n1.pos_rad, n1.tangent);
		tangent = normalize(mul3_mat_vec(params.cam_N, curve_orig.derive().eval(t)));
	}
	else
#else
	const float3
#endif
	tangent = normalize(mul3_mat_vec(params.cam_N, dcurve.eval(ts)));

	// evaluate pre-shading tube surface color
	const float4 color = mix(n0.color, n1.color, t);

	// calculate u texture coordinate (arclength at t)
	const float u = eval_alen(params.alen[seg_id], t);	

	// calculate v texture coordinate
	const float3 bitangent = normalize(
		cross(tangent, mul3_mat_vec(params.cam_N, pos_curve-params.cam_eye)) //w_clip(mul_mat_pos(params.cam_MV, pos_curve))/*-params.cam_cyclops_eyespace*/)
	);
	const float v = acos(dot(bitangent, normal)) * pi_inv;

	// calculate depth value
	const float4 pos_eye = mul_mat_pos(params.cam_MV, pos);
	const float4 pos_screen = mul_mat_vec(params.cam_P, pos_eye);
	const float depth = .5f*(pos_screen.z/pos_screen.w) + .5f;

	// done - store hit results in payload
	set_payload(color, u, v, seg_id, w_clip(pos_eye), normal, tangent, depth);
}
