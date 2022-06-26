
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
#include "optix_curves.h"

// Local includes
#include "optix_tools.h"



//////
//
// Global constant memory
//

extern "C" {
	__constant__ curve_rt_params params;
}



//////
//
// Functions
//

static __forceinline__ __device__ void compute_ray(uint3 idx, uint3 dim, float3& origin, float3& direction)
{
	// determine sub-pixel location
	float2       subpxl_offset = obtain_jittered_subpxl_offset(params.taa_subframe_id, params.taa_jitter_scale);

	// compute exact point on the screen
	const float2 d = 2.f * make_float2(
		(static_cast<float>(idx.x)+subpxl_offset.x) / static_cast<float>(dim.x),
		(static_cast<float>(idx.y)+subpxl_offset.y) / static_cast<float>(dim.y)
	) - 1.f;

	// create resulting ray
	origin = params.cam_eye;
	direction = normalize(d.x*params.cam_u + d.y*params.cam_v + params.cam_w);
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

static __device__ float3 calc_segment_surface_normal (
	const float3 &p_surface, const float3 &p_curve, const linear_interpolator_vec3 &dcurve,
	const float3 &ddcurve, const float ts
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

static __device__ float3 calc_segment_surface_normal (
	const float3 &p_surface, const float3 &p_curve, const quadr_interpolator_vec3 &dcurve,
	const linear_interpolator_vec3 &ddcurve, const float ts
)
{
	// special handling for endcaps
	float3 normal;
	if (ts <= 0.f)
		normal = normalize(-dcurve.eval(0));
	else if (ts >= 1.f)
		normal = normalize(dcurve.eval(1));
	else
	{
		// ps is a point that is near the curve's offset surface,
		// usually ray.origin + ray.direction * rayt.
		// We will push it exactly to the surface by projecting it to the plane(p,d).
		// The function derivation:
		// we (implicitly) transform the curve into coordinate system
		// {p, o1 = normalize(ps - p), o2 = normalize(curve'(t)), o3 = o1 x o2} in which
		// curve'(t) = (0, length(d), 0); ps = (r, 0, 0);
		/*float4 p4 = bc.position4( ts );
		float3 p  = make_float3( p4 );
		float  r  = p4.w;  // == length(ps - p) if ps is already on the surface
		float4 d4 = bc.velocity4( ts );
		float3 d  = make_float3( d4 );
		float  dr = d4.w;
		float  dd = dot( d, d );

		float3 o1 = ps - p;               // dot(modified_o1, d) == 0 by design:
		o1 -= ( dot( o1, d ) / dd ) * d;  // first, project ps to the plane(p,d)
		o1 *= r / length( o1 );           // and then drop it to the surface
		ps = p + o1;                      // fine-tuning the hit point
		if( type == 0 )
			normal = o1;  // cylindrical approximation
		else
		{
			if( type != 1 )
				dd -= dot( bc.acceleration3( ts ), o1 );
			normal = dd * o1 - ( dr * r ) * d;
		}*/
		normal = normalize(p_surface - p_curve);
	}
	return normal;
}

static __forceinline__ __device__ void set_payload (
	const float4 &color, const float u, const float v, const unsigned seg_id, const float3 &position,
	const float3 &normal, const float3 &tangent, const float depth
)
{
	// albedo
	optixSetPayload_0(pack_unorm_4x8(color));  // surface color as 8bit (per channel) RGBA
	optixSetPayload_1(float_as_int(u));
	optixSetPayload_2(float_as_int(v));
	optixSetPayload_3(seg_id);
	// position
	optixSetPayload_4(float_as_int(position.x));
	optixSetPayload_5(float_as_int(position.y));
	optixSetPayload_6(float_as_int(position.z));
	// normal
	optixSetPayload_7(float_as_int(normal.x));
	optixSetPayload_8(float_as_int(normal.y));
	optixSetPayload_9(float_as_int(normal.z));
	// tangent
	optixSetPayload_10(float_as_int(tangent.x));
	optixSetPayload_11(float_as_int(tangent.y));
	optixSetPayload_12(float_as_int(tangent.z));
	// depth
	optixSetPayload_13(float_as_int(depth));
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
	const uint3 idx = optixGetLaunchIndex();
	const uint3 dim = optixGetLaunchDimensions();

	// Map our launch idx to a screen location and create a ray from the camera
	// location through the screen pixel
	float3 ray_origin, ray_direction;
	compute_ray(idx, dim, ray_origin, ray_direction);

	// Trace the ray against our scene hierarchy
	// - create payload storage
	unsigned int
		pl_color, pl_u, pl_v, pl_seg_id,
		pl_position_x, pl_position_y, pl_position_z,
		pl_normal_x, pl_normal_y, pl_normal_z,
		pl_tangent_x, pl_tangent_y, pl_tangent_z,
		pl_depth;
	// - launch ray
	optixTrace(
		params.accelds,
		ray_origin,
		ray_direction,
		0.f,                 // Min intersection distance
		1e16f,               // Max intersection distance
		0.f,                 // rayTime -- used for motion blur
		OptixVisibilityMask(255), // Specify always visible
		OPTIX_RAY_FLAG_NONE,
		0,                   // SBT offset   -- See SBT discussion
		1,                   // SBT stride   -- See SBT discussion
		0,                   // missSBTIndex -- See SBT discussion
		// payloads:
		pl_color, pl_u, pl_v, pl_seg_id,
		pl_position_x, pl_position_y, pl_position_z,
		pl_normal_x, pl_normal_y, pl_normal_z,
		pl_tangent_x, pl_tangent_y, pl_tangent_z,
		pl_depth
	);
	// - process payload	
	float4 albedo;
		albedo.x = int_as_float(pl_color);
		albedo.y = int_as_float(pl_u);
		albedo.z = int_as_float(pl_v);
		albedo.w = int_as_float(pl_seg_id);
	float3 position;
		position.x = int_as_float(pl_position_x);
		position.y = int_as_float(pl_position_y);
		position.z = int_as_float(pl_position_z);
	float3 normal;
		normal.x = int_as_float(pl_normal_x);
		normal.y = int_as_float(pl_normal_y);
		normal.z = int_as_float(pl_normal_z);
	float3 tangent;
		tangent.x = int_as_float(pl_tangent_x);
		tangent.y = int_as_float(pl_tangent_y);
		tangent.z = int_as_float(pl_tangent_z);
	float1 depth;
		depth.x  = int_as_float(pl_depth);

	// Record results in our output raster
	const unsigned pxl = idx.y*params.fb_width + idx.x;
	params.albedo[pxl] = albedo;
	params.position[pxl] = position;
	params.normal[pxl] = normal;
	params.tangent[pxl] = tangent;
	params.depth[pxl] = depth;
	/*//----------------------------
	// BEGIN: DEBUG OUTPUT
	const unsigned mid = (params.fb_height/2)*params.fb_width + (params.fb_width/2);
	if (pxl == mid)
		printf("segid: %d   u=%f  v=%f\n",
		       float_as_int(params.albedo[pxl].w), albedo.y, albedo.z);
	// END:   DEBUG OUTPUT
	//---------------------------*/
}

#ifdef TRAJVIS_PRIMITIVE_RUSSIG
extern "C" __global__ void __intersection__russig (void)
{
	/* do_nothing_yet() */;
}
#endif

extern "C" __global__ void __miss__ms (void)
{
	data_miss *data  = reinterpret_cast<data_miss*>(optixGetSbtDataPointer());
	set_payload(data->bgcolor, -1.f, 0.f, 0, nullvec3, nullvec3, nullvec3, 1.f);
}


extern "C" __global__ void __closesthit__ch (void)
{
	// retrieve curve parameter, hitpoint and segment index
	#ifdef TRAJVIS_PRIMITIVE_BUILTIN
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
	#elif defined(TRAJVIS_PRIMITIVE_BUILTIN_CUBIC)
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
		/*//----------------------------
		// BEGIN: DEBUG OUTPUT
		const uint3 idx = optixGetLaunchIndex();
		if (idx.x==params.fb_width/2 && idx.y==params.fb_height/2)
			printf("segid: %d\:%d - t=%f:%f\n", seg_id, subseg, t, ts);
		// END:   DEBUG OUTPUT
		//---------------------------*/
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
	const float3 normal = normalize(mul_mat_vec(
		params.cam_N, calc_segment_surface_normal(pos, pos_curve, dcurve, dcurve.derive(), ts)
	));
		// in the general setting, we would first call optixTransformNormalFromObjectToWorldSpace()
		// before doing anything with the normal, since the segment could be in a bottom-level acceleration
		// structure and subject to an additional model or instance transform (we don't use those though so
		// we're fine)

	// compute hit tangent in eye-space
	const cuda_node &n0 = params.nodes[node_ids.x], &n1 = params.nodes[node_ids.y];
#ifndef TRAJVIS_PRIMITIVE_BUILTIN_CUBIC
	float3 tangent;
	if (params.cubic_tangents)
	{
		cubic_interpolator_vec3 curve_orig;
		curve_orig.from_hermite(n0.pos_rad, n0.tangent, n1.pos_rad, n1.tangent);
		tangent = normalize(mul_mat_vec(params.cam_N, curve_orig.derive().eval(t)));
	}
	else
#else
	const float3
#endif
	tangent = normalize(mul_mat_vec(params.cam_N, dcurve.eval(ts)));

	// calculate pre-shading surface color
	const float4 color = mix(n0.color, n1.color, t);

	// calculate u texture coordinate (arclength at t)
	const float u = eval_alen(params.alen[seg_id], t);	

	// calculate v texture coordinate
	const float4 pos_eye = mul_mat_pos(params.cam_MV, pos);
	const float3 bitangent = normalize(cross(tangent, w_clip(mul_mat_pos(params.cam_MV, pos_curve))));
	const float v = acos(dot(bitangent, normal)) * pi_inv;

	// calculate depth value
	const float4 pos_screen = mul_mat_vec(params.cam_P, pos_eye);
	const float depth = .5f*(pos_screen.z/pos_screen.w) + .5f;

	// done - store hit results in payload
	set_payload(color, u, v, seg_id, w_clip(pos_eye), normal, tangent, depth);
}