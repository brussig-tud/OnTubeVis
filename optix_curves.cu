
//////
//
// Includes
//

// OptiX SDK
#include <optix.h>

// CUDA SDK
#include <random.h>

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

static __forceinline__ __device__ uchar4 make_rgba (const float4 &c)
{
	return make_uchar4(
		quantizeUnsigned8Bits(c.x), quantizeUnsigned8Bits(c.y),
		quantizeUnsigned8Bits(c.z), quantizeUnsigned8Bits(c.w)
	);
}

static __forceinline__ __device__ void compute_ray(uint3 idx, uint3 dim, float3& origin, float3& direction)
{
	const float2 d = 2.f * make_float2(
		static_cast<float>(idx.x) / static_cast<float>(dim.x),
		static_cast<float>(idx.y) / static_cast<float>(dim.y)
	) - 1.f;
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
	const cubic_interpolator_vec3 &curve, const quadr_interpolator_vec3 &dcurve,
	const float3 &p_surface, const float t
)
{
	// special handling for endcaps
	float3 normal;
	if (t <= 0.f)
		normal = -dcurve.eval(0);
	else if (t >= 1.f)
		normal =  dcurve.eval(1);
	else
		normal = normalize(p_surface - curve.eval(t));
	return normalize(normal);
}

static __forceinline__ __device__ void set_payload (float4 color, float3 position, float3 normal, float3 tangent, float depth)
{
	// albedo
	optixSetPayload_0(pack_unorm_4x8(color));
	optixSetPayload_1(0);
	optixSetPayload_2(0);
	optixSetPayload_3(0);
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


extern "C" __global__ void __raygen__basic (void)
{
	// Lookup our location within the launch grid
	const uint3 idx = optixGetLaunchIndex();
	const uint3 dim = optixGetLaunchDimensions();

	// Map our launch idx to a screen location and create a ray from the camera
	// location through the screen
	float3 ray_origin, ray_direction;
	compute_ray(idx, dim, ray_origin, ray_direction);

	// Trace the ray against our scene hierarchy
	// - create payload storage
	unsigned int
		pl_color, pl_albedo1, pl_albedo2, pl_albedo3,
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
		pl_color, pl_albedo1, pl_albedo2, pl_albedo3,
		pl_position_x, pl_position_y, pl_position_z,
		pl_normal_x, pl_normal_y, pl_normal_z,
		pl_tangent_x, pl_tangent_y, pl_tangent_z,
		pl_depth
	);
	// - process payload	
	float4 albedo;
		albedo.x = int_as_float(pl_color);
		albedo.y = int_as_float(pl_albedo1);
		albedo.z = int_as_float(pl_albedo2);
		albedo.w = int_as_float(pl_albedo3);
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
}


extern "C" __global__ void __miss__ms (void)
{
	data_miss *data  = reinterpret_cast<data_miss*>(optixGetSbtDataPointer());
	set_payload(data->bgcolor, nullvec3, nullvec3, nullvec3, 1.f);
}


extern "C" __global__ void __closesthit__ch (void)
{
	// retrieve curve parameter, hitpoint and segment index
	const float  t = optixGetCurveParameter();
	const unsigned seg_id = optixGetPrimitiveIndex();

	// retrieve actual node data
	cubic_interpolator_vec3 curve;
	float4 nodes[4]; // w-component contains radius
	optixGetCatmullRomVertexData(
		optixGetGASTraversableHandle(), seg_id, optixGetSbtGASIndex(), 0.f, nodes
	);
	curve.from_catmullrom(nodes);
	quadr_interpolator_vec3 dcurve = curve.derive();

	// compute hit position (world space)
	const float3 pos = calc_hit_point();
		// in the general setting, we would first call optixTransformPointFromWorldToObjectSpace() before
		// doing anything with the hitpoint obtained from a ray (e.g. use it to compute surface normals),
		// since the segment could be in a bottom-level acceleration structure and subject to an additional
		// model or instance transform (we don't use those though so we're fine)

	// compute hit normal
	const float3 normal = calc_segment_surface_normal(curve, dcurve, pos, t);
		// in the general setting, we would first call optixTransformNormalFromObjectToWorldSpace()
		// before doing anything with the normal, since the segment could be in a bottom-level acceleration
		// structure and subject to an additional model or instance transform (we don't use those though so
		// we're fine)

	// compute hit tangent
	const float3 tangent = dcurve.eval(t);

	// compute screen-space position of hitpoint for depth map creation
	const float4 p_screen = mul_mat_pos(params.cam_mvp, pos);
	const float  depth = .5f*(p_screen.z/p_screen.w) + .5f;

	// calculate pre-shading surface color
	const float4 color = {1.f-t, t, 0.f, 1.f};  // visualize curve param as red->green

	// done - store hit results in payload
	set_payload(color, pos, normal, tangent, depth);
}
