
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
#include <optix_hspline.h>



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

/*static __forceinline__ __device__ float3 mul_mat_pos (const float *mat, const float3 &pos)
{
	float3 r;
	r.x = mat[0]*pos.x + mat[4]*pos.y + mat[ 8]*pos.z + mat[12];
	r.y = mat[1]*pos.x + mat[5]*pos.y + mat[ 9]*pos.z + mat[13];
	r.z = mat[2]*pos.x + mat[6]*pos.y + mat[10]*pos.z + mat[14];
	const float w = mat[3]*pos.x + mat[7]*pos.y + mat[11]*pos.z + mat[15];
	r.x/=w; r.y/=w; r.z/=w;
	return r;
}
static __forceinline__ __device__ float3 mul_mat_vec (const float *mat, const float3 &vec)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z;
	return r;
}*/
static __forceinline__ __device__ float4 mul_mat_vec (const float *mat, const float4 &vec)
{
	float4 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z + mat[12]*vec.w;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z + mat[13]*vec.w;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z + mat[14]*vec.w;
	r.w = mat[3]*vec.x + mat[7]*vec.y + mat[11]*vec.z + mat[15]*vec.w;
	return r;
}

static __forceinline__ __device__ uchar4 make_rgba (const float4 &c)
{
	// first apply gamma, then convert to unsigned char
	//float3 srgb = toSRGB(clamp(c, 0.f, 1.f));
	return make_uchar4(quantizeUnsigned8Bits(c.x), quantizeUnsigned8Bits(c.y), quantizeUnsigned8Bits(c.z), quantizeUnsigned8Bits(c.w));
}

static __forceinline__ __device__ float3 calc_hit_point (void)
{
    const float  t            = optixGetRayTmax();
    const float3 rayOrigin    = optixGetWorldRayOrigin();
    const float3 rayDirection = optixGetWorldRayDirection();
    return rayOrigin + t*rayDirection;
}

static __forceinline__ __device__ void set_payload (float4 p, float d)
{
	optixSetPayload_0(float_as_int(p.x));
	optixSetPayload_1(float_as_int(p.y));
	optixSetPayload_2(float_as_int(p.z));
	optixSetPayload_3(float_as_int(p.w));
	optixSetPayload_4(float_as_int(d));
}


static __forceinline__ __device__ void computeRay (uint3 idx, uint3 dim, float3& origin, float3& direction)
{
	const float3 U = params.cam_u;
	const float3 V = params.cam_v;
	const float3 W = params.cam_w;
	const float2 d = 2.f * make_float2(
		static_cast<float>(idx.x) / static_cast<float>(dim.x),
		static_cast<float>(idx.y) / static_cast<float>(dim.y)
	) - 1.f;

	origin    = params.cam_eye;
	direction = normalize(d.x * U + d.y * V + W);
}


extern "C" __global__ void __raygen__basic (void)
{
	// Lookup our location within the launch grid
	const uint3 idx = optixGetLaunchIndex();
	const uint3 dim = optixGetLaunchDimensions();

	// Map our launch idx to a screen location and create a ray from the camera
	// location through the screen
	float3 ray_origin, ray_direction;
	computeRay(idx, dim, ray_origin, ray_direction);

	// Trace the ray against our scene hierarchy
	unsigned int p0, p1, p2, p3, p4;
	optixTrace(
		params.accelds,
		ray_origin,
		ray_direction,
		0.f,                 // Min intersection distance
		1e16f,               // Max intersection distance
		0.f,                 // rayTime -- used for motion blur
		OptixVisibilityMask( 255 ), // Specify always visible
		OPTIX_RAY_FLAG_NONE,
		0,                   // SBT offset   -- See SBT discussion
		1,                   // SBT stride   -- See SBT discussion
		0,                   // missSBTIndex -- See SBT discussion
		p0, p1, p2, p3, p4);
	float4 result;
	float1 depth;
	result.x = int_as_float(p0);
	result.y = int_as_float(p1);
	result.z = int_as_float(p2);
	result.w = int_as_float(p3);
	depth.x  = int_as_float(p4);

	// Record results in our output raster
	const unsigned pxl = idx.y*params.fb_width + idx.x;
	params.albedo[pxl] = result;
	params.depth[pxl] = depth;
}


extern "C" __global__ void __miss__ms (void)
{
	data_miss *data  = reinterpret_cast<data_miss*>(optixGetSbtDataPointer());
	set_payload(data->bgcolor, 1.f);
}


extern "C" __global__ void __closesthit__ch (void)
{
	// retrieve curve parameter, hitpoint and segment index
	const float  t = optixGetCurveParameter();
	const float4 p = make_float4(calc_hit_point(), 1.f);
	const unsigned segid = optixGetPrimitiveIndex();

	// retrieve actual node data
	float4 nodes[4]; // w-component contains radius
	optixGetCatmullRomVertexData(
		optixGetGASTraversableHandle(), segid, optixGetSbtGASIndex(), 0.f, nodes
	);

	// compute screen-space position of hitpoint for depth map creation
	const float4 p_screen = mul_mat_vec(params.cam_mvp, p);
	const float  d = .5f*(p_screen.z/p_screen.w) + .5f;

	// linearly interpolate from red to green
	set_payload(make_float4(1.f-t, t, 0.f, 1.f), d);
}
