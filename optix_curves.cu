//
// Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
#include <optix.h>

#include "optix_curves.h"
#include <cuda/helpers.h>
#include <random.h>

#include <sutil/vec_math.h>


extern "C" {
	__constant__ curve_rt_params params;
}

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
	//float3 srgb = toSRGB(clamp(c, 0.0f, 1.0f));
	return make_uchar4(quantizeUnsigned8Bits(c.x), quantizeUnsigned8Bits(c.y), quantizeUnsigned8Bits(c.z), quantizeUnsigned8Bits(c.w));
}

static __forceinline__ __device__ float3 calc_hit_point (void)
{
    const float  t            = optixGetRayTmax();
    const float3 rayOrigin    = optixGetWorldRayOrigin();
    const float3 rayDirection = optixGetWorldRayDirection();
    return rayOrigin + t*rayDirection;
}

static __forceinline__ __device__ void setPayload (float4 p, float d)
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
	const float2 d = 2.0f * make_float2(
		static_cast<float>(idx.x) / static_cast<float>(dim.x),
		static_cast<float>(idx.y) / static_cast<float>(dim.y)
	) - 1.0f;

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
		params.handle,
		ray_origin,
		ray_direction,
		0.0f,                // Min intersection distance
		1e16f,               // Max intersection distance
		0.0f,                // rayTime -- used for motion blur
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
	const unsigned pxl = idx.y*params.image_width + idx.x;
	params.albedo[pxl] = result;
	params.depth[pxl] = depth;
}


extern "C" __global__ void __miss__ms (void)
{
	data_miss *data  = reinterpret_cast<data_miss*>(optixGetSbtDataPointer());
	setPayload(data->bg_color, 1.f);
}


extern "C" __global__ void __closesthit__ch (void)
{
	// When built-in curve intersection is used, the curve parameter u is provided
	// by the OptiX API. The parameter’s range is [0,1] over the curve segment,
	// with u=0 or u=1 only on the end caps.
	const float  u = optixGetCurveParameter();
	const float4 p = make_float4(calc_hit_point(), 1.f);
	const float4 p_screen = mul_mat_vec(params.cam_mvp, p);
	const float  d = .5f * (p_screen.z/p_screen.w) + .5f;
	//printf("%f\n", d);

	// linearly interpolate from red to green
	setPayload(make_float4(1.0f-u, u, 0.0f, 1.0f), d);
}
