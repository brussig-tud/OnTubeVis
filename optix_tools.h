
//////
//
// Includes
//

// OptiX SDK
#include <optix.h>



///////
//
// Utility constants
//

// R^3 null-vector
__constant__ float3 nullvec3 = {0};

// R^4 null-vector
__constant__ float4 nullvec4 = {0};



//////
//
// Functions
//

// Optimized fused multiply-add linear mix implementation (scalar variant)
__device__ __forceinline__ float mix (const float v0, const float v1, const float t)
{
	return fma(t, v1, fma(-t, v0, v0));
}

// Optimized fused multiply-add linear mix implementation (R^2 vector variant)
__device__ __forceinline__ float2 mix (const float2 &v0, const float2 &v1, const float t)
{
	return {
		fma(t, v1.x, fma(-t, v0.x, v0.x)), fma(t, v1.y, fma(-t, v0.y, v0.y))
	};
}

// Optimized fused multiply-add linear mix implementation (R^3 vector variant)
__device__ __forceinline__ float3 mix (const float3 &v0, const float3 &v1, const float t)
{
	return {
		fma(t, v1.x, fma(-t, v0.x, v0.x)), fma(t, v1.y, fma(-t, v0.y, v0.y)), fma(t, v1.z, fma(-t, v0.z, v0.z))
	};
}

// Optimized fused multiply-add linear mix implementation (R^4 vector variant)
__device__ __forceinline__ float4 mix (const float4 &v0, const float4 &v1, const float t)
{
	return {
		fma(t, v1.x, fma(-t, v0.x, v0.x)), fma(t, v1.y, fma(-t, v0.y, v0.y)),
		fma(t, v1.z, fma(-t, v0.z, v0.z)), fma(t, v1.w, fma(-t, v0.w, v0.w))
	};
}

// emulates the GLSL packUnorm4x8 function
__device__ __forceinline__ unsigned pack_unorm_4x8 (const float4 v)
{
	const unsigned char c0 = __float2int_rn(__saturatef(v.x)*255.f),
	                    c1 = __float2int_rn(__saturatef(v.y)*255.f),
	                    c2 = __float2int_rn(__saturatef(v.z)*255.f),
	                    c3 = __float2int_rn(__saturatef(v.w)*255.f);
	return (c3<<24) | (c2<<16) | (c1<<8) | c0;
}

// right-multiply R^3 position vector to 4x4 homogenous transformation matrix, return
// result as homogenous R^3 position vector with w-component
__device__ __forceinline__ float4 mul_mat_pos (const float *mat, const float3 &pos)
{
	float4 r;
	r.x = mat[0]*pos.x + mat[4]*pos.y + mat[ 8]*pos.z + mat[12];
	r.y = mat[1]*pos.x + mat[5]*pos.y + mat[ 9]*pos.z + mat[13];
	r.z = mat[2]*pos.x + mat[6]*pos.y + mat[10]*pos.z + mat[14];
	r.w = mat[3]*pos.x + mat[7]*pos.y + mat[11]*pos.z + mat[15];
	return r;
}

// right-multiply R^3 (direction-)vector to 4x4 homogenous transformation matrix
__device__ __forceinline__ float3 mul_mat_vec (const float *mat, const float3 &vec)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z;
	return r;
}

// right-multiply R^4 vector to 4x4 matrix
__device__ __forceinline__ float4 mul_mat_vec(const float* mat, const float4& vec)
{
	float4 r;
	r.x = mat[0] * vec.x + mat[4] * vec.y + mat[8] * vec.z + mat[12] * vec.w;
	r.y = mat[1] * vec.x + mat[5] * vec.y + mat[9] * vec.z + mat[13] * vec.w;
	r.z = mat[2] * vec.x + mat[6] * vec.y + mat[10] * vec.z + mat[14] * vec.w;
	r.w = mat[3] * vec.x + mat[7] * vec.y + mat[11] * vec.z + mat[15] * vec.w;
	return r;
}



//////
//
// Structs
//

// a quadratic interpolator in R^3 space - uses Bezier basis internally for efficient evaluation
struct quadr_interpolator_vec3
{
	// quadratic bezier basis coefficients
	float3 b[3];

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_bezier (const float3 *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
		this->b[2] = b[2];
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float3 eval (const float t) const
	{
		// De-Casteljau level 0
		float3 v[2] = {
			mix(b[0], b[1], t), mix(b[1], b[2], t)
		};

		// De-Casteljau level 1 (final result)
		return mix(v[0], v[1], t);
	}
};

// a cubic interpolator in R^3 space - uses Bezier basis internally for efficient evaluation
struct cubic_interpolator_vec3
{
	// cubic bezier basis coefficients
	float3 b[4];

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_catmullrom (const float3 *cr)
	{
		// b1
		b[0].x = cr[1].x;
		b[0].y = cr[1].y;
		b[0].z = cr[1].z;

		// b2
		b[1].x = cr[1].x + (cr[2].x-cr[0].x)/6;
		b[1].y = cr[1].y + (cr[2].y-cr[0].y)/6;
		b[1].z = cr[1].z + (cr[2].z-cr[0].z)/6;

		// b3
		b[2].x = cr[2].x - (cr[3].x-cr[1].x)/6;
		b[2].y = cr[2].y - (cr[3].y-cr[1].y)/6;
		b[2].z = cr[2].z - (cr[3].z-cr[1].z)/6;

		// b4
		b[3].x = cr[2].x;
		b[3].y = cr[2].y;
		b[3].z = cr[2].z;
	}

	// initialize from Catmull-Rom control points with radius
	__device__ __forceinline__ void from_catmullrom (const float4 *cr)
	{
		// b1
		b[0].x = cr[1].x;
		b[0].y = cr[1].y;
		b[0].z = cr[1].z;

		// b2
		b[1].x = cr[1].x + (cr[2].x-cr[0].x)/6;
		b[1].y = cr[1].y + (cr[2].y-cr[0].y)/6;
		b[1].z = cr[1].z + (cr[2].z-cr[0].z)/6;

		// b3
		b[2].x = cr[2].x - (cr[3].x-cr[1].x)/6;
		b[2].y = cr[2].y - (cr[3].y-cr[1].y)/6;
		b[2].z = cr[2].z - (cr[3].z-cr[1].z)/6;

		// b4
		b[3].x = cr[2].x;
		b[3].y = cr[2].y;
		b[3].z = cr[2].z;
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float3 eval (const float t) const
	{
		// De-Casteljau level 0
		float3 v[3] = {
			mix(b[0], b[1], t), mix(b[1], b[2], t), mix(b[2], b[3], t)
		};

		// De-Casteljau level 1 (reuse local storage from above)
		v[0] = mix(v[0], v[1], t);
		v[1] = mix(v[1], v[2], t);

		// De-Casteljau level 2 (final result)
		return mix(v[0], v[1], t);
	}

	// compute first derivative and return resulting curve as a quadratic interpolator
	// (trusts nvcc to properly perform RVO/copy-elision)
	__device__ __forceinline__ quadr_interpolator_vec3 derive (void) const
	{
		quadr_interpolator_vec3 der;

		// b1
		der.b[0].x = 3.f*(b[1].x - b[0].x);
		der.b[0].y = 3.f*(b[1].y - b[0].y);
		der.b[0].z = 3.f*(b[1].z - b[0].z);
		// b2
		der.b[1].x = 3.f*(b[2].x - b[1].x);
		der.b[1].y = 3.f*(b[2].y - b[1].y);
		der.b[1].z = 3.f*(b[2].z - b[1].z);
		// b3
		der.b[2].x = 3.f*(b[3].x - b[2].x);
		der.b[2].y = 3.f*(b[3].y - b[2].y);
		der.b[2].z = 3.f*(b[3].z - b[2].z);

		return der;
	}
};
