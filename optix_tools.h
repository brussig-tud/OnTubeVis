
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

// the inverse of pi
constexpr float pi_inv = 0.31830988618379067153776752674503f;

// one-over-three (aka 1/3)
constexpr float _1o3 = 1.f/3.f;

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
	const unsigned char
		c0 = __float2int_rn(__saturatef(v.x)*255.f), c1 = __float2int_rn(__saturatef(v.y)*255.f),
		c2 = __float2int_rn(__saturatef(v.z)*255.f), c3 = __float2int_rn(__saturatef(v.w)*255.f);
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
__device__ __forceinline__ float4 mul_mat_vec (const float *mat, const float4 &vec)
{
	float4 r;
	r.x = mat[0] * vec.x + mat[4] * vec.y + mat[8] * vec.z + mat[12] * vec.w;
	r.y = mat[1] * vec.x + mat[5] * vec.y + mat[9] * vec.z + mat[13] * vec.w;
	r.z = mat[2] * vec.x + mat[6] * vec.y + mat[10] * vec.z + mat[14] * vec.w;
	r.w = mat[3] * vec.x + mat[7] * vec.y + mat[11] * vec.z + mat[15] * vec.w;
	return r;
}

// apply w-clip to homogenous R^3 vector, returning ordinary R^3 vector
__device__ __forceinline__ float3 w_clip (const float4 &hvec)
{
	float3 r;
	r.x = hvec.x/hvec.w;
	r.y = hvec.y/hvec.w;
	r.z = hvec.z/hvec.w;
	return r;
}

// quick evaluation of scalar cuvic bezier function (without building interpolator first)
__device__ __forceinline__ float eval_cubic_bezier (const float4 &cp, const float t)
{
	// De-Casteljau level 0
	float3 v = {
		mix(cp.x, cp.y, t), mix(cp.y, cp.z, t), mix(cp.z, cp.w, t)
	};

	// De-Casteljau level 1 (reuse local storage from above)
	v.x = mix(v.x, v.y, t);
	v.y = mix(v.y, v.z, t);

	// De-Casteljau level 2 (final result)
	return mix(v.x, v.y, t);
}

// calculate n-th value of Van-der-Corput sequence
__device__ __forceinline__ float van_der_corput (int n, int base)
{
	float vdc = 0.0f;
	int den = 1;
	while (n > 0)
	{
		den *= base;
		int remainder = n % base;
		n /= base;
		vdc += remainder / static_cast<float>(den);
	}
	return vdc;
}

// return the k-th location of a 2D halton sampling pattern
__device__ __forceinline__ float2 sample_halton_2d (unsigned k, int base1, int base2) {
	return {van_der_corput(k, base1), van_der_corput(k, base2)};
}

// obtain a sub-pixel jitter offset for some subframe i
__device__ __forceinline__ float2 obtain_jittered_subpxl_offset (unsigned i, float scale=1.f)
{
	float2 sample = sample_halton_2d(i+1, 2, 3);
	float2 offset = .5f + scale*((sample+sample) - 1.f);
	return offset;
}



//////
//
// Structs
//

// a linear interpolator in R^3 space - uses Bezier basis internally for coherence with higher-order interpolators below
struct linear_interpolator_vec3
{
	// linear bezier basis coefficients
	float3 b[2];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float3 *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
	}

	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (const float4 *b)
	{
		// b1
		this->b[0].x = b[0].x;
		this->b[0].y = b[0].y;
		this->b[0].z = b[0].z;

		// b2
		this->b[1].x = b[1].x;
		this->b[1].y = b[1].y;
		this->b[1].z = b[1].z;
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float3 eval (const float t) const
	{
		// De-Casteljau level 0 (final result)
		return mix(b[0], b[1], t);
	}

	// compute first derivative and return resulting curve (actually, just a constant in case
	// of our linear interpolator). Trusts nvcc to properly perform RVO/copy-elision.
	__device__ __forceinline__ float3 derive (void) const
	{
		return {b[1].x-b[0].x, b[1].y-b[0].y, b[1].z-b[0].z};
	}
};

// a quadratic interpolator in R^3 space - uses Bezier basis internally for efficient evaluation
struct quadr_interpolator_vec3
{
	// quadratic bezier basis coefficients
	float3 b[3];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float3 *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
		this->b[2] = b[2];
	}

	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (const float4 *b)
	{
		// b1
		this->b[0].x = b[0].x;
		this->b[0].y = b[0].y;
		this->b[0].z = b[0].z;

		// b2
		this->b[1].x = b[1].x;
		this->b[1].y = b[1].y;
		this->b[1].z = b[1].z;

		// b3
		this->b[2].x = b[2].x;
		this->b[2].y = b[2].y;
		this->b[2].z = b[2].z;
	}

	// initialize from B-spline control points
	__device__ __forceinline__ void from_bspline (const float3 *s)
	{
		b[0] = .5f*(s[0] + s[1]);
		b[1] = s[1];
		b[2] = .5f*(s[1] + s[2]);
	}

	// initialize from B-spline control points with radius
	__device__ __forceinline__ void from_bspline (const float4 *s)
	{
		// b1
		b[0].x = .5f*(s[0].x + s[1].x);
		b[0].y = .5f*(s[0].y + s[1].y);
		b[0].z = .5f*(s[0].z + s[1].z);

		// b2
		b[1].x = s[1].x;
		b[1].y = s[1].y;
		b[1].z = s[1].z;

		// b3
		b[2].x = .5f*(s[1].x + s[2].x);
		b[2].y = .5f*(s[1].y + s[2].y);
		b[2].z = .5f*(s[1].z + s[2].z);
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

	// compute first derivative and return resulting curve as a quadratic interpolator
	// (trusts nvcc to properly perform RVO/copy-elision)
	__device__ __forceinline__ linear_interpolator_vec3 derive (void) const
	{
		linear_interpolator_vec3 der;

		// b0
		der.b[0].x = 2.f*(b[1].x-b[0].x);
		der.b[0].y = 2.f*(b[1].y-b[0].y);
		der.b[0].z = 2.f*(b[1].z-b[0].z);

		// b1
		der.b[1].x = 2.f*(b[2].x-b[1].x);
		der.b[1].y = 2.f*(b[2].y-b[1].y);
		der.b[1].z = 2.f*(b[2].z-b[1].z);

		return der;
	}
};

// a cubic interpolator in R^3 space - uses Bezier basis internally for efficient evaluation
struct cubic_interpolator_vec3
{
	// cubic bezier basis coefficients
	float3 b[4];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float3 *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
		this->b[2] = b[2];
		this->b[3] = b[3];
	}

	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (const float4 *b)
	{
		// b0
		this->b[0].x = b[0].x;
		this->b[0].y = b[0].y;
		this->b[0].z = b[0].z;

		// b1
		this->b[1].x = b[1].x;
		this->b[1].y = b[1].y;
		this->b[1].z = b[1].z;

		// b2
		this->b[2].x = b[2].x;
		this->b[2].y = b[2].y;
		this->b[2].z = b[2].z;

		// b3
		this->b[3].x = b[3].x;
		this->b[3].y = b[3].y;
		this->b[3].z = b[3].z;
	}

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_catmullrom (const float3 *cr)
	{
		b[0] = cr[1];
		b[1] = cr[1] + (cr[2]-cr[0])/6;
		b[2] = cr[2] - (cr[3]-cr[1])/6;
		b[3] = cr[2];
	}

	// initialize from Catmull-Rom control points with radius
	__device__ __forceinline__ void from_catmullrom (const float4 *cr)
	{
		// b0
		b[0].x = cr[1].x;
		b[0].y = cr[1].y;
		b[0].z = cr[1].z;

		// b1
		b[1].x = cr[1].x + (cr[2].x-cr[0].x)/6;
		b[1].y = cr[1].y + (cr[2].y-cr[0].y)/6;
		b[1].z = cr[1].z + (cr[2].z-cr[0].z)/6;

		// b2
		b[2].x = cr[2].x - (cr[3].x-cr[1].x)/6;
		b[2].y = cr[2].y - (cr[3].y-cr[1].y)/6;
		b[2].z = cr[2].z - (cr[3].z-cr[1].z)/6;

		// b3
		b[3].x = cr[2].x;
		b[3].y = cr[2].y;
		b[3].z = cr[2].z;
	}

	// initialize from Hermite control points
	__device__ __forceinline__ void from_hermite (const float3 &p0, const float3 &m0,
	                                              const float3 &p1, const float3 &m1)
	{
		b[0] = p0;
		b[1] = p0 + _1o3*m0;
		b[2] = p1 - _1o3*m1;
		b[3] = p1;
	}

	// initialize from Hermite control points with radius
	__device__ __forceinline__ void from_hermite (const float4 &p0, const float4 &m0,
	                                              const float4 &p1, const float4 &m1)
	{
		// b0
		b[0].x = p0.x;
		b[0].y = p0.y;
		b[0].z = p0.z;

		// b1
		b[1].x = p0.x + _1o3*m0.x;
		b[1].y = p0.y + _1o3*m0.y;
		b[1].z = p0.z + _1o3*m0.z;

		// b2
		b[2].x = p1.x - _1o3*m1.x;
		b[2].y = p1.y - _1o3*m1.y;
		b[2].z = p1.z - _1o3*m1.z;

		// b3
		b[3].x = p1.x;
		b[3].y = p1.y;
		b[3].z = p1.z;
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

		// b0
		der.b[0].x = 3.f*(b[1].x - b[0].x);
		der.b[0].y = 3.f*(b[1].y - b[0].y);
		der.b[0].z = 3.f*(b[1].z - b[0].z);

		// b1
		der.b[1].x = 3.f*(b[2].x - b[1].x);
		der.b[1].y = 3.f*(b[2].y - b[1].y);
		der.b[1].z = 3.f*(b[2].z - b[1].z);

		// b2
		der.b[2].x = 3.f*(b[3].x - b[2].x);
		der.b[2].y = 3.f*(b[3].y - b[2].y);
		der.b[2].z = 3.f*(b[3].z - b[2].z);

		return der;
	}
};