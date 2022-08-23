
#ifndef __OPTIX_TOOLS_H__
#define __OPTIX_TOOLS_H__

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

// what we consider positive infinity
constexpr float pos_inf = 3e+37;

// what we consider negative infinity
constexpr float neg_inf = -3e+37;

// 32bit floating point epsilon
constexpr float flt_eps = 1.19209290e-07;

// pi
constexpr float pi = 3.1415926535897932384626433832795f;

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

__forceinline__ __device__ float3 get_ortho_vec (const float3 &v)
{
	return abs(v.x) > abs(v.z) ? make_float3(-v.y, v.x, 0.f) : make_float3(0.f, -v.z, v.y);
}

__device__ __forceinline__ void make_orthonormal_basis (float3 &e1, float3 &e2, const float3 &e3)
{
	e2 = normalize(get_ortho_vec(e3));
    e1 = cross(e2, e3);
}

// set a column of the given 4x4 matrix to the given vector
__device__ __forceinline__ void set_mat_col (float *mat, const unsigned col, const float4 &vec)
{
	*(((float4*)mat) + col) = vec;
}

// set a column of the given 4x4 homogenous transformation matrix to the given 3D position vector
// (i.e.  its homogenous coordinate is assumed to be 1)
__device__ __forceinline__ void set_mat_colp (float *mat, const unsigned col, const float3 &vec)
{
	float *c0 = (float*)(((float4*)mat) + col);
	*((float3*)c0) = vec;	// base vector
	c0[3] = 1.f;			// w=1
}

// set a column of the given 4x4 homogenous transformation matrix to the given 3D direction vector
// (i.e.  its homogenous coordinate is assumed to be 0)
__device__ __forceinline__ void set_mat_colv (float *mat, const unsigned col, const float3 &vec)
{
	float *c0 = (float*)(((float4*)mat) + col);
	*((float3*)c0) = vec;	// base vector
	c0[3] = 0.f;			// w=0
}

// reference a column of the given 4x4 matrix
__device__ __forceinline__ const float4& get_mat_col (const float *mat, const unsigned col)
{
	return ((float4*)mat)[col];
}

// reference a column of the given 4x4 homogenous transformation as a 3-vector
__device__ __forceinline__ const float3& get_mat_col3 (const float* mat, const unsigned col)
{
	return *(float3*)(((float4*)mat)+col);
}

// right-multiply R^3 position vector to 4x4 homogenous transformation matrix, return
// result as homogenous R^3 vector with w-component
__device__ __forceinline__ float4 mul_mat_pos (const float *mat, const float3 &pos)
{
	float4 r;
	r.x = mat[0]*pos.x + mat[4]*pos.y + mat[ 8]*pos.z + mat[12];
	r.y = mat[1]*pos.x + mat[5]*pos.y + mat[ 9]*pos.z + mat[13];
	r.z = mat[2]*pos.x + mat[6]*pos.y + mat[10]*pos.z + mat[14];
	r.w = mat[3]*pos.x + mat[7]*pos.y + mat[11]*pos.z + mat[15];
	return r;
}
// right-multiply R^3 position vector to 4x4 homogenous transformation matrix, return
// result as 3D position vector with implied homogenous coordinate w=1
__device__ __forceinline__ float3 mul3_mat_pos(const float* mat, const float3& pos)
{
	float3 r;
	r.x = mat[0]*pos.x + mat[4]*pos.y + mat[ 8]*pos.z + mat[12];
	r.y = mat[1]*pos.x + mat[5]*pos.y + mat[ 9]*pos.z + mat[13];
	r.z = mat[2]*pos.x + mat[6]*pos.y + mat[10]*pos.z + mat[14];
	return r;
}

// left-multiply R^3 position vector to 4x4 homogenous transformation matrix, return
// result as homogenous R^3 vector with w-component
__device__ __forceinline__ float4 mul_pos_mat (const float3 &pos, const float *mat)
{
	float4 r;
	r.x = mat[ 0]*pos.x + mat[ 1]*pos.y + mat[ 2]*pos.z + mat[ 3];
	r.y = mat[ 4]*pos.x + mat[ 5]*pos.y + mat[ 6]*pos.z + mat[ 7];
	r.z = mat[ 8]*pos.x + mat[ 9]*pos.y + mat[10]*pos.z + mat[11];
	r.w = mat[12]*pos.x + mat[13]*pos.y + mat[14]*pos.z + mat[15];
	return r;
}
// left-multiply R^3 position vector to 4x4 homogenous transformation matrix, return
//result as 3D position vector with implied homogenous coordinate w=1
__device__ __forceinline__ float3 mul3_pos_mat (const float3 &pos, const float *mat)
{
	float3 r;
	r.x = mat[0]*pos.x + mat[1]*pos.y + mat[ 2]*pos.z + mat[ 3];
	r.y = mat[4]*pos.x + mat[5]*pos.y + mat[ 6]*pos.z + mat[ 7];
	r.z = mat[8]*pos.x + mat[9]*pos.y + mat[10]*pos.z + mat[11];
	return r;
}

// right-multiply R^3 (direction-)vector to 4x4 homogenous transformation matrix, return
// result as homogenous R^3 vector with w-component
__device__ __forceinline__ float4 mul_mat_vec (const float *mat, const float3 &vec)
{
	float4 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z;
	r.w = mat[3]*vec.x + mat[7]*vec.y + mat[11]*vec.z;
	return r;
}
// right-multiply R^3 (direction-)vector to 4x4 homogenous transformation matrix, return
// result as 3D direction vector with implied homogenous coordinate w=0
__device__ __forceinline__ float3 mul3_mat_vec (const float *mat, const float3 &vec)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z;
	return r;
}

// left-multiply R^3 (direction-)vector to 4x4 homogenous transformation matrix, return
// result as homogenous R^3 vector with w-component
__device__ __forceinline__ float4 mul_vec_mat (const float3 &vec, const float *mat)
{
	float4 r;
	r.x = mat[ 0]*vec.x + mat[ 1]*vec.y + mat[ 2]*vec.z;
	r.y = mat[ 4]*vec.x + mat[ 5]*vec.y + mat[ 6]*vec.z;
	r.z = mat[ 8]*vec.x + mat[ 9]*vec.y + mat[10]*vec.z;
	r.w = mat[12]*vec.x + mat[13]*vec.y + mat[14]*vec.z;
	return r;
}
// left-multiply R^3 (direction-)vector to 4x4 homogenous transformation matrix
// result as 3D direction vector with implied homogenous coordinate w=0
__device__ __forceinline__ float3 mul3_vec_mat (const float3 &vec, const float *mat)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[1]*vec.y + mat[ 2]*vec.z;
	r.y = mat[4]*vec.x + mat[5]*vec.y + mat[ 6]*vec.z;
	r.z = mat[8]*vec.x + mat[9]*vec.y + mat[10]*vec.z;
	return r;
}

// right-multiply R^4 vector to 4x4 matrix
__device__ __forceinline__ float4 mul_mat_vec (const float *mat, const float4 &vec)
{
	float4 r;
	r.x = mat[0]*vec.x + mat[4]*vec.y + mat[ 8]*vec.z + mat[12]*vec.w;
	r.y = mat[1]*vec.x + mat[5]*vec.y + mat[ 9]*vec.z + mat[13]*vec.w;
	r.z = mat[2]*vec.x + mat[6]*vec.y + mat[10]*vec.z + mat[14]*vec.w;
	r.w = mat[3]*vec.x + mat[7]*vec.y + mat[11]*vec.z + mat[15]*vec.w;
	return r;
}
// left-multiply R^4 vector to 4x4 matrix
__device__ __forceinline__ float4 mul_vec_mat (const float4 &vec, const float *mat)
{
	float4 r;
	r.x = mat[ 0]*vec.x + mat[ 1]*vec.y + mat[ 2]*vec.z + mat[ 3]*vec.w;
	r.y = mat[ 4]*vec.x + mat[ 5]*vec.y + mat[ 6]*vec.z + mat[ 7]*vec.w;
	r.z = mat[ 8]*vec.x + mat[ 9]*vec.y + mat[10]*vec.z + mat[11]*vec.w;
	r.w = mat[12]*vec.x + mat[13]*vec.y + mat[14]*vec.z + mat[15]*vec.w;
	return r;
}

// create a system transformation matrix into the given local reference frame from basis vector and origin
__device__ __forceinline__ void make_local_frame (float *mat, const float3 &x, const float3 &y, const float3 &z, const float3 &o)
{
	mat[0] = x.x; mat[4] = x.y; mat[ 8] = x.z;	mat[12] = -x.x*o.x - x.y*o.y - x.z*o.z;
	mat[1] = y.y; mat[5] = y.y; mat[ 7] = y.y;	mat[13] = -y.x*o.x - y.y*o.y - y.z*o.z;
	mat[2] = z.z; mat[6] = z.z; mat[10] = z.z;	mat[14] = -z.x*o.x - z.y*o.y - z.z*o.z;
	mat[3] = 0; mat[7] = 0; mat[11] = 0;	mat[15] = 1;
}

// calculate the determinant of the given 2x2 matrix
__device__ __forceinline__ float det2 (const float m00, const float m01, const float m10, const float m11)
{
    return m00*m11 - m10*m01;
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


// Static-size dense matrix template
template <int N /* rows */,  int M /* columns */>
struct Matrix
{};

// 4x4 matrix
template <> class Matrix<4, 4>
{
public:

	using col_type = float4;

	union {
		float c[16];
		col_type col[4];
		struct {
			col_type col0, col1, col2, col3;
		};
		struct {
			float c00, c01, c02, c03, c10, c11, c12, c13, c20, c21, c22, c23, c30, c31, c32, c33;
		};
	};

public:

	__device__ __forceinline__ Matrix() {}

	__device__ __forceinline__ Matrix(const Matrix<4, 4> &other)
		: col0(other.col0), col1(other.col1), col2(other.col2), col3(other.col3)
	{}

	__device__ __forceinline__ Matrix(const float data[16])
	{
		const auto &cols = (const col_type*)data;
		col[0] = cols[0];
		col[1] = cols[1];
		col[2] = cols[2];
		col[3] = cols[3];
	}

	__device__ __forceinline__ Matrix(
		const float c00, const float c10, const float c20, const float c30,
		const float c01, const float c11, const float c21, const float c31,
		const float c02, const float c12, const float c22, const float c32,
		const float c03, const float c13, const float c23, const float c33
	)
	{
		const auto &cols = (const col_type*)&c00;
		col[0] = cols[0];
		col[1] = cols[1];
		col[2] = cols[2];
		col[3] = cols[3];
	}

	__device__ __forceinline__ Matrix(const col_type cols[4])
		: col0(cols[0]), col1(cols[1]), col2(cols[2]), col3(cols[3])
	{}

	__device__ __forceinline__ Matrix(
		const col_type &col0, const col_type &col1, const col_type &col2, const col_type &col3
	)
		: col0(col0), col1(col1), col2(col2), col3(col3)
	{}

	__device__ __forceinline__ Matrix(const float c00, const float c11, const float c22, const float c33)
		: c00(c00), c01(0), c02(0), c03(0), c10(0), c11(c11), c12(0), c13(0),
		  c20(0), c21(0), c22(c22), c23(0), c30(0), c31(0), c32(0), c33(c33)
	{}

	__device__ __forceinline__ Matrix(const float diagonal)
		: c00(diagonal), c01(0), c02(0), c03(0), c10(0), c11(diagonal), c12(0), c13(0),
		  c20(0), c21(0), c22(diagonal), c23(0), c30(0), c31(0), c32(0), c33(diagonal)
	{}

	__device__ __forceinline__ Matrix& operator= (const Matrix<4,4> &other)
	{
		col[0] = other.col[0];
		col[1] = other.col[1];
		col[2] = other.col[2];
		col[3] = other.col[3];
		return *this;
	}

	__device__ __forceinline__ float& operator() (const unsigned row, const unsigned col)
	{
		return c[col*4 + row];
	}
	__device__ __forceinline__ float operator() (const unsigned row, const unsigned col) const
	{
		return c[col*4 + row];
	}

	__device__ __forceinline__ operator float* (void) { return c; }
	__device__ __forceinline__ operator const float* (void) const { return c; }

	__device__ __forceinline__ float* data (void) { return c; };
	__device__ __forceinline__ const float* data (void) const { return c; };

	__device__ __forceinline__ float4 operator * (const float4 &vec)
	{
		return mul_mat_vec(c, vec);
	}

	__device__ Matrix inverse (void) const
	{
		const float s0 = det2(c00, c01, c10, c11), s1 = det2(c00, c02, c10, c12),
		            s2 = det2(c00, c03, c10, c13), s3 = det2(c01, c02, c11, c12),
		            s4 = det2(c01, c03, c11, c13), s5 = det2(c02, c03, c12, c13),
		            c5 = det2(c22, c23, c32, c33), c4 = det2(c21, c23, c31, c33),
		            c3 = det2(c21, c22, c31, c32), c2 = det2(c20, c23, c30, c33),
		            c1 = det2(c20, c22, c30, c32), c0 = det2(c20, c21, c30, c31),
		            det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;

		return {
			(+c11 * c5 - c12 * c4 + c13 * c3) / det,
			(-c10 * c5 + c12 * c2 + c13 * c1) / det,
			(+c10 * c4 - c11 * c2 + c13 * c0) / det,
			(-c10 * c3 + c11 * c1 + c12 * c0) / det,
			(-c01 * c5 + c02 * c4 - c03 * c3) / det,
			(+c00 * c5 - c02 * c2 + c03 * c1) / det,
			(-c00 * c4 + c01 * c2 - c03 * c0) / det,
			(+c00 * c3 - c01 * c1 + c02 * c0) / det,
			(+c31 * s5 - c32 * s4 + c33 * s3) / det,
			(-c30 * s5 + c32 * s2 - c33 * s1) / det,
			(+c30 * s4 - c31 * s2 + c33 * s0) / det,
			(-c30 * s3 + c31 * s1 - c32 * s0) / det,
			(-c21 * s5 + c22 * s4 - c23 * s3) / det,
			(+c20 * s5 - c22 * s2 + c23 * s1) / det,
			(-c20 * s4 + c21 * s2 - c23 * s0) / det,
			(+c20 * s3 - c21 * s1 + c22 * s0) / det
		};
	}

	// creates an identity matrix
	static __device__ __forceinline__ Matrix identity (void) { return 1.f; }

	// creates a matrix with diagonal set to value
	static __device__ __forceinline__ Matrix diagonal (const float value) { return value; }

	// creates a matrix with all components set to value
	static __device__ __forceinline__ Matrix components (const float value) { return {
		value, value, value, value, value, value, value, value, value, value, value, value, value, value, value, value
	};}
};

// common instances of a dense matrix
typedef Matrix<4, 4> mat4;

// a scalar linear interpolator - uses Bezier basis internally as that maps directly onto fused-multiply-add instructions
struct linear_interpolator_float
{
	// linear bezier basis coefficients
	float b[2];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
	}
	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float b0, const float b1)
	{
		b[0] = b0;
		b[1] = b1;
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float eval (const float t) const
	{
		// De-Casteljau level 0 (final result)
		return mix(b[0], b[1], t);
	}

	// compute first derivative and return resulting curve (actually, just a constant in case
	// of our linear interpolator).
	__device__ __forceinline__ float derive (void) const { return b[1]-b[0]; }
};

// a linear interpolator in R^3 space - uses Bezier basis internally as that maps directly onto fused-multiply-add instructions
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
	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float3 &b0, const float3 &b1)
	{
		b[0] = b0;
		b[1] = b1;
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

	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (const float4 &b0, const float4 &b1)
	{
		// b1
		this->b[0].x = b0.x;
		this->b[0].y = b0.y;
		this->b[0].z = b0.z;

		// b2
		this->b[1].x = b1.x;
		this->b[1].y = b1.y;
		this->b[1].z = b1.z;
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float3 eval (const float t) const
	{
		// De-Casteljau level 0 (final result)
		return mix(b[0], b[1], t);
	}

	// compute first derivative and return resulting curve (actually, just a constant in case
	// of our linear interpolator).
	__device__ __forceinline__ float3 derive (void) const
	{
		return {b[1].x-b[0].x, b[1].y-b[0].y, b[1].z-b[0].z};
	}
};

// a scalar quadratic interpolator - uses Bezier basis internally for efficient evaluation
struct quadr_interpolator_float
{
	// quadratic bezier basis coefficients
	float b[3];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
		this->b[2] = b[2];
	}
	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float b0, const float b1, const float b2)
	{
		b[0] = b0;
		b[1] = b1;
		b[2] = b2;
	}

	// initialize from B-spline control points
	__device__ __forceinline__ void from_bspline (const float *s)
	{
		b[0] = .5f*(s[0] + s[1]);
		b[1] = s[1];
		b[2] = .5f*(s[1] + s[2]);
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float eval (const float t) const
	{
		// De-Casteljau level 0
		float v[2] = {
			mix(b[0], b[1], t), mix(b[1], b[2], t)
		};

		// De-Casteljau level 1 (final result)
		return mix(v[0], v[1], t);
	}

	// compute first derivative and return resulting curve as a quadratic interpolator
	// (trusts nvcc to properly perform RVO/copy-elision)
	__device__ __forceinline__ linear_interpolator_float derive (void) const
	{
		linear_interpolator_float der;

		// b0
		der.b[0] = 2.f*(b[1]-b[0]);

		// b1
		der.b[1] = 2.f*(b[2]-b[1]);

		return der;
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
	__device__ __forceinline__ void from_bezier (
		const float3 &b0, const float3 &b1, const float3 &b2
	)
	{
		b[0] = b0;
		b[1] = b1;
		b[2] = b2;
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
	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (
		const float4 &b0, const float4 &b1, const float4 &b2
	)
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

// a scalar cubic interpolator - uses Bezier basis internally for efficient evaluation
struct cubic_interpolator_float
{
	// cubic bezier basis coefficients
	float b[4];

	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (const float *b)
	{
		this->b[0] = b[0];
		this->b[1] = b[1];
		this->b[2] = b[2];
		this->b[3] = b[3];
	}
	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (
		const float&b0, const float b1, const float b2, const float b3
	)
	{
		b[0] = b0;
		b[1] = b1;
		b[2] = b2;
		b[3] = b3;
	}

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_catmullrom (const float *cr)
	{
		b[0] = cr[1];
		b[1] = cr[1] + (cr[2]-cr[0])/6;
		b[2] = cr[2] - (cr[3]-cr[1])/6;
		b[3] = cr[2];
	}

	// initialize from Hermite control points
	__device__ __forceinline__ void from_hermite (const float p0, const float m0,
	                                              const float p1, const float m1)
	{
		b[0] = p0;
		b[1] = p0 + _1o3*m0;
		b[2] = p1 - _1o3*m1;
		b[3] = p1;
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float eval (const float t) const
	{
		// De-Casteljau level 0
		float v[3] = {
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
	__device__ __forceinline__ quadr_interpolator_float derive (void) const
	{
		quadr_interpolator_float der;

		// b0
		der.b[0] = 3.f*(b[1] - b[0]);

		// b1
		der.b[1] = 3.f*(b[2] - b[1]);

		// b2
		der.b[2] = 3.f*(b[3] - b[2]);

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
	// initialize from Bezier control points
	__device__ __forceinline__ void from_bezier (
		const float3 &b0, const float3 &b1, const float3 &b2, const float3 &b3
	)
	{
		b[0] = b0;
		b[1] = b1;
		b[2] = b2;
		b[3] = b3;
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
	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (
		const float4 &b0, const float4 &b1, const float4 &b2, const float4 &b3
	)
	{
		// b0
		this->b[0].x = b0.x;
		this->b[0].y = b0.y;
		this->b[0].z = b0.z;

		// b1
		this->b[1].x = b1.x;
		this->b[1].y = b1.y;
		this->b[1].z = b1.z;

		// b2
		this->b[2].x = b2.x;
		this->b[2].y = b2.y;
		this->b[2].z = b2.z;

		// b3
		this->b[3].x = b3.x;
		this->b[3].y = b3.y;
		this->b[3].z = b3.z;
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


#endif // ifndef __OPTIX_TOOLS_H__
