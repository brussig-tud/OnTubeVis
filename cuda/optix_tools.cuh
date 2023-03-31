
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

// one-over-six (aka 1/6)
constexpr float _1o6 = 1.f/6.f;

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

__device__ __forceinline__ void make_orthonormal_basis (float3 &e0, float3 &e1, const float3 &ref)
{
	e1 = normalize(get_ortho_vec(ref));
    e0 = cross(e1, ref);
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
	mat[0] = x.x; mat[4] = x.y; mat[ 8] = x.z; mat[12] = -x.x*o.x - x.y*o.y - x.z*o.z;
	mat[1] = y.x; mat[5] = y.y; mat[ 9] = y.z; mat[13] = -y.x*o.x - y.y*o.y - y.z*o.z;
	mat[2] = z.x; mat[6] = z.y; mat[10] = z.z; mat[14] = -z.x*o.x - z.y*o.y - z.z*o.z;
	mat[3] = 0;   mat[7] = 0;   mat[11] = 0;   mat[15] = 1;
}

// calculate the determinant of the given 2x2 matrix
__device__ __forceinline__ float det2 (const float m00, const float m01, const float m10, const float m11)
{
    return m00*m11 - m10*m01;
}

// apply w-clip to homogenous R^3 vector, returning ordinary R^3 vector
__device__ __forceinline__ float3 w_clip (const float4 &hvec) {
	const float w_inv = 1.f/hvec.w;
	return { hvec.x*w_inv, hvec.y*w_inv, hvec.z*w_inv };
}

// quick evaluation of scalar cubic bezier function (without building interpolator first)
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

// quick evaluation of scalar cubic bezier function (without building interpolator first)
__device__ __forceinline__ float eval_cubic_bezier (const float *cp, const float t)
{
	// De-Casteljau level 0
	float3 v = {
		mix(cp[0], cp[1], t), mix(cp[1], cp[2], t), mix(cp[2], cp[3], t)
	};

	// De-Casteljau level 1 (reuse local storage from above)
	v.x = mix(v.x, v.y, t);
	v.y = mix(v.y, v.z, t);

	// De-Casteljau level 2 (final result)
	return mix(v.x, v.y, t);
}

// quick evaluation of a vector-valued cubic bezier function (without building interpolator first)
template <class vec_type>
__device__ __forceinline__ vec_type eval_cubic_bezier (const vec_type *cp, const float t)
{
	// De-Casteljau level 0
	vec_type v[3] = {
		mix(cp[0], cp[1], t), mix(cp[1], cp[2], t), mix(cp[2], cp[3], t)
	};

	// De-Casteljau level 1 (reuse local storage from above)
	v[0] = mix(v[0], v[1], t);
	v[1] = mix(v[1], v[2], t);

	// De-Casteljau level 2 (final result)
	return mix(v[0], v[1], t);
}

// calculate n-th value of Van-der-Corput sequence
__device__ __forceinline__ float van_der_corput (int n, int base)
{
	float vdc = 0.0f;
	int den = 1;
	while (n > 0) {
		den *= base;
		int remainder = n % base;
		n /= base;
		vdc += remainder / (float)den;
	}
	return vdc;
}

// return the k-th location of a 2D halton sampling pattern
__device__ __forceinline__ float2 sample_halton_2d (unsigned k, int base1, int base2) {
	return { van_der_corput(k, base1), van_der_corput(k, base2) };
}

// obtain a sub-pixel jitter offset for some subframe i
__device__ __forceinline__ float2 obtain_jittered_subpxl_offset (unsigned i, float scale=1.f) {
	const float2 sample = sample_halton_2d(i+1, 2, 3);
	return .5f + scale*((sample+sample) - 1.f);
}



//////
//
// Structs
//


// Static-size dense matrix template
template <int N /* rows */,  int M /* columns */>
struct matrix
{};

// 4x4 matrix
template <> class matrix<4, 4>
{
public:

	using col_type = float4;

	struct row_tag {};
	static const row_tag rows;

	union {
		float c[16];
		float m[4][4];
	};


public:

	__device__ __forceinline__ matrix() {}

	__device__ __forceinline__ matrix(const matrix<4, 4> &other)
	{
		*(col_type*)m[0] = *(col_type*)other.m[0];
		*(col_type*)m[1] = *(col_type*)other.m[1];
		*(col_type*)m[2] = *(col_type*)other.m[2];
		*(col_type*)m[3] = *(col_type*)other.m[3];
	}

	__device__ __forceinline__ matrix(const float data[16])
		: c{data[0], data[1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
		    data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]}
	{}
	__device__ __forceinline__ matrix(const float data[16], const row_tag)
		: c{data[0], data[4], data[ 8], data[12], data[ 1], data[ 5], data[ 9], data[13],
		    data[2], data[6], data[10], data[14], data[ 3], data[ 7], data[11], data[15]}
	{}

	__device__ __forceinline__ matrix(
		const float c00, const float c10, const float c20, const float c30,
		const float c01, const float c11, const float c21, const float c31,
		const float c02, const float c12, const float c22, const float c32,
		const float c03, const float c13, const float c23, const float c33
	)
		: c{c00, c10, c20, c30, c01, c11, c21, c31, c02, c12, c22, c32, c03, c13, c23, c33}
	{}

	__device__ __forceinline__ matrix(const col_type cols[4])
	{
		*((col_type*)m[0]) = cols[0];
		*((col_type*)m[1]) = cols[1];
		*((col_type*)m[2]) = cols[2];
		*((col_type*)m[3]) = cols[3];
	}
	__device__ __forceinline__ matrix(const col_type rows[4], const row_tag)
		: c{rows[0].x, rows[1].x, rows[2].x, rows[3].x, rows[0].y, rows[1].y, rows[2].y, rows[3].y,
		    rows[0].z, rows[1].z, rows[2].z, rows[3].z, rows[0].w, rows[1].w, rows[2].w, rows[3].w}
	{}

	__device__ __forceinline__ matrix(
		const col_type &col0, const col_type &col1, const col_type &col2, const col_type &col3
	)
		: c{col0.x, col0.y, col0.z, col0.w, col1.x, col1.y, col1.z, col1.w,
		    col2.x, col2.y, col2.z, col2.w, col3.x, col3.y, col3.z, col3.w}
	{}
	__device__ __forceinline__ matrix(
		const col_type &row0, const col_type &row1, const col_type &row2, const col_type &row3,
		const row_tag
	)
		: c{row0.x, row1.x, row2.x, row3.x, row0.y, row1.y, row2.y, row3.y,
		    row0.z, row1.z, row2.z, row3.z, row0.w, row1.w, row2.w, row3.w}
	{}

	__device__ __forceinline__ matrix(const float c00, const float c11, const float c22, const float c33)
		: c{c00, 0.f, 0.f, 0.f, 0.f, c11, 0.f, 0.f, 0.f, 0.f, c22, 0.f, 0.f, 0.f, 0.f, c33}
	{}

	__device__ __forceinline__ matrix(const float diagonal)
		: c{diagonal, 0.f, 0.f, 0.f, 0.f, diagonal, 0.f, 0.f, 0.f, 0.f, diagonal, 0.f, 0.f, 0.f, 0.f, diagonal}
	{}

	__device__ __forceinline__ matrix& operator= (const matrix<4,4> &other)
	{
		*(col_type*)m[0] = *(col_type*)other.m[0];
		*(col_type*)m[1] = *(col_type*)other.m[1];
		*(col_type*)m[2] = *(col_type*)other.m[2];
		*(col_type*)m[3] = *(col_type*)other.m[3];
		return *this;
	}

	__device__ __forceinline__ float& operator() (const unsigned row, const unsigned col) { return m[col][row]; }
	__device__ __forceinline__ float operator() (const unsigned row, const unsigned col) const { return m[col][row]; }

	__device__ __forceinline__ operator float* (void) { return c; }
	__device__ __forceinline__ operator const float* (void) const { return c; }

	__device__ __forceinline__ float4 operator * (const float4 &vec) { return mul_mat_vec(c, vec); }

	__device__ __forceinline__ float* data (void) { return c; };
	__device__ __forceinline__ const float* data (void) const { return c; };

	__device__ __forceinline__ col_type& col (unsigned i) { return *(col_type*)(m[i]); }
	__device__ __forceinline__ const col_type& col (unsigned i) const { return *(col_type*)(m[i]); }

	__device__ __forceinline__ float3 mul_pos (const float3 &pos) const { return mul3_mat_pos(c, pos); }
	__device__ __forceinline__ float3 mul_dir (const float3 &dir) const { return mul3_mat_vec(c, dir); }

	__device__ __forceinline__ void set_cols (
		const col_type &col0, const col_type &col1, const col_type &col2, const col_type &col3
	)
	{
		*(col_type*)m[0] = col0;
		*(col_type*)m[1] = col1;
		*(col_type*)m[2] = col2;
		*(col_type*)m[3] = col3;
	}

	__device__ __forceinline__ void set_rows (
		const col_type &row0, const col_type &row1, const col_type &row2, const col_type &row3
	)
	{
		c[ 0] = row0.x; c[ 1] = row1.x; c[ 2] = row2.x; c[ 3] = row3.x;
		c[ 4] = row0.y; c[ 5] = row1.y; c[ 6] = row2.y; c[ 7] = row3.y;
		c[ 8] = row0.z; c[ 9] = row1.z; c[10] = row2.z; c[11] = row3.z;
		c[12] = row0.w; c[13] = row1.w; c[14] = row2.w; c[15] = row3.w;
	}

	__device__ __forceinline__ matrix transposed (void) const
	{
		return {c[0], c[4], c[ 8], c[12], c[1], c[5], c[ 9], c[13],
		        c[2], c[6], c[10], c[14], c[3], c[7], c[11], c[15]};
	}

	__device__ matrix inverse (void) const
	{
		const float s0 = det2(m[0][0], m[1][0], m[0][1], m[1][1]),
		            s1 = det2(m[0][0], m[2][0], m[0][1], m[2][1]),
		            s2 = det2(m[0][0], m[3][0], m[0][1], m[3][1]),
		            s3 = det2(m[1][0], m[2][0], m[1][1], m[2][1]),
		            s4 = det2(m[1][0], m[3][0], m[1][1], m[3][1]),
		            s5 = det2(m[2][0], m[3][0], m[2][1], m[3][1]),
		            c5 = det2(m[2][2], m[3][2], m[2][3], m[3][3]),
		            c4 = det2(m[1][2], m[3][2], m[1][3], m[3][3]),
		            c3 = det2(m[1][2], m[2][2], m[1][3], m[2][3]),
		            c2 = det2(m[0][2], m[3][2], m[0][3], m[3][3]),
		            c1 = det2(m[0][2], m[2][2], m[0][3], m[2][3]),
		            c0 = det2(m[0][2], m[1][2], m[0][3], m[1][3]),
		            det_inv = 1.f/(s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0);
		return {
			(+m[1][1]*c5 - m[2][1]*c4 + m[3][1]*c3) * det_inv,
			(-m[0][1]*c5 + m[2][1]*c2 + m[3][1]*c1) * det_inv,
			(+m[0][1]*c4 - m[1][1]*c2 + m[3][1]*c0) * det_inv,
			(-m[0][1]*c3 + m[1][1]*c1 + m[2][1]*c0) * det_inv,
			(-m[1][0]*c5 + m[2][0]*c4 - m[3][0]*c3) * det_inv,
			(+m[0][0]*c5 - m[2][0]*c2 + m[3][0]*c1) * det_inv,
			(-m[0][0]*c4 + m[1][0]*c2 - m[3][0]*c0) * det_inv,
			(+m[0][0]*c3 - m[1][0]*c1 + m[2][0]*c0) * det_inv,
			(+m[1][3]*s5 - m[2][3]*s4 + m[3][3]*s3) * det_inv,
			(-m[0][3]*s5 + m[2][3]*s2 - m[3][3]*s1) * det_inv,
			(+m[0][3]*s4 - m[1][3]*s2 + m[3][3]*s0) * det_inv,
			(-m[0][3]*s3 + m[1][3]*s1 - m[2][3]*s0) * det_inv,
			(-m[1][2]*s5 + m[2][2]*s4 - m[3][2]*s3) * det_inv,
			(+m[0][2]*s5 - m[2][2]*s2 + m[3][2]*s1) * det_inv,
			(-m[0][2]*s4 + m[1][2]*s2 - m[3][2]*s0) * det_inv,
			(+m[0][2]*s3 - m[1][2]*s1 + m[2][2]*s0) * det_inv
		};
	}

	// creates an identity matrix
	static __device__ __forceinline__ matrix identity (void) { return 1.f; }

	// creates a matrix with diagonal set to value
	static __device__ __forceinline__ matrix diagonal (const float value) { return value; }

	// creates a matrix with all components set to value
	static __device__ __forceinline__ matrix all (const float value)
	{
		const float4 col{value, value, value, value};
		return {col, col, col, col};
	}
};

// common instances of a dense matrix
typedef matrix<4, 4> mat4;

/// struct encapsulating various helper data and functionality to build - transform geometry into - a ray-centric
/// coordinate system
struct RCC
{
	/// tag indicating the ray forms the x-axis of the RCC
	struct x_axis_tag {};
	static const x_axis_tag x_axis;

	/// tag indicating the ray forms the y-axis of the RCC
	struct y_axis_tag {};
	static const y_axis_tag y_axis;

	/// tag indicating the ray forms the z-axis of the RCC
	struct z_axis_tag {};
	static const z_axis_tag z_axis;

	/// ray-centric coordiante system model transformation matrix
	mat4 model;

	/// ray-centric coordiante system transformation matrix
	mat4 system;

	/// construct the 2-way (system and model) transformation helper for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local x-axis
	__device__ __forceinline__ RCC(const float3 &orig, const float3 &dir, const x_axis_tag)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		model.set_cols(
			make_float4(dir, .0f),
			make_float4(e0, .0f),
			make_float4(e1, .0f),
			make_float4(orig, 1.f)
		);
		make_local_frame(system, dir, e0, e1, orig);
	}

	/// construct the 2-way (system and model) transformation helper for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local y-axis
	__device__ __forceinline__ RCC(const float3 &orig, const float3 &dir, const y_axis_tag)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		model.set_cols(
			make_float4(e0, .0f),
			make_float4(dir, .0f),
			make_float4(e1, .0f),
			make_float4(orig, 1.f)
		);
		make_local_frame(system, e0, dir, e1, orig);
	}

	/// construct the 2-way (system and model) transformation helper for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local z-axis
	__device__ __forceinline__ RCC(const float3 &orig, const float3 &dir, const z_axis_tag)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		model.set_cols(
			make_float4(e0, .0f),
			make_float4(e1, .0f),
			make_float4(dir, .0f),
			make_float4(orig, 1.f)
		);
		make_local_frame(system, e0, e1, dir, orig);
	}

	/// transform a point into the RCC
	__device__ __forceinline__ float3 to (const float3 &pnt) const { return system.mul_pos(pnt); }

	/// transform a vector into the RCC
	__device__ __forceinline__ float3 vec_to (const float3 &vec) const { return system.mul_dir(vec); }

	/// transform a point given in RCC to world-space
	__device__ __forceinline__ float3 from (const float3 &pnt) const { return model.mul_pos(pnt); }

	/// transform a vector given in RCC to world-space
	__device__ __forceinline__ float3 vec_from (const float3 &vec) const { return model.mul_dir(vec); }

	/// calculate the ray-centric coordinate system model transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local x-axis
	static __device__ __forceinline__ mat4 calc_model_transform (
		const float3 &orig, const float3 &dir, const x_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		return {
			make_float4(dir, .0f),
			make_float4(e0, .0f),
			make_float4(e1, .0f),
			make_float4(orig, 1.f)
		};
	}
	/// calculate the ray-centric coordinate system transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local x-axis
	static __device__ __forceinline__ mat4 calc_system_transform (
		const float3 &orig, const float3 &dir, const x_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		mat4 ret;
		make_local_frame(ret, dir, e0, e1, orig);
		return ret;
	}

	/// calculate the ray-centric coordinate system model transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local y-axis
	static __device__ __forceinline__ mat4 calc_model_transform (
		const float3 &orig, const float3 &dir, const y_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		return {
			make_float4(e0, .0f),
			make_float4(dir, .0f),
			make_float4(e1, .0f),
			make_float4(orig, 1.f)
		};
	}
	/// calculate the ray-centric coordinate system transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local y-axis
	static __device__ __forceinline__ mat4 calc_system_transform (
		const float3 &orig, const float3 &dir, const y_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		mat4 ret;
		make_local_frame(ret, e0, dir, e1, orig);
		return ret;
	}

	/// calculate the ray-centric coordinate system model transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local z-axis
	static __device__ __forceinline__ mat4 calc_model_transform (
		const float3 &orig, const float3 &dir, const z_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		return {
			make_float4(e0, .0f),
			make_float4(e1, .0f),
			make_float4(dir, .0f),
			make_float4(orig, 1.f)
		};
	}
	/// calculate the ray-centric coordinate system transformation for the given ray with the ray direction
	/// (assumed to be normalized) coinciding with the local z-axis
	static __device__ __forceinline__ mat4 calc_system_transform (
		const float3 &orig, const float3 &dir, const z_axis_tag
	)
	{
		float3 e0, e1;
		make_orthonormal_basis(e0, e1, dir);
		mat4 ret;
		make_local_frame(ret, e0, e1, dir, orig);
		return ret;
	}
};

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
	__device__ __forceinline__ float derive (void) const { return b[1] - b[0]; }
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
		this->b[0] = make_float3(b[0]);
		this->b[1] = make_float3(b[1]);
	}
	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (const float4 &b0, const float4 &b1)
	{
		b[0] = make_float3(b0);
		b[1] = make_float3(b1);
	}

	// evaluate interpolation at given t
	__device__ __forceinline__ float3 eval (const float t) const
	{
		// De-Casteljau level 0 (final result)
		return mix(b[0], b[1], t);
	}

	// compute first derivative and return resulting curve (actually, just a constant in case
	// of our linear interpolator).
	__device__ __forceinline__ float3 derive (void) const { return b[1] - b[0]; }
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

	// compute first derivative and return resulting curve as a linear interpolator
	__device__ __forceinline__ linear_interpolator_float derive (void) const
	{
		const float b10 = b[1]-b[0], b21 = b[2]-b[1];
		return {b10+b10, b21+b21};
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
	// initialize from Bezier control points
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
		this->b[0] = make_float3(b[0]);
		this->b[1] = make_float3(b[1]);
		this->b[2] = make_float3(b[2]);
	}
	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (
		const float4 &b0, const float4 &b1, const float4 &b2
	)
	{
		b[0] = make_float3(b0);
		b[1] = make_float3(b1);
		b[2] = make_float3(b2);
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

	// compute first derivative and return resulting curve as a linear interpolator
	__device__ __forceinline__ linear_interpolator_vec3 derive (void) const
	{
		const float3 b10 = b[1]-b[0], b21 = b[2]-b[1];
		return {b10+b10, b21+b21};
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
		b[1] = cr[1] + _1o6*(cr[2]-cr[0]);
		b[2] = cr[2] - _1o6*(cr[3]-cr[1]);
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
	__device__ __forceinline__ float eval (const float t) const { return eval_cubic_bezier(b, t); }

	// compute first derivative and return resulting curve as a quadratic interpolator
	__device__ __forceinline__ quadr_interpolator_float derive (void) const
	{
		const float b10 = b[1]-b[0], b21 = b[2]-b[1], b32 = b[3]-b[2];
		return {b10+b10+b10, b21+b21+b21, b32+b32+b32};
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
		this->b[0] = make_float3(b[0]);
		this->b[1] = make_float3(b[1]);
		this->b[2] = make_float3(b[2]);
		this->b[3] = make_float3(b[3]);
	}
	// initialize from Bezier control points with radius
	__device__ __forceinline__ void from_bezier (
		const float4 &b0, const float4 &b1, const float4 &b2, const float4 &b3
	)
	{
		b[0] = make_float3(b0);
		b[1] = make_float3(b1);
		b[2] = make_float3(b2);
		b[3] = make_float3(b3);
	}

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_catmullrom (const float3 *cr)
	{
		b[0] = cr[1];
		b[1] = cr[1] + _1o6*(cr[2]-cr[0]);
		b[2] = cr[2] - _1o6*(cr[3]-cr[1]);
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
		b[1].x = cr[1].x + _1o6*(cr[2].x-cr[0].x);
		b[1].y = cr[1].y + _1o6*(cr[2].y-cr[0].y);
		b[1].z = cr[1].z + _1o6*(cr[2].z-cr[0].z);

		// b2
		b[2].x = cr[2].x - _1o6*(cr[3].x-cr[1].x);
		b[2].y = cr[2].y - _1o6*(cr[3].y-cr[1].y);
		b[2].z = cr[2].z - _1o6*(cr[3].z-cr[1].z);

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
	__device__ __forceinline__ float3 eval (const float t) const { return eval_cubic_bezier(b, t); }

	// compute first derivative and return resulting curve as a quadratic interpolator
	__device__ __forceinline__ quadr_interpolator_vec3 derive (void) const
	{
		const float3 b10 = b[1]-b[0], b21 = b[2]-b[1], b32 = b[3]-b[2];
		return {b10+b10+b10, b21+b21+b21, b32+b32+b32};
	}
};


#endif // ifndef __OPTIX_TOOLS_H__
