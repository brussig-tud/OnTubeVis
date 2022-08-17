
#ifndef __OPTIX_ISECT_H__
#define __OPTIX_ISECT_H__


//////
//
// Includes
//

// OptiX SDK
#include <optix.h>
#include <sutil/vec_math.h>



//////
//
// ToDo: copy-paste-adapted from the rasterization fragment shader and thus introducing a lot of
//       duplicated logic - rewrite to actually integrate better with our existing CUDA code

#define ITERATION_COUNT 10
#define N0 3
#define N1 5

typedef float2 vec2;
typedef float3 vec3;
typedef float4 vec4;

struct Hit {
	float l;
	float t;
};

static __forceinline__ __device__ float Pow2(float x) { return x * x; }
static __forceinline__ __device__ float Pow3(float x) { return x * x * x; }

static __forceinline__ __device__ float sign(float x) {
	const float tmp = x < 0.f ? -1.f : 0.f;
	return x > 0.f ? 1.f : tmp;
}

static __forceinline__ __device__ vec2 SolveQuadratic(float a, float b, float c) {
    if(abs(a) < flt_eps) {
		if(abs(b) < flt_eps)
			return make_float2(-2.f, 2.f);
		else
			return make_float2(-c / b, 2.f);
    } else {
		float discr = b * b - 4.f * a * c;
		if(abs(discr) < flt_eps) return make_float2(-b / (2.f * a), 2.f);
		if(discr < 0.f) return make_float2(-2.f, 2.f);
		const float sqrt_discr = sqrt(discr);
		vec2 r = (-make_float2(b, b) + make_float2(-1.f, 1.f) * make_float2(sqrt_discr, sqrt_discr)) / (2.f * a);
		return r.x < r.y ? make_float2(r.x, r.y) :make_float2(r.y, r.x);
    }
}

static __forceinline__ __device__ void Pow2(const float *c, float *o_c) {
	o_c[0] = c[0] * c[0]; 
	o_c[1] =  2.f * c[0] * c[1];
	o_c[2] = c[1] * c[1] +  2.f * c[0] * c[2];
	o_c[3] =  2.f * c[2] * c[1];
	o_c[4] = c[2] * c[2];
}

static __forceinline__ __device__ void Sub(const float *a, const float *b, float *o_c) {
	o_c[0] = a[0] - b[0];
	o_c[1] = a[1] - b[1];
	o_c[2] = a[2] - b[2];
	o_c[3] = a[3] - b[3];
	o_c[4] = a[4] - b[4];
}

static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4, const float c5, const float c6) { return x * (x * (x * (x * (x * (x * c6 + c5) + c4) + c3) + c2) + c1) + c0; }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4, const float c5) { return EvalPoly(x, c0,c1,c2,c3,c4,c5,0.f); }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4) { return EvalPoly(x, c0,c1,c2,c3,c4,0.f,0.f); }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3) { return EvalPoly(x, c0,c1,c2,c3,0.f,0.f,0.f); }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1, const float c2) { return EvalPoly(x, c0,c1,c2,0.f,0.f,0.f,0.f); }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0, const float c1) { return EvalPoly(x, c0,c1,0.f,0.f,0.f,0.f,0.f); }
static __forceinline__ __device__ float EvalPoly(const float x, const float c0) { return EvalPoly(x, c0,0.f,0.f,0.f,0.f,0.f,0.f); }

static __forceinline__ __device__ float EvalPolyD0_3(const float x, const float *c) { return EvalPoly(x, c[0], c[1], c[2]); }
static __forceinline__ __device__ float EvalPolyD1_3(const float x, const float *c) { return EvalPoly(x, c[1], c[2] + c[2]); }
static __forceinline__ __device__ float EvalPolyD2_3(const float x, const float *c) { return EvalPoly(x, c[2] + c[2]); }
static __forceinline__ __device__ float EvalPolyD0_5(const float x, const float *c) { return EvalPoly(x, c[0], c[1], c[2], c[3], c[4]); }
static __forceinline__ __device__ float EvalPolyD1_5(const float x, const float *c) { return EvalPoly(x, c[1], c[2] + c[2], c[3] + c[3] + c[3], c[4] + c[4] + c[4] + c[4]); }
static __forceinline__ __device__ float EvalPolyD2_5(const float x, const float *c) { return EvalPoly(x, c[2] + c[2], c[3] * 6.f, c[4] * 12.f); }
static __forceinline__ __device__ float EvalPolyD3_5(const float x, const float *c) { return EvalPoly(x, c[3] * 6.f, c[4] * 24.f); }

static __forceinline__ __device__ vec3 EvalCSpline(const vec3 &p1, const vec3 &t1, const vec3 &p2, const vec3 &t2, const float t)
{
	vec3 h1 = p1 + _1o3*t1;
	vec3 h2 = p2 - _1o3*t2;

	vec3 a1 = mix(p1, h1, t);
	vec3 a2 = mix(h1, h2, t);
	vec3 a3 = mix(h2, p2, t);

	vec3 b1 = mix(a1, a2, t);
	vec3 b2 = mix(a2, a3, t);

	return mix(b1, b2, t);
}

#define DEF_FINDROOTS_D1(N)                                                                         \
void FindRootsD1(float poly_C[N1], float x_i[N], int m_i[N], float *x_o, int *m_o)                  \
{	                                                                                                \
    m_o[0] = m_o[N] = 1;                                                                            \
	x_o[0] = x_i[0];                                                                                \
	                                                                                                \
	unsigned j = 0;                                                                                 \
	                                                                                                \
	float x_l = x_i[0];                                                                             \
	float y_l = EvalPolyD1_5(x_l, poly_C);                                                          \
	float sy_l = sign(y_l);                                                                         \
                                                                                                    \
	for(unsigned i = 1; i < N; ++i) {                                                               \
		float x_r = x_i[i];                                                                         \
		float y_r = EvalPolyD1_5(x_r, poly_C);                                                      \
		float sy_r = sign(y_r);                                                                     \
		                                                                                            \
		x_o[i] = 0.f;                                                                               \
		                                                                                            \
		if(m_i[i] == 1) {                                                                           \
			if(sy_l != sy_r) {                                                                      \
				float n = x_l;                                                                      \
				float p = x_r;                                                                      \
				float ny = EvalPolyD1_5(n, poly_C);                                                 \
				float py = EvalPolyD1_5(p, poly_C);                                                 \
				                                                                                    \
				if(ny > 0.f && py < 0.f) {                                                          \
					float t = n;                                                                    \
					n = p; p = t;                                                                   \
				}                                                                                   \
				                                                                                    \
				for(unsigned j = 0; j < ITERATION_COUNT; ++j) {                                     \
					float m = (n + p) * 0.5f;                                                       \
					float f = EvalPolyD1_5(m, poly_C);                                              \
					                                                                                \
					if(f < 0.f) n = m;                                                              \
					else p = m;                                                                     \
				}                                                                                   \
				                                                                                    \
				x_o[i] = (n + p) * 0.5f;                                                            \
                                                                                                    \
				m_o[i] = 1;                                                                         \
			} else {			                                                                    \
				m_o[i] = 0;                                                                         \
			}                                                                                       \
			                                                                                        \
			x_l = x_r;                                                                              \
			y_l = y_r;                                                                              \
			sy_l = sy_r;                                                                            \
		} else {                                                                                    \
			m_o[i] = 0;                                                                             \
		}                                                                                           \
	}                                                                                               \
	                                                                                                \
	x_o[N] = x_i[N - 1];                                                                            \
}

#define DEF_FINDROOTS_D0(N)                                                                         \
void FindRootsD0(float poly_C[N1], float x_i[N], int m_i[N], float *x_o, int *m_o)                  \
{	                                                                                                \
    m_o[0] = m_o[N] = 1;                                                                            \
	x_o[0] = x_i[0];                                                                                \
	                                                                                                \
	unsigned j = 0;                                                                                 \
	                                                                                                \
	float x_l = x_i[0];                                                                             \
	float y_l = EvalPolyD0_5(x_l, poly_C);                                                          \
	float sy_l = sign(y_l);                                                                         \
                                                                                                    \
	for(unsigned i = 1; i < N; ++i) {                                                               \
		float x_r = x_i[i];                                                                         \
		float y_r = EvalPolyD0_5(x_r, poly_C);                                                      \
		float sy_r = sign(y_r);                                                                     \
		                                                                                            \
		x_o[i] = 0.f;                                                                               \
		                                                                                            \
		if(m_i[i] == 1) {                                                                           \
			if(sy_l != sy_r) {                                                                      \
				float n = x_l;                                                                      \
				float p = x_r;                                                                      \
				float ny = EvalPolyD0_5(n, poly_C);                                                 \
				float py = EvalPolyD0_5(p, poly_C);                                                 \
				                                                                                    \
				if(ny > 0.f && py < 0.f) {                                                          \
					float t = n;                                                                    \
					n = p; p = t;                                                                   \
				}                                                                                   \
				                                                                                    \
				for(unsigned j = 0; j < ITERATION_COUNT; ++j) {                                     \
					float m = (n + p) * 0.5f;                                                       \
					float f = EvalPolyD0_5(m, poly_C);                                              \
					                                                                                \
					if(f < 0.f) n = m;                                                              \
					else p = m;                                                                     \
				}                                                                                   \
				                                                                                    \
				x_o[i] = (n + p) * 0.5f;                                                            \
                                                                                                    \
				m_o[i] = 1;                                                                         \
			} else {			                                                                    \
				m_o[i] = 0;                                                                         \
			}                                                                                       \
			                                                                                        \
			x_l = x_r;                                                                              \
			y_l = y_r;                                                                              \
			sy_l = sy_r;                                                                            \
		} else {                                                                                    \
			m_o[i] = 0;                                                                             \
		}                                                                                           \
	}                                                                                               \
	                                                                                                \
	x_o[N] = x_i[N - 1];                                                                            \
}

static __device__ DEF_FINDROOTS_D1(4)
static __device__ DEF_FINDROOTS_D0(5)

static __forceinline__ __device__ vec3 GetOrthoVec(const vec3 &v)
{
    return abs(v.x) > abs(v.z) ? make_float3(-v.y, v.x, 0.f) : make_float3(0.f, -v.z, v.y);
}

static __forceinline__ __device__ void SplinePointsToPolyCoeffs(float p0, float h, float p1, float *o_c)
{
	o_c[0] = p0;
	o_c[1] = -2.f * p0 + 2.f * h;
	o_c[2] =   p0 + p1 - 2.f * h;
}

static __forceinline__ __device__ vec3 qSplineEval(float l, float curveX[N0], float curveY[N0], float curveZ[N0])
{
	return make_float3(
		EvalPolyD0_3(l, curveX),
		EvalPolyD0_3(l, curveY),
		EvalPolyD0_3(l, curveZ)
	);
}

static __forceinline__ __device__ float qSplineIDistEval(float t, float curveX[N0], float polyB_C[N1])
{		
	float term  = EvalPolyD0_3(t, curveX);
	float discr = EvalPolyD0_5(t, polyB_C);
		
	if(discr < 0.f) return pos_inf;
	else return term - sqrt(discr);
}
#define qSplineIDist_ParasDec float curveX[N0], float polyB_C[N1]
#define qSplineIDist_Paras curveX, polyB_C

static __forceinline__ __device__ float qSplineD1Eval(float t, float curveX[N0], float polyB_C[N1])
{	 		
	float f1D1 = EvalPolyD1_3(t, curveX);	
	float f2D0 = EvalPolyD0_5(t, polyB_C);
	float f2D1 = EvalPolyD1_5(t, polyB_C);

	return f1D1 - f2D1 * 0.5f * __frsqrt_rn(max(0.f, f2D0));
}
#define qSplineD1_ParasDec float curveX[N0], float polyB_C[N1]
#define qSplineD1_Paras curveX, polyB_C

static __forceinline__ __device__ float qSplineD2Eval(float t, float curveX[N0], float polyB_C[N1])
{		
	float f1D1 = EvalPolyD1_3(t, curveX);
	float f1D2 = EvalPolyD2_3(t, curveX);
		
	float f2D0 = EvalPolyD0_5(t, polyB_C);
	float f2D1 = EvalPolyD1_5(t, polyB_C);
	float f2D2 = EvalPolyD2_5(t, polyB_C);
	
	f2D0 = max(0.f, f2D0);
	
	return (Pow2(f2D1) / f2D0 * 0.25 - f2D2 * 0.5f) * __frsqrt_rn(f2D0) + f1D2;
}
#define qSplineD2_ParasDec float curveX[N0], float polyB_C[N1]
#define qSplineD2_Paras curveX, polyB_C

static __forceinline__ __device__ float qSplineD3Eval(float t, float polyB_C[N1])
{
	float f2D0 = EvalPolyD0_5(t, polyB_C);
	float f2D1 = EvalPolyD1_5(t, polyB_C);
	float f2D2 = EvalPolyD2_5(t, polyB_C);
	float f2D3 = EvalPolyD3_5(t, polyB_C);

	f2D0 = max(0.f, f2D0);

	return (-3.f * Pow3(f2D1) + 6.f * f2D0 * f2D1 * f2D2 - 4.f * Pow2(f2D0) * f2D3) / Pow2(f2D0) * __frsqrt_rn(f2D0);
}
#define qSplineD3_ParasDec float polyB_C[N1]
#define qSplineD3_Paras polyB_C

#define DEF_binRootFinder(func)                                    \
float func##_BinRootFinder_Eval(float n, float p, func##_ParasDec) \
{		                                                           \
	if(func##Eval(n, func##_Paras) > 0.f) return n;                \
	if(func##Eval(p, func##_Paras) < 0.f) return p;		           \
		                                                           \
	for(unsigned i = 0; i < ITERATION_COUNT; ++i) {                \
		float m = (n + p) * 0.5f;                                  \
		float f = func##Eval(m, func##_Paras);                     \
			                                                       \
		if(f < 0.f) n = m;                                         \
		else p = m;                                                \
	}                                                              \
		                                                           \
	return (n + p) * 0.5f;                                         \
}

static __device__ DEF_binRootFinder(qSplineIDist)
static __device__ DEF_binRootFinder(qSplineD1)
static __device__ DEF_binRootFinder(qSplineD2)
static __device__ DEF_binRootFinder(qSplineD3)
#define binRootFinder_Eval(n, p, func) func##_BinRootFinder_Eval(n, p, func##_Paras)

__device__ __forceinline__ void set_mat3_col (float *mat, const unsigned col, const float3 &vec) {
	((float3*)mat)[col] = vec;
}
__device__ __forceinline__ const float3& get_mat3_col(const float *mat, const unsigned col) {
	return ((float3*)mat)[col];
}

// right-multiply R^3 vector to 3x3 matrix
__device__ __forceinline__ float3 mul_mat3_vec (const float *mat, const float3 &vec)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[3]*vec.y + mat[6]*vec.z;
	r.y = mat[1]*vec.x + mat[4]*vec.y + mat[7]*vec.z;
	r.z = mat[2]*vec.x + mat[5]*vec.y + mat[8]*vec.z;
	return r;
}

// left-multiply R^3 vector to 3x3 matrix
__device__ __forceinline__ float3 mul_vec_mat3 (const float3 &vec, const float *mat)
{
	float3 r;
	r.x = mat[0]*vec.x + mat[1]*vec.y + mat[2]*vec.z;
	r.y = mat[3]*vec.x + mat[4]*vec.y + mat[5]*vec.z;
	r.z = mat[6]*vec.x + mat[7]*vec.y + mat[8]*vec.z;
	return r;
}

static __device__ Hit EvalSplineISect(const vec3 &ray_orig, const vec3 &dir, vec3 s, vec3 h, vec3 t, const float rs, const float rh, const float rt)
{
	Hit hit;
	hit.t = 0.f;
	hit.l = pos_inf;
	
	// transform control points into ray space
	// - basis vectors
	float RM3[9];
	set_mat3_col(RM3, 0, dir);
	set_mat3_col(RM3, 1, normalize(GetOrthoVec(dir)));
	set_mat3_col(RM3, 2, cross(dir, get_mat3_col(RM3, 1)));
	/*float RS[16];
	{ const float3 dir_ortho = normalize(GetOrthoVec(dir));
	  make_local_frame(RS, dir, dir_ortho, cross(dir, dir_ortho), ray_orig); }*/

	// - transform control points
	s = mul_vec_mat3(s, RM3);
	t = mul_vec_mat3(t, RM3);
	h = mul_vec_mat3(h, RM3);
	/*s = mul3_pos_mat(s, RS);
	t = mul3_pos_mat(t, RS);
	h = mul3_pos_mat(h, RS);*/

	float curveX[N0];
	float curveY[N0];
	float curveZ[N0];
	float rcurve[N0];
	
	SplinePointsToPolyCoeffs(s.x, h.x, t.x, curveX);
	SplinePointsToPolyCoeffs(s.y, h.y, t.y, curveY);
	SplinePointsToPolyCoeffs(s.z, h.z, t.z, curveZ);
	
	SplinePointsToPolyCoeffs(rs, rh, rt, rcurve);
	
	float polyB_C[N1];
		
	Pow2(rcurve, polyB_C);

	/* local scope */ {
		float c[N1];
		Pow2(curveY, c);
		
		Sub(polyB_C, c, polyB_C);
	}

	/* local scope */ {
		float c[N1];
		Pow2(curveZ, c);
		
		Sub(polyB_C, c, polyB_C);
	}
	
	float t1 = 0.f;
	float t2 = 0.f;
	
	vec4 roots = make_float4(-1.f);
	
	float x2[4];
	int m2[4];
	
	vec2 r = SolveQuadratic(polyB_C[4] * 12.f, polyB_C[3] * 6.f, polyB_C[2] * 2.f);
	
	x2[0] = 0.f;
	x2[1] = r.x;
	x2[2] = r.y;
	x2[3] = 1.f;

	m2[0] = 1;
	m2[1] = (x2[1] <= 0.f || x2[1] >= 1.f) ? 0 : 1;
	m2[2] = (x2[2] <= 0.f || x2[2] >= 1.f) ? 0 : 1;
	m2[3] = 1;

	float x3[5];
	int m3[5];
	FindRootsD1(polyB_C, x2, m2, x3, m3);

	float x4[6];
	int m4[6];
	
	/* local scope */ {
		FindRootsD0(polyB_C, x3, m3, x4, m4);

		int rootType = 0;
		int rn = 0;
		if (EvalPolyD0_5(0.f, polyB_C) >= 0.f) {
			roots.x =  0; rn = 1; rootType = 15;
		}
		
		if (m4[1] == 1) {
			if(rn == 0) 		{ roots.x = x4[1]; rn = 1; rootType = 15;						}
			else				{ roots.y = x4[1]; rn = 2; rootType = 10; 						}	
		}
		else if(rootType == 15) {
			rootType = 0;
		}
		
		if (m4[2] == 1) {
			if     (rn == 0) 	{ roots.x = x4[2]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[2]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else 				{ roots.z = x4[2]; rn = 3; rootType = 20; 						}	
		}
		else if(rootType == 15) {
			rootType = 0;
		}
		
		if (m4[3] == 1) {
			if     (rn == 0) 	{ roots.x = x4[3]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[3]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else if(rn == 2)	{ roots.z = x4[3]; rn = 3; rootType = 20; 						}	
			else 				{ roots.w = x4[3]; rn = 4; rootType = 20; 						}	
		}
		else if(rootType == 15) {
			rootType = 0;
		}
		
		if (m4[4] == 1) {
			if     (rn == 0) 	{ roots.x = x4[4]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[4]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else if(rn == 2)	{ roots.z = x4[4]; rn = 3; rootType = 20; 						}	
			else 				{ roots.w = x4[4]; rn = 4; rootType = 20; 						}	
		}
		else if(rootType == 15) {
			rootType = 0;
		}
		
		if (EvalPolyD0_5(1.f, polyB_C) > 0.f) {
			if     (rn == 1)	{ roots.y =   1.f; rn = 1; rootType = rootType == 0 ? 30 : 10;	}
			else				{ roots.w =   1.f; rn = 2; rootType = 20; 						}	
		}
		
		if(rootType == 10 || rootType == 15) rootType = 30;

		//region finalize
		if (rootType > 0) {
			if (rootType == 30) {
				float rootD3 = binRootFinder_Eval(roots.x, roots.y, qSplineD3);
				
				vec2 rootsD2;
				rootsD2.x = binRootFinder_Eval(rootD3, roots.x, qSplineD2);
				rootsD2.y = binRootFinder_Eval(rootD3, roots.y, qSplineD2);
				
				t1 = binRootFinder_Eval(roots.x, rootsD2.x, qSplineD1);
				t2 = binRootFinder_Eval(rootsD2.y, roots.y, qSplineD1);
			}
			else {
				t1 = binRootFinder_Eval(roots.x, roots.y, qSplineD1);
				
				if(rootType == 20)
					t2 = binRootFinder_Eval(roots.z, roots.w, qSplineD1);
				else
					t2 = t1;
			}
			
			float l1 = qSplineIDistEval(t1, curveX, polyB_C);
			float l2 = qSplineIDistEval(t2, curveX, polyB_C);
			
			float r1 = EvalPolyD0_3(t1, rcurve);
			float r2 = EvalPolyD0_3(t2, rcurve);
			
			bool hit1 = l1 > 0.f && r1 > 0.f;
			bool hit2 = l2 > 0.f && r2 > 0.f;
			
			if (hit1)
			{
				if (l1 < l2 || !hit2) {
					hit.l = l1;
					hit.t = t1;
				}
				else {
					hit.l = l2;
					hit.t = t2;
				}
			}
			else {
				hit.l = l2;
				hit.t = t2;
			}
		}
		//endregion
	}

	// done!
	return hit;
}


#endif // ifndef __OPTIX_ISECT_H__
