
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

struct QTubeNode {
	vec3 pos;
	float rad;
};

struct QTube {
	QTubeNode s;
	QTubeNode h;
	QTubeNode e;
};

struct Hit {
	float l;
	float t;
	vec3 normal;
	vec3 sp;
	bool cap;
};

float Pow2(float x) { return x * x; }
float Pow3(float x) { return x * x * x; }

vec2 SolveQuadratic(float a, float b, float c) {
    if(abs(a) < flt_eps) {
		if(abs(b) < flt_eps)
			return make_float2(-2.f, 2.f);
		else
			return make_float2(-c / b, 2.f);
    } else {
		float discr = b * b - 4.f * a * c;
		if(abs(discr) < flt_eps) return make_float2(-b / (2.f * a), 2.f);
		if(discr < 0.f) return make_float2(-2.f, 2.f);
		vec2 r = (-make_float2(b) + make_float2(-1.f, 1.f) * make_float2(sqrt(discr))) / (2.f * a);
		return r.x < r.y ? make_float2(r.x, r.y) : make_float2(r.y, r.x); // <-- TODO: eliminate first of the two make_float2's
    }
}

void Pow2(const float *c, float *o_c) {
	o_c[0] = c[0] * c[0]; 
	o_c[1] =  2.f * c[0] * c[1];
	o_c[2] = c[1] * c[1] +  2.f * c[0] * c[2];
	o_c[3] =  2.f * c[2] * c[1];
	o_c[4] = c[2] * c[2];
}

void Sub(const float *a, const float *b, float *o_c) {
	o_c[0] = a[0] - b[0];
	o_c[1] = a[1] - b[1];
	o_c[2] = a[2] - b[2];
	o_c[3] = a[3] - b[3];
	o_c[4] = a[4] - b[4];
}

float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4, const float c5, const float c6) { return x * (x * (x * (x * (x * (x * c6 + c5) + c4) + c3) + c2) + c1) + c0; }
float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4, const float c5) { return EvalPoly(x, c0,c1,c2,c3,c4,c5,0.f); }
float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3, const float c4) { return EvalPoly(x, c0,c1,c2,c3,c4,0.f,0.f); }
float EvalPoly(const float x, const float c0, const float c1, const float c2, const float c3) { return EvalPoly(x, c0,c1,c2,c3,0.f,0.f,0.f); }
float EvalPoly(const float x, const float c0, const float c1, const float c2) { return EvalPoly(x, c0,c1,c2,0.f,0.f,0.f,0.f); }
float EvalPoly(const float x, const float c0, const float c1) { return EvalPoly(x, c0,c1,0.f,0.f,0.f,0.f,0.f); }
float EvalPoly(const float x, const float c0) { return EvalPoly(x, c0,0.f,0.f,0.f,0.f,0.f,0.f); }

float EvalPolyD0_3(const float x, const float *c) { return EvalPoly(x, c[0], c[1], c[2]); }
float EvalPolyD1_3(const float x, const float *c) { return EvalPoly(x, c[1], c[2] + c[2]); }
float EvalPolyD2_3(const float x, const float *c) { return EvalPoly(x, c[2] + c[2]); }
float EvalPolyD0_5(const float x, const float *c) { return EvalPoly(x, c[0], c[1], c[2], c[3], c[4]); }
float EvalPolyD1_5(const float x, const float *c) { return EvalPoly(x, c[1], c[2] + c[2], c[3] + c[3] + c[3], c[4] + c[4] + c[4] + c[4]); }
float EvalPolyD2_5(const float x, const float *c) { return EvalPoly(x, c[2] + c[2], c[3] * 6.f, c[4] * 12.f); }
float EvalPolyD3_5(const float x, const float *c) { return EvalPoly(x, c[3] * 6.f, c[4] * 24.f); }

vec3 EvalCSpline(const vec3 &p1, const vec3 &t1, const vec3 &p2, const vec3 &t2, const float t)
{
	vec3 h1 = p1 + _1o3*t1;	// DANGEROUS EDIT!!!
	vec3 h2 = p2 - _1o3*t2;	// DANGEROUS EDIT!!!

	vec3 a1 = mix(p1, h1, t);
	vec3 a2 = mix(h1, h2, t);
	vec3 a3 = mix(h2, p2, t);

	vec3 b1 = mix(a1, a2, t);
	vec3 b2 = mix(a2, a3, t);

	return mix(b1, b2, t);
}

// DANGEROUS EDIT!!! -----------------------------------------------|---------|
#define DEF_FINDROOTS_D1(N)                                                                         \
void FindRootsD1(float poly_C[N1], float x_i[N], int m_i[N], float *x_o, int *m_o)                  \
{	                                                                                                \
    m_o[0] = m_o[N] = 1;                                                                            \
	x_o[0] = x_i[0];                                                                                \
	                                                                                                \
	unsigned j = 0;                                                                                 \
	                                                                                                \
	float x_l = x_i[0];                                                                             \
	float y_l = EvalPolyD1_5(x_l, poly_C); /* DANGEROUS EDIT!!! */                                  \
	float sy_l = copysignf(1.f, y_l);  /* DANGEROUS EDIT!!! */                                      \
                                                                                                    \
	for(unsigned i = 1; i < N; ++i) {                                                               \
		float x_r = x_i[i];                                                                         \
		float y_r = EvalPolyD1_5(x_r, poly_C); /* DANGEROUS EDIT!!! */                              \
		float sy_r = copysignf(1.f, y_r); /* DANGEROUS EDIT!!! */                                   \
		                                                                                            \
		x_o[i] = 0.f;                                                                               \
		                                                                                            \
		if(m_i[i] == 1) {                                                                           \
			if(sy_l != sy_r) {                                                                      \
				float n = x_l;                                                                      \
				float p = x_r;                                                                      \
				float ny = EvalPolyD1_5(n, poly_C); /* DANGEROUS EDIT!!! */                         \
				float py = EvalPolyD1_5(p, poly_C); /* DANGEROUS EDIT!!! */                         \
				                                                                                    \
				if(ny > 0.f && py < 0.f) {                                                          \
					float t = n;                                                                    \
					n = p; p = t;                                                                   \
				}                                                                                   \
				                                                                                    \
				for(unsigned j = 0; j < ITERATION_COUNT; ++j) {                                     \
					float m = (n + p) * 0.5f;                                                       \
					float f = EvalPolyD1_5(m, poly_C); /* DANGEROUS EDIT!!! */                      \
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

// DANGEROUS EDIT!!! -----------------------------------------------|---------|
#define DEF_FINDROOTS_D0(N)                                                                         \
void FindRootsD0(float poly_C[N1], float x_i[N], int m_i[N], float *x_o, int *m_o)                  \
{	                                                                                                \
    m_o[0] = m_o[N] = 1;                                                                            \
	x_o[0] = x_i[0];                                                                                \
	                                                                                                \
	unsigned j = 0;                                                                                 \
	                                                                                                \
	float x_l = x_i[0];                                                                             \
	float y_l = EvalPolyD0_5(x_l, poly_C); /* DANGEROUS EDIT!!! */                                  \
	float sy_l = copysignf(1.f, y_l);  /* DANGEROUS EDIT!!! */                                      \
                                                                                                    \
	for(unsigned i = 1; i < N; ++i) {                                                               \
		float x_r = x_i[i];                                                                         \
		float y_r = EvalPolyD0_5(x_r, poly_C); /* DANGEROUS EDIT!!! */                              \
		float sy_r = copysignf(1.f, y_r); /* DANGEROUS EDIT!!! */                                   \
		                                                                                            \
		x_o[i] = 0.f;                                                                               \
		                                                                                            \
		if(m_i[i] == 1) {                                                                           \
			if(sy_l != sy_r) {                                                                      \
				float n = x_l;                                                                      \
				float p = x_r;                                                                      \
				float ny = EvalPolyD0_5(n, poly_C); /* DANGEROUS EDIT!!! */                         \
				float py = EvalPolyD0_5(p, poly_C); /* DANGEROUS EDIT!!! */                         \
				                                                                                    \
				if(ny > 0.f && py < 0.f) {                                                          \
					float t = n;                                                                    \
					n = p; p = t;                                                                   \
				}                                                                                   \
				                                                                                    \
				for(unsigned j = 0; j < ITERATION_COUNT; ++j) {                                     \
					float m = (n + p) * 0.5f;                                                       \
					float f = EvalPolyD0_5(m, poly_C); /* DANGEROUS EDIT!!! */                      \
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

DEF_FINDROOTS_D1(4)
DEF_FINDROOTS_D0(5)

vec3 GetOrthoVec(const vec3 &v)
{
    return abs(v.x) > abs(v.z) ? make_float3(-v.y, v.x, 0.f) : make_float3(0.f, -v.z, v.y);
}

void SplinePointsToPolyCoeffs(float p0, float h, float p1, float *o_c) // <-- DANGEROUS EDIT!!!
{
	o_c[0] = p0;
	o_c[1] = -2.f * p0 + 2.f * h;
	o_c[2] =   p0 + p1 - 2.f * h;
}

vec3 qSplineEval(float l, float curveX[N0], float curveY[N0], float curveZ[N0])
{
	return make_float3(
		EvalPolyD0_3(l, curveX),
		EvalPolyD0_3(l, curveY),	// DANGEROUS EDITS!!!
		EvalPolyD0_3(l, curveZ)
	);
}

float qSplineIDistEval(float t, float curveX[N0], float polyB_C[N1])
{		
	float term  = EvalPolyD0_3(t, curveX);  // DANGEROUS EDITS!!!
	float discr = EvalPolyD0_5(t, polyB_C); // DANGEROUS EDITS!!!
		
	if(discr < 0.f) return pos_inf;
	else return term - sqrt(discr);
}
#define qSplineIDist_ParasDec float curveX[N0], float polyB_C[N1]
#define qSplineIDist_Paras curveX, polyB_C

float qSplineD1Eval(float t, float curveX[N0], float polyB_C[N1])
{	 		
	float f1D1 = EvalPolyD1_3(t, curveX);	
	float f2D0 = EvalPolyD0_5(t, polyB_C);
	float f2D1 = EvalPolyD1_5(t, polyB_C);

	return f1D1 - f2D1 * 0.5f * __frsqrt_rn(max(0.f, f2D0));
}
#define qSplineD1_ParasDec float curveX[N0], float polyB_C[N1]
#define qSplineD1_Paras curveX, polyB_C

float qSplineD2Eval(float t, float curveX[N0], float polyB_C[N1])
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

float qSplineD3Eval(float t, float polyB_C[N1])
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

DEF_binRootFinder(qSplineIDist)
DEF_binRootFinder(qSplineD1)
DEF_binRootFinder(qSplineD2)
DEF_binRootFinder(qSplineD3)
#define binRootFinder_Eval(n, p, func) func##_BinRootFinder_Eval(n, p, func##_Paras)

void set_mat3_col (float *mat_out, const unsigned col, const float3 &vec) {
	*(((float3*)mat_out)+col) = vec;
}
float3& get_mat3_col(const float *mat, const unsigned col) {
	return *(((float3*)mat)+col);
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
	r.x = mat[0]*vec.x + mat[3]*vec.y + mat[6]*vec.z;
	r.y = mat[1]*vec.x + mat[4]*vec.y + mat[7]*vec.z;
	r.z = mat[2]*vec.x + mat[5]*vec.y + mat[8]*vec.z;
	return r;
}

Hit EvalSplineISect(const vec3 &dir, vec3 s, vec3 h, vec3 t, const float rs, const float rh, const float rt)
{
	Hit hit;
	hit.l = 0.f;
	hit.t = pos_inf;
	hit.normal = make_float3(0.f);
	
	float RM[9]; const vec3 ov=normalize(GetOrthoVec(dir));
	set_mat3_col(RM, 0, dir);
	set_mat3_col(RM, 1, ov);
	const vec3 on=cross(dir, get_mat3_col(RM, 1));
	set_mat3_col(RM, 2, on);
	
	s = mul_vec_mat3(s, RM);
	t = mul_vec_mat3(t, RM); // DANGEROUS EDITS!!!
	h = mul_vec_mat3(h, RM);
	//----------------------------
	// BEGIN: DEBUG OUTPUT
	const uint3 idx = optixGetLaunchIndex();
	if (idx.x==params.fb_width/2 && idx.y==params.fb_height/2)
		printf(
			"dir = (%f, %f, %f)\n"
			" ov = (%f, %f, %f)\n"
			" on = (%f, %f, %f)\n",
			dir.x, dir.y, dir.z, ov.x, ov.y, ov.z, on.x, on.y, on.z
		);
	// END:   DEBUG OUTPUT
	//---------------------------
	/*s *= RM;
	t *= RM;
	h *= RM;*/

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

	{
		float c[N1];
		Pow2(curveY, c);
		
		Sub(polyB_C, c, polyB_C);
	}
	
	{
		float c[N1];
		Pow2(curveZ, c);
		
		Sub(polyB_C, c, polyB_C);
	}
	
	float l1 = 0.f;
	float l2 = 0.f;
	
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
	
	{
		FindRootsD0(polyB_C, x3, m3, x4, m4);

		int rootType = 0;
		int rn = 0;
		if(EvalPolyD0_5(0.f, polyB_C) >= 0.f) {
			roots.x =  0; rn = 1; rootType = 15;
		}
		
		if(m4[1] == 1) {
			if(rn == 0) 		{ roots.x = x4[1]; rn = 1; rootType = 15;						}
			else				{ roots.y = x4[1]; rn = 2; rootType = 10; 						}	
		} else if(rootType == 15) {
			rootType = 0;
		}
		
		if(m4[2] == 1) {
			if     (rn == 0) 	{ roots.x = x4[2]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[2]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else 				{ roots.z = x4[2]; rn = 3; rootType = 20; 						}	
		} else if(rootType == 15) {
			rootType = 0;
		}
		
		if(m4[3] == 1) {
			if     (rn == 0) 	{ roots.x = x4[3]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[3]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else if(rn == 2)	{ roots.z = x4[3]; rn = 3; rootType = 20; 						}	
			else 				{ roots.w = x4[3]; rn = 4; rootType = 20; 						}	
		} else if(rootType == 15) {
			rootType = 0;
		}
		
		if(m4[4] == 1) {
			if     (rn == 0) 	{ roots.x = x4[4]; rn = 1; rootType = 15;						}
			else if(rn == 1)	{ roots.y = x4[4]; rn = 2; rootType = rootType == 0 ? 30 : 10;	}	
			else if(rn == 2)	{ roots.z = x4[4]; rn = 3; rootType = 20; 						}	
			else 				{ roots.w = x4[4]; rn = 4; rootType = 20; 						}	
		} else if(rootType == 15) {
			rootType = 0;
		}
		
		if(EvalPolyD0_5(1.f, polyB_C) > 0.f) {
			if     (rn == 1)	{ roots.y =   1.f; rn = 1; rootType = rootType == 0 ? 30 : 10;	}
			else				{ roots.w =   1.f; rn = 2; rootType = 20; 						}	
		}
		
		if(rootType == 10 || rootType == 15) rootType = 30;

		//region finalize
		if(rootType > 0) {
			if(rootType == 30) {
				float rootD3 = binRootFinder_Eval(roots.x, roots.y, qSplineD3);
				
				vec2 rootsD2;
				rootsD2.x = binRootFinder_Eval(rootD3, roots.x, qSplineD2);
				rootsD2.y = binRootFinder_Eval(rootD3, roots.y, qSplineD2);
				
				l1 = binRootFinder_Eval(roots.x, rootsD2.x, qSplineD1);
				l2 = binRootFinder_Eval(rootsD2.y, roots.y, qSplineD1);
			} else {
				l1 = binRootFinder_Eval(roots.x, roots.y, qSplineD1);
				
				if(rootType == 20)
					l2 = binRootFinder_Eval(roots.z, roots.w, qSplineD1);
				else
					l2 = l1;
			}
			
			float t1 = qSplineIDistEval(l1, curveX, polyB_C);
			float t2 = qSplineIDistEval(l2, curveX, polyB_C);
			
			float r1 = EvalPolyD0_3(l1, rcurve);
			float r2 = EvalPolyD0_3(l2, rcurve);
			
			bool hit1 = t1 > 0.f && r1 > 0.f;
			bool hit2 = t2 > 0.f && r2 > 0.f;
			
			if(hit1) {
				if(t1 < t2 || !hit2) {
					hit.t = t1;
					hit.l = l1;
				} else {
					hit.t = t2;
					hit.l = l2;
				}
			} else {
				hit.t = t2;
				hit.l = l2;
			}

			hit.cap = false;
			if (hit.l == 0.f)
				hit.cap = true;
			if (hit.l == 1.f)
				hit.cap = true;

			// get the position on the spline at l
			hit.sp = qSplineEval(hit.l, curveX, curveY, curveZ);
			// transform from ray space to eye space
			hit.sp = mul_mat3_vec(RM, hit.sp);	// <-- DANGEROUS EDIT!!!   <-- hit.sp = RM * hit.sp;
			// calculate the intersection point of the ray with the spline tube
			vec3 ip = hit.t * dir;
			hit.normal = normalize(ip - hit.sp);
		}
		//endregion
	}

	return hit;
}


#endif // ifndef __OPTIX_ISECT_H__
