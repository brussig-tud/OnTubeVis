
#ifndef __OPTIX_ISECT_PHANTOM_CUH__
#define __OPTIX_ISECT_PHANTOM_CUH__


//////
//
// Includes
//

// OptiX SDK
#include <optix.h>
#include <sutil/vec_math.h>

// Local includes
#include "optix_tools.cuh"



//////
//
// Typedefs and constants
//

#define MAX_ITERATIONS 24
#define TOLERANCE 0.000244140625f  // = 2^-12



//////
//
// Structs
//

/// struct representing a ray intersection with a spline tube
struct Hit {
	/// the ray parameter at the intersection
	float l;

	/// the curve parameter at the intersection
	float t;
};

/// struct encapsulating various helper data and functionality to transform geometry into a
/// ray-centric coordinate system
struct RCC
{
	/// ray-centric coordiante system model transformation matrix
	mat4 model;

	/// ray-centric coordiante system transformation matrix
	mat4 system;

	/// construct for given ray
	__device__ __forceinline__ RCC(const float3 &ray_orig, const float3 &dir)
	{
		float3 e1, e2, e3 = normalize(dir);
		make_orthonormal_basis(e1, e2, e3);
		model = mat4(
			make_float4(e1, .0f),
			make_float4(e2, .0f),
			make_float4(e3, .0f),
			make_float4(ray_orig, 1.f)
		);
		system = model.inverse();
	}

	/// transform a point into the RCC
	__device__ __forceinline__ float3 to (const float3 &pnt) { return mul3_mat_pos(system, pnt); }

	/// transform a vector into the RCC
	__device__ __forceinline__ float3 vec_to (const float3 &vec) { return mul3_mat_vec(system, vec); }

	/// transform a point given in RCC to world-space
	__device__ __forceinline__ float3 from (const float3 &pnt) { return mul3_mat_pos(model, pnt); }

	/// transform a vector given in RCC to world-space
	__device__ __forceinline__ float3 vec_from (const float3 &vec) { return mul3_mat_vec(model, vec); }
};

struct Curve
{
	/// interpolator along the curve axis position
	quadr_interpolator_vec3 pos;

	/// first derivative of the curve axis position (aka the tangent)
	linear_interpolator_vec3 dpos;

	/// radius interpolator along the curve
	quadr_interpolator_float rad;

	/// first derivative of the radius curve
	linear_interpolator_float drad;

	__device__ __forceinline__ float3 f (const float t) const { return pos.eval(t); }
	__device__ __forceinline__ float3 dfdt (const float t) const { return dpos.eval(t); }

	__device__ __forceinline__ float r (const float t) const { return rad.eval(t); }
	__device__ __forceinline__ float drdt (const float t) const { return drad.eval(t); }

	/// construct a curve from the given geometric Bezier control points and constant radius
	/// (trusts nvcc to properly perform RVO/copy-elision)
	static __device__ __forceinline__ Curve make (
		const float3 &s, const float3 &h, const float3 &t, const float r
	)
	{
		Curve curve;
		curve.pos.from_bezier(s, h, t);
		curve.rad.from_bezier(r, r, r);
		curve.dpos = curve.pos.derive();
		curve.drad = curve.rad.derive();
		return curve;
	}

	/// construct a curve from the given geometric and radius Bezier control points
	/// (trusts nvcc to properly perform RVO/copy-elision)
	static __device__ __forceinline__ Curve make (
		const float3 &s, const float3 &h, const float3 &t, const float rs, const float rh, const float rt
	)
	{
		Curve curve;
		curve.pos.from_bezier(s, h, t);
		curve.rad.from_bezier(rs, rh, rt);
		curve.dpos = curve.pos.derive();
		curve.drad = curve.rad.derive();
		return curve;
	}
};



//////
//
// Functions
//

__device__ bool intersect_cylinder(
	const float3 &ray_orig, const float3 &dir, const float3 &p0, const float3 &p1, float ra
)
{
	float3 ba = p1 - p0;
	float3 oc = ray_orig - p0;

	float baba = dot(ba, ba);
	float bard = dot(ba, dir);
	float baoc = dot(ba, oc);

	float k2 = baba - bard * bard;
	float k1 = baba * dot(oc, dir) - baoc * bard;
	float k0 = baba * dot(oc, oc) - baoc * baoc - ra * ra * baba;

	float h = k1 * k1 - k2 * k0;

	if (h < .0f)
		return false;

	h = sqrtf(h);
	float t = (-k1 - h) / k2;

	// body
	float y = baoc + t * bard;
	if (y > .0f && y < baba)
		return true;

	// caps
	t = ((y < .0f ? .0f : baba) - baoc) / bard;
	if (fabsf(k1 + k2 * t) < h)
		return true;

	return false;
}

struct RayConeIntersection
{
	__device__ bool intersect (float r, float dr)
	{
		float r2  = r * r;
		float drr = r * dr;

		float ddd = cd.x * cd.x + cd.y * cd.y;
		dp        = c0.x * c0.x + c0.y * c0.y;
		float cdd = c0.x * cd.x + c0.y * cd.y;
		float cxd = c0.x * cd.y - c0.y * cd.x;

		float c = ddd;
		float b = cd.z * (drr - cdd);
		float cdz2 = cd.z * cd.z;
		ddd += cdz2;
		float a = 2.0f * drr * cdd + cxd * cxd - ddd * r2 + dp * cdz2;

		float discr = b * b - a * c;
		s   = (b - (discr > .0f ? sqrtf(discr) : .0f)) / c;
		dt  = (s * cd.z - cdd) / ddd;
		dc  = s * s + dp;
		sp  = cdd / cd.z;
		dp += sp * sp;

		return discr > .0f;
	}

	float3 c0;
	float3 cd;
	float s;
	float dt;
	float dp;
	float dc;
	float sp;
};

static __device__ Hit intersect_spline_tube (
	const float3 &ray_orig, const float3 &dir, const float3 &s, const float3 &h, const float3 &t,
	const float rs, const float rh, const float rt
)
{
	// prelude
	// - return value initialization
	Hit hit;
	hit.t = .0f;
	hit.l = pos_inf;
	// - local helpers
	const auto dist_to_cylinder = [&s, &t](float3 pt) {
		return length(cross(pt-s, pt-t)) / length(t-s);
	};

	// construct (untransformed) curve representation
	const float radius_bound = fmax(rs, fmax(rh, rt));
	const auto curve = Curve::make(s, h, t, radius_bound);

	// early rejection test on bounding cylinder
	// ToDo: center cylinder inside convex hull of Bezier control points for a tighter fit, consider performing
	//       exact extrema calculation as in Russig (and probably builtin) intersector
	const float rmax = dist_to_cylinder(curve.f(.5f)) + radius_bound;
	const float3 axis = normalize(t-s), p0 = s - axis*radius_bound, p1 = t + axis*radius_bound;
	if (!intersect_cylinder(ray_orig, dir, p0, p1, rmax))
		return hit;
	/*else
	{
		hit.t = .5f;
		hit.l = 1.f;
		return hit;
	}*/

	// transform curve to RCC
	RCC rcc(ray_orig, dir);
	auto xcurve = Curve::make(rcc.to(s), rcc.to(h), rcc.to(t), rs, rh, rt);

	/* DEBUG *//*{
		const uint3 idx = optixGetLaunchIndex();
		const unsigned pxl = idx.y*params.fb_width + idx.x,
		               mid = (params.fb_height/2)*params.fb_width + (params.fb_width/2);
		if (pxl == mid)
		{
			const auto &m = rcc.xform, &mi = rcc.xformInv;
			printf(" ===================================================\nRCC:\n"
			       "  c = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n"
			       "  m = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n"
			       "col = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n",
			       m.c[0], m.c[4], m.c[ 8], m.c[12], m.c[1], m.c[5], m.c[ 9], m.c[13],
			       m.c[2], m.c[6], m.c[10], m.c[14], m.c[3], m.c[7], m.c[11], m.c[15],
			       m.m[0][0], m.m[1][0], m.m[2][0], m.m[3][0], m.m[0][1], m.m[1][1], m.m[2][1], m.m[3][1],
			       m.m[0][2], m.m[1][2], m.m[2][2], m.m[3][2], m.m[0][3], m.m[1][3], m.m[2][3], m.m[3][3],
			       m.col(0).x, m.col(1).x, m.col(2).x, m.col(3).x, m.col(0).y, m.col(1).y, m.col(2).y, m.col(3).y,
			       m.col(0).z, m.col(1).z, m.col(2).z, m.col(3).z, m.col(0).w, m.col(1).w, m.col(2).w, m.col(3).w);
			printf("  -------\nRCC_inv:\n"
			       "  c = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n"
			       "  m = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n"
			       "col = { %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f,\n"
			       "        %f,  %f,  %f,  %f,\n        %f,  %f,  %f,  %f}\n"
			       " ===================================================\n",
			       mi.c[0], mi.c[4], mi.c[ 8], mi.c[12], mi.c[1], mi.c[5], mi.c[ 9], mi.c[13],
			       mi.c[2], mi.c[6], mi.c[10], mi.c[14], mi.c[3], mi.c[7], mi.c[11], mi.c[15],
			       mi.m[0][0], mi.m[1][0], mi.m[2][0], mi.m[3][0], mi.m[0][1], mi.m[1][1], mi.m[2][1], mi.m[3][1],
			       mi.m[0][2], mi.m[1][2], mi.m[2][2], mi.m[3][2], mi.m[0][3], mi.m[1][3], mi.m[2][3], mi.m[3][3],
			       mi.col(0).x, mi.col(1).x, mi.col(2).x, mi.col(3).x, mi.col(0).y, mi.col(1).y, mi.col(2).y, mi.col(3).y,
			       mi.col(0).z, mi.col(1).z, mi.col(2).z, mi.col(3).z, mi.col(0).w, mi.col(1).w, mi.col(2).w, mi.col(3).w);
		}
	}*/

	// determine which curve end to start iterating from
	float tstart = dot(t - s, dir) > .0f ? .0f : 1.f;

	// also attempt from the other end in case iteration from selected one fails
	for (unsigned end=0; end<2; end++)
	{
		float t    = tstart;
		float told = .0f;
		float dt1  = .0f;
		float dt2  = .0f;

		RayConeIntersection rci;
		for (unsigned i=0; i<MAX_ITERATIONS; i++)
		{
			rci.c0 = xcurve.f(t);
			rci.cd = xcurve.dfdt(t);
			bool phantom = !rci.intersect(xcurve.r(t), xcurve.drdt(t));

			// check convergence
			if (   !phantom && fabsf(rci.dt) < TOLERANCE
				&& t > .0f && t < 1.f) // <-- seems necessary to prevent segment transition artifacts (ToDo: investigate!)
			{
				hit.l = rci.s + rci.c0.z;
				hit.t = t;
				return hit;
			}

			rci.dt = fmin(rci.dt,  .5f);
			rci.dt = fmax(rci.dt, -.5f);

			dt1 = dt2;
			dt2 = rci.dt;

			// regula falsi
			if (dt1 * dt2 < .0f)
			{
				// "we use the simplest possible approach by switching
				//  to the bisection every 4th iteration:"
				told = t;
				t = ((i&3) == 0) ? (.5f*(told+t)) : ((dt2*told - dt1*t) / (dt2-dt1));
			}
			else
			{
				told = t;
				t += rci.dt;
			}

			if (t < .0f || t > 1.f)
				break;
		}

		// re-attempt iteration from the direction of the other end
		tstart = 1.f - tstart;
	}

	// done!
	return hit;
}


#endif // ifndef __OPTIX_ISECT_PHANTOM_CUH__
