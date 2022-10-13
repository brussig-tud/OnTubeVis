
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
struct Hit
{
	/// the ray parameter at the intersection
	float l;

	/// the curve parameter at the intersection
	float t;
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

__device__ __forceinline__ float dist_to_cylinder (const float3 &s, const float3 &t, const float3 &pt)
{
	return length(cross(pt-s, pt-t)) / length(t-s);
}


///// MOVE TO optix_tools.cuh ///////////////////////////////////////////////////////////////////////////////
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



static __device__ Hit intersect_spline_tube (
	const mat4 &rcc, const float3 &ray_orig, const float3 &dir,
	const float3 &s, const float3 &h, const float3 &t, const float rs, const float rh, const float rt
)
{
	// prelude
	Hit hit;
	hit.t = .0f;
	hit.l = pos_inf;

	// construct (untransformed) curve representation
	const float radius_bound = fmax(rs, fmax(rh, rt));
	const auto curve = Curve::make(s, h, t, radius_bound);

	// early rejection test on bounding cylinder
	// Todos:
	// 	- move these calculations to ray-centric coordinates (more efficient + gets rid of ray orig/dir params
	// 	- center cylinder inside convex hull of Bezier control points for a tighter fit, consider performing
	//    exact extrema calculation as in Russig et al. intersector (and probably the builtin one too)
	const float rmax = dist_to_cylinder(s, t, curve.f(.5f)) + radius_bound;
	const float3 axis = normalize(t-s), p0 = s - axis*radius_bound, p1 = t + axis*radius_bound;
	if (!intersect_cylinder(ray_orig, dir, p0, p1, rmax))
		return hit;
	/*else {
		hit.t = .5f;
		hit.l = 1.f;
		return hit;
	}*/

	// transform curve into ray-centric coordinate system
	const auto xcurve = Curve::make(rcc.mul_pos(s), rcc.mul_pos(h), rcc.mul_pos(t), rs, rh, rt);

	/* DEBUG *//*{
		const uint3 idx = optixGetLaunchIndex();
		const unsigned pxl = idx.y*params.fb_width + idx.x,
		               mid = (params.fb_height/2)*params.fb_width + (params.fb_width/2);
		if (pxl == mid)
			// do printf output here
	}*/

	// determine which curve end to start iterating from
	float tstart = dot(t-s, dir) > .0f ? .0f : 1.f;

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
			const bool phantom = !rci.intersect(xcurve.r(t), xcurve.drdt(t));

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
			else {
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
