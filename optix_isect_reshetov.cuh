
#ifndef __OPTIX_ISECT_RESHETOV_CUH__
#define __OPTIX_ISECT_RESHETOV_CUH__


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

typedef float2 vec2;
typedef float3 vec3;
typedef float4 vec4;



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
struct TransformToRCC
{
	/// ray-centric coordiante system model transformation matrix
	mat4 xform;

	/// ray-centric coordiante system transformation matrix
	mat4 xformInv;

	/// construct for given ray
	inline TransformToRCC(const float3 &ray_orig, const float3 &dir)
	{
		float3 e1;
		float3 e2;
		float3 e3 = normalize(dir);
		make_orthonormal_basis(e1, e2, e3);
		xformInv = mat4(
			make_float4(e1, 0.f),
			make_float4(e2, 0.f),
			make_float4(e3, 0.f),
			make_float4(ray_orig, 1.f)
		);
		xform = xformInv.inverse();
	}

	/// transform a point into the RCC
	inline vec3 xfmPoint (const vec3 &pnt) { return mul3_mat_pos(xform, pnt); }

	/// transform a vector into the RCC
	inline vec3 xfmVector (const vec3 &vec) { return mul3_mat_vec(xform, vec); }

	/// transform a point given in RCC to world-space
	inline vec3 xfmPointInv (const vec3 &pnt) { return mul3_mat_pos(xformInv, pnt); }

	/// transform a vector given in RCC to world-space
	inline vec3 xfmVectorInv (const vec3 &vec) { return mul3_mat_vec(xformInv, vec); }
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

	vec3 f (const float t) const { return pos.eval(t); }
	vec3 dfdt (const float t) const { return dpos.eval(t); }

	float r (const float t) const { return rad.eval(t); }
	float drdt (const float t) const { return drad.eval(t); }

	/// construct a curve from the given geometric bezier control points and constant radius
	/// (trusts nvcc to properly perform RVO/copy-elision)
	static Curve make (
		const float3 &s, const float3 &h, const float3 &t, const float r
	)
	{
		Curve curve;
		curve.pos.from_bezier(s, h, t);
		curve.rad.from_bezier(r, r, r);
		curve.dpos = curve.pos.derive();
		curve.drad = curve.rad.derive();
		return {};
	}
};



//////
//
// Functions
//

__device__ bool intersectCylinder (
	const float3 &ray_orig, const float3 &dir, const vec3 &p0, const vec3 &p1, float ra
)
{
	vec3 ba = p1 - p0;
	vec3 oc = ray_orig - p0;

	float baba = dot(ba, ba);
	float bard = dot(ba, dir);
	float baoc = dot(ba, oc);

	float k2 = baba - bard * bard;
	float k1 = baba * dot(oc, dir) - baoc * bard;
	float k0 = baba * dot(oc, oc) - baoc * baoc - ra * ra * baba;

	float h = k1 * k1 - k2 * k0;

	if (h < 0.0f)
		return false;

	h = sqrtf(h);
	float t = (-k1 - h) / k2;

	// body
	float y = baoc + t * bard;
	if (y > 0.0f && y < baba)
		return true;

	// caps
	t = ((y < 0.0f ? 0.0f : baba) - baoc) / bard;
	if (fabsf(k1 + k2 * t) < h)
		return true;

	return false;
}

static __device__ Hit isect (
	const float3 &ray_orig, const float3 &dir, const float3 &s, const float3 &h, const float3 &t,
	const float rs, const float rh, const float rt
)
{
	// return value initialization
	Hit hit;
	hit.t = 0.f;
	hit.l = pos_inf;

	// construct our curve representation
	const float radius = rs;
	auto curve = Curve::make(s, h, t, radius);

	// Early exit check against enclosing cylinder
	auto distToCylinder = [&s, &t](vec3 pt) {
		return length(cross(pt-s, pt-t)) / length(t-s);
	};

	// TODO: center cylinder inside convex hull of Bezier control points for a tighter fit
	float rmax = distToCylinder(curve.f(0.5f)) + radius;

	vec3 axis = normalize(t - s);
	vec3 p0   = s - axis*radius;
	vec3 p1   = t + axis*radius;

	if (!intersectCylinder(ray_orig, dir, p0, p1, rmax))
		return hit;

	// Transform curve to RCC
	/*TransformToRCC rcc(r);
	Curve xcurve = make_curve(
		rcc.xfmPoint(curve.w0),
		rcc.xfmPoint(curve.w1),
		rcc.xfmPoint(curve.w2),
		rcc.xfmPoint(curve.w3),
		curve.r
	);

	// "Test for convergence. If the intersection is found,
	// report it, otherwise start at the other endpoint."

	// Compute curve end to start at
	float tstart = dot(xcurve.w3 - xcurve.w0, r.dir) > 0.0f ? 0.0f : 1.0f;

	for (int ep = 0; ep < 2; ++ep)
	{
		float t   = tstart;

		RayConeIntersection rci;

		float told = 0.0f;
		float dt1 = 0.0f;
		float dt2 = 0.0f;

		for (int i = 0; i < 40; ++i)
		{
			rci.c0 = xcurve.f(t);
			rci.cd = xcurve.dfdt(t);

			bool phantom = !rci.intersect(curve.r, 0.0f/*cylinder*//*);

			// "In all examples in this paper we stop iterations when dt < 5x10^−5"
			if (!phantom && fabsf(rci.dt) < 5e-5f)
			{
				//vec3 n = normalize(curve.dfdt(t));
				rci.s += rci.c0.z;
				result.t = rci.s;
				result.u = t; // abuse param u to store curve's t
				result.hit = true;
				result.isect_pos = r.ori + result.t * r.dir;
				break;
			}

			rci.dt = min(rci.dt, 0.5f);
			rci.dt = max(rci.dt, -0.5f);

			dt1 = dt2;
			dt2 = rci.dt;

			// Regula falsi
			if (dt1 * dt2 < 0.0f)
			{
				float tnext = 0.0f;
				// "we use the simplest possible approach by switching
				// to the bisection every 4th iteration:"
				if ((i & 3) == 0)
				{
					tnext = 0.5f * (told + t);
				}
				else
				{
					tnext = (dt2 * told - dt1 * t) / (dt2 - dt1);
				}
				told = t;
				t = tnext;
			}
			else
			{
				told = t;
				t += rci.dt;
			}

			if (t < 0.0f || t > 1.0f)
			{
				break;
			}
		}

		if (!result.hit)
		{
			tstart = 1.0f - tstart;
		}
		else
		{
			break;
		}
	}*/

	// done!
	return hit;
}


#endif // ifndef __OPTIX_ISECT_RESHETOV_CUH__
