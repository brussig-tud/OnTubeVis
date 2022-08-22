
#ifndef __OPTIX_ISECT_RESHETOV_CUH__
#define __OPTIX_ISECT_RESHETOV_CUH__


//////
//
// Includes
//

// OptiX SDK
#include <optix.h>
#include <sutil/vec_math.h>



//////
//
// Structs
//

/// Struct representing a ray intersection with a spline tube
struct Hit {
	/// the ray parameter at the intersection
	float l;

	/// the curve parameter at the intersection
	float t;
};



//////
//
// Functions
//

static __device__ Hit isect (const float3 &ray_orig, const float3 &dir, float3 s, float3 h, float3 t, const float rs, const float rh, const float rt)
{
	// return value initialization
	Hit hit;
	hit.t = 0.f;
	hit.l = pos_inf;

	// done!
	return hit;
}


#endif // ifndef __OPTIX_ISECT_RESHETOV_CUH__
