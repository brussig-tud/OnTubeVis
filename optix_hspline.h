
//////
//
// Includes
//

// OptiX SDK
#include <optix.h>



//////
//
// Structs
//

// a hermite interpolator in R^3 space - uses Bezier basis internally for efficient evaluation.
struct hermite_interpolator_vec3
{
	// cubic bezier basis coefficients
	float3 b[4];

	// initialize from Catmull-Rom control points
	__device__ __forceinline__ void from_catmullrom (const float4* cr)
	{
		const float3 t0 = make_float3(cr[0].x, cr[1].x, cr[2].x);
		b[0] = t0;

		/*// Catrom-to-Poly = Matrix([[-1/2, 3/2, -3/2,  1/2],
		//                          [1,   -5/2,    2, -1/2],
		//                          [-1/2,   0,  1/2,    0],
		//                          [0,      1,    0,    0]])
		p[0] = ((-1.f)*cr[0] +   3.f *cr[1] + (-3.f)*cr[2] +   1.f *cr[3]) / 2.f;
		p[1] = (  2.f *cr[0] + (-5.f)*cr[1] +   4.f *cr[2] + (-1.f)*cr[3]) / 2.f;
		p[2] = ((-1.f)*cr[0] +   1.f *cr[2]) / 2.f;
		p[3] = (  2.f *cr[1]) / 2.f;*/
	}
};
