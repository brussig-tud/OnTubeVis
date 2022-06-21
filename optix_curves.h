
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

// OptiX launch parameters
struct curve_rt_params
{
	// output buffers
	float4*                albedo;
	float3*                position;
	float3*                normal;
	float3*                tangent;
	float1*                depth;

	// framebuffer dimensions
	unsigned int           fb_width;
	unsigned int           fb_height;

	// camera parameters
	float3                 cam_eye;
	float3                 cam_u, cam_v, cam_w;
	float2                 cam_clip;
	float                  cam_mvp[16];

	// the accelleration datastructure to trace
	OptixTraversableHandle accelds;
};

// OptiX raygen constants
struct data_raygen {
	/* nothing */
};

// OptiX miss program constants
struct data_miss
{
	// the background color to use
	float4 bgcolor;
};

// OptiX hit program constants
struct data_hit {
	/* nothing */
};
