
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

// GPU representation of tube node attribute data
struct cuda_node
{
	float4 pos_rad;
	float4 color;
	float4 tangent;
	float4 t;
};

// GPU representation of arclength parametrization
struct cuda_arclen
{
	float4 span [4];
};

// OptiX launch parameters
struct curve_rt_params
{
	// input buffers
	cuda_node*     nodes;
	uint2*         node_ids;
	cuda_arclen*   alen;

	// custom attribute input buffers (unused for the built-in variants)
	float3*        positions;
	float1*        radii;

	// output buffers
	float4*        albedo;
	float3*        position;
	float3*        normal;
	float3*        tangent;
	float1*        depth;

	// tube primitive params
	bool           cubic_tangents;

	// TAA params
	unsigned       taa_subframe_id;
	float          taa_jitter_scale;

	// framebuffer dimensions
	unsigned int   fb_width;
	unsigned int   fb_height;

	// camera parameters
	float3         cam_eye, cam_u, cam_v, cam_w;
	float2         cam_clip;
	float          cam_MV[16];
	float          cam_P[16];
	float          cam_N[16];

	// Misc options
	bool show_bvol;

	// the accelleration datastructure to trace
	OptixTraversableHandle accelds;
};

// OptiX raygen constants
struct data_raygen {
	/* nothing */
};

// OptiX miss program constants
struct data_miss {
	// the background color to use
	float4 bgcolor;
};

// OptiX hit program constants
struct data_hit {
	/* nothing */
};
