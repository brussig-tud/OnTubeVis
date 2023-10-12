
//////
//
// Includes
//

// OptiX SDK
#include <optix.h>



//////
//
// Macros & constants
//

/// Convenience helper for treating float arrays like a mat4
#define MAT4(marray) (*(mat4*)(&marray))



//////
//
// Structs
//

/// GPU representation of tube node attribute data
struct cuda_node
{
	float4 pos_rad;
	float4 color;
	float4 tangent;
	float4 frame_normal_t;
};

/// GPU representation of arclength parametrization
struct cuda_arclen {
	float4 span [4];
};

/// holographic rendering mode
enum Holo {
	// disabled
	OFF = 0,

	// render directly into hologram
	ON,

	// render a lightfield quilt
	QUILT
};

/// OptiX launch parameters
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

	// tube primitive params (reflects properties from the textured spline tube render style)
	bool           cubic_tangents;
	float          max_t;

	// TAA params
	unsigned       taa_subframe_id;
	float          taa_jitter_scale;

	// viewport and framebuffer dimensions
	// (framebuffer dims should be 3x as wide as viewport dims in holography mode, and identical
	// when holography is disabled)
	uint3          viewport_dims;
	uint3          framebuf_dims;

	// camera parameters
	float3         cam_eye;	// in holo mode, this is the cyclopic eye
	float2         cam_clip;
	float          cam_MV[16];
	float          cam_invMV[16];
	float          cam_P[16];
	float          cam_invP[16];
	float          cam_N[16];

	// Misc options
	bool show_bvol;

	// holography
	Holo holo;
	float2 screen_size;
	float parallax_zero_depth;
	float holo_eye, holo_eyes_dist;
	float holo_MV_left[16], holo_invMV_left[16], holo_MV_right[16], holo_invMV_right[16];
	float holo_P_left[16], holo_invP_left[16], holo_P_right[16], holo_invP_right[16];
	float holo_invMVP_left[16], holo_invMVP_right[16];
	bool unproject_mode_dbg;

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
