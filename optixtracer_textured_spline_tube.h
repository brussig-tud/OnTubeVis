#pragma once

//////
//
// Includes
//

// C++ STL
#include <vector>

// CGV framework
#include <cgv/render/render_types.h>

// local includes
#include "optix_integration.h"
	#undef min
	#undef max
#include "traj_loader.h"




//////
//
// Structs
//

/// a set of launch parameters provided by an optixtracer
struct optix_launch_params
{
	/// the acceleration datastructure to pass as the traversable
	OptixTraversableHandle accelds;

	/// the OptiX pipeline
	OptixPipeline pipeline;

	/// the device pointer to the kernel parameter storage
	CUdeviceptr params;

	/// the size of the parameter storage
	size_t params_size;

	/// the shader binding table for the launch
	OptixShaderBindingTable *sbt;
};



//////
//
// Class definitions
//

// an optixtracer that uses the builtin phantom-ray-hair-intersector and its disc-based cubic spline tube primitive
class optixtracer_textured_spline_tube_builtin : public cgv::render::render_types
{

public:

	// default constructor - instance will be unusable until a functioning instance is moved in!
	optixtracer_textured_spline_tube_builtin();

	/// move constructor
	optixtracer_textured_spline_tube_builtin(optixtracer_textured_spline_tube_builtin &&other);

	/// destructor
	~optixtracer_textured_spline_tube_builtin();

	/// move assign another instance of the tracer, destroying the current one
	optixtracer_textured_spline_tube_builtin& operator= (optixtracer_textured_spline_tube_builtin&& other);

	/// destroy the current instance
	void destroy (void);

	/// obtain the launch parameters for this build
	inline optix_launch_params& ref_launch_params (void) { return lp; }

	/// check if the tracer is built and ready
	bool built (void);

	/// build an instance for the given hermite spline tube defintion
	static optixtracer_textured_spline_tube_builtin build (OptixDeviceContext context,
	                                                       const traj_manager<float>::render_data *render_data);

	/// update the acceleration ds with new geometry
	bool update_accelds (const traj_manager<float>::render_data *render_data);


private:

	// helpers for internal structuring
	void destroy_accelds(void);
	void destroy_pipeline(void);
	bool update_pipeline (void);

	// the optix context we're operating on
	OptixDeviceContext context = nullptr;

	// the acceleration datastructure memory we're managing
	CUdeviceptr accelds_mem = 0;

	// the program group resources we're managing
	OptixProgramGroup prg_hit = nullptr;
	OptixProgramGroup prg_miss = nullptr;
	OptixProgramGroup prg_raygen = nullptr;

	// the pipeline module resources we're managing
	OptixModule mod_shading = nullptr;
	OptixModule mod_geom = nullptr;

	// our shader binding table
	OptixShaderBindingTable sbt = {};

	// the launch parameters
	optix_launch_params lp = {};
};
