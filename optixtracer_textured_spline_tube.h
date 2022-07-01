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

	/// the value of the positions attribute input buffer kernel parameter
	float3* positions;

	/// the value of the radii attribute input buffer kernel parameter
	float1* radii;

	/// the shader binding table for the launch
	OptixShaderBindingTable *sbt;
};



//////
//
// Class definitions
//

// base class with common functionality for all texture spline tube tracers
class optixtracer_textured_spline_tube
{

public:

	/// default constructor
	optixtracer_textured_spline_tube() {};

	/// move constructor
	optixtracer_textured_spline_tube(optixtracer_textured_spline_tube &&other);

	/// virtual base destructor (causes vtable creation)
	virtual ~optixtracer_textured_spline_tube();

	/// destroy the tracer
	virtual void destroy (void) = 0;

	/// obtain the launch parameters for this build
	inline const optix_launch_params& ref_launch_params (void) const { return lp; }

	/// check if the tracer is built and ready
	virtual bool built (void) const = 0;


protected:

	/// move assign another instance of the tracer, destroying the current one
	optixtracer_textured_spline_tube& operator= (optixtracer_textured_spline_tube &&other);

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

// an optixtracer that uses the sphere-based Hermite spline tube primitive intersector by Russig et al.
class optixtracer_textured_spline_tube_russig : public optixtracer_textured_spline_tube, public cgv::render::render_types
{

public:

	/// main superclass type
	typedef optixtracer_textured_spline_tube super;

	/// default constructor - instance will be unusable until a functioning instance is moved in!
	optixtracer_textured_spline_tube_russig();

	/// move constructor
	optixtracer_textured_spline_tube_russig(optixtracer_textured_spline_tube_russig &&other);

	/// destructor
	virtual ~optixtracer_textured_spline_tube_russig();

	/// move assign another instance of the tracer, destroying the current one
	optixtracer_textured_spline_tube_russig& operator= (optixtracer_textured_spline_tube_russig &&other);

	/// destroy the current instance
	virtual void destroy (void);

	/// check if the tracer is built and ready
	virtual bool built (void) const;

	/// build an instance for the given hermite spline tube defintion
	static optixtracer_textured_spline_tube_russig build (
		OptixDeviceContext context, const traj_manager<float>::render_data *render_data
	);

	/// update the acceleration ds with new geometry
	bool update_accelds (const traj_manager<float>::render_data *render_data);


private:

	// helpers for internal structuring
	void destroy_accelds(void);
	void destroy_pipeline(void);
	bool update_pipeline (void);
};

// an optixtracer that uses the built-in disc-based quadratic spline tube primitive defined for the Phantom-Ray-Hair-Intersector by Reshetov et al.
class optixtracer_textured_spline_tube_reshetov : public optixtracer_textured_spline_tube, public cgv::render::render_types
{

public:

	/// main superclass type
	typedef optixtracer_textured_spline_tube super;

	/// default constructor - instance will be unusable until a functioning instance is moved in!
	optixtracer_textured_spline_tube_reshetov();

	/// move constructor
	optixtracer_textured_spline_tube_reshetov(optixtracer_textured_spline_tube_reshetov &&other);

	/// destructor
	virtual ~optixtracer_textured_spline_tube_reshetov();

	/// move assign another instance of the tracer, destroying the current one
	optixtracer_textured_spline_tube_reshetov& operator= (optixtracer_textured_spline_tube_reshetov &&other);

	/// destroy the current instance
	virtual void destroy (void);

	/// check if the tracer is built and ready
	virtual bool built (void) const;

	/// build an instance for the given hermite spline tube defintion
	static optixtracer_textured_spline_tube_reshetov build (
		OptixDeviceContext context, const traj_manager<float>::render_data *render_data
	);

	/// update the acceleration ds with new geometry
	bool update_accelds (const traj_manager<float>::render_data *render_data);


private:

	// helpers for internal structuring
	void destroy_accelds(void);
	void destroy_pipeline(void);
	bool update_pipeline (void);
};

// an optixtracer that uses the built-in disc-based cubic spline tube primitive defined for the Phantom-Ray-Hair-Intersector by Reshetov et al.
class optixtracer_textured_spline_tube_reshetovcubic : public optixtracer_textured_spline_tube, public cgv::render::render_types
{

public:

	/// main superclass type
	typedef optixtracer_textured_spline_tube super;

	/// default constructor - instance will be unusable until a functioning instance is moved in!
	optixtracer_textured_spline_tube_reshetovcubic();

	/// move constructor
	optixtracer_textured_spline_tube_reshetovcubic(optixtracer_textured_spline_tube_reshetovcubic &&other);

	/// destructor
	virtual ~optixtracer_textured_spline_tube_reshetovcubic();

	/// move assign another instance of the tracer, destroying the current one
	optixtracer_textured_spline_tube_reshetovcubic& operator= (optixtracer_textured_spline_tube_reshetovcubic &&other);

	/// destroy the current instance
	virtual void destroy (void);

	/// check if the tracer is built and ready
	virtual bool built (void) const;

	/// build an instance for the given hermite spline tube defintion
	static optixtracer_textured_spline_tube_reshetovcubic build (
		OptixDeviceContext context, const traj_manager<float>::render_data *render_data
	);

	/// update the acceleration ds with new geometry
	bool update_accelds (const traj_manager<float>::render_data *render_data);


private:

	// helpers for internal structuring
	void destroy_accelds(void);
	void destroy_pipeline(void);
	bool update_pipeline (void);
};
