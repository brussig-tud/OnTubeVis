
//////
//
// Includes
//

// C++ STL
#include <fstream>

// OpenGL
#include <GL/glew.h>

// CGV framework
#include <cgv/render/render_types.h>
#include <cgv/render/texture.h>

// local includes
#include "optix_integration.h"
#include "curveutils.h"

// OptiX/CUDA kernels
#include "cuda/optix_interface.h"

// implemented header
#include "optixtracer_textured_spline_tube.h"



//////
//
// Local helpers
//

namespace {

	template <class int_type>
	inline int_type ceil_unsigned (const int_type v, const int_type base) {
		return ((v+base-1)/base)*base;
	}

	inline float get_s0 (const float b0, const float b1) {
		return b0 - (b1-b0);
	}
	template <class T>
	inline T get_s0 (const T &b0, const T &b1) {
		return b0 - (b1-b0);
	}

	inline float get_s2 (const float b1, const float b2) {
		return b2 + (b2-b1);
	}
	template <class T>
	inline T get_s2 (const T &b1, const T &b2) {
		return b2 + (b2-b1);
	}

	inline float get_cr0 (const float p0, const float m0, const float p1)
	{
		const float b1 = p0 + _1o3<float>*m0;
		return p1 + 6.f*(p0 - b1);
	}
	template <class T>
	inline T get_cr0 (const T &p0, const T &m0, const T &p1)
	{
		const T b1 = p0 + _1o3<T::value_type>*m0;
		return p1 + 6.f*(p0 - b1);
	}

	inline float get_cr3 (const float p0, const float p1, const float m1)
	{
		const float b2 = p1 - _1o3<float>*m1;
		return p0 + 6.f*(p1 - b2);
	}
	template <class T>
	inline T get_cr3 (const T &p0, const T &p1, const T &m1)
	{
		const T b2 = p1 - _1o3<T::value_type>*m1;
		return p0 + 6.f*(p1 - b2);
	}

	inline void split_quadseg (
		float *b0, float *b1, float *b2, float *g0, float *g1, float *g2,
		const float p0, const float m0, const float p1, const float m1
	)
	{
		*b0 = p0;
		*b1 = p0 + 0.2667f*m0;
		*g1 = p1 - 0.2667f*m1;
		*g0 = *b2 = .5f*(*b1 + *g1);
		*g2 = p1;
	}
	template <class T>
	inline void split_quadseg (
		T *b0, T *b1, T *b2, T *g0, T *g1, T *g2,
		const T &p0, const T &m0, const T &p1, const T &m1
	)
	{
		*b0 = p0;
		*b1 = p0 + 0.2667f*m0;
		*g1 = p1 - 0.2667f*m1;
		*g0 = *b2 = .5f*(*b1 + *g1);
		*g2 = p1;
	}

	template <unsigned dims>
	Vec<dims, float> get_curve_minimum (
		const QuadraticCurve<float, dims> &bezier, const LinearRoots3<float> &extrema
	)
	{
		const Vec<dims, float> bstart = evalBezier(bezier, .0f),
		                       bend   = evalBezier(bezier, 1.f);
		Vec<dims, float> pmin;
		for (int i=0; i<3; i++)
		{
			float extremum = extrema.num[i] && isBetween01(extrema.roots[i]) ?
				  evalBezier(bezier.row(i), extrema.roots[i])
				: _posInf<float>;
			pmin[i] = std::min(bstart[i], std::min(extremum, bend[i]));
		}
		return pmin;
	}

	template <unsigned dims>
	Vec<dims, float> get_curve_maximum (
		const QuadraticCurve<float, dims> &bezier, const LinearRoots3<float> &extrema
	)
	{
		const Vec<dims, float> bstart = evalBezier(bezier, .0f),
		                       bend   = evalBezier(bezier, 1.f);
		Vec<dims, float> pmax;
		for (int i=0; i<3; i++)
		{
			float extremum = extrema.num[i] && isBetween01(extrema.roots[i]) ?
				  evalBezier(bezier.row(i), extrema.roots[i])
				: _negInf<float>;
			pmax[i] = std::max(bstart[i], std::max(extremum, bend[i]));
		}
		return pmax;
	}

	OptixAabb get_quadr_bezier_tube_aabb (
		const cgv::render::render_types::vec3 &b0,
		const cgv::render::render_types::vec3 &b1,
		const cgv::render::render_types::vec3 &b2,
		const float r0, const float r1, const float r2
	)
	{
		// convenience shorthand
		using vec3 = cgv::render::render_types::vec3;

		// represent all curves as control matrices for more wieldy notation from here on
		const QuadraticCurve3<float> b{b0, b1, b2};
		const QuadraticCurve1<float> r(r0, r1, r2);

		// differentiate component min/max curves
		const QuadraticCurve3<float> r33({vec3(r[0]), vec3(r[1]), vec3(r[2])});
		const QuadraticCurve3<float> qminus = b - r33, qplus = b + r33;
		const LinearCurve3<float>    dqminus = toMonomial(deriveBezier(qminus)),
		                             dqplus = toMonomial(deriveBezier(qplus));

		// determine extrema 
		const LinearRoots3<float> extrema = solveLinear(dqminus);
		const vec3 pmin = get_curve_minimum(qminus, extrema),
		           pmax = get_curve_maximum(qplus,  extrema);

		// done!
		return {pmin.x(), pmin.y(), pmin.z(), pmax.x(), pmax.y(), pmax.z()};
	}

	OptixPayloadType set_optix_custom_isect_payload_semantics (unsigned *semantics_out)
	{
		// Payloads 0 and 1: two constant inputs for the custom intersection shader(s) defined at raygen,
		// encoding a 64-bit pointer to the ray-centric coordinate system transformation living on the stack
		// of the thread running an OptiX tracing kernel.
		semantics_out[1] = semantics_out[0] =
			OPTIX_PAYLOAD_SEMANTICS_TRACE_CALLER_WRITE | OPTIX_PAYLOAD_SEMANTICS_IS_READ;

		// Payloads 2 to 15: encodes the tracing results reported by the closest-hit or miss shaders. This
		// includes tube color at hit point (1 slot), uv surface coordinates of hit point (2 slots), id of
		// intersected curve segment (1 slot), hit position, surface normal and curve tangent (3x3=9 slots)
		// and depth value at hit (1 slot), occupying 14 payload slots in total, which will remain unused
		// until the very end of a trace, allowing OptiX to free up a lot of registers for general use.
		for (unsigned i=2; i<16; i++)
			semantics_out[i] =   OPTIX_PAYLOAD_SEMANTICS_TRACE_CALLER_READ
			                   | OPTIX_PAYLOAD_SEMANTICS_MS_WRITE | OPTIX_PAYLOAD_SEMANTICS_CH_WRITE;

		// return ready-made OptiX payload descriptor
		return {16, semantics_out};
	}
};



//////
//
// Class implementations
//


////
// optixtracer_textured_spline_tube

optixtracer_textured_spline_tube::~optixtracer_textured_spline_tube() {}

optixtracer_textured_spline_tube::optixtracer_textured_spline_tube(
	optixtracer_textured_spline_tube &&other
)
	: context(other.context), accelds_mem(other.accelds_mem), prg_hit(other.prg_hit), prg_miss(other.prg_miss),
	  prg_raygen(other.prg_raygen), mod_shading(other.mod_shading), mod_geom(other.mod_geom), sbt(other.sbt),
	  lp(other.lp)
{
	// redirect launch params SBT pointer
	lp.sbt = &sbt;

	// invalidate moved-from instance
	other.lp = {};
	other.context = nullptr;
	other.accelds_mem = 0;
	other.prg_hit = nullptr;
	other.prg_miss = nullptr;
	other.prg_raygen = nullptr;
	other.mod_shading = nullptr;
	other.mod_geom = nullptr;
	other.sbt = {};
}

optixtracer_textured_spline_tube& optixtracer_textured_spline_tube::operator= (
	optixtracer_textured_spline_tube &&other
)
{
	// we're going to overwrite
	destroy();

	// take ownership of moved-in resources
	context = other.context;
	accelds_mem = other.accelds_mem;
	prg_hit = other.prg_hit;
	prg_miss = other.prg_miss;
	prg_raygen = other.prg_raygen;
	mod_shading = other.mod_shading;
	mod_geom = other.mod_geom;
	sbt = other.sbt;
	lp = other.lp;
	lp.sbt = &sbt; // redirect launch params SBT pointer

	// invalidate moved-from instance
	other.context = nullptr;
	other.accelds_mem = 0;
	other.prg_hit = nullptr;
	other.prg_miss = nullptr;
	other.prg_raygen = nullptr;
	other.mod_shading = nullptr;
	other.mod_geom = nullptr;
	other.sbt = {};
	other.lp = {};

	// done!
	return *this;
}


////
// optixtracer_textured_spline_tube_russig

optixtracer_textured_spline_tube_russig::optixtracer_textured_spline_tube_russig()
{}

optixtracer_textured_spline_tube_russig::optixtracer_textured_spline_tube_russig(
	optixtracer_textured_spline_tube_russig &&other
)
	: optixtracer_textured_spline_tube(std::move(other))
{
	// we don't have any additional data, so just delegate to base...
}

optixtracer_textured_spline_tube_russig::~optixtracer_textured_spline_tube_russig()
{
	// delegate to destroy method
	destroy();
}

optixtracer_textured_spline_tube_russig& optixtracer_textured_spline_tube_russig::operator= (
	optixtracer_textured_spline_tube_russig &&other
)
{
	// we don't have any additional data, so just delegate to base
	super::operator=(std::move(other));

	// done!
	return *this;
}

void optixtracer_textured_spline_tube_russig::destroy (void)
{
	destroy_pipeline();
	destroy_accelds();
	context = nullptr;
}

bool optixtracer_textured_spline_tube_russig::built (void) const
{
	// we _must_ be good to go if somehow we ended up with a valid pipeline
	return lp.pipeline;
}

optixtracer_textured_spline_tube_russig optixtracer_textured_spline_tube_russig::build (
	OptixDeviceContext context, const traj_manager<float>::render_data *render_data
)
{
	optixtracer_textured_spline_tube_russig tracer;
	tracer.context = context;
	if (!tracer.update_accelds(render_data))
		return optixtracer_textured_spline_tube_russig();
	if (!tracer.update_pipeline())
		return optixtracer_textured_spline_tube_russig();
	tracer.lp.sbt = &tracer.sbt;
	return tracer;
}

void optixtracer_textured_spline_tube_russig::destroy_accelds (void)
{
	lp.accelds = 0; // ToDo: check if we can really leave the accelds handle dangling like this (there appears to be no destroy function for it?)
	CUDA_SAFE_FREE(accelds_mem);
	CUDA_SAFE_FREE(lp.positions);
	CUDA_SAFE_FREE(lp.radii);
}

void optixtracer_textured_spline_tube_russig::destroy_pipeline (void)
{
	CUDA_SAFE_FREE(lp.params); lp.params_size = 0;
	OPTIX_SAFE_DESTROY_PIPELINE(lp.pipeline);
	OPTIX_SAFE_DESTROY_MODULE(mod_shading);
	OPTIX_SAFE_DESTROY_MODULE(mod_geom);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_hit);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_miss);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_raygen);
	sbt = {}; lp.sbt = nullptr;
}

bool optixtracer_textured_spline_tube_russig::update_accelds (const traj_manager<float>::render_data *render_data)
{
	// make sure we start with a blank slate
	destroy_accelds();

	// use default options for simplicity
	OptixAccelBuildOptions accel_options = {};
	accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
	accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

	// stage geometry for upload - we don't use the available render data directly since we need to store the
	// quadratic segments explicitly if we don't want to double the workload on the intersection shader
	unsigned num;
	std::vector<float3> positions;
	std::vector<float> radii;
	std::vector<OptixAabb> aabbs;

	// prepare CPU-side vertex staging area
	/* local scope */ {
		const auto &rd_pos = render_data->positions;
		const auto &rd_tan = render_data->tangents;
		const auto &rd_rad = render_data->radii;
		const auto &rd_idx = render_data->indices;
		aabbs.reserve(rd_idx.size());
		num = (unsigned)rd_idx.size()*3;
		positions.reserve(num);
		radii.reserve(num);

		// convert data representation:
		// - split Hermite curves into two quadratic Beziers
		// - build axis aligned boundinx boxes (ToDo: build oriented BBs instead and use one BLAS per segment)
		for (const auto &ds : render_data->datasets) for (const auto &traj : ds.trajs)
		{
			const unsigned num_segs = traj.n / 2;
			for (unsigned i=0; i<num_segs; i++)
			{
				// obtain shortcuts to node data
				const unsigned idx = rd_idx[traj.i0]+i;
				const vec4  &t0 = rd_tan[idx], &t1 = rd_tan[idx+1];
				const vec3  &p0 = rd_pos[idx], &p1 = rd_pos[idx+1],
				             m0 = vec3(t0),     m1 = vec3(t1);
				const float &r0 = rd_rad[idx], &r1 = rd_rad[idx+1];

				// split hermite segment into 2 quadratic Beziers
				vec3  b0,  b1,  b2,  g0,  g1,  g2;
				float br0, br1, br2, gr0, gr1, gr2;
				split_quadseg(&b0,  &b1,  &b2,  &g0,  &g1,  &g2,	p0, m0,     p1, m1);
				split_quadseg(&br0, &br1, &br2, &gr0, &gr1, &gr2,	r0, t0.w(), r1, t1.w());

				// commit to staging buffers
				// - 1st segment
				aabbs.emplace_back(get_quadr_bezier_tube_aabb(b0, b1, b2, br0, br1, br2));
				positions.emplace_back(to_float3(b0));
				radii.emplace_back(r0);
				positions.emplace_back(to_float3(b1));
				radii.emplace_back(br1);
				positions.emplace_back(to_float3(b2));
				radii.emplace_back(br2);
				// - 2nd segment
				aabbs.emplace_back(get_quadr_bezier_tube_aabb(g0, g1, g2, gr0, gr1, gr2));
				positions.emplace_back(to_float3(g0));
				radii.emplace_back(gr0);
				positions.emplace_back(to_float3(g1));
				radii.emplace_back(gr1);
				positions.emplace_back(to_float3(g2));
				radii.emplace_back(gr2);
			}
		}
	}

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;

	// prepare geometry device memory
	// - prelude
	CUdeviceptr aabbs_dev=0;
	const size_t positions_size = num*sizeof(float3), radii_size = num*sizeof(float), aabbs_size = aabbs.size()*sizeof(OptixAabb);
	// - nodes
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.positions), positions_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(lp.positions), positions.data(), positions_size, cudaMemcpyHostToDevice), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.radii), radii_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(lp.radii), radii.data(), radii_size, cudaMemcpyHostToDevice), success);
	// - AABBs
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&aabbs_dev), aabbs_size), success);
    CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(aabbs_dev), aabbs.data(), aabbs_size, cudaMemcpyHostToDevice), success);

	// OptiX accel-ds build input descriptor
	// - SBT indices
	const unsigned sbtindex[] = {0}, inputflags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};  // we only use one SBT record
	CUdeviceptr    sbtindex_dev = 0;
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&sbtindex_dev), sizeof(sbtindex)), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(sbtindex_dev), sbtindex, sizeof(sbtindex), cudaMemcpyHostToDevice), success);
	// - input descriptor
	OptixBuildInput input_desc = {};
	input_desc.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
	input_desc.customPrimitiveArray.aabbBuffers   = &aabbs_dev;
	input_desc.customPrimitiveArray.flags = inputflags;
	input_desc.customPrimitiveArray.numSbtRecords = 1;
	input_desc.customPrimitiveArray.numPrimitives = (unsigned)aabbs.size();
	input_desc.customPrimitiveArray.sbtIndexOffsetBuffer      = sbtindex_dev;
	input_desc.customPrimitiveArray.sbtIndexOffsetSizeInBytes = sizeof(unsigned);
	// - working memory
	CUdeviceptr tmpbuf = 0;
	OptixAccelBufferSizes accelds_buffer_sizes = {0};
	OPTIX_CHECK_SET(
		optixAccelComputeMemoryUsage(context, &accel_options, &input_desc, 1/* num build inputs */, &accelds_buffer_sizes),
		success
	);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&tmpbuf), accelds_buffer_sizes.tempSizeInBytes), success);
	// - output memory
	CUdeviceptr mem_precompact = 0;
    size_t compactsize_feedback_offset = ceil_unsigned<size_t>(accelds_buffer_sizes.outputSizeInBytes, 8);
		/* |-- squeeze in a little 8-byte-aligned space for our compacted size feedback at the end of the output buffer */
    CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&mem_precompact), compactsize_feedback_offset+8), success);
	// - build accel-ds
	OptixAccelEmitDesc emitted_prop;
    emitted_prop.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
	emitted_prop.result = (CUdeviceptr)((char*)mem_precompact + compactsize_feedback_offset);
	OPTIX_CHECK_SET(
		optixAccelBuild(
			context, 0/*CUDA stream*/, &accel_options, &input_desc, 1/*num build inputs*/,
			tmpbuf, accelds_buffer_sizes.tempSizeInBytes, mem_precompact, accelds_buffer_sizes.outputSizeInBytes,
			&lp.accelds,  // <-- our acceleration datastructure!!!
			&emitted_prop, 1/*num emitted properties*/
		),
		success
	);
	// - fetch our emitted property (the size we can compact to)
	size_t compacted_size = accelds_buffer_sizes.outputSizeInBytes;
    CUDA_CHECK_SET(cudaMemcpy(&compacted_size, (void*)emitted_prop.result, sizeof(size_t), cudaMemcpyDeviceToHost), success);
	// - free scratch memory
	CUDA_SAFE_FREE(tmpbuf);

	// perform compaction if possible
	if (compacted_size < accelds_buffer_sizes.outputSizeInBytes) {
        CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&accelds_mem), compacted_size), success);
		OPTIX_CHECK_SET(
			optixAccelCompact(context, 0/*CUDA stream*/, lp.accelds, accelds_mem, compacted_size, &lp.accelds),
			success
		);
		CUDA_SAFE_FREE(mem_precompact);
    }
    else
		accelds_mem = mem_precompact;

	// We can now free the aabb buffer used during build /** and the vertex
	// inputs, since they are not needed by our trivial shading method **/
	// (we won't consider cudaFree failing a failure of the whole function)
	CUDA_SAFE_FREE(aabbs_dev);

	// done!
	return success;
}

bool optixtracer_textured_spline_tube_russig::update_pipeline (void)
{
	////
	// Prelude

	// make sure we start with a blank slate
	destroy_pipeline();

	// CUDA/OptiX log storage
	std::string compiler_log(8192, 0);
	char* log = compiler_log.data();
	size_t sizeof_log = compiler_log.size();

	// pipeline build options
	constexpr unsigned max_trace_depth = 1;
	OptixPipelineCompileOptions pipeline_options = {};

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;


	////
	// Create modules

	// fine-grained payload usage (enables OptiX to optimize register consumption)
	unsigned payloads[16];
	OptixPayloadType payloadType = set_optix_custom_isect_payload_semantics(payloads);

	/* local scope */ {
		OptixModuleCompileOptions mod_options = {};
		mod_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
		mod_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
		mod_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
		mod_options.numPayloadTypes = 1;
		mod_options.payloadTypes = &payloadType;

		pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
		pipeline_options.numPayloadValues = 0; // we use module-defined payloads
		// - we report the curve parameter (1 attrib) as well as the first two bezier nodes (3 attribs each = 6). Unfortunately, the
		//   closest-hit shader will have to fetch the third node from global memory, as we are out of attribute registers at this point.
		pipeline_options.numAttributeValues = 7;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_options.exceptionFlags =
			OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
	#else
		pipeline_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	#endif
		pipeline_options.pipelineLaunchParamsVariableName = "params";
		pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;

		const auto ptx = compile_cu2ptx("cuda/optix_curves_russig.cu", "rayc_russig", {CUDA_NVRTC_OPTIONS}, &compiler_log);
		const size_t ptx_size = ptx.size();
		if (!ptx_size)
		{
			std::cerr << "ERROR compiling OptiX device code! Log:" << std::endl
			          << compiler_log << std::endl<<std::endl;
			return false; // no resources allocated yet, so we can just return
		}
		OPTIX_CHECK_LOG_SET(
			optixModuleCreateFromPTX(
				context, &mod_options, &pipeline_options, ptx.data(), ptx_size, log, &sizeof_log, &mod_shading
			),
			log, sizeof_log, success
		);
		mod_geom = mod_shading; // all in one file for now
	}


	////
	// Create program groups

	/* local scope */ {
		// common options
		OptixProgramGroupOptions prg_options = {&payloadType};

		// raygen shader
		OptixProgramGroupDesc prg_raygen_desc = {};
		prg_raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
		prg_raygen_desc.raygen.module = mod_shading;
		prg_raygen_desc.raygen.entryFunctionName = "__raygen__basic";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_raygen_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_raygen
			),
			log, sizeof_log, success
		);

		// miss shader
		OptixProgramGroupDesc prg_miss_desc = {};
		prg_miss_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
		prg_miss_desc.miss.module = mod_shading;
		prg_miss_desc.miss.entryFunctionName = "__miss__ms";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_miss_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_miss
			),
			log, sizeof_log, success
		);

		// hit shader group
		OptixProgramGroupDesc prg_hit_desc = {};
		prg_hit_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
		prg_hit_desc.hitgroup.moduleCH = mod_shading;
		prg_hit_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
		prg_hit_desc.hitgroup.moduleIS = mod_geom;
		prg_hit_desc.hitgroup.entryFunctionNameIS = "__intersection__russig";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_hit_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_hit
			),
			log, sizeof_log, success
		);
	}


	////
	// Link pipeline

	/* local scope */ {
		constexpr unsigned num_prgs = 3;
		OptixProgramGroup prgs[num_prgs] = {prg_raygen, prg_miss, prg_hit};
		OptixPipelineLinkOptions pipeline_linkoptions = {};
		pipeline_linkoptions.maxTraceDepth = max_trace_depth;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
	#else
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
	#endif
		OPTIX_CHECK_LOG_SET(
			optixPipelineCreate(
				context, &pipeline_options, &pipeline_linkoptions, prgs, num_prgs, log, &sizeof_log, &lp.pipeline
			),
			log, sizeof_log, success
		);

		OptixStackSizes stacksizes = {};
		for (const auto &prg : prgs)
			OPTIX_CHECK_SET(optixUtilAccumulateStackSizes(prg, &stacksizes), success);

		// ToDo: ??? investigate what exactly this is, why it is needed, what to best do here, etc.
		uint32_t direct_callable_stack_size_from_traversal;
		uint32_t direct_callable_stack_size_from_state;
		uint32_t continuation_stack_size;
		OPTIX_CHECK_SET(
			optixUtilComputeStackSizes(
				&stacksizes, max_trace_depth, 0/* maxCCDepth */, 0/* maxDCDepth */,
				&direct_callable_stack_size_from_traversal,
				&direct_callable_stack_size_from_state, &continuation_stack_size
			),
			success
		);
		OPTIX_CHECK_SET(
			optixPipelineSetStackSize(
				lp.pipeline, direct_callable_stack_size_from_traversal,
				direct_callable_stack_size_from_state, continuation_stack_size,
				1 // <-- max traversable depth - we only use the top-level GAS currently
			),
			success
		);
	}


	////
	// Set up shader binding table

	/* local scope */ {
		// our SBT record types
		typedef sbt_record<data_raygen> sbt_record_raygen;
		typedef sbt_record<data_miss>   sbt_record_miss;
		typedef sbt_record<data_hit>    sbt_record_hit;

		// prepare entries
		// - raygen shaders
		CUdeviceptr  raygen_record;
		const size_t raygen_record_size = sizeof(sbt_record_raygen);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&raygen_record), raygen_record_size), success);
		sbt_record_raygen rg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_raygen, &rg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(raygen_record), &rg_sbt, raygen_record_size, cudaMemcpyHostToDevice), success);
		// - miss shaders
		CUdeviceptr miss_record;
		size_t      miss_record_size = sizeof(sbt_record_miss);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&miss_record), miss_record_size), success);
		sbt_record_miss ms_sbt;
		ms_sbt.data = {0.0f, 0.0f, 0.0f, 0.0f};  // background color (fully transparent black)
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_miss, &ms_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(miss_record), &ms_sbt, miss_record_size, cudaMemcpyHostToDevice), success);
		// - hit shaders
		CUdeviceptr hitgroup_record;
		size_t      hitgroup_record_size = sizeof(sbt_record_hit);
		CUDA_CHECK_SET (cudaMalloc(reinterpret_cast<void**>(&hitgroup_record), hitgroup_record_size), success);
		sbt_record_hit hg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_hit, &hg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(hitgroup_record), &hg_sbt, hitgroup_record_size, cudaMemcpyHostToDevice), success);

		// build up the SBT
		sbt = {};
		sbt.raygenRecord = raygen_record;
		sbt.missRecordBase = miss_record;
		sbt.missRecordStrideInBytes = sizeof(sbt_record_miss);
		sbt.missRecordCount = 1;
		sbt.hitgroupRecordBase = hitgroup_record;
		sbt.hitgroupRecordStrideInBytes = sizeof(sbt_record_hit);
		sbt.hitgroupRecordCount = 1;
	}

	// Create the device memory for our launch params
	/* local scope */ {
		bool params_malloc_success = true;
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.params), sizeof(curve_rt_params)), params_malloc_success);
		if (params_malloc_success)
			lp.params_size = sizeof(curve_rt_params);
		else
			success = false;
	}

	// done!
	return success;
}


////
// optixtracer_textured_spline_tube_phantom

optixtracer_textured_spline_tube_phantom::optixtracer_textured_spline_tube_phantom()
{}

optixtracer_textured_spline_tube_phantom::optixtracer_textured_spline_tube_phantom(
	optixtracer_textured_spline_tube_phantom &&other
)
	: optixtracer_textured_spline_tube(std::move(other))
{
	// we don't have any additional data, so just delegate to base...
}

optixtracer_textured_spline_tube_phantom::~optixtracer_textured_spline_tube_phantom()
{
	// delegate to destroy method
	destroy();
}

optixtracer_textured_spline_tube_phantom& optixtracer_textured_spline_tube_phantom::operator= (
	optixtracer_textured_spline_tube_phantom &&other
)
{
	// we don't have any additional data, so just delegate to base
	super::operator=(std::move(other));

	// done!
	return *this;
}

void optixtracer_textured_spline_tube_phantom::destroy (void)
{
	destroy_pipeline();
	destroy_accelds();
	context = nullptr;
}

bool optixtracer_textured_spline_tube_phantom::built (void) const
{
	// we _must_ be good to go if somehow we ended up with a valid pipeline
	return lp.pipeline;
}

optixtracer_textured_spline_tube_phantom optixtracer_textured_spline_tube_phantom::build (
	OptixDeviceContext context, const traj_manager<float>::render_data *render_data
)
{
	optixtracer_textured_spline_tube_phantom tracer;
	tracer.context = context;
	if (!tracer.update_accelds(render_data))
		return optixtracer_textured_spline_tube_phantom();
	if (!tracer.update_pipeline())
		return optixtracer_textured_spline_tube_phantom();
	tracer.lp.sbt = &tracer.sbt;
	return tracer;
}

void optixtracer_textured_spline_tube_phantom::destroy_accelds (void)
{
	lp.accelds = 0; // ToDo: check if we can really leave the accelds handle dangling like this (there appears to be no destroy function for it?)
	CUDA_SAFE_FREE(accelds_mem);
	CUDA_SAFE_FREE(lp.positions);
	CUDA_SAFE_FREE(lp.radii);
}

void optixtracer_textured_spline_tube_phantom::destroy_pipeline (void)
{
	CUDA_SAFE_FREE(lp.params); lp.params_size = 0;
	OPTIX_SAFE_DESTROY_PIPELINE(lp.pipeline);
	OPTIX_SAFE_DESTROY_MODULE(mod_shading);
	OPTIX_SAFE_DESTROY_MODULE(mod_geom);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_hit);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_miss);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_raygen);
	sbt = {}; lp.sbt = nullptr;
}

bool optixtracer_textured_spline_tube_phantom::update_accelds (const traj_manager<float>::render_data *render_data)
{
	// make sure we start with a blank slate
	destroy_accelds();

	// use default options for simplicity
	OptixAccelBuildOptions accel_options = {};
	accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_COMPACTION;
	accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

	// stage geometry for upload - we don't use the available render data directly since we need to store the
	// quadratic segments explicitly if we don't want to double the workload on the intersection shader
	unsigned num;
	std::vector<float3> positions;
	std::vector<float> radii;
	std::vector<OptixAabb> aabbs;

	// prepare CPU-side vertex staging area
	/* local scope */ {
		const auto &rd_pos = render_data->positions;
		const auto &rd_tan = render_data->tangents;
		const auto &rd_rad = render_data->radii;
		const auto &rd_idx = render_data->indices;
		aabbs.reserve(rd_idx.size());
		num = (unsigned)rd_idx.size()*3;
		positions.reserve(num);
		radii.reserve(num);

		// convert data representation:
		// - split Hermite curves into two quadratic Beziers
		// - build axis aligned boundinx boxes (ToDo: build oriented BBs instead and use one BLAS per segment)
		for (const auto &ds : render_data->datasets) for (const auto &traj : ds.trajs)
		{
			const unsigned num_segs = traj.n / 2;
			for (unsigned i=0; i<num_segs; i++)
			{
				// obtain shortcuts to node data
				const unsigned idx = rd_idx[traj.i0]+i;
				const vec4  &t0 = rd_tan[idx], &t1 = rd_tan[idx+1];
				const vec3  &p0 = rd_pos[idx], &p1 = rd_pos[idx+1],
				             m0 = vec3(t0),     m1 = vec3(t1);
				const float &r0 = rd_rad[idx], &r1 = rd_rad[idx+1];

				// split hermite segment into 2 quadratic Beziers
				vec3  b0,  b1,  b2,  g0,  g1,  g2;
				float br0, br1, br2, gr0, gr1, gr2;
				split_quadseg(&b0,  &b1,  &b2,  &g0,  &g1,  &g2,	p0, m0,     p1, m1);
				split_quadseg(&br0, &br1, &br2, &gr0, &gr1, &gr2,	r0, t0.w(), r1, t1.w());

				// commit to staging buffers
				// ToDo: AABBs are optimized for swept-sphere, can be marginally tighter in case of swept-disc
				// - 1st segment
				aabbs.emplace_back(get_quadr_bezier_tube_aabb(b0, b1, b2, br0, br1, br2));
				positions.emplace_back(to_float3(b0));
				radii.emplace_back(r0);
				positions.emplace_back(to_float3(b1));
				radii.emplace_back(br1);
				positions.emplace_back(to_float3(b2));
				radii.emplace_back(br2);
				// - 2nd segment
				aabbs.emplace_back(get_quadr_bezier_tube_aabb(g0, g1, g2, gr0, gr1, gr2));
				positions.emplace_back(to_float3(g0));
				radii.emplace_back(gr0);
				positions.emplace_back(to_float3(g1));
				radii.emplace_back(gr1);
				positions.emplace_back(to_float3(g2));
				radii.emplace_back(gr2);
			}
		}
	}

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;

	// prepare geometry device memory
	// - prelude
	CUdeviceptr aabbs_dev=0;
	const size_t positions_size = num*sizeof(float3), radii_size = num*sizeof(float), aabbs_size = aabbs.size()*sizeof(OptixAabb);
	// - nodes
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.positions), positions_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(lp.positions), positions.data(), positions_size, cudaMemcpyHostToDevice), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.radii), radii_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(lp.radii), radii.data(), radii_size, cudaMemcpyHostToDevice), success);
	// - AABBs
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&aabbs_dev), aabbs_size), success);
    CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(aabbs_dev), aabbs.data(), aabbs_size, cudaMemcpyHostToDevice), success);

	// OptiX accel-ds build input descriptor
	// - SBT indices
	const unsigned sbtindex[] = {0}, inputflags[] = {OPTIX_GEOMETRY_FLAG_DISABLE_ANYHIT};  // we only use one SBT record
	CUdeviceptr    sbtindex_dev = 0;
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&sbtindex_dev), sizeof(sbtindex)), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(sbtindex_dev), sbtindex, sizeof(sbtindex), cudaMemcpyHostToDevice), success);
	// - input descriptor
	OptixBuildInput input_desc = {};
	input_desc.type = OPTIX_BUILD_INPUT_TYPE_CUSTOM_PRIMITIVES;
	input_desc.customPrimitiveArray.aabbBuffers   = &aabbs_dev;
	input_desc.customPrimitiveArray.flags = inputflags;
	input_desc.customPrimitiveArray.numSbtRecords = 1;
	input_desc.customPrimitiveArray.numPrimitives = (unsigned)aabbs.size();
	input_desc.customPrimitiveArray.sbtIndexOffsetBuffer      = sbtindex_dev;
	input_desc.customPrimitiveArray.sbtIndexOffsetSizeInBytes = sizeof(unsigned);
	// - working memory
	CUdeviceptr tmpbuf = 0;
	OptixAccelBufferSizes accelds_buffer_sizes = {0};
	OPTIX_CHECK_SET(
		optixAccelComputeMemoryUsage(context, &accel_options, &input_desc, 1/* num build inputs */, &accelds_buffer_sizes),
		success
	);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&tmpbuf), accelds_buffer_sizes.tempSizeInBytes), success);
	// - output memory
	CUdeviceptr mem_precompact = 0;
    size_t compactsize_feedback_offset = ceil_unsigned<size_t>(accelds_buffer_sizes.outputSizeInBytes, 8);
		/* |-- squeeze in a little 8-byte-aligned space for our compacted size feedback at the end of the output buffer */
    CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&mem_precompact), compactsize_feedback_offset+8), success);
	// - build accel-ds
	OptixAccelEmitDesc emitted_prop;
    emitted_prop.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
	emitted_prop.result = (CUdeviceptr)((char*)mem_precompact + compactsize_feedback_offset);
	OPTIX_CHECK_SET(
		optixAccelBuild(
			context, 0/*CUDA stream*/, &accel_options, &input_desc, 1/*num build inputs*/,
			tmpbuf, accelds_buffer_sizes.tempSizeInBytes, mem_precompact, accelds_buffer_sizes.outputSizeInBytes,
			&lp.accelds,  // <-- our acceleration datastructure!!!
			&emitted_prop, 1/*num emitted properties*/
		),
		success
	);
	// - fetch our emitted property (the size we can compact to)
	size_t compacted_size = accelds_buffer_sizes.outputSizeInBytes;
    CUDA_CHECK_SET(cudaMemcpy(&compacted_size, (void*)emitted_prop.result, sizeof(size_t), cudaMemcpyDeviceToHost), success);
	// - free scratch memory
	CUDA_SAFE_FREE(tmpbuf);

	// perform compaction if possible
	if (compacted_size < accelds_buffer_sizes.outputSizeInBytes) {
        CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&accelds_mem), compacted_size), success);
		OPTIX_CHECK_SET(
			optixAccelCompact(context, 0/*CUDA stream*/, lp.accelds, accelds_mem, compacted_size, &lp.accelds),
			success
		);
		CUDA_SAFE_FREE(mem_precompact);
    }
    else
		accelds_mem = mem_precompact;

	// We can now free the aabb buffer used during build /** and the vertex
	// inputs, since they are not needed by our trivial shading method **/
	// (we won't consider cudaFree failing a failure of the whole function)
	CUDA_SAFE_FREE(aabbs_dev);

	// done!
	return success;
}

bool optixtracer_textured_spline_tube_phantom::update_pipeline (void)
{
	////
	// Prelude

	// make sure we start with a blank slate
	destroy_pipeline();

	// CUDA/OptiX log storage
	std::string compiler_log(8192, 0);
	char* log = compiler_log.data();
	size_t sizeof_log = compiler_log.size();

	// pipeline build options
	constexpr unsigned max_trace_depth = 1;
	OptixPipelineCompileOptions pipeline_options = {};

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;


	////
	// Create modules

	// fine-grained payload usage (enables OptiX to optimize register consumption)
	unsigned payloads[16];
	OptixPayloadType payloadType = set_optix_custom_isect_payload_semantics(payloads);

	/* local scope */ {
		OptixModuleCompileOptions mod_options = {};
		mod_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
		mod_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
		mod_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
		mod_options.numPayloadTypes = 1;
		mod_options.payloadTypes = &payloadType;

		pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
		pipeline_options.numPayloadValues = 0; // we use module-defined payloads
		// - we report the curve parameter (1 attrib) as well as the first two bezier nodes (3 attribs each = 6). Unfortunately, the
		//   closest-hit shader will have to fetch the third node from global memory, as we are out of attribute registers at this point.
		pipeline_options.numAttributeValues = 7;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_options.exceptionFlags =
			OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
	#else
		pipeline_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	#endif
		pipeline_options.pipelineLaunchParamsVariableName = "params";
		pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_CUSTOM;

		const auto ptx = compile_cu2ptx("cuda/optix_curves_phantom.cu", "rayc_phantom", {CUDA_NVRTC_OPTIONS}, &compiler_log);
		const size_t ptx_size = ptx.size();
		if (!ptx_size)
		{
			std::cerr << "ERROR compiling OptiX device code! Log:" << std::endl
			          << compiler_log << std::endl<<std::endl;
			return false; // no resources allocated yet, so we can just return
		}
		OPTIX_CHECK_LOG_SET(
			optixModuleCreateFromPTX(
				context, &mod_options, &pipeline_options, ptx.data(), ptx_size, log, &sizeof_log, &mod_shading
			),
			log, sizeof_log, success
		);
		mod_geom = mod_shading; // all in one file for now
	}


	////
	// Create program groups

	/* local scope */ {
		// common options
		OptixProgramGroupOptions prg_options = {&payloadType};

		// raygen shader
		OptixProgramGroupDesc prg_raygen_desc = {};
		prg_raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
		prg_raygen_desc.raygen.module = mod_shading;
		prg_raygen_desc.raygen.entryFunctionName = "__raygen__basic";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_raygen_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_raygen
			),
			log, sizeof_log, success
		);

		// miss shader
		OptixProgramGroupDesc prg_miss_desc = {};
		prg_miss_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
		prg_miss_desc.miss.module = mod_shading;
		prg_miss_desc.miss.entryFunctionName = "__miss__ms";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_miss_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_miss
			),
			log, sizeof_log, success
		);

		// hit shader group
		OptixProgramGroupDesc prg_hit_desc = {};
		prg_hit_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
		prg_hit_desc.hitgroup.moduleCH = mod_shading;
		prg_hit_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
		prg_hit_desc.hitgroup.moduleIS = mod_geom;
		prg_hit_desc.hitgroup.entryFunctionNameIS = "__intersection__phantom";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_hit_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_hit
			),
			log, sizeof_log, success
		);
	}


	////
	// Link pipeline

	/* local scope */ {
		constexpr unsigned num_prgs = 3;
		OptixProgramGroup prgs[num_prgs] = {prg_raygen, prg_miss, prg_hit};
		OptixPipelineLinkOptions pipeline_linkoptions = {};
		pipeline_linkoptions.maxTraceDepth = max_trace_depth;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
	#else
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
	#endif
		OPTIX_CHECK_LOG_SET(
			optixPipelineCreate(
				context, &pipeline_options, &pipeline_linkoptions, prgs, num_prgs, log, &sizeof_log, &lp.pipeline
			),
			log, sizeof_log, success
		);

		OptixStackSizes stacksizes = {};
		for (const auto &prg : prgs)
			OPTIX_CHECK_SET(optixUtilAccumulateStackSizes(prg, &stacksizes), success);

		// ToDo: ??? investigate what exactly this is, why it is needed, what to best do here, etc.
		uint32_t direct_callable_stack_size_from_traversal;
		uint32_t direct_callable_stack_size_from_state;
		uint32_t continuation_stack_size;
		OPTIX_CHECK_SET(
			optixUtilComputeStackSizes(
				&stacksizes, max_trace_depth, 0/* maxCCDepth */, 0/* maxDCDepth */,
				&direct_callable_stack_size_from_traversal,
				&direct_callable_stack_size_from_state, &continuation_stack_size
			),
			success
		);
		OPTIX_CHECK_SET(
			optixPipelineSetStackSize(
				lp.pipeline, direct_callable_stack_size_from_traversal,
				direct_callable_stack_size_from_state, continuation_stack_size,
				1 // <-- max traversable depth - we only use the top-level GAS currently
			),
			success
		);
	}


	////
	// Set up shader binding table

	/* local scope */ {
		// our SBT record types
		typedef sbt_record<data_raygen> sbt_record_raygen;
		typedef sbt_record<data_miss>   sbt_record_miss;
		typedef sbt_record<data_hit>    sbt_record_hit;

		// prepare entries
		// - raygen shaders
		CUdeviceptr  raygen_record;
		const size_t raygen_record_size = sizeof(sbt_record_raygen);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&raygen_record), raygen_record_size), success);
		sbt_record_raygen rg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_raygen, &rg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(raygen_record), &rg_sbt, raygen_record_size, cudaMemcpyHostToDevice), success);
		// - miss shaders
		CUdeviceptr miss_record;
		size_t      miss_record_size = sizeof(sbt_record_miss);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&miss_record), miss_record_size), success);
		sbt_record_miss ms_sbt;
		ms_sbt.data = {0.0f, 0.0f, 0.0f, 0.0f};  // background color (fully transparent black)
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_miss, &ms_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(miss_record), &ms_sbt, miss_record_size, cudaMemcpyHostToDevice), success);
		// - hit shaders
		CUdeviceptr hitgroup_record;
		size_t      hitgroup_record_size = sizeof(sbt_record_hit);
		CUDA_CHECK_SET (cudaMalloc(reinterpret_cast<void**>(&hitgroup_record), hitgroup_record_size), success);
		sbt_record_hit hg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_hit, &hg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(hitgroup_record), &hg_sbt, hitgroup_record_size, cudaMemcpyHostToDevice), success);

		// build up the SBT
		sbt = {};
		sbt.raygenRecord = raygen_record;
		sbt.missRecordBase = miss_record;
		sbt.missRecordStrideInBytes = sizeof(sbt_record_miss);
		sbt.missRecordCount = 1;
		sbt.hitgroupRecordBase = hitgroup_record;
		sbt.hitgroupRecordStrideInBytes = sizeof(sbt_record_hit);
		sbt.hitgroupRecordCount = 1;
	}

	// Create the device memory for our launch params
	/* local scope */ {
		bool params_malloc_success = true;
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.params), sizeof(curve_rt_params)), params_malloc_success);
		if (params_malloc_success)
			lp.params_size = sizeof(curve_rt_params);
		else
			success = false;
	}

	// done!
	return success;
}


////
// optixtracer_textured_spline_tube_builtin

optixtracer_textured_spline_tube_builtin::optixtracer_textured_spline_tube_builtin()
{}

optixtracer_textured_spline_tube_builtin::optixtracer_textured_spline_tube_builtin(
	optixtracer_textured_spline_tube_builtin &&other
)
	: optixtracer_textured_spline_tube(std::move(other))
{
	// we don't have any additional data, so just delegate to base...
}

optixtracer_textured_spline_tube_builtin::~optixtracer_textured_spline_tube_builtin()
{
	// delegate to destroy method
	destroy();
}

optixtracer_textured_spline_tube_builtin& optixtracer_textured_spline_tube_builtin::operator= (
	optixtracer_textured_spline_tube_builtin &&other
)
{
	// we don't have any additional data, so just delegate to base
	super::operator=(std::move(other));

	// done!
	return *this;
}

void optixtracer_textured_spline_tube_builtin::destroy (void)
{
	destroy_pipeline();
	destroy_accelds();
	context = nullptr;
}

bool optixtracer_textured_spline_tube_builtin::built (void) const
{
	// we _must_ be good to go if somehow we ended up with a valid pipeline
	return lp.pipeline;
}

optixtracer_textured_spline_tube_builtin optixtracer_textured_spline_tube_builtin::build (
	OptixDeviceContext context, const traj_manager<float>::render_data *render_data
)
{
	optixtracer_textured_spline_tube_builtin tracer;
	tracer.context = context;
	if (!tracer.update_accelds(render_data))
		return optixtracer_textured_spline_tube_builtin();
	if (!tracer.update_pipeline())
		return optixtracer_textured_spline_tube_builtin();
	tracer.lp.sbt = &tracer.sbt;
	return tracer;
}

void optixtracer_textured_spline_tube_builtin::destroy_accelds (void)
{
	lp.accelds = 0; // ToDo: check if we can really leave the accelds handle dangling like this (there appears to be no destroy function for it?)
	CUDA_SAFE_FREE(accelds_mem);
}

void optixtracer_textured_spline_tube_builtin::destroy_pipeline (void)
{
	CUDA_SAFE_FREE(lp.params); lp.params_size = 0;
	OPTIX_SAFE_DESTROY_PIPELINE(lp.pipeline);
	OPTIX_SAFE_DESTROY_MODULE(mod_shading);
	OPTIX_SAFE_DESTROY_MODULE(mod_geom);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_hit);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_miss);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_raygen);
	sbt = {}; lp.sbt = nullptr;
}

bool optixtracer_textured_spline_tube_builtin::update_accelds (const traj_manager<float>::render_data *render_data)
{
	// make sure we start with a blank slate
	destroy_accelds();

	// use default options for simplicity
	OptixAccelBuildOptions accel_options = {};
	accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS;
	accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

	// stage geometry for upload - we don't use the available render data directly since in order to use the built-in
	// quadratic curve primitive, we need to store the quadratic sub-segments explicitly (plus we don't want to double
	// the workload on the intersection shader even if we could use the existing render data directly)
	unsigned num;
	std::vector<float3> positions;
	std::vector<float> radii;
	std::vector<unsigned> indices;

	// prepare CPU-side vertex staging area
	/* local scope */ {
		const auto &rd_pos = render_data->positions;
		const auto &rd_tan = render_data->tangents;
		const auto &rd_rad = render_data->radii;
		const auto &rd_idx = render_data->indices;
		indices.reserve(rd_idx.size());
		num = (unsigned)rd_idx.size()*3;
		positions.reserve(num);
		radii.reserve(num);

		// convert data representation:
		// - split Hermite curves into two quadratic Beziers
		// - adapt indices to OptiX curve primitive scheme
		for (const auto &ds : render_data->datasets) for (const auto &traj : ds.trajs)
		{
			const unsigned num_segs = traj.n / 2;
			for (unsigned i=0; i<num_segs; i++)
			{
				// obtain shortcuts to node data
				const unsigned idx = rd_idx[traj.i0]+i;
				const vec4  &t0 = rd_tan[idx], &t1 = rd_tan[idx+1];
				const vec3  &p0 = rd_pos[idx], &p1 = rd_pos[idx+1],
				             m0 = vec3(t0),     m1 = vec3(t1);
				const float &r0 = rd_rad[idx], &r1 = rd_rad[idx+1];

				// split hermite segment into 2 quadratic Beziers
				vec3  b0,  b1,  b2,  g0,  g1,  g2;
				float br0, br1, br2, gr0, gr1, gr2;
				split_quadseg(&b0,  &b1,  &b2,  &g0,  &g1,  &g2,	p0, m0,     p1, m1);
				split_quadseg(&br0, &br1, &br2, &gr0, &gr1, &gr2,	r0, t0.w(), r1, t1.w());

				// commit to buffer (adjust subsegment endpoints for the uniform knot vector used by OptiX)
				// - 1st segment
				indices.emplace_back(unsigned(positions.size()));
				positions.emplace_back(to_float3(get_s0(b0, b1)));
				radii.emplace_back(get_s0(br0, br1));
				positions.emplace_back(to_float3(b1));
				radii.emplace_back(br1);
				positions.emplace_back(to_float3(get_s2(b1, b2)));
				radii.emplace_back(get_s2(br1, br2));
				// - 2nd segment
				indices.emplace_back(unsigned(positions.size()));
				positions.emplace_back(to_float3(get_s0(g0, g1)));
				radii.emplace_back(get_s0(gr0, gr1));
				positions.emplace_back(to_float3(g1));
				radii.emplace_back(gr1);
				positions.emplace_back(to_float3(get_s2(g1, g2)));
				radii.emplace_back(get_s2(gr1, gr2));
			}
		}
	}

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;

	// prepare geometry device memory
	CUdeviceptr positions_dev=0, radii_dev=0;
	const size_t positions_size = num*sizeof(float3), radii_size = num*sizeof(float);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&positions_dev), positions_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(positions_dev), positions.data(), positions_size, cudaMemcpyHostToDevice), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&radii_dev), radii_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(radii_dev), radii.data(), radii_size, cudaMemcpyHostToDevice), success);

	// upload segment indices
	CUdeviceptr indices_dev = 0;
	const size_t indices_size = indices.size()*sizeof(unsigned);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&indices_dev), indices_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(indices_dev), indices.data(), indices_size, cudaMemcpyHostToDevice), success);

	// OptiX accel-ds build input descriptor
	OptixBuildInput input_desc = {};
	input_desc.type = OPTIX_BUILD_INPUT_TYPE_CURVES;
	input_desc.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
	input_desc.curveArray.numPrimitives = (unsigned)indices.size();
	input_desc.curveArray.vertexBuffers = &positions_dev;
	input_desc.curveArray.numVertices = num;
	input_desc.curveArray.vertexStrideInBytes = sizeof(float3);
	input_desc.curveArray.widthBuffers = &radii_dev;
	input_desc.curveArray.widthStrideInBytes = sizeof(float);
	input_desc.curveArray.normalBuffers = 0;
	input_desc.curveArray.normalStrideInBytes = 0;
	input_desc.curveArray.indexBuffer = indices_dev;
	input_desc.curveArray.indexStrideInBytes = sizeof(unsigned);
	input_desc.curveArray.flag = OPTIX_GEOMETRY_FLAG_NONE;
	input_desc.curveArray.primitiveIndexOffset = 0;
	input_desc.curveArray.endcapFlags = OPTIX_CURVE_ENDCAP_ON;

	OptixAccelBufferSizes accelds_buffer_sizes = {0};
	OPTIX_CHECK_SET(
		optixAccelComputeMemoryUsage(
			context, &accel_options, &input_desc, 1/* num build inputs */, &accelds_buffer_sizes
		),
		success
	);

	CUdeviceptr tmpbuf = 0;
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&tmpbuf), accelds_buffer_sizes.tempSizeInBytes), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&accelds_mem), accelds_buffer_sizes.outputSizeInBytes), success);

	OPTIX_CHECK_SET(
		optixAccelBuild(
			context, 0/*CUDA stream*/, &accel_options, &input_desc, 1/*num build inputs*/,
			tmpbuf, accelds_buffer_sizes.tempSizeInBytes, accelds_mem, accelds_buffer_sizes.outputSizeInBytes,
			&lp.accelds,  // <-- our acceleration datastructure!!!
			nullptr/*emitted property list*/, 0/*num emitted properties*/
		),
		success
	);

	// we can now free the scratch buffer used during build and the vertex inputs, since they are stored
	// directly in the accel-ds (we won't consider cudaFree failing a failure of the whole function)
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(tmpbuf)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(positions_dev)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(radii_dev)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(indices_dev)));

	// done!
	return success;
}

bool optixtracer_textured_spline_tube_builtin::update_pipeline (void)
{
	////
	// Prelude

	// make sure we start with a blank slate
	destroy_pipeline();

	// CUDA/OptiX log storage
	std::string compiler_log(8192, 0);
	char* log = compiler_log.data();
	size_t sizeof_log = compiler_log.size();

	// pipeline build options
	constexpr unsigned max_trace_depth = 1;
	OptixPipelineCompileOptions pipeline_options = {};

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;


	////
	// Create modules

	/* local scope */ {
		OptixModuleCompileOptions mod_options = {};
		mod_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
		mod_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
		mod_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;

		pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
		pipeline_options.numPayloadValues = 14;
		pipeline_options.numAttributeValues = 1;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_options.exceptionFlags =
			OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
	#else
		pipeline_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	#endif
		pipeline_options.pipelineLaunchParamsVariableName = "params";
		pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_QUADRATIC_BSPLINE;

		const auto ptx = compile_cu2ptx("cuda/optix_curves_builtin2.cu", "rayc_builtin2", {CUDA_NVRTC_OPTIONS}, &compiler_log);
		const size_t ptx_size = ptx.size();
		if (!ptx_size)
		{
			std::cerr << "ERROR compiling OptiX device code! Log:" << std::endl
			          << compiler_log << std::endl<<std::endl;
			return false; // no resources allocated yet, so we can just return
		}
		OPTIX_CHECK_LOG_SET(
			optixModuleCreateFromPTX(
				context, &mod_options, &pipeline_options, ptx.data(), ptx_size, log, &sizeof_log, &mod_shading
			),
			log, sizeof_log, success
		);

		OptixBuiltinISOptions builtin_isectshader_options = {};
		builtin_isectshader_options.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_QUADRATIC_BSPLINE;
		builtin_isectshader_options.curveEndcapFlags = OPTIX_CURVE_ENDCAP_ON;
		OPTIX_CHECK_SET(
			optixBuiltinISModuleGet(context, &mod_options, &pipeline_options, &builtin_isectshader_options, &mod_geom),
			success
		);
	}


	////
	// Create program groups

	/* local scope */ {
		// common options
		OptixProgramGroupOptions prg_options = {};

		// raygen shader
		OptixProgramGroupDesc prg_raygen_desc = {};
		prg_raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
		prg_raygen_desc.raygen.module = mod_shading;
		prg_raygen_desc.raygen.entryFunctionName = "__raygen__basic";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_raygen_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_raygen
			),
			log, sizeof_log, success
		);

		// miss shader
		OptixProgramGroupDesc prg_miss_desc = {};
		prg_miss_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
		prg_miss_desc.miss.module = mod_shading;
		prg_miss_desc.miss.entryFunctionName = "__miss__ms";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_miss_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_miss
			),
			log, sizeof_log, success
		);

		// hit shader group
		OptixProgramGroupDesc prg_hit_desc = {};
		prg_hit_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
		prg_hit_desc.hitgroup.moduleCH = mod_shading;
		prg_hit_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
		prg_hit_desc.hitgroup.moduleIS = mod_geom;
		prg_hit_desc.hitgroup.entryFunctionNameIS = 0; // automatically supplied for built-in intersection shader
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_hit_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_hit
			),
			log, sizeof_log, success
		);
	}


	////
	// Link pipeline

	/* local scope */ {
		constexpr unsigned num_prgs = 3;
		OptixProgramGroup prgs[num_prgs] = {prg_raygen, prg_miss, prg_hit};
		OptixPipelineLinkOptions pipeline_linkoptions = {};
		pipeline_linkoptions.maxTraceDepth = max_trace_depth;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
	#else
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
	#endif
		OPTIX_CHECK_LOG_SET(
			optixPipelineCreate(
				context, &pipeline_options, &pipeline_linkoptions, prgs, num_prgs, log, &sizeof_log, &lp.pipeline
			),
			log, sizeof_log, success
		);

		OptixStackSizes stacksizes = {};
		for (const auto &prg : prgs)
			OPTIX_CHECK_SET(optixUtilAccumulateStackSizes(prg, &stacksizes), success);

		// ToDo: ??? investigate what exactly this is, why it is needed, what to best do here, etc.
		uint32_t direct_callable_stack_size_from_traversal;
		uint32_t direct_callable_stack_size_from_state;
		uint32_t continuation_stack_size;
		OPTIX_CHECK_SET(
			optixUtilComputeStackSizes(
				&stacksizes, max_trace_depth, 0/* maxCCDepth */, 0/* maxDCDepth */,
				&direct_callable_stack_size_from_traversal,
				&direct_callable_stack_size_from_state, &continuation_stack_size
			),
			success
		);
		OPTIX_CHECK_SET(
			optixPipelineSetStackSize(
				lp.pipeline, direct_callable_stack_size_from_traversal,
				direct_callable_stack_size_from_state, continuation_stack_size,
				1 // <-- max traversable depth - we only use the top-level GAS currently
			),
			success
		);
	}


	////
	// Set up shader binding table

	/* local scope */ {
		// our SBT record types
		typedef sbt_record<data_raygen> sbt_record_raygen;
		typedef sbt_record<data_miss>   sbt_record_miss;
		typedef sbt_record<data_hit>    sbt_record_hit;

		// prepare entries
		// - raygen shaders
		CUdeviceptr  raygen_record;
		const size_t raygen_record_size = sizeof(sbt_record_raygen);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&raygen_record), raygen_record_size), success);
		sbt_record_raygen rg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_raygen, &rg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(raygen_record), &rg_sbt, raygen_record_size, cudaMemcpyHostToDevice), success);
		// - miss shaders
		CUdeviceptr miss_record;
		size_t      miss_record_size = sizeof(sbt_record_miss);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&miss_record), miss_record_size), success);
		sbt_record_miss ms_sbt;
		ms_sbt.data = {0.0f, 0.0f, 0.0f, 0.0f};  // background color (fully transparent black)
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_miss, &ms_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(miss_record), &ms_sbt, miss_record_size, cudaMemcpyHostToDevice), success);
		// - hit shaders
		CUdeviceptr hitgroup_record;
		size_t      hitgroup_record_size = sizeof(sbt_record_hit);
		CUDA_CHECK_SET (cudaMalloc(reinterpret_cast<void**>(&hitgroup_record), hitgroup_record_size), success);
		sbt_record_hit hg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_hit, &hg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(hitgroup_record), &hg_sbt, hitgroup_record_size, cudaMemcpyHostToDevice), success);

		// build up the SBT
		sbt = {};
		sbt.raygenRecord = raygen_record;
		sbt.missRecordBase = miss_record;
		sbt.missRecordStrideInBytes = sizeof(sbt_record_miss);
		sbt.missRecordCount = 1;
		sbt.hitgroupRecordBase = hitgroup_record;
		sbt.hitgroupRecordStrideInBytes = sizeof(sbt_record_hit);
		sbt.hitgroupRecordCount = 1;
	}

	// Create the device memory for our launch params
	/* local scope */ {
		bool params_malloc_success = true;
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.params), sizeof(curve_rt_params)), params_malloc_success);
		if (params_malloc_success)
			lp.params_size = sizeof(curve_rt_params);
		else
			success = false;
	}

	// done!
	return success;
}


////
// optixtracer_textured_spline_tube_builtincubic

optixtracer_textured_spline_tube_builtincubic::optixtracer_textured_spline_tube_builtincubic()
{}

optixtracer_textured_spline_tube_builtincubic::optixtracer_textured_spline_tube_builtincubic(
	optixtracer_textured_spline_tube_builtincubic &&other
)
	: optixtracer_textured_spline_tube(std::move(other))
{
	// we don't have any additional data, so just delegate to base...
}

optixtracer_textured_spline_tube_builtincubic::~optixtracer_textured_spline_tube_builtincubic()
{
	// delegate to destroy method
	destroy();
}

optixtracer_textured_spline_tube_builtincubic& optixtracer_textured_spline_tube_builtincubic::operator= (
	optixtracer_textured_spline_tube_builtincubic &&other
)
{
	// we don't have any additional data, so just delegate to base
	super::operator=(std::move(other));

	// done!
	return *this;
}

void optixtracer_textured_spline_tube_builtincubic::destroy (void)
{
	destroy_pipeline();
	destroy_accelds();
	context = nullptr;
}

bool optixtracer_textured_spline_tube_builtincubic::built (void) const
{
	// we _must_ be good to go if somehow we ended up with a valid pipeline
	return lp.pipeline;
}

optixtracer_textured_spline_tube_builtincubic optixtracer_textured_spline_tube_builtincubic::build (
	OptixDeviceContext context, const traj_manager<float>::render_data *render_data
)
{
	optixtracer_textured_spline_tube_builtincubic tracer;
	tracer.context = context;
	if (!tracer.update_accelds(render_data))
		return optixtracer_textured_spline_tube_builtincubic();
	if (!tracer.update_pipeline())
		return optixtracer_textured_spline_tube_builtincubic();
	tracer.lp.sbt = &tracer.sbt;
	return tracer;
}

void optixtracer_textured_spline_tube_builtincubic::destroy_accelds (void)
{
	lp.accelds = 0; // ToDo: check if we can really leave the accelds handle dangling like this (there appears to be no destroy function for it?)
	CUDA_SAFE_FREE(accelds_mem);
}

void optixtracer_textured_spline_tube_builtincubic::destroy_pipeline (void)
{
	CUDA_SAFE_FREE(lp.params); lp.params_size = 0;
	OPTIX_SAFE_DESTROY_PIPELINE(lp.pipeline);
	OPTIX_SAFE_DESTROY_MODULE(mod_shading);
	OPTIX_SAFE_DESTROY_MODULE(mod_geom);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_hit);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_miss);
	OPTIX_SAFE_DESTROY_PROGGROUP(prg_raygen);
	sbt = {}; lp.sbt = nullptr;
}

bool optixtracer_textured_spline_tube_builtincubic::update_accelds (const traj_manager<float>::render_data *render_data)
{
	// make sure we start with a blank slate
	destroy_accelds();

	// use default options for simplicity
	OptixAccelBuildOptions accel_options = {};
	accel_options.buildFlags = OPTIX_BUILD_FLAG_ALLOW_RANDOM_VERTEX_ACCESS;
	accel_options.operation = OPTIX_BUILD_OPERATION_BUILD;

	// stage geometry for upload - we don't use the available render data directly since in order to use the built-in
	// cubic curve primitives, we need to store the segments in a different basis (here, we use Catmull-Rom)
	unsigned num;
	std::vector<float3> positions;
	std::vector<float> radii;
	std::vector<unsigned> indices;

	// prepare CPU-side vertex staging area
	/* local scope */ {
		const auto &rd_pos = render_data->positions;
		const auto &rd_tan = render_data->tangents;
		const auto &rd_rad = render_data->radii;
		const auto &rd_idx = render_data->indices;
		indices.reserve(rd_idx.size()/2);
		num = (unsigned)rd_idx.size()*2;
		positions.reserve(num);
		radii.reserve(num);

		// convert data representation:
		// - convert Hermite-basis control points to Catmull-Rom basis
		// - adapt indices to OptiX curve primitive scheme
		constexpr float mscale = 1.09375f; // <-- hand-tuned tangent scaling factor to get results visually closer to split quadratic curves
		for (const auto &ds : render_data->datasets) for (const auto &traj : ds.trajs)
		{
			const unsigned num_segs = traj.n / 2;
			for (unsigned i=0; i<num_segs; i++)
			{
				const unsigned idx = rd_idx[traj.i0]+i;
				const vec4  &t0 = rd_tan[idx],    &t1 = rd_tan[idx+1];
				const vec3  &p0 = rd_pos[idx],    &p1 = rd_pos[idx+1],
				             m0 = mscale*vec3(t0), m1 = mscale*vec3(t1);
				const float &r0 = rd_rad[idx],    &r1 = rd_rad[idx+1];
				indices.emplace_back(unsigned(positions.size()));
				// cr0
				positions.emplace_back(to_float3(get_cr0(p0, m0, p1)));
				radii.emplace_back(get_cr0(r0, /*mscale**/t0.w(), r1));
				// cr1
				positions.emplace_back(to_float3(p0));
				radii.push_back(r0);
				// cr2
				positions.emplace_back(to_float3(p1));
				radii.push_back(r1);
				// cr3
				positions.emplace_back(to_float3(get_cr3(p0, p1, m1)));
				radii.emplace_back(get_cr3(r0, r1, /*mscale**/t1.w()));
			}
		}
	}

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;

	// prepare geometry device memory
	CUdeviceptr positions_dev=0, radii_dev=0;
	const size_t positions_size = num*sizeof(float3), radii_size = num*sizeof(float);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&positions_dev), positions_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(positions_dev), positions.data(), positions_size, cudaMemcpyHostToDevice), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&radii_dev), radii_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(radii_dev), radii.data(), radii_size, cudaMemcpyHostToDevice), success);

	// upload segment indices
	CUdeviceptr indices_dev = 0;
	const size_t indices_size = indices.size()*sizeof(unsigned);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&indices_dev), indices_size), success);
	CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(indices_dev), indices.data(), indices_size, cudaMemcpyHostToDevice), success);

	// OptiX accel-ds build input descriptor
	OptixBuildInput input_desc = {};
	input_desc.type = OPTIX_BUILD_INPUT_TYPE_CURVES;
	input_desc.curveArray.curveType = OPTIX_PRIMITIVE_TYPE_ROUND_CATMULLROM;
	input_desc.curveArray.numPrimitives = (unsigned)indices.size();
	input_desc.curveArray.vertexBuffers = &positions_dev;
	input_desc.curveArray.numVertices = num;
	input_desc.curveArray.vertexStrideInBytes = sizeof(float3);
	input_desc.curveArray.widthBuffers = &radii_dev;
	input_desc.curveArray.widthStrideInBytes = sizeof(float);
	input_desc.curveArray.normalBuffers = 0;
	input_desc.curveArray.normalStrideInBytes = 0;
	input_desc.curveArray.indexBuffer = indices_dev;
	input_desc.curveArray.indexStrideInBytes = sizeof(unsigned);
	input_desc.curveArray.flag = OPTIX_GEOMETRY_FLAG_NONE;
	input_desc.curveArray.primitiveIndexOffset = 0;
	input_desc.curveArray.endcapFlags = OPTIX_CURVE_ENDCAP_ON;

	OptixAccelBufferSizes accelds_buffer_sizes = {0};
	OPTIX_CHECK_SET(
		optixAccelComputeMemoryUsage(
			context, &accel_options, &input_desc, 1/* num build inputs */, &accelds_buffer_sizes
		),
		success
	);

	CUdeviceptr tmpbuf = 0;
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&tmpbuf), accelds_buffer_sizes.tempSizeInBytes), success);
	CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&accelds_mem), accelds_buffer_sizes.outputSizeInBytes), success);

	OPTIX_CHECK_SET(
		optixAccelBuild(
			context, 0/*CUDA stream*/, &accel_options, &input_desc, 1/*num build inputs*/,
			tmpbuf, accelds_buffer_sizes.tempSizeInBytes, accelds_mem, accelds_buffer_sizes.outputSizeInBytes,
			&lp.accelds,  // <-- our acceleration datastructure!!!
			nullptr/*emitted property list*/, 0/*num emitted properties*/
		),
		success
	);

	// we can now free the scratch buffer used during build and the vertex inputs, since they are stored
	// directly in the accel-ds (we won't consider cudaFree failing a failure of the whole function)
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(tmpbuf)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(positions_dev)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(radii_dev)));
	CUDA_CHECK(cudaFree(reinterpret_cast<void*>(indices_dev)));

	// done!
	return success;
}

bool optixtracer_textured_spline_tube_builtincubic::update_pipeline (void)
{
	////
	// Prelude

	// make sure we start with a blank slate
	destroy_pipeline();

	// CUDA/OptiX log storage
	std::string compiler_log(8192, 0);
	char* log = compiler_log.data();
	size_t sizeof_log = compiler_log.size();

	// pipeline build options
	constexpr unsigned max_trace_depth = 1;
	OptixPipelineCompileOptions pipeline_options = {};

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that would result in both host and device memory leaks
	bool success = true;


	////
	// Create modules

	/* local scope */ {
		OptixModuleCompileOptions mod_options = {};
		mod_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
		mod_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
		mod_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;

		pipeline_options.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_GAS;
		pipeline_options.numPayloadValues = 14;
		pipeline_options.numAttributeValues = 1;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_options.exceptionFlags =
			OPTIX_EXCEPTION_FLAG_DEBUG | OPTIX_EXCEPTION_FLAG_TRACE_DEPTH | OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
	#else
		pipeline_options.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;
	#endif
		pipeline_options.pipelineLaunchParamsVariableName = "params";
		pipeline_options.usesPrimitiveTypeFlags = OPTIX_PRIMITIVE_TYPE_FLAGS_ROUND_CATMULLROM;

		const auto ptx = compile_cu2ptx("cuda/optix_curves_builtin3.cu", "rayc_builtin3", {CUDA_NVRTC_OPTIONS}, &compiler_log);
		const size_t ptx_size = ptx.size();
		if (!ptx_size)
		{
			std::cerr << "ERROR compiling OptiX device code! Log:" << std::endl
			          << compiler_log << std::endl<<std::endl;
			return false; // no resources allocated yet, so we can just return
		}
		OPTIX_CHECK_LOG_SET(
			optixModuleCreateFromPTX(
				context, &mod_options, &pipeline_options, ptx.data(), ptx_size, log, &sizeof_log, &mod_shading
			),
			log, sizeof_log, success
		);

		OptixBuiltinISOptions builtin_isectshader_options = {};
		builtin_isectshader_options.builtinISModuleType = OPTIX_PRIMITIVE_TYPE_ROUND_CATMULLROM;
		builtin_isectshader_options.curveEndcapFlags = OPTIX_CURVE_ENDCAP_ON;
		OPTIX_CHECK_SET(
			optixBuiltinISModuleGet(context, &mod_options, &pipeline_options, &builtin_isectshader_options, &mod_geom),
			success
		);
	}


	////
	// Create program groups

	/* local scope */ {
		// common options
		OptixProgramGroupOptions prg_options = {};

		// raygen shader
		OptixProgramGroupDesc prg_raygen_desc = {};
		prg_raygen_desc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
		prg_raygen_desc.raygen.module = mod_shading;
		prg_raygen_desc.raygen.entryFunctionName = "__raygen__basic";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_raygen_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_raygen
			),
			log, sizeof_log, success
		);

		// miss shader
		OptixProgramGroupDesc prg_miss_desc = {};
		prg_miss_desc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
		prg_miss_desc.miss.module = mod_shading;
		prg_miss_desc.miss.entryFunctionName = "__miss__ms";
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_miss_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_miss
			),
			log, sizeof_log, success
		);

		// hit shader group
		OptixProgramGroupDesc prg_hit_desc = {};
		prg_hit_desc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
		prg_hit_desc.hitgroup.moduleCH = mod_shading;
		prg_hit_desc.hitgroup.entryFunctionNameCH = "__closesthit__ch";
		prg_hit_desc.hitgroup.moduleIS = mod_geom;
		prg_hit_desc.hitgroup.entryFunctionNameIS = 0; // automatically supplied for built-in intersection shader
		OPTIX_CHECK_LOG_SET(
			optixProgramGroupCreate(
				context, &prg_hit_desc, 1/*num program groups*/, &prg_options, log, &sizeof_log, &prg_hit
			),
			log, sizeof_log, success
		);
	}


	////
	// Link pipeline

	/* local scope */ {
		constexpr unsigned num_prgs = 3;
		OptixProgramGroup prgs[num_prgs] = {prg_raygen, prg_miss, prg_hit};
		OptixPipelineLinkOptions pipeline_linkoptions = {};
		pipeline_linkoptions.maxTraceDepth = max_trace_depth;
	#ifdef _DEBUG  // Enables debug exceptions during optix launches. This may incur significant performance cost and should only be done during development.
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;
	#else
		pipeline_linkoptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_NONE;
	#endif
		OPTIX_CHECK_LOG_SET(
			optixPipelineCreate(
				context, &pipeline_options, &pipeline_linkoptions, prgs, num_prgs, log, &sizeof_log, &lp.pipeline
			),
			log, sizeof_log, success
		);

		OptixStackSizes stacksizes = {};
		for (const auto &prg : prgs)
			OPTIX_CHECK_SET(optixUtilAccumulateStackSizes(prg, &stacksizes), success);

		// ToDo: ??? investigate what exactly this is, why it is needed, what to best do here, etc.
		uint32_t direct_callable_stack_size_from_traversal;
		uint32_t direct_callable_stack_size_from_state;
		uint32_t continuation_stack_size;
		OPTIX_CHECK_SET(
			optixUtilComputeStackSizes(
				&stacksizes, max_trace_depth, 0/* maxCCDepth */, 0/* maxDCDepth */,
				&direct_callable_stack_size_from_traversal,
				&direct_callable_stack_size_from_state, &continuation_stack_size
			),
			success
		);
		OPTIX_CHECK_SET(
			optixPipelineSetStackSize(
				lp.pipeline, direct_callable_stack_size_from_traversal,
				direct_callable_stack_size_from_state, continuation_stack_size,
				1 // <-- max traversable depth - we only use the top-level GAS currently
			),
			success
		);
	}


	////
	// Set up shader binding table

	/* local scope */ {
		// our SBT record types
		typedef sbt_record<data_raygen> sbt_record_raygen;
		typedef sbt_record<data_miss>   sbt_record_miss;
		typedef sbt_record<data_hit>    sbt_record_hit;

		// prepare entries
		// - raygen shaders
		CUdeviceptr  raygen_record;
		const size_t raygen_record_size = sizeof(sbt_record_raygen);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&raygen_record), raygen_record_size), success);
		sbt_record_raygen rg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_raygen, &rg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(raygen_record), &rg_sbt, raygen_record_size, cudaMemcpyHostToDevice), success);
		// - miss shaders
		CUdeviceptr miss_record;
		size_t      miss_record_size = sizeof(sbt_record_miss);
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&miss_record), miss_record_size), success);
		sbt_record_miss ms_sbt;
		ms_sbt.data = {0.0f, 0.0f, 0.0f, 0.0f};  // background color (fully transparent black)
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_miss, &ms_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(miss_record), &ms_sbt, miss_record_size, cudaMemcpyHostToDevice), success);
		// - hit shaders
		CUdeviceptr hitgroup_record;
		size_t      hitgroup_record_size = sizeof(sbt_record_hit);
		CUDA_CHECK_SET (cudaMalloc(reinterpret_cast<void**>(&hitgroup_record), hitgroup_record_size), success);
		sbt_record_hit hg_sbt;
		OPTIX_CHECK_SET(optixSbtRecordPackHeader(prg_hit, &hg_sbt), success);
		CUDA_CHECK_SET(cudaMemcpy(reinterpret_cast<void*>(hitgroup_record), &hg_sbt, hitgroup_record_size, cudaMemcpyHostToDevice), success);

		// build up the SBT
		sbt = {};
		sbt.raygenRecord = raygen_record;
		sbt.missRecordBase = miss_record;
		sbt.missRecordStrideInBytes = sizeof(sbt_record_miss);
		sbt.missRecordCount = 1;
		sbt.hitgroupRecordBase = hitgroup_record;
		sbt.hitgroupRecordStrideInBytes = sizeof(sbt_record_hit);
		sbt.hitgroupRecordCount = 1;
	}

	// Create the device memory for our launch params
	/* local scope */ {
		bool params_malloc_success = true;
		CUDA_CHECK_SET(cudaMalloc(reinterpret_cast<void**>(&lp.params), sizeof(curve_rt_params)), params_malloc_success);
		if (params_malloc_success)
			lp.params_size = sizeof(curve_rt_params);
		else
			success = false;
	}

	// done!
	return success;
}
