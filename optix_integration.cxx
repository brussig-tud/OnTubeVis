
//////
//
// Includes
//

// C++ STL
#include <fstream>

// OpenGL
#include <GL/glew.h>

// CUDA/Optix
#include <nvrtc.h>
#include <optix_function_table_definition.h>
#include <cuda_gl_interop.h>

// Implemented header
#include "optix_integration.h"



//////
//
// Function implementations
//

std::vector<char> compile_cu2ptx (
	const std::string &filename,                      // Cuda C input file name
	const std::string &name,                          // arbitrary name the compiled code should be assigned in CUDA
	const std::vector<const char*> &compiler_options, // CUDA compiler options
	std::string *log_out                              // [out] the compilation log
)
{
	// CUDA runtime compiler include directories
	const std::string mydir = std::filesystem::path(filename).parent_path().string();
	const std::vector<std::string> include_args = {
		"-I" CUDA_RUNTIME_COMPILER__INC_DIR_CUDA, "-I" CUDA_RUNTIME_COMPILER__INC_DIR_OPTIX,
		"-I" CUDA_RUNTIME_COMPILER__INC_DIR_OPTIXSDK, "-I" CUDA_RUNTIME_COMPILER__INC_DIR_OPTIXSDK_CUDA,
		"-I"+mydir
	};

	// adapted from getCuStringFromFile(cu, location, sampleDir, filename)
	std::string cu;
	{
		std::ifstream file(filename, std::ios::binary);
		if (!file.good()) {
			if (log_out)
				*log_out = "Could not open file '"+filename+"'";
			return std::vector<char>();
		}
		std::vector<unsigned char> buffer =
			std::vector<unsigned char>(std::istreambuf_iterator<char>(file), {});
		cu.assign(buffer.begin(), buffer.end());
	}

	// adapted from getPtxFromCuString(*ptx, sampleDir, cu.c_str(), location.c_str(), log, compilerOptions)
	std::vector<char> ptx;
	{
		// create program
		nvrtcProgram prog = 0;
		NVRTC_CHECK(nvrtcCreateProgram(&prog, cu.c_str(), name.c_str(), 0, nullptr, nullptr));

		// gather compiler options
		std::vector<const char*> options;
		for (const auto &inc : include_args)
			options.emplace_back(inc.c_str());
		std::copy(compiler_options.begin(), compiler_options.end(), std::back_inserter(options));

		// JIT compile CU to PTX
		// - the compile call
		const nvrtcResult compileRes = nvrtcCompileProgram(prog, (int)options.size(), options.data());
		// - retrieve log output
		size_t log_size = 0;
		NVRTC_CHECK(nvrtcGetProgramLogSize(prog, &log_size));
		if (log_out && log_size > 1) {
			log_out->resize(log_size);
			NVRTC_CHECK(nvrtcGetProgramLog(prog, log_out->data()));
		}
		// - retrieve PTX code if compilation successful
		if (compileRes == NVRTC_SUCCESS) {
			size_t ptx_size = 0;
			NVRTC_CHECK(nvrtcGetPTXSize(prog, &ptx_size));
			ptx.resize(ptx_size);
			NVRTC_CHECK(nvrtcGetPTX(prog, ptx.data()));
		}
		// - cleanup
		NVRTC_CHECK(nvrtcDestroyProgram(&prog));
	}

	// done!
	return std::move(ptx);
}



//////
//
// Class implementations
//

////
// cuda_output_buffer

template <class pxl_fmt>
cuda_output_buffer<pxl_fmt>::cuda_output_buffer(CUDAOutputBufferType type, int width, int height)
	: m_type(type)
{
	// Output dimensions must be at least 1 in both x and y to avoid an error
	// with cudaMalloc.
	ensure_min_size(width, height);
 
	// If using GL Interop, expect that the active device is also the display device.
	if (type == CUDAOutputBufferType::GL_INTEROP)
	{
		int current_device, is_display_device;
		CUDA_CHECK(cudaGetDevice(&current_device));
		CUDA_CHECK(cudaDeviceGetAttribute(&is_display_device, cudaDevAttrKernelExecTimeout, current_device));
		if (!is_display_device)
			std::cerr << "[OptiX] ERROR: GL interop is only available on display device, please use display device"
			             " for optimal performance. Alternatively you can disable GL interop with --no-gl-interop and run with"
			             " degraded performance." << std::endl<<std::endl;
    }
    resize(width, height);
}

template <class pxl_fmt>
cuda_output_buffer<pxl_fmt>::~cuda_output_buffer()
{
	try
	{
		make_current();
		if (m_type == CUDAOutputBufferType::CUDA_DEVICE || m_type == CUDAOutputBufferType::CUDA_P2P)
			CUDA_CHECK(cudaFree(reinterpret_cast<void*>(m_device_pixels)));
		else if (m_type == CUDAOutputBufferType::ZERO_COPY)
			CUDA_CHECK(cudaFreeHost(reinterpret_cast<void*>(m_host_zcopy_pixels)));
		// no action needed for CUDAOutputBufferType::GL_INTEROP

		if (m_pbo != 0u)
		{
			GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
			GL_CHECK(glDeleteBuffers(1, &m_pbo));
		}
	}
	catch (std::exception& e) {
		std::cerr << "[OptiX] ERROR: cuda_output_buffer destructor caught exception: " << e.what() << std::endl;
	}
}

template <class pxl_fmt>
void cuda_output_buffer<pxl_fmt>::resize (int width, int height)
{
	// Output dimensions must be at least 1 in both x and y to avoid an error
	// with cudaMalloc.
	ensure_min_size(width, height);

	if (m_width == width && m_height == height)
		return;

	m_width = width;
	m_height = height;

	make_current();

	if (m_type == CUDAOutputBufferType::CUDA_DEVICE || m_type == CUDAOutputBufferType::CUDA_P2P)
	{
		CUDA_CHECK(cudaFree(reinterpret_cast<void*>(m_device_pixels)));
		CUDA_CHECK(cudaMalloc(
			reinterpret_cast<void**>(&m_device_pixels),
			m_width * m_height * sizeof(pxl_fmt)
		));
	}

	if (m_type == CUDAOutputBufferType::GL_INTEROP || m_type == CUDAOutputBufferType::CUDA_P2P)
	{
		// GL buffer gets resized below
		GL_CHECK(glGenBuffers(1, &m_pbo));
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, m_pbo));
		GL_CHECK(glBufferData(GL_ARRAY_BUFFER, sizeof(pxl_fmt) * m_width * m_height, nullptr, GL_STREAM_DRAW));
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0u));

		CUDA_CHECK(cudaGraphicsGLRegisterBuffer(
			&m_cuda_gfx_resource,
			m_pbo,
			cudaGraphicsMapFlagsWriteDiscard
		));
	}

	if (m_type == CUDAOutputBufferType::ZERO_COPY)
	{
		CUDA_CHECK(cudaFreeHost(reinterpret_cast<void*>(m_host_zcopy_pixels)));
		CUDA_CHECK(cudaHostAlloc(
			reinterpret_cast<void**>(&m_host_zcopy_pixels),
			m_width * m_height * sizeof(pxl_fmt),
			cudaHostAllocPortable | cudaHostAllocMapped
		));
		CUDA_CHECK(cudaHostGetDevicePointer(
			reinterpret_cast<void**>(&m_device_pixels),
			reinterpret_cast<void*>(m_host_zcopy_pixels),
			0 /*flags*/
		));
	}

	if (m_type != CUDAOutputBufferType::GL_INTEROP && m_type != CUDAOutputBufferType::CUDA_P2P && m_pbo != 0u)
	{
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, m_pbo));
		GL_CHECK(glBufferData(GL_ARRAY_BUFFER, sizeof(pxl_fmt) * m_width * m_height, nullptr, GL_STREAM_DRAW));
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0u));
	}

	if (!m_host_pixels.empty())
		m_host_pixels.resize(m_width * m_height);
}

template <class pxl_fmt>
pxl_fmt* cuda_output_buffer<pxl_fmt>::map (void)
{
	if (m_type == CUDAOutputBufferType::GL_INTEROP)
	{
		make_current();

		size_t buffer_size = 0u;
		CUDA_CHECK(cudaGraphicsMapResources(1, &m_cuda_gfx_resource, m_stream));
		CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(
			reinterpret_cast<void**>(&m_device_pixels),
			&buffer_size,
			m_cuda_gfx_resource
		));
	}
	// no action needed for CUDA_DEVICE, CUDA_P2P or ZERO_COPY

	return m_device_pixels;
}

template <class pxl_fmt>
void cuda_output_buffer<pxl_fmt>::unmap (void)
{
	make_current();

	if (m_type == CUDAOutputBufferType::CUDA_DEVICE || m_type == CUDAOutputBufferType::CUDA_P2P)
		CUDA_CHECK(cudaStreamSynchronize(m_stream));
	else if (m_type == CUDAOutputBufferType::GL_INTEROP)
		CUDA_CHECK(cudaGraphicsUnmapResources(1, &m_cuda_gfx_resource, m_stream));
	else // m_type == CUDAOutputBufferType::ZERO_COPY
		CUDA_CHECK(cudaStreamSynchronize(m_stream));
}

template <class pxl_fmt>
GLuint cuda_output_buffer<pxl_fmt>::get_pbo (void)
{
	if (m_pbo == 0u)
		GL_CHECK(glGenBuffers(1, &m_pbo));

	const size_t buffer_size = m_width * m_height * sizeof(pxl_fmt);

	if (m_type == CUDAOutputBufferType::CUDA_DEVICE)
	{
		// We need a host buffer to act as a way-station
		if (m_host_pixels.empty())
			m_host_pixels.resize(m_width * m_height);

		make_current();
		CUDA_CHECK(cudaMemcpy(
			static_cast<void*>(m_host_pixels.data()),
			m_device_pixels,
			buffer_size,
			cudaMemcpyDeviceToHost
		));

		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, m_pbo));
		GL_CHECK(glBufferData(
			GL_ARRAY_BUFFER,
			buffer_size,
			static_cast<void*>(m_host_pixels.data()),
			GL_STREAM_DRAW
		));
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
	}
	else if (m_type == CUDAOutputBufferType::GL_INTEROP)
		/* DoNothing() */;	// <-- no action needed
	else if (m_type == CUDAOutputBufferType::CUDA_P2P)
	{
		make_current();
		void* pbo_buff = nullptr;
		size_t dummy_size = 0;

		CUDA_CHECK(cudaGraphicsMapResources(1, &m_cuda_gfx_resource, m_stream));
		CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(&pbo_buff, &dummy_size, m_cuda_gfx_resource));
		CUDA_CHECK(cudaMemcpy(pbo_buff, m_device_pixels, buffer_size, cudaMemcpyDeviceToDevice));
		CUDA_CHECK(cudaGraphicsUnmapResources(1, &m_cuda_gfx_resource, m_stream));
	}
	else // m_type == CUDAOutputBufferType::ZERO_COPY
	{
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, m_pbo));
		GL_CHECK(glBufferData(
			GL_ARRAY_BUFFER,
			buffer_size,
			static_cast<void*>(m_host_zcopy_pixels),
			GL_STREAM_DRAW
		));
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
	}

	return m_pbo;
}

template <class pxl_fmt>
void cuda_output_buffer<pxl_fmt>::delete_pbo (void)
{
	if (m_pbo)
	{
		GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
		GL_CHECK(glDeleteBuffers(1, &m_pbo));
		m_pbo = 0;
	}
}

template <class pxl_fmt>
pxl_fmt* cuda_output_buffer<pxl_fmt>::get_host_ptr (void)
{
	if (m_type == CUDAOutputBufferType::CUDA_DEVICE || m_type == CUDAOutputBufferType::CUDA_P2P ||
	    m_type == CUDAOutputBufferType::GL_INTEROP)
	{
		m_host_pixels.resize(m_width * m_height);

		make_current();
		CUDA_CHECK(cudaMemcpy(
			static_cast<void*>(m_host_pixels.data()),
			map(),
			m_width * m_height * sizeof(pxl_fmt),
			cudaMemcpyDeviceToHost
		));
		unmap();

		return m_host_pixels.data();
	}
	else // m_type == CUDAOutputBufferType::ZERO_COPY
		return m_host_zcopy_pixels;
}



//////
//
// Explicit template instantiations
//

// Only variants for CUDA-compatible integral types are intended
// - single-byte components
template class cuda_output_buffer<char1>;
template class cuda_output_buffer<uchar1>;
template class cuda_output_buffer<char2>;
template class cuda_output_buffer<uchar2>;
template class cuda_output_buffer<char3>;
template class cuda_output_buffer<uchar3>;
template class cuda_output_buffer<char4>;
template class cuda_output_buffer<uchar4>;
// - two-byte components
template class cuda_output_buffer<short1>;
template class cuda_output_buffer<ushort1>;
template class cuda_output_buffer<short2>;
template class cuda_output_buffer<ushort2>;
template class cuda_output_buffer<short3>;
template class cuda_output_buffer<ushort3>;
template class cuda_output_buffer<short4>;
template class cuda_output_buffer<ushort4>;
// - four-byte components
template class cuda_output_buffer<int1>;
template class cuda_output_buffer<uint1>;
template class cuda_output_buffer<int2>;
template class cuda_output_buffer<uint2>;
template class cuda_output_buffer<int3>;
template class cuda_output_buffer<uint3>;
template class cuda_output_buffer<int4>;
template class cuda_output_buffer<uint4>;
template class cuda_output_buffer<long1>;
template class cuda_output_buffer<ulong1>;
template class cuda_output_buffer<long2>;
template class cuda_output_buffer<ulong2>;
template class cuda_output_buffer<long3>;
template class cuda_output_buffer<ulong3>;
template class cuda_output_buffer<long4>;
template class cuda_output_buffer<ulong4>;
// - eight-byte components
template class cuda_output_buffer<longlong1>;
template class cuda_output_buffer<ulonglong1>;
template class cuda_output_buffer<longlong2>;
template class cuda_output_buffer<ulonglong2>;
template class cuda_output_buffer<longlong3>;
template class cuda_output_buffer<ulonglong3>;
template class cuda_output_buffer<longlong4>;
template class cuda_output_buffer<ulonglong4>;
// - single-precision floating point components
template class cuda_output_buffer<float1>;
template class cuda_output_buffer<float2>;
template class cuda_output_buffer<float3>;
template class cuda_output_buffer<float4>;
// - double-precision floating point components
template class cuda_output_buffer<double1>;
template class cuda_output_buffer<double2>;
template class cuda_output_buffer<double3>;
template class cuda_output_buffer<double4>;
