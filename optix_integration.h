#pragma once

//////
//
// Includes
//

// C++ STL
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

// OpenGL
#include <GL/glew.h>

// CUDA/OptiX
#include <optix.h>
#include <optix_stack_size.h>
#include <optix_stubs.h>
#include <cuda_runtime.h>

// CGV framework
#include <cgv/render/texture.h>



//////
//
// Macros & constants
//

// provide default CUDA compiler options if the including file doesn't provide its own
#ifndef CUDA_NVRTC_OPTIONS
	/// default CUDA compiler options that will be used when compile_cu2ptx is called without
	/// explicitly specifying them
	#define CUDA_NVRTC_OPTIONS                                                   \
		"-std=c++11", "-arch", "compute_50", "-use_fast_math", "-lineinfo",      \
		"-default-device", "-rdc", "true", "-D__x86_64"
#endif

#define CUDA_CHECK(call)                                                         \
	do                                                                           \
	{                                                                            \
		const cudaError_t error = call;                                          \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA call (" << #call << " ) failed with error: '"     \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
		}                                                                        \
	} while (false)

#define CUDA_CHECK_FAIL(call)                                                    \
	do                                                                           \
	{                                                                            \
		const cudaError_t error = call;                                          \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA call (" << #call << " ) failed with error: '"     \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
		}                                                                        \
	} while (false)

#define CUDA_CHECK_SET(call, success_var)                                        \
	do                                                                           \
	{                                                                            \
		const cudaError_t error = call;                                          \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA call (" << #call << " ) failed with error: '"     \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
		}                                                                        \
	} while (false)

#define CUDA_SYNC_CHECK()                                                        \
	do                                                                           \
	{                                                                            \
		cudaDeviceSynchronize();                                                 \
		const cudaError_t error = cudaGetLastError();                            \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA error on synchronize: '"                          \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
	    }                                                                        \
	} while (false)

#define CUDA_SYNC_CHECK_FAIL()                                                   \
	do                                                                           \
	{                                                                            \
		cudaDeviceSynchronize();                                                 \
		const cudaError_t error = cudaGetLastError();                            \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA error on synchronize: '"                          \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
	    }                                                                        \
	} while (false)

#define CUDA_SYNC_CHECK_SET(success_var)                                         \
	do                                                                           \
	{                                                                            \
		cudaDeviceSynchronize();                                                 \
		cudaError_t error = cudaGetLastError();                                  \
		if (error != cudaSuccess)                                                \
		{                                                                        \
			std::cerr << "CUDA error on synchronize: '"                          \
			          << cudaGetErrorString(error)                               \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
		}                                                                        \
	} while (false)

#define NVRTC_CHECK(call)                                                        \
	do                                                                           \
	{                                                                            \
		const nvrtcResult result = call;                                         \
		if (result != NVRTC_SUCCESS)                                             \
			std::cerr << "CUDA Compiler error: '" << nvrtcGetErrorString(result) \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
	} while(false)

#define NVRTC_CHECK_FAIL(call)                                                   \
	do                                                                           \
	{                                                                            \
		const nvrtcResult result = call;                                         \
		if (result != NVRTC_SUCCESS)                                             \
			std::cerr << "CUDA Compiler error: '" << nvrtcGetErrorString(result) \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
	} while(false)

#define NVRTC_CHECK_SET(call, success_var)                                       \
	do                                                                           \
	{                                                                            \
		const nvrtcResult result = call;                                         \
		if (result != NVRTC_SUCCESS)                                             \
			std::cerr << "CUDA Compiler error: '" << nvrtcGetErrorString(result) \
			          << "' (" __FILE__ << ":" << __LINE__ << ")"                \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
	} while(false)

#define OPTIX_CHECK(call)                                                        \
	do                                                                           \
	{                                                                            \
		OptixResult result = call;                                               \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
		}                                                                        \
	} while (false)

#define OPTIX_CHECK_FAIL(call)                                                   \
	do                                                                           \
	{                                                                            \
		OptixResult result = call;                                               \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
		}                                                                        \
	} while (false)

#define OPTIX_CHECK_SET(call, success_var)                                       \
	do                                                                           \
	{                                                                            \
		OptixResult result = call;                                               \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
		}                                                                        \
	} while (false)

#define OPTIX_CHECK_LOG(call, log, sizeof_log)                                   \
	do                                                                           \
	{                                                                            \
		const size_t sizeof_log_user = sizeof_log;                               \
		const OptixResult result = call;                                         \
		const size_t log_needed_size = sizeof_log;                               \
		sizeof_log = sizeof_log_user; /* hide side-effect to user */             \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")" << std::endl         \
			          << "Log:" << std::endl << log                              \
			          << (log_needed_size>sizeof_log_user ? "<TRUNCATED>" : "" ) \
			          << std::endl << std::endl;                                 \
		}                                                                        \
	} while(false)

#define OPTIX_CHECK_LOG_FAIL(call, log, sizeof_log)                              \
	do                                                                           \
	{                                                                            \
		const size_t sizeof_log_user = sizeof_log;                               \
		const OptixResult result = call;                                         \
		const size_t log_needed_size = sizeof_log;                               \
		sizeof_log = sizeof_log_user; /* hide side-effect to user */             \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")" << std::endl         \
			          << "Log:" << std::endl << log                              \
			          << (log_needed_size>sizeof_log_user ? "<TRUNCATED>" : "" ) \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
		}                                                                        \
	} while(false)

#define OPTIX_CHECK_LOG_SET(call, log, sizeof_log, success_var)                  \
	do                                                                           \
	{                                                                            \
		const size_t sizeof_log_user = sizeof_log;                               \
		const OptixResult result = call;                                         \
		const size_t log_needed_size = sizeof_log;                               \
		sizeof_log = sizeof_log_user; /* hide side-effect to user */             \
		if (result != OPTIX_SUCCESS)                                             \
		{                                                                        \
			std::cerr << "Optix call '" << #call << "' failed ("                 \
			          << __FILE__ << ":" << __LINE__ << ")" << std::endl         \
			          << "Log:" << std::endl << log                              \
			          << (log_needed_size>sizeof_log_user ? "<TRUNCATED>" : "" ) \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
		}                                                                        \
	} while(false)

#define GL_CHECK(call)                                                           \
	do                                                                           \
	{                                                                            \
		call;                                                                    \
		GLenum err = glGetError();                                               \
		if (err != GL_NO_ERROR)                                                  \
		{                                                                        \
			std::cerr << "OpenGL call '" << #call << "' failed: "                \
			          << gl_error_string(err) << " ("                            \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
		}                                                                        \
	} while (false)

#define GL_CHECK_FAIL(call)                                                      \
	do                                                                           \
	{                                                                            \
		call;                                                                    \
		GLenum err = glGetError();                                               \
		if (err != GL_NO_ERROR)                                                  \
		{                                                                        \
			std::cerr << "OpenGL call '" << #call << "' failed: "                \
			          << gl_error_string(err) << " ("                            \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
			return false;                                                        \
		}                                                                        \
	} while (false)

#define GL_CHECK_SET(call, success_var)                                          \
	do                                                                           \
	{                                                                            \
		call;                                                                    \
		GLenum err = glGetError();                                               \
		if (err != GL_NO_ERROR)                                                  \
		{                                                                        \
			std::cerr << "OpenGL call '" << #call << "' failed: "                \
			          << gl_error_string(err) << " ("                            \
			          << __FILE__ << ":" << __LINE__ << ")"                      \
			          << std::endl << std::endl;                                 \
			success_var = false;                                                 \
		}                                                                        \
	} while (false)

#define CUDA_SAFE_FREE(memptr)                                                   \
	if (memptr) {                                                                \
		CUDA_CHECK(cudaFree(reinterpret_cast<void*>(memptr)));                   \
		memptr = 0;                                                              \
	}

#define CUDA_SAFE_DESTROY_STREAM(strm)                                           \
	if (strm) {                                                                  \
		CUDA_CHECK(cudaStreamDestroy(strm));                                     \
		strm = 0;                                                                \
	}

#define OPTIX_SAFE_DESTROY_PIPELINE(pl)                                          \
	if (pl) {                                                                    \
		OPTIX_CHECK(optixPipelineDestroy(pl));                                   \
		pl = nullptr;                                                            \
	}

#define OPTIX_SAFE_DESTROY_PROGGROUP(proggroup)                                  \
	if (proggroup) {                                                             \
		OPTIX_CHECK(optixProgramGroupDestroy(proggroup));                        \
		proggroup = nullptr;                                                     \
	}

#define OPTIX_SAFE_DESTROY_MODULE(mod)                                           \
	if (mod) {                                                                   \
		OPTIX_CHECK(optixModuleDestroy(mod));                                    \
		mod = nullptr;                                                           \
	}

/// resolves to an expression creating an std::string that contains the directory of the current source file
#define CUR_SRC_FILE_DIR flipped_backslashes(std::filesystem::absolute(__FILE__"\\..").string())


///////
//
// Structs & enums
//

/// ToDo: document this
enum class CUDAOutputBufferType
{
	CUDA_DEVICE = 0, // not preferred, typically slower than ZERO_COPY
	GL_INTEROP = 1, // single device only, preferred for single device
	ZERO_COPY = 2, // general case, preferred for multi-gpu if not fully nvlink connected
	CUDA_P2P = 3  // fully connected only, preferred for fully nvlink connected
};

/// shorthand alias for CUDAOutputBufferType
typedef CUDAOutputBufferType CUOutBuf;

/// Shader binding table record template
template <typename T>
struct sbt_record
{
	__align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
	T data;
};



//////
//
// Functions
//

/// returns a version of the given path with all backslashes flipped to forward slashes
inline std::string flipped_backslashes (const std::string &path)
{
	const size_t len = path.length();
	std::string ret; ret.reserve(len);
	for (unsigned i=0; i< len; i++)
	{
		const std::string::value_type &ch = path[i];
		ret.push_back(ch != '\\' ? ch : '/');
	}
	return ret;
}

/// compiles the given .cu file to a device code PTX object
std::vector<char> compile_cu2ptx (
	const std::string &filename,                      // Cuda C input file name
	const std::string &name,                          // arbitrary name the compiled code should be assigned in CUDA
	const std::vector<const char*> &compiler_options, // CUDA compiler options
	std::string *log_out                              // [out] the compilation log
);

/// version of compile_cu2ptx with no log output and default compiler options
inline std::vector<char> compile_cu_to_ptx(
	const std::string &filename,  // Cuda C input file name
	const std::string &name       // arbitrary name the compiled code should be assigned in CUDA
)
{
	return compile_cu2ptx(filename, name, {CUDA_NVRTC_OPTIONS}, nullptr);
}

/// version of compile_cu2ptx with log output and default compiler options
inline std::vector<char> compile_cu2ptx (
	const std::string &filename,  // Cuda C input file name
	const std::string &name,      // arbitrary name the compiled code should be assigned in CUDA
	std::string *log_out          // string that will receive the compilation log
)
{
	return compile_cu2ptx(filename, name, {CUDA_NVRTC_OPTIONS}, log_out);
}

/// version of compile_cu2ptx with custom compiler options and no log output
inline std::vector<char> compile_cu2ptx (
	const std::string &filename,                     // Cuda C input file name
	const std::string &name,                         // arbitrary name the compiled code should be assigned in CUDA
	const std::vector<const char*> &compiler_options // CUDA compiler options
)
{
	return compile_cu2ptx(filename, name, {CUDA_NVRTC_OPTIONS}, nullptr);
}

/// utility for obtaining a human-readable representation of an OpenGL error value
inline const char* gl_error_string (GLenum error)
{
	switch (error)
	{
		case GL_NO_ERROR:          return "No error";
		case GL_INVALID_ENUM:      return "Invalid enum";
		case GL_INVALID_VALUE:     return "Invalid value";
		case GL_INVALID_OPERATION: return "Invalid operation";
		case GL_STACK_OVERFLOW:    return "Stack overflow";
		case GL_STACK_UNDERFLOW:   return "Stack underflow";
		case GL_OUT_OF_MEMORY:     return "Out of memory";
		case GL_TABLE_TOO_LARGE:   return "Table too large";
		default:                   return "Unknown GL error";
	}
}



//////
//
// Classes
//

/// reference the CGV pixel format string corresponding to an element data type supported by cuda_output_buffer
template <class pxl_fmt>
const std::string& ref_cgv_format_string (void);

/// manages an output buffer strategy for retrieving results of CUDA computations (shamelessly stolen
/// from OptiX SDK samples)
template <class pxl_fmt>
class cuda_output_buffer
{
public:
	/// Default-constructed instances will be unusable until reset() is called!
	cuda_output_buffer();

	cuda_output_buffer(CUDAOutputBufferType type, int width, int height);
	~cuda_output_buffer();

	bool reset (CUDAOutputBufferType type, int width, int height);

	void set_device (int device_idx) { m_device_idx = device_idx; }
	void set_stream (CUstream stream) { m_stream = stream; }

	bool resize (int width, int height);

	// Allocate or update device pointer as necessary for CUDA access
	pxl_fmt* map (void);
	void unmap (void);

	int width() const { return m_width; }
	int height() const { return m_height; }

	// Get output buffer
	GLuint   get_pbo (void);
	void     delete_pbo (void);
	pxl_fmt* get_host_ptr (void);

	// transfer output buffer to given texture
	bool into_texture (cgv::render::context &ctx, cgv::render::texture &tex);

	static void ensure_min_size (int &width, int &height) {
		if (width <= 0) width = 1;
		if (height <= 0) height = 1;
	}
	static void ensure_min_size (unsigned &width, unsigned &height) {
		if (width == 0) width = 1;
		if (height == 0) height = 1;
	}

private:
	bool make_current() { CUDA_CHECK_FAIL(cudaSetDevice(m_device_idx)); return true; }

	bool initialized;

	CUDAOutputBufferType m_type;
	int m_width = 0u;
	int m_height = 0u;

	cudaGraphicsResource* m_cuda_gfx_resource = nullptr;
	GLuint m_pbo = 0u;
	pxl_fmt* m_device_pixels = nullptr;
    pxl_fmt* m_host_zcopy_pixels = nullptr;
	std::vector<pxl_fmt>  m_host_pixels;

	CUstream m_stream = nullptr;
	int m_device_idx = 0;
};
