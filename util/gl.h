#pragma once

#include <string_view>

#include <cgv_gl/gl/gl.h>


/// Map an enum value as returned by `glGetError` to its corresponding identifier.
[[nodiscard]] constexpr auto gl_error_name (GLenum error) noexcept -> std::string_view
{
	using std::operator""sv;
	switch (error) {
	case GL_NO_ERROR:                      return "GL_NO_ERROR";
	case GL_INVALID_ENUM:                  return "GL_INVALID_ENUM";
	case GL_INVALID_VALUE:                 return "GL_INVALID_VALUE";
	case GL_INVALID_OPERATION:             return "GL_INVALID_OPERATION";
	case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
	case GL_OUT_OF_MEMORY:                 return "GL_OUT_OF_MEMORY";
	case GL_STACK_UNDERFLOW:               return "GL_STACK_UNDERFLOW";
	case GL_STACK_OVERFLOW:                return "GL_STACK_OVERFLOW";
	default:                               return "unknown GL error";
	}
}

/// Clear all OpenGL error flags, printing any errors they stored to stderr alongside the given
/// `context`. If `throwing` is true and at least one error was encountered, an exception is thrown.
/// Otherwise the function returns any one of the errors, or `GL_NO_ERROR` if there was none.
auto gl_errors (std::string_view context, bool throwing = true) -> GLenum;


/// Nullable RAII wrapper around a GL buffer object.
class gl_buffer {
public:
	/// Construct and initialize a new buffer object using `glCreateBuffers`.
	[[nodiscard]] static auto create () -> gl_buffer;

	/// Take ownership of the buffer object with the given handle.
	[[nodiscard]] constexpr gl_buffer(GLuint handle = 0) noexcept
		: _handle {handle}
	{}

	// Prevent copying.
	gl_buffer(gl_buffer const&) = delete;
	auto operator= (gl_buffer const&) = delete;

	// Allow moving.
	[[nodiscard]] constexpr gl_buffer(gl_buffer&& src) noexcept
		: _handle {src._handle}
	{
		src._handle = 0;
	}
	auto operator= (gl_buffer&& src) noexcept -> gl_buffer&
	{
		if (&src == this) return *this;
		if (_handle) destroy();
		_handle     = src._handle;
		src._handle = 0;
		return *this;
	}

	~gl_buffer() noexcept
	{
		if (_handle) try {destroy();} catch (...) {}
	}

	/// Return the GL object handle for this buffer.
	[[nodiscard]] constexpr auto handle () const noexcept -> GLuint
	{
		return _handle;
	}

private:
	GLuint _handle {};

	void destroy();
};

