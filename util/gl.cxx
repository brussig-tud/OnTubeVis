#include <iostream>
#include <stdexcept>

#include "gl.h"


auto gl_errors (std::string_view context, bool throwing) -> GLenum
{
	// If there is no error, exit immediately.
	auto error = glGetError();
	if (!error) return GL_NO_ERROR;

	// Otherwise print context and errors.
	using std::operator""sv;
	std::clog << "\x1b[1;31m[error]\x1b[39m " << context;

	auto sep = "\x1b[m: "sv;
	while (true) {
		std::clog << sep << gl_error_name(error);
		if (auto const next_error = glGetError()) error = next_error; else break;
		sep = ", "sv;
	}
	std::clog << "\n";

	// Report the last error as return value or exception.
	if (!throwing) return error;
	throw std::runtime_error{"Encountered an OpenGL error, see stderr for details."};
}


auto gl_buffer::create () -> gl_buffer
{
	GLuint handle;
	glCreateBuffers(1, &handle);
	if (gl_errors("in gl_buffer::create")) return {};
	return {handle};
}

void gl_buffer::destroy()
{
	glDeleteBuffers(1, &_handle);
	gl_errors("in gl_buffer::~gl_buffer", false);
}

