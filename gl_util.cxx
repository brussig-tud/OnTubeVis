// C++ STL
#include <iostream>
#include <stdexcept>

// CGV framework
#include <cgv_gl/gl/gl.h>

// implemented header
#include "gl_util.h"


bool check_gl_errors (std::string_view function)
	#ifndef _DEBUG
		noexcept
	#endif
{
	using namespace std::literals;


	// In case of no error, exit immediately with no output.
	auto error = glGetError();

	if (!error) {
		return true;
	}

	// Otherwise print source location...
	std::clog << "\x1b[1;31mgl errors\x1b[m in \x1b[1m" << function;

	auto sep = "\x1b[m: "sv;

	// ...and all errors.
	do {
		std::cerr << sep << gluErrorString(error);
		sep = ", "sv;
	} while ((error = glGetError()));

	std::clog << "\n";

	// In debug mode, throw an exception to create a core dump for inspection.
	#ifdef _DEBUG
		throw std::runtime_error("Encountered an OpenGL error, see stderr for details.");
	#endif

	return false;
}
