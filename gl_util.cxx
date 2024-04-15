// C++ STL
#include <iostream>

// CGV framework
#include <cgv_gl/gl/gl.h>

// implemented header
#include "gl_util.h"


bool check_gl_errors (std::string_view function) noexcept
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
	return false;
}
