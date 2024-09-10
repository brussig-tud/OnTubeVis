#pragma once

// C++ STL
#include <string_view>


/// Clear OpenGL error flags, printing any encountered errors to `std::cerr`.
/// Returns `true` if there were no errors, `false` otherwise.
/// In debug mode, errors trigger an exception.
bool check_gl_errors (std::string_view function)
	#ifndef _DEBUG
		noexcept
	#endif
;
