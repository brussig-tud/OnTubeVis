#pragma once

// C++ STL
#include <string_view>


/// Returns `true` if there were no errors, `false` otherwise.
bool check_gl_errors (std::string_view function) noexcept;
