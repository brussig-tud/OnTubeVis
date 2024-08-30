#pragma once

#include <cassert>

// CGV framework
#include <cgv_gl/gl/gl.h>


/// Provides containers and allocators using persistently mapped OpenGL buffers that can be accessed
/// by both CPU and GPU.
/// Unless stated otherwise, users are responsible for ensuring synchronization between CPU and GPU
/// as well as between threads.
namespace otv::gpumem {

/// Represents the size of and offset into an OpenGL buffer's memory in bytes.
using size_type = GLsizeiptr;
/// Represents the difference in bytes between offsets into an OpenGL buffer's memory.
using index_type = GLintptr;
/// Type used identify OpenGL buffer objects.
using handle_type = GLuint;


/// Cast `sizeof` for use with OpenGL buffers.
template <typename T>
constexpr size_type memsize {static_cast<size_type>(sizeof(T))};

} // namespace otv::gpumem
