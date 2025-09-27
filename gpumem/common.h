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

/// Specifies how writes to a persistently mapped buffer by either host or GPU are made visible to
/// the other, depending on the flags that the buffer is mapped with.
enum class sync_mode : uint8_t {
	/// Writes automatically become visible at some point.
	/// `GL_MAP_COHERENT_BIT` is set.
	coherent,
	/// All writes are made visible by calling `glMemoryBarrier`.
	/// Neither `GL_MAP_COHERENT_BIT` nor `GL_MAP_FLUSH_EXPLICIT_BIT` is set.
	flush_all,
	/// Only writes in ranges indicated with `glFlushMappedBufferRange` become visible.
	/// `GL_MAP_FLUSH_EXPLICIT_BIT` is set.
	flush_ranges,
};


/// Cast `sizeof` for use with OpenGL buffers.
template <typename T>
constexpr size_type memsize {static_cast<size_type>(sizeof(T))};

} // namespace otv::gpumem
