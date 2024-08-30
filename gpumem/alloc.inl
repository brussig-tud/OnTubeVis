#pragma once
#include "alloc.h"

#include "span.inl"


namespace otv::gpumem {

template <class Elem>
span<Elem> buffer_alloc::alloc (size_type length, size_type alignment)
{
	// Calculate the required amount of memory in bytes.
	// Add padding to ensure that alignment restrictions can be met.
	size_type num_bytes {length * memsize<Elem>};

	GLint min_alignment;
	glGetIntegerv(GL_MIN_MAP_BUFFER_ALIGNMENT, &min_alignment);

	if (alignment > min_alignment) {
		num_bytes += (alignment > min_alignment ? alignment - min_alignment : 0);
	}

	// Create buffer object.
	handle_type handle;
	glGenBuffers(1, &handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, handle);

	// Allocate GPU-accessible storage and persistently map it to the CPU's address space without
	// implicit synchronization.
	const auto flags {GL_MAP_WRITE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT};
	glBufferStorage(GL_COPY_WRITE_BUFFER, num_bytes, nullptr, flags);

	auto *const data {reinterpret_cast<std::byte*>(glMapBufferRange(
			GL_COPY_WRITE_BUFFER, 0, num_bytes, flags | GL_MAP_FLUSH_EXPLICIT_BIT))};

	// Check for errors and construct an aligned span over the buffer's memory.
	if (check_gl_errors("gpumem::buffer_alloc::alloc")) {
		return span{data, num_bytes, handle}.aligned_to(alignment).reinterpret_as<Elem>();
	} else {
		glDeleteBuffers(1, &handle);
		throw std::bad_alloc();
	}
}

template <class Elem>
void buffer_alloc::dealloc (span<Elem> memory) noexcept
{
	const auto handle {memory.handle()};
	glDeleteBuffers(1, &handle);
	check_gl_errors("gpumem::buffer_alloc::dealloc");
}

} // namespace otv::gpumem
