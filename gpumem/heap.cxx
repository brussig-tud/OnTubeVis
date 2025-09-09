// local includes
#include <gl_util.h>

// implemented header
#include "heap.h"


namespace otv::gpumem {

heap::heap(size_t buffer_size, uint8_t chunk_order, uint8_t min_order, sync_mode sync_mode)
	: _buffer_size {static_cast<size_type>(buffer_size)}
{
	// Check parameters.
	assert(min_order <= chunk_order && buffer_size >= 1 << chunk_order);

	// Create buffer.
	glGenBuffers(1, &_buffer.handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, _buffer.handle);

	// Allocate buffer.
	auto const flags = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT
		| (sync_mode == sync_mode::coherent ? GL_MAP_COHERENT_BIT : 0);
	glBufferStorage(GL_COPY_WRITE_BUFFER, buffer_size, nullptr, flags);

	// Map buffer for host access.
	auto const mapping = glMapBufferRange(
		GL_COPY_WRITE_BUFFER,
		0,
		buffer_size,
		flags | (sync_mode == sync_mode::flush_ranges ? GL_MAP_FLUSH_EXPLICIT_BIT : 0)
	);

	// Ensure that everything was successful.
	check_gl_errors("gpumem::heap::heap");
	if (!mapping)
		throw std::runtime_error{std::format(
			"Failed to allocate gpumem::heap of {} bytes.",
			buffer_size
		)};

	// Create allocators.
	_buddy_alloc = std::make_unique<gpumem::buddy_alloc>(
		std::span{static_cast<std::byte*>(mapping), buffer_size}, chunk_order);
	_pool_alloc = {*_buddy_alloc, 1uz << chunk_order, min_order};
}

void heap::free_buffer (GLuint handle)
{
	glDeleteBuffers(1, &handle);
	check_gl_errors("gpumem::heap::free_buffer");
}

} // namespace otv::gpumem
