// local includes
#include <gl_util.h>

// implemented header
#include "gpumem/heap_buffer.h"


namespace otv::gpumem {

namespace {

/// Create a persistently mapped GL buffer object.
[[nodiscard]] auto create_buffer (size_type num_bytes, gpumem::sync_mode sync)
	-> span<std::byte>
{
	handle_type handle;

	// Create buffer.
	glGenBuffers(1, &handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, handle);

	// Allocate buffer.
	auto const flags = GL_MAP_READ_BIT | GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT
		| (sync == sync_mode::coherent ? GL_MAP_COHERENT_BIT : 0);
	glBufferStorage(GL_COPY_WRITE_BUFFER, num_bytes, nullptr, flags);

	// Map buffer for host access.
	auto const mapping = static_cast<std::byte*>(glMapBufferRange(
		GL_COPY_WRITE_BUFFER,
		0,
		num_bytes,
		flags | (sync == sync_mode::flush_ranges ? GL_MAP_FLUSH_EXPLICIT_BIT : 0)
	));

	// Ensure that everything was successful.
	check_gl_errors("gpumem::heap::heap");
	if (!mapping)
		throw std::runtime_error{std::format(
			"Failed to allocate gpumem::heap of {} bytes.",
			num_bytes
		)};

	return {mapping, num_bytes, handle};
}

} // namespace


heap_buffer::heap_buffer(size_type num_bytes, sync_mode sync, size_t granularity)
	: heap_buffer{create_buffer(num_bytes, sync), granularity}
{}

heap_buffer::heap_buffer(gpumem::span<std::byte> buffer, size_t granularity)
	: heap{buffer.to_std(), granularity}
	, _buffer {buffer.handle()}
{}

void heap_buffer::free_buffer (handle_type handle) noexcept
{
	if (handle == 0) return;

	glDeleteBuffers(1, &handle);
	check_gl_errors("gpumem::heap_buffer::free");
}

} // namespace otv::gpumem
