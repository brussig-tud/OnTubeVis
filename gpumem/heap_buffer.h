#pragma once

// local includes
#include "gpumem/buffer.h"
#include "gpumem/common.h"
#include "gpumem/span.h"
#include "pmr/heap.h"


namespace otv::gpumem {

/// A persistently mapped buffer used as a memory resource for allocations.
class heap_buffer : public pmr::heap {
public:
	/// Create an instance with no backing buffer that cannot perform any allocations.
	[[nodiscard]] constexpr heap_buffer() noexcept = default;
	/// Create an instance that manages a new buffer object.
	/// `granularity` is a hint for the size in bytes of the smallest allocation to expect.
	[[nodiscard]] heap_buffer(size_type num_bytes, gpumem::sync_mode, size_t granularity = 1);

	/// Handle to the GL buffer object whose memory this instance manages.
	[[nodiscard]] constexpr auto handle () const noexcept -> handle_type
	{
		return _buffer.handle;
	}

private:
	/// Persistently mapped buffer containing the managed memory.
	buffer _buffer {};

	/// Create an instance that owns and manages the given buffer.
	[[nodiscard]] heap_buffer(gpumem::span<std::byte>, size_t granularity);
};

} // namespace otv::gpumem
