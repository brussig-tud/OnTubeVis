#pragma once

// C++ STL
#include <memory>

// CGV framework
#include <cgv_gl/gl/gl.h>

// local includes
#include "buddy_alloc.h"
#include "pool_alloc.h"
#include "span.h"
#include <util.h>


namespace otv::gpumem {

/// Adapts a persistently mapped GL buffer as a `std::pmr::memory_resource` that can be used as a
/// memory pool for allocators.
class heap {
public:
	/// Specifies how writes to the buffer by either host or GPU are made visible to the other.
	/// Determines with which flags the buffer is mapped.
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

	/// Create an instance with no backing buffer that cannot perform any allocations.
	[[nodiscard]] heap() = default;
	/// Create an instance that manages `buffer_size` bytes at a granularity of 2 ^ `min_order`
	/// byte blocks, allocated in chunks of 2 ^ `chunk_order` bytes.
	[[nodiscard]] heap(size_t buffer_size, uint8_t chunk_order, uint8_t min_order, sync_mode);

	/// Allow implicit use as a memory resource.
	[[nodiscard]] constexpr operator std::pmr::memory_resource& () noexcept
	{
		return as_memory_resource();
	}
	/// Allow implicit use as a memory resource.
	[[nodiscard]] constexpr operator std::pmr::memory_resource const& () const noexcept
	{
		return as_memory_resource();
	}

	/// Provide the interface for allocations.
	[[nodiscard]] constexpr auto as_memory_resource () noexcept -> std::pmr::memory_resource&
	{
		return _pool_alloc;
	}
	/// Provide the interface for allocations.
	[[nodiscard]] constexpr auto as_memory_resource () const noexcept
		-> std::pmr::memory_resource const&
	{
		return _pool_alloc;
	}

	/// The mapped buffer memory managed by this instance.
	[[nodiscard]] constexpr auto as_span () const noexcept -> span<std::byte>
	{
		if (!_buddy_alloc) return {};
		return {_buddy_alloc->data(), _buffer_size, _buffer.handle};
	}

private:
	/// Destroy a GL buffer object.
	/// Used to clean up `_buffer` in its destructor.
	static void free_buffer (GLuint handle);

	/// Coarse memory management for larger allocations.
	/// Boxed so its address remains stable for `_pool_alloc` to point to.
	std::unique_ptr<buddy_alloc> _buddy_alloc {};
	/// Granular memory management for smaller allocations.
	/// Provides the heap's primary interface in `as_memory_resource`.
	pool_alloc _pool_alloc {};
	/// Size of the managed buffer in bytes.
	gpumem::size_type _buffer_size {};
	/// Persistently mapped buffer containing the managed memory.
	RAII<GLuint, free_buffer> _buffer {0};
};

} // namespace otv::gpumem
