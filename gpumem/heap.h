#pragma once

// C++ STL
#include <memory>

// CGV framework
#include <cgv_gl/gl/gl.h>

// local includes
#include "gpumem/buddy_alloc.h"
#include "gpumem/pool_alloc.h"
#include "gpumem/span.h"
#include "util.h"


namespace otv::gpumem {

/// Uses a single persistently mapped GL buffer as a `std::pmr::memory_resource`.
class heap final : public std::pmr::memory_resource {
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
	/// Create an instance that manages a buffer of `buffer_size` bytes.
	[[nodiscard]] heap(size_t buffer_size, sync_mode);

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

	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_allocate.html.
	[[nodiscard]] auto do_allocate (size_t num_bytes, size_t align) -> void* final
	{
		return _pool_alloc.allocate(num_bytes, align);
	}
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_deallocate.html.
	void do_deallocate (void* ptr, size_t num_bytes, size_t align) noexcept final
	{
		return _pool_alloc.deallocate(ptr, num_bytes, align);
	}
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_is_equal.html.
	[[nodiscard]] auto do_is_equal (std::pmr::memory_resource const& other) const noexcept
		-> bool final
	{
		return _pool_alloc.is_equal(other);
	}

};

} // namespace otv::gpumem
