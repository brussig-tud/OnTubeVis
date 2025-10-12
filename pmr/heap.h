#pragma once

// local includes
#include "pmr/buddy_alloc.h"
#include "pmr/pool_alloc.h"


namespace otv::pmr {

/// Provides allocations from a contiguous region of memory.
class heap : public memory_region {
public:
	/// Create an instance with no backing memory that cannot perform any allocations.
	[[nodiscard]] heap() = default;
	/// Create an instance that provides allocations from the given memory region.
	/// `granularity` is a hint for the size in bytes of the smallest allocation to expect.
	[[nodiscard]] heap(std::span<std::byte>, size_t granularity = 1);

	// Forbid copying.
	heap(heap const&) = delete;
	void operator= (heap const&) = delete;

	// Forbid moving since `_pool_alloc` points to `_buddy_alloc`.
	heap(heap&&) = delete;
	void operator= (heap&&) = delete;

	/// Free all allocations on the heap.
	/// Allocated memory remains valid, but can no longer be freed.
	void clear ();

	/// Count how many bytes of memory are currently allocated from this instance.
	/// May include internal fragmentation, thus exceeding requested allocations.
	[[nodiscard]] virtual auto allocated_bytes () const -> size_t
	{
		return _pool_alloc.allocated_bytes();
	}

private:
	/// Coarse memory management for larger allocations.
	buddy_alloc _buddy_alloc {};
	/// Granular memory management for smaller allocations.
	/// Used to implement `std::pmr::memory_resource`.
	pool_alloc _pool_alloc {};

	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_allocate.html.
	[[nodiscard]] auto do_allocate (size_t num_bytes, size_t align) -> void* override
	{
		return _pool_alloc.allocate(num_bytes, align);
	}
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_deallocate.html.
	void do_deallocate (void* ptr, size_t num_bytes, size_t align) noexcept override
	{
		return _pool_alloc.deallocate(ptr, num_bytes, align);
	}
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_is_equal.html.
	[[nodiscard]] auto do_is_equal (std::pmr::memory_resource const& other) const noexcept
		-> bool override
	{
		return &other == this;
	}
};

} // namespace otv::pmr
