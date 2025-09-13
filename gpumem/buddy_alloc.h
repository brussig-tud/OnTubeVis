#pragma once

// C++ STL
#include <cstdint>
#include <memory>
#include <memory_resource>
#include <span>


namespace otv::gpumem {

/// Allocates from a span of memory using the buddy system due to Knowlton 1965.
/// Bookkeeping is done outside the managed span using the default allocator.
/// NOTE: The buddy allocator does not own, and therefore does not free, the memory it manages.
class buddy_alloc final : public std::pmr::memory_resource {
private:
	/// Start of the memory range managed by the allocator.
	std::byte* _memory {};
#ifndef NDEBUG
	/// Combined size in bytes of all blocks currently in use.
	/// Only used for a sanity check.
	size_t _allocated_bytes {};
#endif
	/// Full and complete binary tree storing the order of the largest free block in each subtree.
	/// The children of block n are found at indices 2n and 2n + 1, with the root at index 1.
	/// Order 0 indicates a full block, order 1 a free block of size `_base_order` etc.
	std::unique_ptr<uint8_t[]> _capacity {};
	/// The size of the smallest block (order 1) is 2 ^ `_base_order` bytes.
	uint8_t _base_order {};
	/// Order of the root block containing all other blocks.
	/// The root block may be larger than the managed memory range.
	uint8_t _max_order {};

public:
	/// Create a buddy allocator that manages no memory.
	[[nodiscard]] buddy_alloc() = default;
	/// Create a buddy allocator that manages the given memory in blocks of 2^n bytes, where
	/// n >= `base_order`.
	[[nodiscard]] buddy_alloc(std::span<std::byte> memory, uint8_t base_order);

	// Prevent copying.
	buddy_alloc(buddy_alloc const&) = delete;
	auto operator= (buddy_alloc const&) -> buddy_alloc& = delete;

	// Allow moving.
	buddy_alloc(buddy_alloc&&) = default;
	auto operator= (buddy_alloc&&) noexcept -> buddy_alloc&;

	~buddy_alloc() noexcept
	{
		clean_up();
	}

	/// Start of the memory range managed by this instance.
	[[nodiscard]] constexpr auto data () const noexcept -> std::byte*
	{
		return _memory;
	}

private:
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_allocate.html.
	[[nodiscard]] auto do_allocate (size_t num_bytes, size_t align) -> void* final;
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_deallocate.html.
	void do_deallocate (void*, size_t num_bytes, size_t align) noexcept final;
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_is_equal.html.
	[[nodiscard]] auto do_is_equal (std::pmr::memory_resource const& other) const noexcept
		-> bool final
	{
		// Allocations can only be freed by the instance that created them.
		return &other == this;
	}

	/// Calculate the order of the smallest block that contains the requested number of bytes.
	/// `num_bytes` must not be 0.
	/// NOTE: The smallest blocks have order 1, so order 0 can be used to mark unavailable blocks.
	[[nodiscard]] constexpr auto required_order (size_t num_bytes) const noexcept -> uint8_t;

	/// Called before an instance is destroyed or replaced.
	void clean_up () noexcept;
};

} // namespace otv::gpumem
