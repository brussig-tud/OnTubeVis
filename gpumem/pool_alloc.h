#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <memory_resource>
#include <vector>


namespace otv::gpumem {

/// A segmented storage fixed-size block allocator that adapts a coarse parent allocator to
/// efficiently provide many smaller allocations by using chunks of memory obtained from the parent
/// as pools of smaller, individually managed blocks.
/// Larger allocation requests are delegeted directly to the parent.
/// The size of each individually allocated block must be a power of two.
class pool_alloc : public std::pmr::memory_resource {
public:
	/// A default-constructed instance has no parent and cannot perform any allocations.
	pool_alloc() = default;
	/// Create a pool allocator that obtains chunks no larger than `chunk_size` bytes from `parent`
	/// and divides them into blocks no smaller than 2 ^ `min_order` bytes.
	[[nodiscard]] pool_alloc(
		std::pmr::memory_resource& parent,
		size_t  chunk_size,
		uint8_t min_order
	);

	// Prevent copying.
	pool_alloc(pool_alloc const&) = delete;
	auto operator= (pool_alloc const&) -> pool_alloc& = delete;

	// Allow moving.
	pool_alloc(pool_alloc&&) = default;
	auto operator= (pool_alloc&&) -> pool_alloc& = default;

private:
	/// A chunk of memory obtained from the parent allocator, subdivided into smaller blocks.
	struct chunk {
		/// Beginning address of the chunk.
		std::byte* data {};
		/// Linked list of free blocks, with the next pointers stored in the blocks themselves.
		/// Only valid when `capacity` > 0.
		void** next_free {};
		/// Number of free blocks in this chunk.
		size_t capacity {};
	};

	/// Pointer to the parent allocator providing chunks and larger allocations.
	std::pmr::memory_resource* _parent {};
	/// List of allocated chunks/pools for blocks of every order available.
	/// Index i stores all pools with block size 2^(i + `_min_order`) bytes.
	std::unique_ptr<std::vector<chunk>[]> _pools {};
	/// Blocks are allocated in chunks of this number of bytes.
	size_t _chunk_size {};
	/// The smallest block that can be individually allocated has a size of 2 ^ `_min_order` bytes.
	uint8_t _min_order {};
	/// Requests for more than 2 ^ `_max_order` bytes are forwarded to the parent allocator.
	uint8_t _max_order {};

	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_allocate.html.
	[[nodiscard]] auto do_allocate (size_t bytes, size_t align) -> void* override;
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_deallocate.html.
	void do_deallocate (void*, size_t bytes, size_t align) noexcept override;
	/// See https://en.cppreference.com/w/cpp/memory/memory_resource/do_is_equal.html.
	[[nodiscard]] auto do_is_equal (std::pmr::memory_resource const& other) const noexcept
		-> bool override
	{
		// Allocations can only be freed by the instance that created them.
		return &other == this;
	}

	/// Calculate the order of the smallest block that contains the requested number of bytes.
	/// `num_bytes` must not be 0.
	/// A block of order n contains 2^n bytes.
	[[nodiscard]] constexpr auto required_order (size_t num_bytes) const noexcept -> uint8_t;
};

} // namespace otv::gpumem
