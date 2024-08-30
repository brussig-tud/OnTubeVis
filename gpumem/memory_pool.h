#pragma once

#include <vector>

#include "alloc.h"


namespace otv::gpumem {
/// Divides a span of memory into blocks of equal length which can be individually allocated.
/// Backing memory must be provided by derived classes, see `memory_pool`.
class memory_pool_alloc {
private:
	/// Start of each unallocated block.
	std::vector<std::byte*> _free_blocks {};
	/// Length of each block in bytes.
	size_type _block_length {0};

protected:
	/// The entire backing memory managed by this allocator.
	/// Must be allocated and freed by derived classes.
	span<std::byte> _memory {};

	/// Construct a null instance with no backing memory.
	[[nodiscard]] memory_pool_alloc() = default;

	/// Initialize this object with new backing memory.
	/// All blocks will be free.
	/// Any previous state is overwritten, attempting to free old allocations is an error.
	[[nodiscard]] bool create (
		size_type       num_blocks,
		size_type       block_length,
		span<std::byte> memory
	);

public:
	/// Return the `memory_pool_alloc` subobject of `*this`.
	[[nodiscard]] memory_pool_alloc &allocator () noexcept
	{
		return *this;
	}

	/// The length of each block in bytes.
	[[nodiscard]] constexpr size_type block_length () const noexcept
	{
		return _block_length;
	}

	/// The entire backing memory managed by this object.
	[[nodiscard]] constexpr const span<std::byte> &as_span () const noexcept
	{
		return _memory.as_span();
	}

	/// In a free block, create an allocation that holds at least `length` objects of type `Elem`
	/// and is aligned to `alignment` bytes, which must be a power of two.
	/// Always uses exactly one block, even if less memory is requested.
	/// Fails if the requested allocation does not fit into a block or all blocks are occupied.
	template <class Elem = std::byte>
	[[nodiscard]] span<Elem> alloc (size_type length, size_type alignment = alignof(Elem));

	/// Free an allocated block.
	/// `memory` must be an allocation produced by this object that has not been freed yet.
	template <class Elem>
	void dealloc (span<Elem> memory) noexcept;
};

/// Divides a span of memory into blocks of equal length which can be individually allocated.
/// This class only provides the backing memory, for managing the blocks see `memory_pool_alloc`.
template <class Alloc = buffer_alloc>
class memory_pool : protected Alloc, public memory_pool_alloc {
public:
	/// Allocator type used to manage the backing memory, in which blocks are placed, as a whole.
	/// Not to be confused with the block allocator itself, `memory_pool_alloc`.
	using alloc_type = Alloc;

public:
	/// Construct a null instance with no backing buffer, but an allocator that can be used to
	/// obtain backing memory in the future.
	[[nodiscard]] explicit memory_pool(const alloc_type &allocator = {})
		: alloc_type{allocator}
	{}

	memory_pool(const memory_pool &src) = delete;
	[[nodiscard]] constexpr memory_pool(memory_pool &&src) noexcept;

	memory_pool &operator= (const memory_pool &src) = delete;
	memory_pool &operator= (memory_pool &&src) noexcept;

	~memory_pool() noexcept;

	// Disambiguate between `memory_pool_alloc` and `alloc_type`.
	using alloc_type::allocator;
	using memory_pool_alloc::alloc;
	using memory_pool_alloc::dealloc;

	/// Allocate new backing memory and subdivide it into blocks of at least `block_length` bytes
	/// aligned to `block_alignment` bytes, which must be a power of two.
	/// See also `memory_pool_alloc::create`.
	[[nodiscard]] bool create (
		size_type num_blocks,
		size_type block_length,
		size_type block_alignment
	);
};

/// Allocates memory from a pool shared with others.
using memory_pool_ptr = alloc_ptr<memory_pool_alloc>;

} // namespace otv::gpumem
