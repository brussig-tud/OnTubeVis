// C++ STL
#include <bit>
#include <cassert>
#include <iostream>

// implemented header
#include "pmr/buddy_alloc.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[buddy_alloc]\x1b[m"


namespace otv::pmr {

namespace {

/// Controls the amount of debug messages produced by the allocator.
/// Ranges from 0 (no output) to 3 (full output).
constexpr auto log_level = 1u;

/// Stream arguments to `std::clog`.
template<typename... Args>
void log(Args&&... args)
{
	(std::clog << ... << std::forward<Args>(args));
}

} // namespace


buddy_alloc::buddy_alloc(std::span<std::byte> memory, uint8_t base_order)
	: memory_region{memory}
	, _base_order {base_order}
{
	/// Number of 1-blocks that fit within the managed memory.
	auto const min_blocks = memory.size() >> _base_order;

	// The root block contains all memory, rounded up to the next block order.
	_max_order = required_order(memory.size());
	// Allocate binary tree, initializing every node to 0 (block unavailable).
	_capacity = std::make_unique<uint8_t[]>(
		// Perfect binary tree for all levels except the last one, plus an unused node at index 0
		// to simplify traversal with 1-based indexing.
		(1 << (_max_order - 1))
		// 1-blocks on the lowest level, rounded up to an even number so every block has a buddy.
		+ min_blocks + min_blocks%2
	);

	auto const block_size = 1 << _base_order;

	if constexpr (log_level > 0)
		log(LOG_TAG" Create instance.\n"
			"\tRange:       ",_span.data(),"..",&*_span.end()," (",_span.size()," bytes)\n"
			"\tCapacity:    ",block_size * min_blocks," bytes\n"
			"\tGranularity: ",block_size," bytes\n"
			"\tLevels:      ",_max_order,"\n"
		);

	// Initialize blocks as free.
	clear();
}

auto buddy_alloc::operator= (buddy_alloc&& src) noexcept -> buddy_alloc&
{
	// Self-assignment is a NOP.
	if (&src == this) return *this;

	// Clean up the current state.
	clean_up();

	// Move members.
	_span = src._span;
#ifndef NDEBUG
	_allocated_bytes = src._allocated_bytes;
#endif
	_capacity   = std::move(src._capacity);
	_base_order = src._base_order;
	_max_order  = src._max_order;
	return *this;
}

void buddy_alloc::clear ()
{
	if constexpr (log_level > 0) log(LOG_TAG" Free all blocks.\n");

	/// Number of 1-blocks that fit within the managed memory.
	auto const min_blocks = _span.size() >> _base_order;

	// Start at the root block.
	auto first_block = 1;

	for (auto order = _max_order; order > 0; --order) {
		// The number of full blocks is halved with every order as their size doubles.
		auto const full_blocks = min_blocks >> (order - 1);
		// Iff a block is completely free, its capacity equals its order.
		std::fill_n(&_capacity[first_block], full_blocks, order);

		// If the size in bytes of the managed region is not a power of two, the last block of every
		// order may extend beyond it.
		// Their capacity is equal to the largest block that still fits within the memory remaining
		// beyond the last full block.
		auto const rem       = min_blocks - (full_blocks << (order - 1));
		auto const rem_order = std::bit_width(rem); // floor(log2 rem) + 1

		if (rem_order != 0) {
			_capacity[first_block + full_blocks] = rem_order;
		}

		if constexpr (log_level > 2)
			log("\tOrder ",order,": ",full_blocks," blocks, remainder ",rem_order,".\n");

		// The next level starts at the left child of the current layer's first node.
		first_block *= 2;
	}

#ifndef NDEBUG
	_allocated_bytes = 0;
#endif
}

auto buddy_alloc::do_allocate (size_t num_bytes, size_t align) -> void*
{
	// Allocator must be initialized and not moved from.
	assert(_capacity);

	// Round up the requested allocation to the next block size.
	auto const req_order = required_order(num_bytes);

	if constexpr (log_level > 1)
		log(LOG_TAG" Allocating ",num_bytes," bytes in a block of order ",req_order,".\n");

	auto block = size_t{1};
	// The root node stores the order of the largest block currently available.
	// This check is both necessary and sufficient to determine whether the requested allocation is
	// possible (ignoring alignment).
	if (_capacity[block] < req_order) throw std::bad_alloc{};

	// Descend the tree of blocks down to the lowest order large enough to fulfill the request.
	for (auto order = _max_order; order > req_order; --order) {
		if constexpr (log_level > 2)
			log("Order ",order,", block ",block,", capacity ",_capacity[block],".\n");

		auto const lchild = block * 2;

		// Reduce fragmentation by allocating preferably from blocks that have already been split,
		// so long as they are still large enough.
		if (_capacity[lchild] < _capacity[lchild + 1])
			block = lchild + (_capacity[lchild] < req_order);
		else
			block = lchild + (_capacity[lchild + 1] >= req_order);

		// Sanity check.
		assert(_capacity[block] >= req_order);
	}

	// Allocate the block we ended up at.
	auto const allocation = static_cast<void*>(&_span[
		(block - (1 << (_max_order - req_order))) // Block index within level.
		<< (_base_order + req_order - 1) // Times block size in bytes.
	]);
	if constexpr (log_level > 1) log("Allocated block ",block," at ",allocation,".\n");

	// Check alignment.
	if (reinterpret_cast<uintptr_t>(allocation) & (align - 1)) throw std::bad_alloc{};

#ifndef NDEBUG
	// Track memory usage.
	_allocated_bytes += 1 << (_base_order + req_order - 1);
#endif

	// Mark the allocated block as occupied.
	auto new_cap = _capacity[block] = 0;

	// Propagate reduced capacity up the tree.
	for (auto order = req_order; order < _max_order; ++order) {
		auto const parent = block / 2;
		// The largest free block within a subtree is the largest free block within either child.
		// We do not need to consider the possibility of merging empty children, since we know that
		// the block we just allocated from is not empty.
		new_cap = std::max(new_cap, _capacity[block ^ 1]); // Find our buddy by flipping the LSB.
		// If a block does not change, the rest of the tree is unaffected.
		if (_capacity[parent] == new_cap) break;
		// Otherwise store the new capacity and continue up the tree.
		_capacity[parent] = new_cap;
		block = parent;

		if constexpr (log_level > 2)
			log("Order ",order + 1,", block ",block,", capacity ",_capacity[block],".\n");
	}

	return allocation;
}

void buddy_alloc::do_deallocate (void* ptr, size_t num_bytes, size_t align) noexcept
{
	// Determine the order of the allocated block.
	auto order = required_order(num_bytes);

	if constexpr (log_level > 1)
		log(LOG_TAG" Free ",num_bytes," bytes at ",ptr,".\n");

#ifndef NDEBUG
	// Track memory usage.
	_allocated_bytes -= 1 << (_base_order + order - 1);
#endif

	// Calculate which block the allocation corresponds to.
	auto const offset = static_cast<size_t>(static_cast<std::byte*>(ptr) - _span.data());
	auto block        =
		(1 << (_max_order - order)) // Number of higher-order blocks.
		+ (offset >> (_base_order + order - 1)); // Offset within the level.

	if constexpr (log_level > 2)
		log("Allocated in block ",block,", order ",order,".\n");

	// A free block's capacity is equal to its order.
	auto new_cap = _capacity[block] = order;

	// Propagate new capacity up the tree, merging buddies where possible.
	for (; order < _max_order; ++order) {
		auto const buddy_cap = _capacity[block ^ 1];

		// A block's capacity is the maximum of its children's capacities.
		// Therefore, the current block's ancestors only change if it has a larger capacity than
		// its buddy, or both are at full capacity and can be recombined.
		if (buddy_cap >= new_cap) {
			// The current block is still partially occupied, cannot recombine.
			if (new_cap < order) break;
			// Both buddies are completely free, merge them.
			++new_cap;
		}

		// Continue up the tree.
		_capacity[block / 2] = new_cap;
		block /= 2;

		if constexpr (log_level > 2)
			log("Order ",order + 1,", block ",block,", capacity ",new_cap,".\n");
	}
}


constexpr auto buddy_alloc::required_order (size_t num_bytes) const noexcept -> uint8_t
{
	assert(num_bytes > 0);

	// Round up the number of bytes to the next power of two.
	// Divide by the size of each 1-block, take log 2, then add 1 since order 0 is an empty block.
	// In actuality the logarithm is done first together with the rounding, so the division becomes
	// a subtraction.

	auto const order = std::bit_width(num_bytes - 1); // ceil(log2 num_bytes)
	return order <= _base_order ? 1 : order - _base_order + 1;
}

void buddy_alloc::clean_up () noexcept
{
#ifndef NDEBUG
	// Check that all allocations have been freed.
	if (_capacity && _allocated_bytes > 0)
		log("\x1b[1;33m[warning]\x1b[m" LOG_TAG" ",_allocated_bytes," bytes leaked.\n");
#endif
}

} // namespace otv::pmr
