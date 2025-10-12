// C++ STL
#include <bit>
#include <cassert>
#include <iostream>
#include <print>

// implemented header
#include "pmr/pool_alloc.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[pool_alloc]\x1b[m"


namespace otv::pmr {

namespace {

/// Controls the amount of debug messages produced by the allocator.
/// Ranges from 0 (no output) to 3 (full output).
constexpr auto log_level = 1u;

/// Print a message to `std::clog` using a C++ 20 format string.
/// Does not append a newline.
template <class... Args>
void log(std::format_string<Args...> fmt, Args&&... args)
{
	std::print(std::clog, fmt, std::forward<Args>(args)...);
}

/// Calculate the alignment used for blocks of a given order.
[[nodiscard]] constexpr auto block_align (uint8_t order) noexcept -> size_t
{
	// Blocks are aligned no more than the size of each block or the maximum alignment of any type.
	return std::min(1uz << order, alignof(std::max_align_t));
}

} // namespace


pool_alloc::pool_alloc (std::pmr::memory_resource* parent, size_t chunk_size, uint8_t min_order)
	: _parent {parent}
	// Blocks must be large enough to store a free-list pointer.
	, _min_order {std::max<uint8_t>(min_order, std::bit_width(sizeof(void*) - 1)/*ceil(log2)*/)}
	, _max_order {std::max<uint8_t>(std::bit_width(chunk_size) - 1/*floor(log2)*/, 0)}
{
	// Round down chunk size to the next multiple of the smallest block.
	_chunk_size = (chunk_size >> _min_order) << _min_order;

	if constexpr (log_level > 0)
		log(LOG_TAG" Create instance.\n"
			"\tChunk size: {} bytes\n"
			"\tMin block:  {} bytes\n"
			"\tMax block:  {} bytes\n",
			_chunk_size, 1 << _min_order, 1 << (_max_order - 1)
		);

	// Chunks must be large enough to store at least two blocks of the smallest size.
	assert(_min_order < _max_order);

	// Create empty pool lists, since no chunks have been allocated.
	_pools = std::make_unique<std::vector<chunk>[]>(_max_order - _min_order);
}

auto pool_alloc::operator= (pool_alloc&& src) noexcept -> pool_alloc&
{
	// Self-assignment is a NOP.
	if (&src == this) return *this;

	// Clean up the current state.
	clean_up();

	// Move members.
	_parent     = src._parent;
	_pools      = std::move(src._pools);
	_chunk_size = src._chunk_size;
	_min_order  = src._min_order;
	_max_order  = src._max_order;
	return *this;
}

void pool_alloc::leak ()
{
	if (log_level > 0) log(LOG_TAG" Leak all chunks.\n");
	for (auto* pool = &_pools[0]; pool < &_pools[_max_order - _min_order]; ++pool) pool->clear();
}

auto pool_alloc::allocated_bytes () const -> size_t
{
	if (!_pools) return 0;

	auto num_bytes = 0uz;
	for (auto order = _min_order; order < _max_order; ++order)
		for (auto const& chunk : _pools[order - _min_order])
			num_bytes += _chunk_size - chunk.capacity * (1uz << order);
	return num_bytes;
}

auto pool_alloc::do_allocate (size_t num_bytes, size_t align) -> void*
{
	// Allocator must be initialized and not moved from.
	assert(_pools);

	// Round up the requested allocation to the next block size.
	auto const order = required_order(num_bytes);

	if constexpr (log_level > 1)
		log(LOG_TAG" Allocating {} bytes in a block of order {}.\n", num_bytes, order);

	// Requests for more than half a chunk are forwarded to the parent allocator.
	if (order >= _max_order) {
		if constexpr (log_level > 2) log("Delegated to parent.\n");
		return _parent->allocate(num_bytes, align);
	}

	// Vector of chunks used for blocks of the required order.
	auto& chunks = _pools[order - _min_order];
	// Search for a chunk with a free block.
	auto chunk = std::find_if(chunks.begin(), chunks.end(), [](auto const& chunk) {
		return chunk.capacity > 0;
	});

	// If all chunks are full, allocate a new one.
	if (chunk == chunks.end()) {
		auto const num_blocks = _chunk_size >> order;

		if constexpr (log_level > 2)
			log("Allocating a new chunk of {} blocks.\n", num_blocks);

		// Chunks have the same alignment as the blocks the contain.
		auto const chunk_mem = static_cast<void**>(_parent->allocate(
			_chunk_size,
			block_align(order)
		));
		// Initially, all blocks are free and the free list starts with the first block.
		chunks.push_back({reinterpret_cast<std::byte*>(chunk_mem), chunk_mem, num_blocks});
		chunk = chunks.end() - 1;

		// Add all blocks to the free list by having each point to its successor.
		auto next_ptr = chunk_mem;
		/// Number of pointers that fit into each block.
		auto const stride     = (1 << order) / sizeof(void*);
		auto const last_block = chunk_mem + (num_blocks - 1)*stride;

		for (; next_ptr < last_block; next_ptr += stride) {
			*next_ptr = next_ptr + stride;
		}

		// The last block has no successor.
		*last_block = nullptr;
	}

	// Allocate the block at the head of the free list.
	auto const allocation = chunk->next_free;
	// Check alignment.
	if (reinterpret_cast<uintptr_t>(allocation) & (align - 1)) throw std::bad_alloc{};
	// Remove the head from the free list.
	chunk->next_free = static_cast<void**>(*allocation);
	// Update the number of free blocks.
	--chunk->capacity;

	if (log_level > 2) {
		// Calculate the index of a block in the current chunk to its index.
		auto const block_idx = [&](void* block) {
			return (reinterpret_cast<std::byte*>(block) - chunk->data) >> order;
		};

		log(LOG_TAG" Allocated block {} at {} in chunk {}."
			"\nBlocks remaining: {}"
			"\nNext free block:  {}\n",
			block_idx(allocation), static_cast<void*>(allocation), chunk - chunks.begin(),
			chunk->capacity,
			block_idx(chunk->next_free)
		);
	}
	return allocation;
}

void pool_alloc::do_deallocate (void* ptr, size_t num_bytes, size_t align) noexcept
{
	// Determine the order of the allocated block.
	auto const order = required_order(num_bytes);

	if constexpr (log_level > 1)
		log(LOG_TAG" Free {} bytes at {}.\n", num_bytes, ptr);

	// Large allocations are handled by the parent.
	if (order >= _max_order) {
		if constexpr (log_level > 2) log("Delegated to parent.\n");
		return _parent->deallocate(ptr, num_bytes, align);
	}

	// Chunks used for blocks of the required order.
	auto& chunks = _pools[order - _min_order];
	// Find the chunk that contains the allocation being freed.
	auto chunk = std::find_if(chunks.begin(), chunks.end(), [&](auto const& chunk) {
		auto const end = chunk.data + _chunk_size;
		return chunk.data <= ptr && ptr < end;
	});
	// If the allocation does not lie within any chunk, something has gone wrong.
	assert(chunk != chunks.end());

	// Update the number of free blocks in the chunk.
	++chunk->capacity;

	// Calculate the index of a block in the current chunk to its index for debug output.
	auto const block_idx = [&](void* block) {
		return (reinterpret_cast<std::byte*>(block) - chunk->data) >> order;
	};

	if (log_level > 2)
		log("Returned block {} to chunk {}, which now has {} free blocks.\n",
			block_idx(ptr), chunk - chunks.begin(), chunk->capacity
		);

	// If all blocks in the chunk are free, return it to the parent allocator.
	if (chunk->capacity == _chunk_size >> order) {
		if (log_level > 2)
			log("Free empty chunk.\n");

		// Deallocate the chunk's memory.
		_parent->deallocate(chunk->data, _chunk_size, block_align(order));
		// Remove the chunk from the pool.
		chunks.erase(chunk);
		return;
	}

	// Otherwise push the newly freed block to the free list.
	auto const next_free = static_cast<void**>(ptr);
	// Point to the previous head.
	*next_free = chunk->next_free;
	// Replace the head pointer.
	chunk->next_free = next_free;

	if (log_level > 2)
		log("Next free block: {}.\n", (static_cast<std::byte*>(*next_free) - chunk->data) >> order);
}

constexpr auto pool_alloc::required_order (size_t num_bytes) const noexcept -> uint8_t
{
	assert(num_bytes > 0);
	return std::max<uint8_t>(std::bit_width(num_bytes - 1)/*ceil(log2)*/, _min_order);
}

void pool_alloc::clean_up () noexcept
{
#ifndef NDEBUG
	if (!_pools) return;

	// Check if there are any allocations that have not been freed
	// Remaining chunks are not freed; this leaks memory but avoids potential crashes from freeing
	// objects still in use.
	if (auto const allocated_bytes = this->allocated_bytes(); allocated_bytes > 0)
		log("\x1b[1;31m[error]\x1b[m" LOG_TAG" {} bytes leaked.\n", allocated_bytes);
#endif
}

} // namespace otv::pmr
