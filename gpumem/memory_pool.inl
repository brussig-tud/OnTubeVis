#pragma once
#include "memory_pool.h"

#include <stdexcept>

#include "alloc.inl"


namespace otv::gpumem {

template <class Elem>
span<Elem> memory_pool_alloc::alloc (size_type length, size_type alignment)
{
	// Get a free block.
	if (_free_blocks.empty()) {
		throw std::runtime_error("[gpumem::memory_pool_alloc::alloc] No free blocks available.");
	}

	// Align and check size.
	const auto memory {
			span(_free_blocks.back(), _block_length, _memory.handle()).aligned_to(alignment)};

	if (memory.length() < length) {
		throw std::runtime_error("[gpumem::memory_pool_alloc::alloc] Requested allocation does not fit into block.");
	}

	// Mark the block as occupied.
	_free_blocks.pop_back();
	return memory.reinterpret_as<Elem>();
}

template <class Elem>
void memory_pool_alloc::dealloc (span<Elem> memory) noexcept
{
	assert(_memory.as_range().contains(memory.bytes().data()));

	// Calculate the offset of the containing block in case the allocation had to be offset to meet
	// a certain alignment.
	auto block_idx = (memory.bytes().data() - _memory.data()) / _block_length;
	_free_blocks.push_back(_memory.data() + block_idx * _block_length);
}



template <class Alloc>
constexpr memory_pool<Alloc>::memory_pool(memory_pool &&src) noexcept
	: alloc_type{std::move(src)}
	, memory_pool_alloc{std::move(src)}
{
	src._memory = {};
}

template <class Alloc>
auto memory_pool<Alloc>::operator= (memory_pool &&src) noexcept -> memory_pool&
{
	if (&src != this) {
		// Free current memory.
		this->~memory_pool();
		this->allocator()                      = {std::move(src)};
		static_cast<memory_pool_alloc&>(*this) = {std::move(src)};
		src._memory                            = {};
	}

	return *this;
}

template <class Alloc>
memory_pool<Alloc>::~memory_pool() noexcept
{
	if (this->as_span().data()) {
		this->allocator().dealloc(this->_memory);
		this->_memory = {};
	}
}

template <class Alloc>
bool memory_pool<Alloc>::create (size_type num_blocks, size_type block_length, size_type alignment)
{
	// For all blocks to be aligned, the block size must be a multiple of the alignment.
	// The formula for rounding to the next multiple of a power of two is adopted from the GLIBCXX
	// implementation of `std::align`.
	block_length = (block_length - 1 + alignment) & -alignment;

	const auto memory {
			this->allocator().template alloc(num_blocks * block_length, alignment)};
	return memory.data() ? memory_pool_alloc::create(num_blocks, block_length, memory) : false;
}

} // namespace otv::gpumem
