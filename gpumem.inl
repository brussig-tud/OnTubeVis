#pragma once

// C++ STL
#include <cassert>
#include <memory>
#include <stdexcept>

// local includes
#include "gl_util.h"

// implemented header
#include "gpumem.h"


namespace gpumem {

template <class Elem>
span<std::byte> span<Elem>::buffer () const
{
	glBindBuffer(GL_COPY_READ_BUFFER, _handle);

	void *data       {nullptr};
	size_type length {0};

	glGetBufferPointerv(GL_COPY_READ_BUFFER, GL_BUFFER_MAP_POINTER, &data);
	glGetBufferParameteri64v(GL_COPY_READ_BUFFER, GL_BUFFER_MAP_LENGTH, &length);

	return {reinterpret_cast<std::byte*>(data), length, _handle};
}

template <class Elem>
template <class T>
span<T> span<Elem>::reinterpret_as () const noexcept
{
	return {
		reinterpret_cast<T*>(_data),
		_length * memsize<elem_type> / memsize<T>,
		_handle
	};
}

template <class Elem>
auto span<Elem>::aligned_to(size_type alignment) const -> span
{
	// Ensure the requested alignment is valid for the element type.
	assert(alignment >= alignof(elem_type));

	// Determine the largest subspan with the requested alignment.
	void *aligned_ptr {_data};
	auto aligned_size {static_cast<std::size_t>(_length) * sizeof(elem_type)}; // In bytes, not elements!

	if (! std::align(static_cast<std::size_t>(alignment),
	                 sizeof(elem_type),
	                 aligned_ptr,
	                 aligned_size
	                 )) {
		// If the span cannot fit at least one element with the given alignment, return a null span.
		return {};
	}

	// Construct a new span over the aligned memory.
	return {
		reinterpret_cast<elem_type*>(aligned_ptr),
		static_cast<size_type>(aligned_size / sizeof(elem_type)),
		_handle
	};
}

template <class Elem>
bool span<Elem>::flush_wrapping (ro_range<index_type> range) const noexcept
{
	// Nothing to do for an empty range.
	if (range.is_empty()) {
		return true;
	}

	// Bind the Buffer Object and obtain its base address to calculate the span's offset within the
	// buffer.
	glBindBuffer(GL_COPY_READ_BUFFER, _handle);

	void *buffer_data {nullptr};
	glGetBufferPointerv(GL_COPY_READ_BUFFER, GL_BUFFER_MAP_POINTER, &buffer_data);
	index_type base_offset {bytes().data() - reinterpret_cast<std::byte*>(buffer_data)};

	auto [begin, end] = range;

	// Flush the wrap-around, if there is any, then truncate the range to the end of the span.
	if (end < begin) {
		glFlushMappedBufferRange(GL_COPY_READ_BUFFER, base_offset, end * memsize<elem_type>);
		end = _length;
	}

	// Flush the contiguous range.
	glFlushMappedBufferRange(
		GL_COPY_READ_BUFFER,
		base_offset + begin,
		(end - begin) * memsize<elem_type>
	);

	return check_gl_errors("gpumem::span::flush_wrapping");
}


template <class Elem>
span<Elem> buffer_alloc::alloc (size_type length, size_type alignment)
{
	// Calculate the required amount of memory in bytes.
	// Add padding to ensure that alignment restrictions can be met.
	size_type num_bytes {length * memsize<Elem>};

	GLint min_alignment;
	glGetIntegerv(GL_MIN_MAP_BUFFER_ALIGNMENT, &min_alignment);

	if (alignment > min_alignment) {
		num_bytes += (alignment > min_alignment ? alignment - min_alignment : 0);
	}

	// Create buffer object.
	handle_type handle;
	glGenBuffers(1, &handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, handle);

	// Allocate GPU-accessible storage and persistently map it to the CPU's address space without
	// implicit synchronization.
	const auto flags {GL_MAP_WRITE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT};
	glBufferStorage(GL_COPY_WRITE_BUFFER, num_bytes, nullptr, flags);

	auto *const data {reinterpret_cast<std::byte*>(glMapBufferRange(
			GL_COPY_WRITE_BUFFER, 0, num_bytes, flags | GL_MAP_FLUSH_EXPLICIT_BIT))};

	// Check for errors and construct an aligned span over the buffer's memory.
	if (check_gl_errors("gpumem::buffer_alloc::alloc")) {
		return span{data, num_bytes, handle}.aligned_to(alignment).reinterpret_as<Elem>();
	} else {
		glDeleteBuffers(1, &handle);
		throw std::bad_alloc();
	}
}

template <class Elem>
void buffer_alloc::dealloc (span<Elem> memory) noexcept
{
	const auto handle {memory.handle()};
	glDeleteBuffers(1, &handle);
	check_gl_errors("gpumem::buffer_alloc::dealloc");
}


template <class Elem, class Alloc>
constexpr array<Elem, Alloc>::array(array &&src) noexcept
	: alloc_type{std::move(src)}
	, span<Elem>{std::move(src)}
{
	src.as_span() = {};
}

template <class Elem, class Alloc>
auto array<Elem, Alloc>::operator= (array &&src) noexcept -> array&
{
	if (&src != this) {
		// Free current memory.
		this->~array();
		this->allocator() = {std::move(src)};
		this->as_span()   = {std::move(src)};
		src.as_span()     = {};
	}

	return *this;
}

template <class Elem, class Alloc>
array<Elem, Alloc>::~array() noexcept
{
	if (this->as_span().data()) {
		this->dealloc(*this);
		this->as_span() = {};
	}
}

template <class Elem, class Alloc>
bool array<Elem, Alloc>::create (size_type length, size_type alignment)
{
	this->as_span() = this->template alloc<Elem>(length, alignment);
	assert(this->length() == length);
	return this->handle();
}


bool memory_pool_alloc::create (size_type num_blocks, size_type block_length, span<std::byte> memory)
{
	// Set members.
	_block_length = block_length;
	_memory       = memory;

	// Generate blocks.
	_free_blocks.clear();
	_free_blocks.reserve(num_blocks);

	for (auto i {num_blocks - 1}; i != -1; --i) {
		_free_blocks.emplace_back(_memory.data() + i * block_length);
	}

	return true;
}

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
	const auto memory {
			this->allocator().template alloc(num_blocks * block_length, alignment)};
	return memory.data() ? memory_pool_alloc::create(num_blocks, block_length, memory) : false;
}


template <class Elem, class Alloc>
void ring_buffer<Elem, Alloc>::push_back (const elem_type &elem)
{
	auto new_back = index_after(_idcs.back);

	// Cannot add a new element if the buffer is full.
	if (new_back == _idcs.gpu_front) {
		throw std::runtime_error("[gpumem::ring_buffer::push_back] The buffer is full.");
	}

	// Otherwise store the element and advance the back index.
	_memory[_idcs.back] = elem;
	_idcs.back          = new_back;
}

template <class Elem, class Alloc>
template <class Iter>
void ring_buffer<Elem, Alloc>::push_back (ro_range<Iter> elems)
{
	auto &[begin, end] = elems;
	auto new_back      = _idcs.back + (end - begin);

	// If the used part of the buffer is contiguous in memory, the unused part wraps around.
	if (_idcs.gpu_front <= _idcs.back) {
		// If there is enough room at the end of the backing memory, a single copy is enough.
		if (new_back < _memory.length()) {
			goto copy;
		}

		// Otherwise the back pointer wraps around.
		new_back -= _memory.length();

		// Don't add any elements unless there is room for all of them.
		if (new_back >= _idcs.gpu_front) {
			throw std::runtime_error("[gpumem::ring_buffer::push_back] The buffer has insufficient capacity.");
		}

		// Fill the back part of the allocation in a first copy...
		auto split_point = begin + (_memory.length() - _idcs.back);
		std::copy(begin, split_point, &_memory[_idcs.back]);

		// ...and the remaining data is added at the front.
		begin     = split_point;
		_idcs.back = 0;
	} else {
		// Don't add any elements unless there is room for all of them.
		if (new_back >= _idcs.gpu_front) {
			throw std::runtime_error("[gpumem::ring_buffer::push_back] The buffer has insufficient capacity.");
		}
	}

	// At this point, the unused part of the allocation is contiguous in memory.

	// Copy the new elements and update the back index.
	copy:
	std::copy(begin, end, &_memory[_idcs.back]);
	_idcs.back = new_back;
}

template <class Elem, class Alloc>
void ring_buffer<Elem, Alloc>::pop_front () noexcept
{
	// Sanity check.
	assert(_idcs.front != _idcs.back);

	auto new_front = index_after(_idcs.front);

	// Ensure that `gpu_back` remains in the used range of the allocation so that no unused memory
	// is flushed.
	if (_idcs.front == _idcs.gpu_back) {
		_idcs.gpu_back = new_front;
	}

	_idcs.front = new_front;
}

template <class Elem, class Alloc>
[[nodiscard]] bool ring_buffer<Elem, Alloc>::flush () noexcept
{
	auto result = _memory.flush_wrapping(added_idcs());

	// The GPU is now caught up to the CPU.
	_idcs.gpu_back = _idcs.back;
	return result;
}

}
