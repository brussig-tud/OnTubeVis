#pragma once

// C++ STL
#include <cstring>
#include <stdexcept>

// local includes
#include "gl_util.h"

// implemented header
#include "ring_buffer.h"


template <class Elem, class Sfinae>
constexpr auto ring_buffer<Elem, Sfinae>::index_after(index_type idx) const noexcept -> index_type
{
	return ++idx == self.alloc_len ? 0 : idx;
}

template <class Elem, class Sfinae>
constexpr ring_buffer<Elem, Sfinae>::ring_buffer(ring_buffer &&src) noexcept
	: self {std::move(src.self)}
{
	// Mark source as unallocated to prevent double free.
	src.self.handle = 0;
}

template <class Elem, class Sfinae>
auto ring_buffer<Elem, Sfinae>::operator=(ring_buffer &&src) noexcept -> ring_buffer&
{
	// Explicit NOP on self-assignment to prevent deallocation.
	if (&src == this) {
		return *this;
	}

	// Clean up current allocation.
	ring_buffer::~ring_buffer();

	// Take source state.
	self = std::move(src.self);

	// Mark source as unallocated to prevent double free.
	src.self.handle = 0;

	return *this;
}

template <class Elem, class Sfinae>
ring_buffer<Elem, Sfinae>::~ring_buffer() noexcept
{
	if (self.handle) {
		glDeleteBuffers(1, &self.handle);
		check_gl_errors("ring_buffer::~ring_buffer");
	}
}

template <class Elem, class Sfinae>
[[nodiscard]] bool ring_buffer<Elem, Sfinae>::create (size_type capacity) noexcept
{
	// Destroy any existing allocation and reset member variables.
	*this = {};

	// The allocation must have one additional element slot since there must always be at least one unused slot to
	// distinguish a full buffer from an empty one
	self.alloc_len  = capacity + 1;
	auto alloc_size = self.alloc_len * sizeof(elem_type);

	// Create GL buffer object.
	glGenBuffers(1, &self.handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, self.handle);

	// Allocate GPU-accessible storage and persistently map it to the CPU's address space without implicit
	// synchronization.
	auto flags = GL_MAP_WRITE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT;
	glBufferStorage(GL_COPY_WRITE_BUFFER, alloc_size, nullptr, flags);
	self.data = reinterpret_cast<elem_type*>(glMapBufferRange(
			GL_COPY_WRITE_BUFFER, 0, alloc_size, flags | GL_MAP_FLUSH_EXPLICIT_BIT));

	return check_gl_errors("ring_buffer::create");
}

template <class Elem, class Sfinae>
void ring_buffer<Elem, Sfinae>::push_back (const elem_type &elem)
{
	// If the buffer is full, raise an exception.
	auto new_back = index_after(self.back);

	if (new_back == self.gpu_front) {
		throw std::runtime_error("ring_buffer::push_back would exceed capacity");
	}

	// Otherwise store the element and advance the back index.
	self.data[self.back] = elem;
	self.back            = new_back;
}

template <class Elem, class Sfinae>
void ring_buffer<Elem, Sfinae>::push_back (const elem_type *elems, size_type num_elems)
{
	auto new_back = self.back + num_elems;

	// If the used part of the buffer is sequential in memory, the unused part wraps around.
	if (self.gpu_front <= self.back) {
		// If there is enough room at the end of the allocation, a single copy is enough.
		if (new_back < self.alloc_len) {
			goto copy;
		}

		// Otherwise the back part of the allocation is filled in a first copy...
		auto back_vacancy_len = self.alloc_len - self.back;
		std::memcpy(self.data + self.back, elems, back_vacancy_len * sizeof(elem_type));

		// ...and the rest is copied to the front.
		elems     += back_vacancy_len;
		num_elems -= back_vacancy_len;
		self.back  = 0;
		new_back  -= self.alloc_len;
	}

	// At this point, the unused part of the allocation is sequential in memory.

	// If there is not enough room to store the new values without overwriting data still used by GPU commands, raise an
	// exception.
	if (new_back >= self.gpu_front) {
		throw std::runtime_error("ring_buffer::push_back would exceed capacity");
	}

	// Otherwise copy the new elements and update the back index.
	copy:
	std::memcpy(self.data + self.back, elems, num_elems * sizeof(elem_type));
	self.back = new_back;
}

template <class Elem, class Sfinae>
bool ring_buffer<Elem, Sfinae>::try_pop_front () noexcept
{
	// Cannot pop from an empty buffer.
	if (self.front == self.back) {
		return false;
	}

	auto new_front = index_after(self.front);

	// Ensure that `gpu_back` remains in the used region of the allocation so that no unused memory is flushed.
	if (self.front == self.gpu_back) {
		self.gpu_back = new_front;
	}

	self.front = new_front;
	return true;
}


template <class Elem, class Sfinae>
[[nodiscard]] bool ring_buffer<Elem, Sfinae>::flush () noexcept
{
	// Nothing to do if the GPU is already up to date.
	if (self.gpu_back == self.back) {
		return true;
	}

	glBindBuffer(GL_COPY_READ_BUFFER, self.handle);

	auto end = self.back;

	// If the unflushed region of the allocation wraps around, two flush calls are required.
	if (self.back < self.gpu_back) {
		glFlushMappedBufferRange(GL_COPY_READ_BUFFER, 0, self.back * sizeof(elem_type));
		end = self.alloc_len;
	}

	glFlushMappedBufferRange(
		GL_COPY_READ_BUFFER,
		self.gpu_back * sizeof(elem_type),
		(end - self.gpu_back) * sizeof(elem_type)
	);

	// The GPU is now caught up to the CPU.
	self.gpu_back = self.back;
	return check_gl_errors("ring_buffer::flush");
}
