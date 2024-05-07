#pragma once 

// C++ STL
#include <stdexcept>

// local includes
#include "gl_util.h"

// implemented header
#include "gpumem.h"


namespace gpumem {

template <class Elem, class Sfinae>
constexpr array<Elem, Sfinae>::array(array &&src) noexcept
	: self {std::move(src.self)}
{
	// Mark source as empty to prevent double free.
	src.self.handle = 0;
}

template <class Elem, class Sfinae>
auto array<Elem, Sfinae>::operator=(array &&src) noexcept -> array&
{
	// Explicit NOP on self-assignment to prevent deallocation.
	if (&src == this) {
		return *this;
	}

	// Clean up current allocation.
	array::~array();

	// Take source state.
	self = std::move(src.self);

	// Mark source as empty to prevent double free.
	src.self.handle = 0;

	return *this;
}

template <class Elem, class Sfinae>
array<Elem, Sfinae>::~array() noexcept
{
	if (self.handle) {
		glDeleteBuffers(1, &self.handle);
		check_gl_errors("gpumem::array::~array");
	}
}

template <class Elem, class Sfinae>
[[nodiscard]] bool array<Elem, Sfinae>::create (size_type length) noexcept
{
	// Destroy any existing buffer object and reset state.
	*this = {};

	// Store length and calculate size in bytes.
	self.length     = length;
	auto size_bytes = self.length * sizeof(elem_type);

	// Create buffer object.
	glGenBuffers(1, &self.handle);
	glBindBuffer(GL_COPY_WRITE_BUFFER, self.handle);

	// Allocate GPU-accessible storage and persistently map it to the CPU's address space without implicit
	// synchronization.
	auto flags = GL_MAP_WRITE_BIT | GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT;
	glBufferStorage(GL_COPY_WRITE_BUFFER, size_bytes, nullptr, flags);
	self.data = reinterpret_cast<elem_type*>(glMapBufferRange(
			GL_COPY_WRITE_BUFFER, 0, size_bytes, flags | GL_MAP_FLUSH_EXPLICIT_BIT));

	return check_gl_errors("gpumem::array::create");
}

template <class Elem, class Sfinae>
[[nodiscard]] bool array<Elem, Sfinae>::flush_wrapping (index_type begin, index_type end) noexcept
{
	// Nothing to do for an empty range.
	if (begin == end) {
		return true;
	}

	glBindBuffer(GL_COPY_READ_BUFFER, self.handle);

	// If the range wraps around, two flush calls are required.
	if (end < begin) {
		glFlushMappedBufferRange(GL_COPY_READ_BUFFER, 0, end * sizeof(elem_type));
		end = self.length;
	}

	glFlushMappedBufferRange(
		GL_COPY_READ_BUFFER,
		begin * sizeof(elem_type),
		(end - begin) * sizeof(elem_type)
	);

	return check_gl_errors("gpumem::array::flush");
}


template <class Elem, class Sfinae>
constexpr auto ring_buffer<Elem, Sfinae>::index_after(index_type idx) const noexcept -> index_type
{
	return ++idx == self.alloc.length() ? 0 : idx;
}

template <class Elem, class Sfinae>
void ring_buffer<Elem, Sfinae>::push_back (const elem_type &elem)
{
	// If the buffer is full, raise an exception.
	auto new_back = index_after(self.back);

	if (new_back == self.gpu_front) {
		throw std::runtime_error("gpumem::ring_buffer::push_back would exceed capacity");
	}

	// Otherwise store the element and advance the back index.
	self.alloc[self.back] = elem;
	self.back             = new_back;
}

template <class Elem, class Sfinae>
void ring_buffer<Elem, Sfinae>::push_back (const elem_type *elems, size_type num_elems)
{
	auto new_back = self.back + num_elems;

	// If the used part of the buffer is sequential in memory, the unused part wraps around.
	if (self.gpu_front <= self.back) {
		// If there is enough room at the end of the allocation, a single copy is enough.
		if (new_back < self.alloc.length()) {
			goto copy;
		}

		// Otherwise the back part of the allocation is filled in a first copy...
		auto back_vacancy_len = self.alloc.length() - self.back;
		std::copy(&self.alloc[self.back], elems, back_vacancy_len);

		// ...and the rest is copied to the front.
		elems     += back_vacancy_len;
		num_elems -= back_vacancy_len;
		self.back  = 0;
		new_back  -= self.alloc.length();
	}

	// At this point, the unused part of the allocation is sequential in memory.

	// If there is not enough room to store the new values without overwriting data still used by GPU commands, raise an
	// exception.
	if (new_back >= self.gpu_front) {
		throw std::runtime_error("gpumem::ring_buffer::push_back would exceed capacity");
	}

	// Otherwise copy the new elements and update the back index.
	copy:
	std::copy(&self.alloc[self.back], elems, num_elems);
	self.back = new_back;
}

template <class Elem, class Sfinae>
void ring_buffer<Elem, Sfinae>::pop_front () noexcept
{
	// Sanity check.
	assert(self.front != self.back);

	auto new_front = index_after(self.front);

	// Ensure that `gpu_back` remains in the used region of the allocation so that no unused memory is flushed.
	if (self.front == self.gpu_back) {
		self.gpu_back = new_front;
	}

	self.front = new_front;
}

template <class Elem, class Sfinae>
[[nodiscard]] bool ring_buffer<Elem, Sfinae>::flush () noexcept
{
	auto result = self.alloc.flush_wrapping(self.gpu_back, self.back);

	// The GPU is now caught up to the CPU.
	self.gpu_back = self.back;
	return result;
}

}
