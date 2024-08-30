#pragma once
#include "span.h"

#include <memory>

#include "gl_util.h"


namespace otv::gpumem {

template <class Elem>
constexpr auto span<Elem>::wrapping_iterator_type::operator++ () noexcept -> wrapping_iterator_type&
{
	++_elem;

	// Wrap-around.
	if (_elem == _span.data() + _span.length()) {
		_elem = _span.data();
	}

	return *this;
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
constexpr auto span<Elem>::wrapping_iterator (index_type offset) const noexcept
	-> wrapping_iterator_type
{
	assert(offset < _length);
	return {*this, _data + offset};
}

template <class Elem>
bool span<Elem>::flush () const noexcept
{
	// Nothing to do for an empty buffer.
	if (_length == 0) {
		return true;
	}
	// Flush.
	glFlushMappedBufferRange(
		GL_COPY_READ_BUFFER,
		bytes().data() - buffer().data(),
		bytes().length()
	);

	return check_gl_errors("gpumem::span::flush");
}

template <class Elem>
bool span<Elem>::flush_wrapping (ro_range<index_type> range) const noexcept
{
	// Nothing to do for an empty range.
	if (range.is_empty()) {
		return true;
	}

	index_type offset {bytes().data() - buffer().data()};
	auto [begin, end] {range};

	// Flush the wrap-around, if there is any, then truncate the range to the end of the span.
	if (end < begin) {
		glFlushMappedBufferRange(GL_COPY_READ_BUFFER, offset, end * memsize<elem_type>);
		end = _length;
	}

	// Flush the contiguous range.
	glFlushMappedBufferRange(
		GL_COPY_READ_BUFFER,
		offset + begin,
		(end - begin) * memsize<elem_type>
	);

	return check_gl_errors("gpumem::span::flush_wrapping");
}

} // namespace otv::gpumem
