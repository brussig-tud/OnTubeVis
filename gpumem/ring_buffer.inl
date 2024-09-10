#pragma once
#include "ring_buffer.h"

#include <stdexcept>

#include "array.inl"


namespace otv::gpumem {

template <class Elem, class Alloc>
constexpr index_type ring_buffer<Elem, Alloc>::index_after (
	index_type idx,
	index_type offset
) const noexcept {
	assert(offset <= _memory.length());
	idx += offset;
	return idx >= _memory.length() ? idx - _memory.length() : idx;
}

template <class Elem, class Alloc>
constexpr ro_range<index_type> ring_buffer<Elem, Alloc>::flush_range () noexcept {
	ro_range range {_idcs.gpu_back, _idcs.back};

	// If the front of the buffer has passed GPU back, we only need to flush from the front index.
	// In case of wrap-around, the inclusion test has to be negated.
	if ((range.begin <= range.end) ^ range.contains(_idcs.front)) {
		range.begin = _idcs.front;
	}

	return range;
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
void ring_buffer<Elem, Alloc>::pop_front (size_type num_elems) noexcept
{
	assert(num_elems >= 0 && num_elems <= length());
	_idcs.front = index_after(_idcs.front, num_elems);
}

template <class Elem, class Alloc>
[[nodiscard]] bool ring_buffer<Elem, Alloc>::flush () noexcept
{
	auto result = _memory.flush_wrapping(flush_range());

	// The GPU is now caught up to the CPU.
	_idcs.gpu_back = _idcs.back;
	return result;
}

} // namespace otv::gpumem
