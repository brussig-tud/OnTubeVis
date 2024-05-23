#pragma once

#include <cstdint>


/// A right-open range [begin, end).
template <class T>
struct ro_range {
	T begin {};
	T end   {};

	std::size_t length () const {
		return end - begin;
	}
};

template <class T>
ro_range(T, T) -> ro_range<T>;
