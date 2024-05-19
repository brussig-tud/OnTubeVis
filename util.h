#pragma once


/// A right-open range [begin, end).
template <class T>
struct ro_range {
	T begin {};
	T end   {};
};
