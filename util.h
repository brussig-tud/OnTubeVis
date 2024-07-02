#pragma once

#include <cstdint>


/// A right-open range [begin, end).
template <class T>
struct ro_range {
	T begin {};
	T end   {};

	[[nodiscard]] constexpr bool is_empty () const noexcept
	{
		return begin == end;
	}

	[[nodiscard]] constexpr std::size_t length () const noexcept
	{
		return end - begin;
	}

	[[nodiscard]] constexpr bool contains (const T &elem) const noexcept
	{
		return elem >= begin && elem < end;
	}
};

template <class T>
ro_range(T, T) -> ro_range<T>;
