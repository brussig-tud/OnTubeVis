#pragma once

#include <cstdint>
#include <optional>


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

	ro_range& operator += (const std::size_t offset) noexcept {
		begin += offset;
		end += offset;
		return *this;
	}

	ro_range& operator -= (const std::size_t offset) noexcept {
		begin -= offset;
		end -= offset;
		return *this;
	}
};

template <class T>
ro_range(T, T) -> ro_range<T>;


template <typename O, typename F>
auto map_optional (O &&o, F &&f) -> std::optional<decltype(f(*std::forward<O>(o)))> {
	if (!o.has_value())
		return std::nullopt;
	return {f(*std::forward<O>(o))};
}
