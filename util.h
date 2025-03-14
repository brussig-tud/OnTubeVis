#pragma once

#include <cstdint>
#include <optional>


/// Find the largest common denominator of two integers.
template <class Integer>
constexpr Integer find_gcd (const Integer &a, const Integer &b) {
	if (b==0)
		return a;
	return find_gcd(b, a%b);
}

/// Find the largest common multiple of two integers.
template <class Integer>
constexpr Integer find_lcm (const Integer &a, const Integer &b) {
	return a/find_gcd(a, b) * b;
}

/// A right-open range [begin, end).
template <class Iter>
struct ro_range
{
	Iter begin {};
	Iter end   {};

	[[nodiscard]] constexpr bool is_empty () const noexcept {
		return begin == end;
	}

	[[nodiscard]] constexpr std::size_t length () const noexcept {
		return end - begin;
	}

	[[nodiscard]] constexpr bool contains (const Iter &elem) const noexcept {
		return elem >= begin && elem < end;
	}

	inline ro_range& operator += (const std::size_t offset) noexcept {
		begin += offset;
		end += offset;
		return *this;
	}
	inline void safe_advance (const std::size_t offset, const Iter &max) noexcept {
		begin += offset;
		const std::size_t safe_offset = max - end;
		end += std::min(offset, safe_offset);
	}
	inline void safe_unadvance(const std::size_t offset) noexcept {
		const std::size_t safe_offset = begin != end ? offset : 0;
		begin -= offset;
		end -= safe_offset;
	}

	[[nodiscard]] inline ro_range operator + (const std::size_t offset) noexcept {
		return ro_range{begin+offset, end+offset};
	}

	inline ro_range& operator -= (const std::size_t offset) noexcept {
		begin -= offset;
		end -= offset;
		return *this;
	}
	inline void safe_retreat (const std::size_t offset, const Iter &min) noexcept {
		const std::size_t safe_offset = begin - min;
		begin -= std::min(offset, safe_offset);
		end -= offset;
	}
	inline void safe_unretreat (const std::size_t offset) noexcept {
		const std::size_t safe_offset = begin != end ? offset : 0;
		begin += safe_offset;
		end += offset;
	}

	[[nodiscard]] inline ro_range operator - (const std::size_t offset) noexcept {
		return ro_range{begin-offset, end-offset};
	}
};

template <class Iter>
ro_range(Iter, Iter) -> ro_range<Iter>;

////
// Various typedefs for iterator-like objects that can be used to greatly shorten explicit template instantiations for
// all functions that have to work with ro_ranges
#if defined(_STL_VECTOR_H) || (defined(_MSC_VER) && defined(_VECTOR_))
	typedef std::vector<float>::iterator std_vector_float_iter;
#endif
#if defined(_STL_DEQUE_H) || (defined(_MSC_VER) && defined(_DEQUE_))
	typedef std::deque<float>::iterator std_deque_float_iter;
#endif


/// A map() operator for optionals, enabling functional programming-style handling of std::optional.
template <typename O, typename F>
auto map_optional (O &&o, F &&f) -> std::optional<decltype(f(*std::forward<O>(o)))> {
	if (!o.has_value())
		return std::nullopt;
	return {f(*std::forward<O>(o))};
}


/// A simple RAII-style finalizer, making sure the finalizer code is called when the helper gets destroyed. In case
/// the finalizer contains code that should not be called under some circumstances (e.g. when the parent function as
/// a whole succeeds and allocated resources should thus not be destroyed), it can be programmatically disarmed.
template <class Finalizer>
struct finalizer
{
	bool armed = true;
	Finalizer f;

	/// Construct for the provided finalizer.
	[[nodiscard]] explicit finalizer(Finalizer &&f) : f(std::move(f))
	{}

	~finalizer() {
		if (armed)
			f();
	}

	/// Disarm iff the provided @a disarmed flag is @c true . Disarming means that when the finalizer is triggered, it
	/// will not actually be executed. Forwards the passed in boolean as return.
	bool disarm (bool disarmed=true) {
		armed = !disarmed;
		return disarmed;
	}
};
