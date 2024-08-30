#pragma once

#include <cstddef>

#include "util.h"
#include "common.h"


namespace otv::gpumem {

/// A non-owning view of a contiguous sequence of objects located in a persistently mapped OpenGL
/// buffer.
template <class Elem>
class span {
public:
	/// Type of the contained objects.
	using elem_type = Elem;

	/// An iterator over a span that wraps back to the start when it reaches the end.
	class wrapping_iterator_type {
		friend class span;

	protected:
		/// The span being iterated over.
		const span &_span;
		/// The iterator's current position.
		elem_type *_elem {nullptr};

		constexpr wrapping_iterator_type(const span &span, elem_type *elem) noexcept
			: _span {span}
			, _elem {elem}
		{}

	public:
		/// Return a pointer to the element at the iterator's current position.
		[[nodiscard]] constexpr elem_type *operator-> () const noexcept
		{
			return _elem;
		}

		/// Return the element at the iterator's current position.
		[[nodiscard]] constexpr elem_type &operator* () const noexcept
		{
			return *_elem;
		}

		/// Advance the iterator by one element.
		constexpr wrapping_iterator_type &operator++ () noexcept;
	};

private:
	/// Address of the first element.
	elem_type *_data {nullptr};
	/// Number of elements in the span.
	size_type _length {0};
	/// Name of the Buffer Object to which the spanned memory belongs.
	handle_type _handle {0};

public:
	/// Construct a null span with no associated memory or Buffer Object.
	[[nodiscard]] constexpr span() noexcept = default;

	/// Construct a span with the given properties.
	/// Should only be called from within the `gpumem` namespace.
	[[nodiscard]] constexpr span(elem_type *data, size_type length, handle_type handle) noexcept
		: _data   {data}
		, _length {length}
		, _handle {handle}
	{}

	/// Return the `span` subobject of `*this` as const.
	[[nodiscard]] constexpr const span &as_span () const noexcept
	{
		return *this;
	}

	/// Return the `span` subobject of `*this`.
	[[nodiscard]] constexpr span &as_span () noexcept
	{
		return *this;
	}

	/// Return the address of the first element.
	/// `nullptr` for a null span.
	[[nodiscard]] constexpr elem_type *data () const noexcept
	{
		return _data;
	}

	/// Return the number of elements in the span.
	[[nodiscard]] constexpr size_type length () const noexcept
	{
		return _length;
	}

	/// Return the name of the Buffer Object to which the spanned memory belongs.
	/// 0 for a null span.
	[[nodiscard]] constexpr handle_type handle () const noexcept
	{
		return _handle;
	}

	/// Create a right-open range over the spanned memory.
	[[nodiscard]] constexpr ro_range<elem_type*> as_range () const noexcept
	{
		return {_data, _data + length()};
	}

	/// Create a read-only span over the same memory.
	[[nodiscard]] span<const elem_type> as_const () const noexcept
	{
		return {_data, _length, _handle};
	}

	/// Create a span over the same memory accessed though `reinterpret_cast`.
	/// The new span is truncated to the last `T` it can fully contain.
	/// In particular, this means that the result may have a length of zero and that round trip
	/// conversion may cut off some elements.
	template <class T>
	[[nodiscard]] span<T> reinterpret_as () const noexcept;

	/// The memory spanned by this object as raw bytes.
	/// Equivalent to `reinterpret_as<std:byte>`.
	[[nodiscard]] span<std::byte> bytes () const noexcept
	{
		return reinterpret_as<std::byte>();
	}

	/// Create a subspan of this object whose begin is aligned to the given number of bytes, which
	/// must be a power of two.
	/// If the requested alignment does not fit into the spanned memory, a null span is returned.
	[[nodiscard]] span aligned_to (size_type alignment) const;

	/// Create a span over a the containing Buffer Object's entire mapping.
	[[nodiscard]] span<std::byte> buffer () const;

	/// Access an element without bounds checking.
	[[nodiscard]] constexpr elem_type &operator[] (index_type idx) const noexcept
	{
		return _data[idx];
	}

	/// Create an iterator at the given index that will wrap from the end of the span to its start.
	/// Precondition: `offset < length()`.
	[[nodiscard]] constexpr wrapping_iterator_type wrapping_iterator (index_type offset = 0)
		const noexcept;

	/// Ensure that the spanned memory is up-to-date for the GPU.
	[[nodiscard]] bool flush () const noexcept;

	/// Ensure that all elements in the given range are up-to-date for the GPU.
	/// The range may wrap around from the end of the buffer to the start, but may not overlap
	/// itself, and the start index must be in bounds.
	[[nodiscard]] bool flush_wrapping (ro_range<index_type> range) const noexcept;
};

} // namespace otv::gpumem
