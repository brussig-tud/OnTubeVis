#pragma once

#include "span.h"


namespace otv::gpumem {

/// A stateless allocator that places each allocation in a separate Buffer Object.
class buffer_alloc {
public:
	/// Return the `buffer_alloc` subobject of `*this`.
	[[nodiscard]] buffer_alloc &allocator () noexcept
	{
		return *this;
	}

	/// Create a Buffer Object with persistently mapped storage and return a subspan of its memory
	/// that is aligned to the given number of bytes and contains at least `length` entries of type
	/// `Elem`.
	/// `alignment` must be a power of two.
	template <class Elem = std::byte>
	[[nodiscard]] span<Elem> alloc (size_type length, size_type alignment = alignof(Elem));

	/// Destroy a previously created Buffer Object, releasing its memory.
	/// `memory` must be an allocation created by this class that has not been deallocated before.
	template <class Elem>
	void dealloc (span<Elem> memory) noexcept;
};


/// An allocator that points to another, to which it delegates all operations.
/// Allows multiple containers to use the same allocator instance.
/// Users are responsible for ensuring that the pointee remains valid for the entire lifetime of
/// this object.
template <class Alloc>
class alloc_ptr {
public:
	/// Type of the allocator to which this one points and delegates.
	using alloc_type = Alloc;

private:
	/// The allocator to which this one points and delegates, the pointee.
	alloc_type *_allocator {nullptr};

public:
	[[nodiscard]] alloc_ptr() noexcept = default;
	/// Construct a new pointer to the given allocator.
	[[nodiscard]] explicit alloc_ptr(alloc_type &allocator)
		: _allocator {&allocator}
	{}

	/// Return the `alloc_ptr` subobject of `*this`.
	[[nodiscard]] alloc_ptr &allocator () noexcept
	{
		return *this;
	}

	/// In a persistently mapped Buffer Object, allocate memory that contains at least `length`
	/// objects of type `Elem` and is aligned to `alignment` bytes, which must be a power of two
	template <class Elem = std::byte>
	[[nodiscard]] span<Elem> alloc (size_type length, size_type alignment = alignof(Elem))
	{
		assert(_allocator);
		return _allocator->template alloc<Elem>(length, alignment);
	}

	/// Free a previous allocation.
	/// `memory` must be an allocation created by an allocator equal to the pointee and must not
	/// have been deallocated before.
	template <class Elem>
	void dealloc (span<Elem> alloc) noexcept
	{
		assert(_allocator);
		return _allocator->dealloc(alloc);
	}
};

} // namespace otv::gpumem
