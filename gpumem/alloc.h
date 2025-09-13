#pragma once

// C++ STL
#include <concepts>

// local includes
#include "gpumem/span.h"

// forward declarations
namespace std::pmr {
	class memory_resource;
}


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


/// Adapts a `std::pmr::memory_resource` as an STL Allocator (named requirement).
/// Similar to `std::pmr::polymorphic_allocator`, but the resource type can be restricted and
/// allocators propagate on container assignment.
template <
	class T = std::byte,
	std::derived_from<std::pmr::memory_resource> Resource = std::pmr::memory_resource
>
class pmr_alloc {
	/// The memory resource providing allocations.
	Resource* _resource {nullptr};

public:
	using value_type = T;
	// Make allocators move with the memory they provide.
	using propagate_on_container_copy_assignment = std::true_type;
	using propagate_on_container_move_assignment = std::true_type;
	using propagate_on_container_swap            = std::true_type;

	/// Create an instance that provides allocations from the given memory resource.
	/// The resource must outlive the allocator.
	/// Instances with a null resource are valid but cannot perform any allocations.
	[[nodiscard]] constexpr explicit pmr_alloc(Resource* resource = nullptr) noexcept
		: _resource {resource}
	{}

	/// Allocators are covariant with their memory resource.
	/// The allocated type can be changed freely (allocator rebinding).
	template <class T2, std::derived_from<Resource> R2>
	[[nodiscard]] constexpr pmr_alloc(pmr_alloc<T2, R2> const& src) noexcept
		: _resource {src.resource()}
	{}

	/// Returns true iff both instances can free each others allocations.
	[[nodiscard]] auto operator== (pmr_alloc const& other) const noexcept -> bool
	{
		// Null allocators are equal only to each other.
		if (!_resource || !other._resource) return !_resource && !other._resource;
		return _resource->is_equal(*other._resource);
	}

	/// The memory resource this instance uses to perform allocations.
	/// May be null.
	[[nodiscard]] constexpr auto resource () const noexcept -> Resource*
	{
		return _resource;
	}

	/// Allocate `count` objects of `value_type` without initialization.
	[[nodiscard]] auto allocate (size_t count) -> T*
	{
		if (!_resource) throw std::bad_alloc{};
		return static_cast<T*>(_resource->allocate(count * sizeof(T), alignof(T)));
	}

	/// Free a previous allocation from a compatible memory resource.
	void deallocate (T* ptr, size_t count) noexcept
	{
		if (!_resource) return;
		_resource->deallocate(ptr, count * sizeof(T), alignof(T));
	}

	/// Copied containers use the same allocator as the original.
	[[nodiscard]] constexpr auto select_on_container_copy_construction() const noexcept -> pmr_alloc
	{
		return *this;
	}
};

} // namespace otv::gpumem
