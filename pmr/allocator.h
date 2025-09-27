#pragma once

// C++ STL
#include <concepts>
#include <cstddef>
#include <new>

namespace std::pmr {
	class memory_resource;
}


namespace otv::pmr {

/// Adapts a `std::pmr::memory_resource` as an STL Allocator (named requirement).
/// Similar to `std::pmr::polymorphic_allocator`, but the resource type can be restricted and
/// allocators propagate on container assignment.
template <
	class T = std::byte,
	std::derived_from<std::pmr::memory_resource> Resource = std::pmr::memory_resource
>
class allocator {
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
	[[nodiscard]] constexpr explicit allocator(Resource* resource = nullptr) noexcept
		: _resource {resource}
	{}

	/// Allocators are covariant with their memory resource.
	/// The allocated type can be changed freely (allocator rebinding).
	template <class T2, std::derived_from<Resource> R2>
	[[nodiscard]] constexpr allocator(allocator<T2, R2> const& src) noexcept
		: _resource {src.resource()}
	{}

	/// Returns true iff both instances can free each others allocations.
	[[nodiscard]] auto operator== (allocator const& other) const noexcept -> bool
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
	[[nodiscard]] constexpr auto select_on_container_copy_construction() const noexcept -> allocator
	{
		return *this;
	}
};

} // namespace otv::pmr
