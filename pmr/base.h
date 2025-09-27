#pragma once

// C++ STL
#include <memory_resource>
#include <span>


namespace otv::pmr {

/// Abstract memory resource whose allocations all lie within a known range.
class memory_region : public std::pmr::memory_resource {
public:
	/// Range containing all memory managed by this instance.
	/// May also include memory it does not manage.
	[[nodiscard]] constexpr auto span () const noexcept -> std::span<std::byte>
	{
		return _span;
	}

protected:
	/// Range containing all memory managed by this instance.
	/// May also include memory it does not manage.
	std::span<std::byte> _span {};

	/// Create an empty resource that does not manage any memory.
	[[nodiscard]] constexpr memory_region() noexcept = default;
	/// Create a resource whose managed memory is contained within the given range.
	[[nodiscard]] constexpr memory_region(std::span<std::byte> span) noexcept
		: _span {span}
	{}
};

} // namespace otv::pmr
