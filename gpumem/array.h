#pragma once

#include <type_traits>

#include "alloc.h"


namespace otv::gpumem {

/// Container holding a contiguous sequence of objects stored in a persistently mapped OpenGL
/// buffer.
template <class Elem, class Alloc = buffer_alloc>
class array : protected Alloc, public span<Elem> {
	// This container does not construct or destroy its elements, so they should be trivial.
	// Triviality is only enforced for the destructor though, since important CGV math types like
	// `fvec` manually declare constructors that are de-facto trivial, but not de-jure.
	static_assert(std::is_trivially_destructible_v<Elem>);

public:
	/// Type of the allocator used to manage the spanned memory.
	using alloc_type = Alloc;

	/// Construct a null instance with no backing bufferm but an allocator that can be used to
	/// obtain backing memory in the future.
	[[nodiscard]] explicit array(const alloc_type &allocator = {})
		: alloc_type{allocator}
	{}

	array(const array &src) = delete;
	[[nodiscard]] constexpr array(array &&src) noexcept;

	array &operator= (const array &src) = delete;
	array &operator= (array &&src) noexcept;

	~array() noexcept;

	/// Replace the backing buffer with newly allocated memory spanning `length` uninitialized
	/// elements and aligned to the requested number of bytes, which must be a power of two.
	/// Previously owned memory is deallocated without the contained elements being destroyed.
	[[nodiscard]] bool create (size_type length, size_type alignment = alignof(Elem));
};

} // namespace otv::gpumem
