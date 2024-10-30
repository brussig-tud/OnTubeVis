#pragma
#include "array.h"

#include "alloc.inl"


namespace otv::gpumem {

template <class Elem, class Alloc>
constexpr array<Elem, Alloc>::array(array &&src) noexcept
	: alloc_type{std::move(src)}
	, span<Elem>{std::move(src)}
{
	src.as_span() = {};
}

template <class Elem, class Alloc>
auto array<Elem, Alloc>::operator= (array &&src) noexcept -> array&
{
	if (&src != this) {
		// Free current memory.
		destroy();
		// Take from source.
		this->allocator() = {std::move(src)};
		this->as_span()   = {std::move(src)};
		// Mark source as empty.
		src.as_span() = {};
	}

	return *this;
}

template <class Elem, class Alloc>
bool array<Elem, Alloc>::create (size_type length, size_type alignment)
{
	// Free previous allocation.
	destroy();

	// Allocate new memory.
	this->as_span() = this->template alloc<Elem>(length, alignment);
	assert(this->length() == length);
	return this->handle();
}

template <class Elem, class Alloc>
void array<Elem, Alloc>::destroy () noexcept
{
	if (this->as_span().data()) {
		this->dealloc(this->as_span());
		this->as_span() = {};
	}
}

} // namespace otv::gpumem
