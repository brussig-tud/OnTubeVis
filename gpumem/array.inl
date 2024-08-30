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
		this->~array();
		this->allocator() = {std::move(src)};
		this->as_span()   = {std::move(src)};
		src.as_span()     = {};
	}

	return *this;
}

template <class Elem, class Alloc>
array<Elem, Alloc>::~array() noexcept
{
	if (this->as_span().data()) {
		this->dealloc(*this);
		this->as_span() = {};
	}
}

template <class Elem, class Alloc>
bool array<Elem, Alloc>::create (size_type length, size_type alignment)
{
	this->as_span() = this->template alloc<Elem>(length, alignment);
	assert(this->length() == length);
	return this->handle();
}

} // namespace otv::gpumem
