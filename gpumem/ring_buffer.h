#pragma once

#include <type_traits>

#include "array.h"



namespace otv::gpumem {

/// A ring buffer using wrap-around indices into contiguous memory.
/// Separately tracks the range of elements synchronized with the GPU.
template <class Elem, class Alloc = buffer_alloc>
class ring_buffer {
	// This container does not construct or destroy its elements, so they should be trivial.
	// Triviality is only enforced for the destructor though, since important CGV math types like
	// `fvec` manually declare constructors that are de-facto trivial, but not de-jure.
	static_assert(std::is_trivially_destructible_v<Elem>);

public:
	/// Type of the contained objects.
	using elem_type = Elem;
	/// Type of the allocator used to manage the backing memory.
	using alloc_type = Alloc;

private:
	// Indices are wrapped in a struct for convenient collective assignment and reset.
	struct {
		/// Index of the oldest entry.
		index_type front {0};
		/// Index one past the newest entry.
		index_type back {0};
		/// Index of the oldest entry that has been flushed to the GPU.
		index_type gpu_front {0};
		/// Index one past the newest entry that has been flushed to the GPU.
		index_type gpu_back {0};
	} _idcs;

	/// The backing memory in which elements are placed.
	array<Elem, Alloc> _memory;

	/// Advance an index, accounting for wrap-around.
	/// `idx` must be valid and `0 <= offset <= _memory.length()`.
	[[nodiscard]] constexpr index_type index_after (
		index_type idx,
		index_type offset = 1
	) const noexcept;

public:
	/// Construct a null instance with no backing buffer, but an allocator that can be used to
	/// obtain backing memory in the future.
	[[nodiscard]] ring_buffer(const alloc_type &allocator = {})
		: _memory {allocator}
	{}

	/// Index of the oldest entry.
	[[nodiscard]] constexpr index_type front () const noexcept
	{
		return _idcs.front;
	}

	/// Index one past the newest entry.
	[[nodiscard]] constexpr index_type back () const noexcept
	{
		return _idcs.back;
	}

	/// Index one past the newest entry that has been flushed to the GPU.
	[[nodiscard]] constexpr index_type gpu_back () const noexcept
	{
		return _idcs.gpu_back;
	}

	/// Return a view of the backing memory.
	[[nodiscard]] constexpr const span<Elem> &as_span() const noexcept {
		return _memory.as_span();
	}

	/// Return true iff the buffer contains no elements.
	[[nodiscard]] constexpr bool is_empty () const noexcept
	{
		return _idcs.front == _idcs.back;
	}

	/// Calculate the number of elements between two indices, accounting for wrap-around.
	[[nodiscard]] constexpr index_type distance (index_type from, index_type to) const noexcept
	{
		return from <= to
			? to - from
			: _memory.length() - from + to;
	}

	/// The maximum number of elements the buffer can hold with its current allocation.
	[[nodiscard]] constexpr size_type capacity () const noexcept
	{
		return _memory.length() - 1;
	}

	/// The number of elements in the buffer.
	[[nodiscard]] constexpr size_type length () const noexcept
	{
		return distance(_idcs.front, _idcs.back);
	}

	/// Calculate the number of elements that can be added right now, i.e. the number of slots that
	/// are unused for both CPU and GPU.
	[[nodiscard]] constexpr size_type free_capacity () const noexcept
	{
		return capacity() - distance(_idcs.gpu_front, _idcs.back);
	}

	/// Return a pointer to the first entry of the buffer, or `nullptr` if it is empty.
	[[nodiscard]] constexpr elem_type *try_first () noexcept
	{
		return _idcs.front == _idcs.back ? nullptr : &_memory[_idcs.front];
	}

	/// The range of elements that have to be flushed so that all elements of the buffer are
	/// up-to-date on the GPU.
	[[nodiscard]] constexpr ro_range<index_type> flush_range () noexcept;


	/// Set the index onward from which elements may be used by the GPU.
	/// Elements before this index (and after `gpu_back`) are not in use by the GPU and may be
	/// overwritten.
	/// The caller is responsible for ensuring that all commands using these elements have actually
	/// completed.
	constexpr void set_gpu_front (index_type gpu_front) noexcept
	{
		_idcs.gpu_front = gpu_front;
	}

	/// Allocate new backing memory large enough for the buffer to hold `capacity` entries.
	/// The buffer will initially be empty.
	/// Previously contained elements are deallocated without being destroyed.
	/// For implementation reasons, the length of the new backing memory is at least `capacity + 1`.
	[[nodiscard]] bool create (size_type capacity)
	{
		_idcs = {};
		return _memory.create(capacity + 1);
	}

	/// Add a new element at the back of the buffer.
	/// The change might not be visible to the GPU until `sync` is called.
	void push_back (const elem_type &elem);
	/// Add new elements at the back of the buffer.
	/// The change might not be visible to the GPU until `sync` is called.
	template <class Iter>
	void push_back (ro_range<Iter> elems);

	/// Remove elements from the front of the buffer.
	void pop_front (size_type num_elems = 1) noexcept;

	/// Make newly added elements visible to the GPU.
	[[nodiscard]] bool flush () noexcept;
};

} // namespae otv::gpumem
