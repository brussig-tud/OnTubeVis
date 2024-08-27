#pragma once

// C++ STL
#include <cassert>
#include <type_traits>
#include <vector>

// CGV framework
#include <cgv_gl/gl/gl.h>

// local includes
#include "util.h"


/// Provides containers and allocators using persistently mapped OpenGL buffers that can be accessed
/// by both CPU and GPU.
/// Unless stated otherwise, users are responsible for ensuring synchronization between CPU and GPU
/// as well as between threads.
namespace gpumem {

/// Represents the size of and offset into an OpenGL buffer's memory in bytes.
using size_type = GLsizeiptr;
/// Represents the difference in bytes between offsets into an OpenGL buffer's memory.
using index_type = GLintptr;
/// Type used identify OpenGL buffer objects.
using handle_type = GLuint;


/// Cast `sizeof` for use with OpenGL buffers.
template <typename T>
constexpr size_type memsize {static_cast<size_type>(sizeof(T))};


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


/// Divides a span of memory into blocks of equal length which can be individually allocated.
/// Backing memory must be provided by derived classes, see `memory_pool`.
class memory_pool_alloc {
private:
	/// Start of each unallocated block.
	std::vector<std::byte*> _free_blocks {};
	/// Length of each block in bytes.
	size_type _block_length {0};

protected:
	/// The entire backing memory managed by this allocator.
	/// Must be allocated and freed by derived classes.
	span<std::byte> _memory {};

	/// Construct a null instance with no backing memory.
	[[nodiscard]] memory_pool_alloc() = default;

	/// Initialize this object with new backing memory.
	/// All blocks will be free.
	/// Any previous state is overwritten, attempting to free old allocations is an error.
	[[nodiscard]] bool create (
		size_type       num_blocks,
		size_type       block_length,
		span<std::byte> memory
	);

public:
	/// Return the `memory_pool_alloc` subobject of `*this`.
	[[nodiscard]] memory_pool_alloc &allocator () noexcept
	{
		return *this;
	}

	/// The length of each block in bytes.
	[[nodiscard]] constexpr size_type block_length () const noexcept
	{
		return _block_length;
	}

	/// The entire backing memory managed by this object.
	[[nodiscard]] constexpr const span<std::byte> &as_span () const noexcept
	{
		return _memory.as_span();
	}

	/// In a free block, create an allocation that holds at least `length` objects of type `Elem`
	/// and is aligned to `alignment` bytes, which must be a power of two.
	/// Always uses exactly one block, even if less memory is requested.
	/// Fails if the requested allocation does not fit into a block or all blocks are occupied.
	template <class Elem = std::byte>
	[[nodiscard]] span<Elem> alloc (size_type length, size_type alignment = alignof(Elem));

	/// Free an allocated block.
	/// `memory` must be an allocation produced by this object that has not been freed yet.
	template <class Elem>
	void dealloc (span<Elem> memory) noexcept;
};

/// Divides a span of memory into blocks of equal length which can be individually allocated.
/// This class only provides the backing memory, for managing the blocks see `memory_pool_alloc`.
template <class Alloc = buffer_alloc>
class memory_pool : protected Alloc, public memory_pool_alloc {
public:
	/// Allocator type used to manage the backing memory, in which blocks are placed, as a whole.
	/// Not to be confused with the block allocator itself, `memory_pool_alloc`.
	using alloc_type = Alloc;

public:
	/// Construct a null instance with no backing buffer, but an allocator that can be used to
	/// obtain backing memory in the future.
	[[nodiscard]] explicit memory_pool(const alloc_type &allocator = {})
		: alloc_type{allocator}
	{}

	memory_pool(const memory_pool &src) = delete;
	[[nodiscard]] constexpr memory_pool(memory_pool &&src) noexcept;

	memory_pool &operator= (const memory_pool &src) = delete;
	memory_pool &operator= (memory_pool &&src) noexcept;

	~memory_pool() noexcept;

	// Disambiguate between `memory_pool_alloc` and `alloc_type`.
	using alloc_type::allocator;
	using memory_pool_alloc::alloc;
	using memory_pool_alloc::dealloc;

	/// Allocate new backing memory and subdivide it into blocks of at least `block_length` bytes
	/// aligned to `block_alignment` bytes, which must be a power of two.
	/// See also `memory_pool_alloc::create`.
	[[nodiscard]] bool create (
		size_type num_blocks,
		size_type block_length,
		size_type block_alignment
	);
};

/// Allocates memory from a pool shared with others.
using memory_pool_ptr = alloc_ptr<memory_pool_alloc>;


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

}
