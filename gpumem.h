#pragma once

// C++ STL
#include <type_traits>

// CGV framework
#include <cgv_gl/gl/gl.h>

// local includes
#include "util.h"


/// Containers using persistently mapped memory that can be accessed by both CPU and GPU.
namespace gpumem {

// Common types.
using size_type   = GLsizeiptr;
using index_type  = GLintptr;
using handle_type = GLuint;


/// A fixed size buffer.
/// Users are responsible for synchronization.
template <class Elem, class = std::enable_if_t<std::is_trivially_destructible_v<Elem>>>
class array {
public:
	using elem_type = Elem;

private:
	/// Member variables wrapped in a struct for convenient assignment and reset.
	struct {
		elem_type *data    = nullptr;
		size_type length   = 0;
		handle_type handle = 0;
	} self;

public:
	/// Create an empty instance with no backing buffer.
	[[nodiscard]] constexpr array() noexcept = default;

	array(const array &src) = delete;
	[[nodiscard]] constexpr array(array &&src) noexcept;

	array &operator=(const array &src) = delete;
	array &operator=(array &&src) noexcept;

	~array() noexcept;


	[[nodiscard]] constexpr elem_type *data () const noexcept
	{
		return self.data;
	}

	[[nodiscard]] constexpr size_type length () const noexcept
	{
		return self.length;
	}

	[[nodiscard]] constexpr handle_type handle () const noexcept
	{
		return self.handle;
	}

	[[nodiscard]] constexpr const elem_type &operator[] (index_type idx) const noexcept
	{
		return self.data[idx];
	}

	[[nodiscard]] constexpr elem_type &operator[] (index_type idx) noexcept
	{
		return self.data[idx];
	}


	/// Create a buffer object and allocate memory for the desired number of elements.
	/// The initial state of that memory is undefined.
	/// Any previous buffer is destroyed.
	[[nodiscard]] bool create (size_type length) noexcept;

	/// Make the elements in the range [begin, end) visible to the GPU.
	/// No bounds checking.
	[[nodiscard]] bool flush_wrapping (ro_range<index_type> range) noexcept;
};


/// A ring buffer using wrap-around indices into sequential memory.
/// Separately tracks the range visible to the GPU to help with synchronization.
template <class Elem, class = std::enable_if_t<std::is_trivially_destructible_v<Elem>>>
class ring_buffer
{
public:
	using elem_type = Elem;

private:
	// Member variables wrapped in a struct for convenient assignment and reset.
	struct {
		index_type front     = 0;
		index_type back      = 0;
		index_type gpu_front = 0;
		index_type gpu_back  = 0;
		array<Elem> alloc;
	} self;

	/// Return the next index modulo allocation length.
	[[nodiscard]] constexpr index_type index_after (index_type idx) const noexcept;

public:
	[[nodiscard]] constexpr index_type front () const noexcept
	{
		return self.front;
	}

	[[nodiscard]] constexpr index_type back () const noexcept
	{
		return self.back;
	}

	[[nodiscard]] constexpr index_type gpu_back () const noexcept
	{
		return self.gpu_back;
	}

	[[nodiscard]] constexpr array<Elem> &alloc() noexcept {
		return self.alloc;
	}

	[[nodiscard]] constexpr const array<Elem> &alloc() const noexcept {
		return self.alloc;
	}

	[[nodiscard]] constexpr bool is_empty () const noexcept
	{
		return self.front == self.back;
	}

	/// Return a pointer to the first entry of the buffer or `nullptr` if it is empty.
	[[nodiscard]] constexpr elem_type *try_first () noexcept
	{
		return self.front == self.back ? nullptr : &self.alloc[self.front];
	}

	/// Return the range of elements that have been added on the CPU, but not yet flushed to the GPU.
	/// Uses absolute indices, meaning that `end` may be less than `begin` in case of a wrap-around.
	[[nodiscard]] constexpr ro_range<index_type> new_elems () noexcept {
		return {self.gpu_back, self.back};
	}


	/// Set the index onward from which elements may be used by the GPU.
	/// Elements before this index (and after `gpu_back`) are not in use by the GPU and may be overwritten.
	/// The caller is responsible for ensuring that all commands using these elements have actually completed.
	constexpr void set_gpu_front (index_type gpu_front) noexcept
	{
		self.gpu_front = gpu_front;
	}

	/// Note that memory for `capacity + 1` elements is allocated.
	/// Any previous state is discarded.
	[[nodiscard]] bool create (size_type capacity) noexcept
	{
		self = {};
		return self.alloc.create(capacity + 1);
	}

	/// Changes might not be visible to the GPU until `sync` is called.
	void push_back (const elem_type &elem);
	/// Changes might not be visible to the GPU until `sync` is called.
	template <class Iter>
	void push_back (ro_range<Iter> elems);

	/// Remove the first element of the buffer.
	/// Undefined behaviour if the buffer is empty.
	void pop_front () noexcept;

	/// Make changes visible to the GPU.
	[[nodiscard]] bool flush () noexcept;
};

}
