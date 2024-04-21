#pragma once

// C++ STL
#include <type_traits>

// CGV framework
#include <cgv_gl/gl/gl.h>


/// Non-generic base class shared between all ring buffer instantiations.
struct ring_buffer_base
{
	using size_type  = GLsizeiptr;
	using index_type = GLintptr;
};

/// A ring buffer using GPU-accessible memory.
/// NOTE: `Elem` must be trivially copyable.
/// This requirement is not enforced since `cgv::math::fvec` could be trivially copyable, but is not implemented as such.
template <class Elem, class = std::enable_if_t<std::is_trivially_destructible_v<Elem>>>
class ring_buffer final : ring_buffer_base
{
public:
	using elem_type = Elem;

private:
	// Member variables wrapped in a struct for convenient assignment.
	struct {
		elem_type *data      = nullptr;
		size_type alloc_len  = 0;
		index_type front     = 0;
		index_type back      = 0;
		index_type gpu_front = 0;
		index_type gpu_back  = 0;
		GLuint handle        = 0;
	} self;

	/// Return the next index module allocation length.
	[[nodiscard]] constexpr index_type index_after (index_type idx) const noexcept;

public:
	[[nodiscard]] constexpr ring_buffer() noexcept = default;

	ring_buffer(const ring_buffer&) = delete;
	[[nodiscard]] constexpr ring_buffer(ring_buffer&&) noexcept;

	ring_buffer &operator=(const ring_buffer &src) = delete;
	ring_buffer &operator=(ring_buffer &&src) noexcept;

	~ring_buffer() noexcept;


	[[nodiscard]] constexpr GLuint handle () const noexcept
	{
		return self.handle;
	}

	[[nodiscard]] constexpr elem_type *data () const noexcept
	{
		return self.data;
	}

	[[nodiscard]] constexpr size_type alloc_len () const noexcept
	{
		return self.alloc_len;
	}

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

	[[nodiscard]] constexpr bool is_empty () const noexcept
	{
		return self.front == self.back;
	}

	/// Return a pointer to the first entry of the buffer or `nullptr` if it is empty.
	[[nodiscard]] constexpr elem_type *try_first () const noexcept
	{
		return self.front == self.back ? nullptr : self.data + self.front;
	}

	/// Set the index onward from which elements may be used by the GPU.
	/// Elements before this index (and after `gpu_back`) are not in use by the GPU and may be overwritten.
	/// The caller is responsible for ensuring that all commands using these elements have actually completed.
	constexpr void set_gpu_front (index_type gpu_front) noexcept
	{
		self.gpu_front = gpu_front;
	}

	
	/// Note that memory for `capacity + 1` elements is allocated.
	/// Any previous data is discarded.
	[[nodiscard]] bool create (size_type capacity) noexcept;

	/// Changes might not be visible to the GPU until `sync` is called.
	void push_back (const elem_type &elem);
	/// Changes might not be visible to the GPU until `sync` is called.
	void push_back (const elem_type *elems, size_type num_elems);

	/// Remove the first element of the buffer.
	/// Undefined behaviour if the buffer is empty.
	void pop_front () noexcept;

	/// Make changes visible to the GPU.
	[[nodiscard]] bool flush () noexcept;
};
