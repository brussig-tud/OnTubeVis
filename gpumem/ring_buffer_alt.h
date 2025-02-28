
#ifndef __PERSISTENT_BUFFER_H__
#define __PERSISTENT_BUFFER_H__


//////
//
// Includes
//

// Local includes
#include "gpumem/alloc.h"


namespace otv::gpumem {

struct ring_buffer_meta
{
	/// Index of the start element in the backing buffer.
	uint32_t begin {0};

	/// Index of the newest element, relative to @ref begin .
	uint32_t head {0};

	/// Index of the oldest element, relative to @ref begin .
	uint32_t tail {0};
};

template <class Elem, class Alloc=buffer_alloc>
struct ring_buffer_arena
{
	/*/// The type of the contained elements.
	using elem_type = Elem;

	/// The type of the used allocators.
	using alloc_type = Alloc;*/

	/// The GPU-side buffer holding each ring buffer's meta data
	array<ring_buffer_meta, Alloc> meta_memory;

	/// The GPU-side buffer holding the actual ring buffer contents
	array<Elem, Alloc> data_memory;

	bool create (unsigned num_buffers, size_t max_capacity, size_t alignment = alignof(Elem))
	{
		// Make sure we don't leak resources when any single one of the steps fails
		auto _ = finalizer([this] {
			this->data_memory.destroy();
			this->meta_memory.destroy();
		});

		// Enforce sane alignment
		#define MINIMUM_ALIGNMENT 64
		constexpr size_t alignment_multiple_meta = alignof(ring_buffer_meta)/MINIMUM_ALIGNMENT;
		constexpr size_t alignment_meta = std::max(
			alignment_multiple_meta*MINIMUM_ALIGNMENT, size_t(MINIMUM_ALIGNMENT)
		);
		const size_t alignment_multple_data = alignment/MINIMUM_ALIGNMENT;
		alignment = std::max(alignment_multple_data*MINIMUM_ALIGNMENT, size_t(MINIMUM_ALIGNMENT));

		// Determine size of data buffer
		const size_t data_buffer_size = num_buffers * max_capacity;

		// Create memory
		bool success = meta_memory.create(num_buffers, alignment_meta);
		success &= data_memory.create(data_buffer_size, alignment);

		// Done!
		return _.disarm(success);
	}

	void destroy (void) {
		meta_memory.destroy();
		data_memory.destroy();
	}
};

template <class Elem/*, class Alloc = buffer_alloc*/>
struct ring_buffer_alt
{
	// This container does not construct or destroy its elements, so they should at least be trivially destructible to
	// to avoid resource leaks.
	static_assert(std::is_trivially_destructible_v<Elem>);

	/// The type of the contained elements.
	using elem_type = Elem;

	/// The type of the allocator used to manage the backing memory.
	/*using alloc_type = Alloc;*/

	// Indices are wrapped in a struct for convenient collective assignment and reset.
	ring_buffer_meta &indices;

	/// The backing memory in which elements are placed.
	span<Elem> mem;

	/// Allocate new backing memory large enough for the buffer to hold `capacity` entries.
	/// The buffer will initially be empty.
	/// Previously contained elements are deallocated without being destroyed.
	/// For implementation reasons, the length of the new backing memory is at least `capacity + 1`.
	bool create (array<Elem> &backing_mem, size_type capacity) {
		indices = {};
		const size_t alignment = std::min(alignof(Elem), size_t(64));
		mem = backing_mem.alloc(capacity, alignment);
	}

	/// Explicitly release the backing memory and leave the ring buffer in an unusable state (until @ref create is
	/// called again). Note that no element destructors will be executed, as elements, by contract, are assumed to be
	/// trivially destructible.
	void destroy (void) {
		indices = {};
		mem.dealloc();
		mem.destroy();
	}

	/// Clears the buffer contents. Note that no element destructors will be executed, as elements, by contract, are
	/// assumed to be trivially destructible.
	void clear (void ) {
		indices = {};
	}
};

}

#endif // ifndef __PERSISTENT_BUFFER_H__
