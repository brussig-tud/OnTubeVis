
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

	/// The number of elements from @ref begin that make up the full capacity of the ring buffer inside the backing
	/// buffer.
	uint32_t len {0};

	/// Index of the newest element, relative to @ref begin .
	uint32_t head {0};

	/// Index of the oldest element, relative to @ref begin .
	uint32_t tail {0};
};

template <class Elem/*, class Alloc = buffer_alloc*/>
struct ring_buffer_alt
{
	// This container does not construct or destroy its elements, so they should at least be trivially destructible to
	// to avoid resource leaks.
	static_assert(std::is_trivially_destructible_v<Elem>);

	/// The backing memory of ring the ring buffer metadata.
	span<ring_buffer_meta> meta_mem;

	/// The backing memory in which elements are placed.
	span<Elem> contents_mem;

	/// Reference to the in-memory struct of the ring buffer metadata.
	ring_buffer_meta &meta;

	// Reference to the in-memory array of elements of the ring buffer.
	Elem *contents;

	/// Create the ring buffer using the given memory. While the internal metadata will be initialized on the host side,
	/// it will NOT be available for the GPU until @ref flush_meta is called at least once.
	[[nodiscard]] explicit ring_buffer_alt (const span<ring_buffer_meta> &meta_mem, const span<Elem> &contents_mem)
		: meta(*meta_mem.data())
	{
		this->meta_mem = meta_mem;
		this->contents_mem = contents_mem;
		contents = this->contents_mem.data();
		init_meta();
	}

	/// The Destructor. Actually, we don't need a custom destructor; this is just here so we can place debug breakpoints
	/// that trigger during destruction.
	~ring_buffer_alt (void) {
		std::clog.flush();
	}

	/// Report max number of elements that can be stored when at full capacity.
	[[nodiscard]] unsigned capacity (void) const {
		return meta.len;
	}

	/// Flush just the ring buffer meta data (i.e. after all elements in the buffer were dropped so there is no reason
	/// to change something in the contents)
	bool flush_meta (void) const {
		return meta_mem.flush();
	}

	/// Flush just the ring buffer contents (i.e. after all elements in the buffer were updated, but none were added or
	/// dropped off the end)
	bool flush_contents (void) const {
		return contents_mem.flush();
	}

	/// Flush both the whole ring buffer and its meta data.
	bool flush_all (void) const {
		bool result = flush_meta();
		result &= flush_contents();
		return result;
	}

private:
	void init_meta (void) {
		meta.begin = pointer2idx(contents_mem.as_range().begin);
		meta.len = contents_mem.length();
		meta.head = meta.tail = 0;
	}

	uint32_t pointer2idx (ring_buffer_meta* ptr) const {
		const auto buf_ptr_bytes = meta_mem.buffer().as_range().begin;
		const auto buf_ptr = reinterpret_cast<ring_buffer_meta*>(buf_ptr_bytes);
		const auto diff = ptr - buf_ptr;
		return (uint32_t)diff;
	}

	uint32_t pointer2idx (Elem* ptr) const {
		const auto buf_ptr_bytes = contents_mem.buffer().as_range().begin;
		const auto buf_ptr = reinterpret_cast<Elem*>(buf_ptr_bytes);
		const auto diff = ptr - buf_ptr;
		return (uint32_t)diff;
	}
};

template <class Elem, class Alloc=buffer_alloc>
struct ring_buffer_arena
{
	/// The GPU-side buffer serving as backing memory each ring buffer's metadata
	array<ring_buffer_meta, Alloc> meta_memory;

	/// The GPU-side buffer serving as backing memory for the actual ring buffer contents
	array<Elem, Alloc> data_memory;

	/// The list of ring buffers in the arena
	std::vector<ring_buffer_alt<Elem>> ring_buffers;

	// The destructor. We probably don't actually need it.
	~ring_buffer_arena (void) {
		destroy();
	}

	/// Create an arena with the given number of ring buffers of the specified capacity, observing the specified minimal
	/// alignment (i.e. the actual alignment can end up being higher).
	bool create (unsigned num_buffers, size_t max_capacity, size_t min_alignment = alignof(Elem))
	{
		// Make sure we don't leak resources when any single one of the steps fails
		auto _ = finalizer(std::bind(&ring_buffer_arena::destroy, this));

		// Clean up previous contents, if any
		_.f(); // re-use finalizer to perform cleanup of GPU memory arenas

		// Enforce sane alignment
		#define MINIMUM_ALIGNMENT size_t(64)
		constexpr size_t alignment_meta = find_lcm(alignof(ring_buffer_meta), MINIMUM_ALIGNMENT);
		const size_t alignment_data = find_lcm(min_alignment, MINIMUM_ALIGNMENT);
		if (alignment_data > MINIMUM_ALIGNMENT*4)
			std::cerr << "\n!!! WARNING !!! - very large GPU ring buffer alignment: "<<alignment_data<<" bytes"
			          << std::endl;

		// Determine size of data buffer
		const size_t data_buffer_size = num_buffers * max_capacity;

		// Create memory pools
		const bool success
		         = meta_memory.create(num_buffers, alignment_meta)
		        && data_memory.create(data_buffer_size, alignment_data);

		// Create ring buffers
		ring_buffers.reserve(num_buffers);
		for (unsigned i=0; i<num_buffers; i++) {
			const span<ring_buffer_meta> meta_span{
				meta_memory.data()+i, 1, meta_memory.handle()
			};
			const unsigned ringbuffer_offset = i*max_capacity;
			const span<Elem> data_span{
				data_memory.data()+ringbuffer_offset, (unsigned)max_capacity, data_memory.handle()
			};
			ring_buffers.emplace_back(meta_span, data_span);
		}

		// Done!
		return _.disarm(success);
	}

	/// Completely reset the arena and free all dynamically allocated GPU and CPU memory. No destructors of elements
	/// in the ring buffers will be called (as they should be trivially destructible by contract, see @ref
	/// gpumem::ring_buffer_alt ).
	void destroy (void) {
		ring_buffers.clear();
		data_memory.destroy();
		meta_memory.destroy();
	}

	/// Report the number of ring buffers in the arena
	[[nodiscard]] unsigned num_ring_buffers (void) const {
		return (unsigned)ring_buffers.size();
	}

	/// Report the capacity of the individual ring buffers in the arena.
	[[nodiscard]] unsigned ring_buffer_capacity (void) const {
		return ring_buffers.front().capacity();
	}

	/// Obtain a reference to the i-th buffer.
	[[nodiscard]] ring_buffer_alt<Elem>& buffer (unsigned i) {
		return ring_buffers[i];
	}

	/// Flush all memory in the arena.
	bool flush_all (void) const {
		bool result = meta_memory.flush();
		result &= data_memory.flush();
		return result;
	}
};

}

#endif // ifndef __PERSISTENT_BUFFER_H__
