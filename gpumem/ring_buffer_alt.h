
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

template <class Elem>
struct ring_buffer_alt
{
	// This container does not construct or destroy its elements, so they should at least be trivially destructible to
	// avoid resource leaks.
	static_assert(std::is_trivially_destructible_v<Elem>);

	/// The iterator type
	struct iterator
	{
		const unsigned len;
		Elem *buf;
		unsigned idx, real_idx;

		iterator() = delete;
		iterator(const iterator &other) = default;

		inline explicit iterator (Elem *buf, const unsigned len, const unsigned idx, const unsigned real_idx)
			: len(len), buf(buf), idx(idx), real_idx(real_idx)
		{}

		inline iterator& operator=(const iterator &other) = default;

		inline Elem& operator* (void) {
			return buf[real_idx];
		}
		inline const Elem& operator* (void) const {
			return buf[real_idx];
		}

		inline bool operator== (const iterator &other) const {
			return idx == other.idx;
		}
		inline bool operator!= (const iterator &other) const {
			return idx != other.idx;
		}

		inline bool operator< (const iterator &other) const {
			return idx < other.idx;
		}
		inline bool operator<= (const iterator &other) const {
			return idx <= other.idx;
		}

		inline bool operator> (const iterator &other) const {
			return idx > other.idx;
		}
		inline bool operator>= (const iterator &other) const {
			return idx >= other.idx;
		}

		inline iterator& operator++ (void) {
			++idx;
			real_idx = (real_idx+1) % len;
			return *this;
		}
		[[nodiscard]] inline iterator operator++ (int) {
			const auto old{this};
			operator++();
			return old;
		}
		template <class Int>
		inline iterator& operator+= (const Int offset) {
			idx += offset;
			real_idx = (real_idx+offset) % len;
			return *this;
		}
		template <class Int>
		inline iterator operator+ (const Int offset) const {
			const auto new_idx = idx+offset;
			const auto new_real_idx = (real_idx+offset) % len;
			return iterator(buf, len, new_idx, new_real_idx);
		}

		inline iterator& operator-- (void) {
			--idx;
			real_idx = (real_idx-1) % len;
			return *this;
		}
		[[nodiscard]] inline iterator operator-- (int) {
			const auto old{this};
			operator--();
			return old;
		}
		template <class Int>
		inline iterator& operator-= (const Int offset) {
			idx -= offset;
			real_idx = (real_idx-offset) % len;
			return *this;
		}
		template <class Int>
		inline iterator operator- (const Int offset) const {
			const auto new_idx = idx-offset;
			const auto new_real_idx = (real_idx-offset) % len;
			return iterator(buf, len, new_idx, new_real_idx);
		}
		inline size_t operator- (const iterator &other) const {
			return idx - other.idx;
		}
	};

	/// The backing memory of ring the ring buffer metadata.
	span<ring_buffer_meta> meta_mem;

	/// The backing memory in which elements are placed.
	span<Elem> contents_mem;

	/// Reference to the in-memory struct of the ring buffer metadata.
	ring_buffer_meta &meta;

	/// Reference to the in-memory array of elements of the ring buffer.
	Elem *contents;

	/// Internal log of current number of elements. Required to distinguish full from empty in case head==tail.
	unsigned num_elems = 0;

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

	/// Return the current number of elements in the buffer.
	[[nodiscard]] unsigned size (void) const {
		return num_elems;
	}

	/// Return whether the ring buffer contains no elements at all.
	[[nodiscard]] inline bool empty (void) const {
		return !size();
	}

	/// Report the number of elements that can still be pushed into the ring buffer without dropping old elements.
	[[nodiscard]] unsigned num_free (void) const {
		return meta.len - num_elems;
	}

	/// Clear the ring buffer, choosing the indicated actual index @a head_tail as the new head and tail position.
	void clear (const unsigned head_tail=0) {
		meta.tail = meta.head = head_tail;
		update_num(false);
	}

	/// Push a single element into the ring buffer, reporting whether an old element had to be discarded to accommodate
	/// the new one.
	bool push_back (const Elem &elem)
	{
		// Determine whether we need to adjust the tail
		const bool drop_one = num_free()==0;

		// Copy in new element
		contents[meta.begin + meta.head] = elem;

		// Update buffer geometry
		meta.head = (meta.head+1) % meta.len;
		meta.tail = (meta.tail+drop_one) % meta.len;
		update_num(true);

		// Done!
		return drop_one;
	}

	/// Push elements into the ring buffer, reporting the number of old elements that had to be discarded to accommodate
	/// for the new ones. Notably, <b>this includes</b> elements from the tail end of the input range, if it was larger
	/// than the buffer's max capacity.
	template <class Iter>
	unsigned push_range (const ro_range<Iter> &new_elems)
	{
		// Don't do anything for empty input range
		const auto num_new_elems = new_elems.length();
		if (!num_new_elems)
			return 0; // nothing could have been overwritten

		// Infer amount of elements we actually need to copy
		const int old_free = num_free();
		const auto copy_count = std::min<unsigned>(meta.len, num_new_elems);

		// Special handling for full buffer overwrite, as that presents an opportunity to eliminate wrap-around
		if (copy_count == meta.len)
		{
			// Set up and perform full-buffer copy
			const auto copy_range = ro_range{new_elems.end-copy_count, new_elems.end};
			assert (copy_range.length() == copy_count);
			Elem *my_begin = contents+meta.begin;
			std::copy(copy_range.begin, copy_range.end, my_begin);

			// Update buffer geometry
			meta.tail = meta.head = 0;
			const auto old_num = num_elems;
			update_num(true);

			// Done! Report number of discarded elements
			return /*all old elements*/old_num  +  /*some of the new elements*/num_new_elems-copy_count;
		}
		assert(num_new_elems-copy_count == 0); // we completely handle this case above

		// Get a picture of the current buffer geometry
		const auto num_from_head = meta.len - meta.head;
		const auto buf_begin = contents + meta.begin;
		const auto buf_insert = buf_begin + meta.head;

		// Build ranges for copying
		const auto older_range_num = std::min(num_from_head, copy_count);
		const auto older_range = ro_range{new_elems.begin, new_elems.begin+older_range_num};
		const auto newer_range_num = copy_count - older_range_num;
		const auto newer_range = ro_range{older_range.end, older_range.end+newer_range_num};
		assert(older_range.length()+newer_range.length() == copy_count);

		// Copy, potentially with wrap-around (if both ranges are not empty)
		std::copy(older_range.begin, older_range.end, buf_insert);
		std::copy(newer_range.begin, newer_range.end, buf_begin);

		// Update buffer geometry
		const auto num_overwritten = std::max(int(copy_count) - int(old_free), 0);
		meta.head = (meta.head+copy_count) % meta.len;
		meta.tail = (meta.tail+num_overwritten) % meta.len;
		update_num(true);

		// Done! Report number of discarded elements
		return num_overwritten;
	}

	/// Pop given number of elements from the tail end of the buffer.
	void pop_front (const unsigned count=1) {
		assert(count <= size()); // sanity check
		meta.tail = (meta.tail+count) % meta.len; // just increase tail index accordingly
		update_num(false);
	}

	/// Return a forward iterator initially pointing to the tail element.
	inline iterator begin (void) {
		return iterator(contents+meta.begin, meta.len, 0, meta.tail);
	}
	/// Return a forward iterator initially pointing to the head (i.e. one after the last actually contained) element.
	inline iterator end (void) {
		return iterator(contents+meta.begin, meta.len, num_elems, meta.head);
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

	/// Re-calculate number of elements in the buffer based on current values of head and tail indices, and disambiguate
	/// the @c head==tail case via the provided hint whether the buffer should be considered @a full in such a case.
	///
	/// @note
	///		How to choose the disambiguation hint @a full will typically be clear from the context (it certainly is for
	///		all internal calls to this function). E.g. if we just added elements, then the buffer certainly can't be
	///		empty afterwards.
	inline void update_num (const bool full) {
		num_elems = meta.head == meta.tail ? full * meta.len
			: (meta.head > meta.tail ?
			   	  meta.head - meta.tail
			   	: /*newer range*/meta.head  +  /*older range*/meta.len-meta.tail
			);
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
		const bool success =    meta_memory.create(num_buffers, alignment_meta)
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
