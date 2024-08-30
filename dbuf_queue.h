#pragma once

#include <vector>


/// A queue implemented through double-buffering with two dynamic arrays, one for reading and one
/// for writing.
template <class Elem>
class dbuf_queue {
private:
	/// Buffer storing newly appended entries.
	/// Elements may only be added, not removed.
	std::vector<Elem> write_buffer;
	/// Buffer storing entries that may be read.
	/// May not be modified.
	std::vector<Elem> read_buffer;
	/// The number of entries in `read_buffer` that have already been popped from the queue.
	std::size_t read_offset {0};

public:
	/// Return the number of unread elements in the read buffer.
	[[nodiscard]] constexpr std::size_t length () const
	{
		return read_buffer.size() - read_offset;
	}

	/// Construct an iterator at the first unread element.
	[[nodiscard]] constexpr auto begin ()
	{
		return read_buffer.begin() + read_offset;
	}

	/// Construct an iterator past the last element in the read buffer.
	[[nodiscard]] constexpr auto end ()
	{
		return read_buffer.end();
	}

	/// Append an element to the write buffer.
	/// It will not appear in the read buffer until that buffer has been cleared, either by popping
	/// all of its elements or by manually calling `clear`:
	void push_back (const Elem &elem)
	{
		write_buffer.push_back(elem);
	}

	/// Mark a number of elements as read.
	/// Popped elements are not destroyed until the entire content of the read buffer has been
	/// marked as read.
	void pop (std::size_t num_elems = 1)
	{
		read_offset += num_elems;

		if (read_offset >= read_buffer.size()) {
			clear();
		}
	}

	/// Clear the read buffer, then swap it with the write buffer.
	/// Elements previously enqueued can now be read.
	void clear ()
	{
		read_buffer.clear();
		std::swap(read_buffer, write_buffer);
		read_offset = 0;
	}
};
