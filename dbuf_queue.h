#pragma once

#include <vector>

#include "util.h"


/// A queue implemented through double-buffering with two dynamic arrays, one for reading and one
/// for writing.
template <class Elem>
class dbuf_queue {
private:
	/// Buffer storing newly appended elements.
	/// Entries may only be added, not removed.
	std::vector<Elem> write_buffer;
	/// Buffer storing visible, usable elements.
	/// May not be modified.
	std::vector<Elem> read_buffer;
	/// The number of elements in `read_buffer`, starting from the front, that are considered
	/// removed from the queue.
	std::size_t read_offset {0};

public:
	/// Return the number of visible elements in the read buffer.
	[[nodiscard]] constexpr std::size_t length () const
	{
		return read_buffer.size() - read_offset;
	}

	/// Construct an iterator at the first visible element.
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
	/// It will not appear in the read buffer until `flush` has been called.
	void push_back (const Elem &elem)
	{
		write_buffer.push_back(elem);
	}

	/// Append a range of elements to the write buffer.
	/// They will not appear in the read buffer until `flush` has been called.
	template <class Iter>
	void push_back (const ro_range<Iter> elems)
	{
		write_buffer.insert(write_buffer.end(), elems.begin, elems.end);
	}

	/// Remove a number of elements from the start of the read buffer.
	/// The elements are not actually destroyed until `flush` is called.
	void pop (std::size_t num_elems = 1)
	{
		read_offset = std::min(read_offset + num_elems, read_buffer.size());
	}

	/// Clear the read buffer, then swap it with the write buffer.
	/// Elements previously enqueued can now be read.
	void flush ()
	{
		read_buffer.clear();
		std::swap(read_buffer, write_buffer);
		read_offset = 0;
	}
};
