#pragma once

#include <cgv/math/fmat.h>

#include "common.h"
#include "dbuf_queue.h"
#include "gpumem/memory_pool.h"
#include "gpumem/ring_buffer.h"


namespace otv {

struct render_state;

/// Manages the render data for a single trajectory.
class trajectory {
public:
	using id_type = uint;


	/// Create a new trajectory.
	trajectory(id_type id, render_state &render);


	/// Return this trajectory's ID.
	[[nodiscard]] constexpr id_type id () const noexcept
	{
		return _id;
	}

	/// Return the number of 32-bit floats used to define each glyph per layer.
	[[nodiscard]] constexpr const per_layer<glyph_size_type> &glyph_sizes () const noexcept
	{
		return _glyph_sizes;
	}

	[[nodiscard]] constexpr gpumem::index_type last_node_idx () const noexcept
	{
		return _last_node_idx;
	}

	[[nodiscard]] constexpr bool is_empty () const noexcept
	{
		return _last_node_idx == nil;
	}

	/// Calculate the number of glyphs defined by a given number of attributes on a given layer,
	/// rounding down.
	[[nodiscard]] constexpr glyph_count_type attrib_to_glyph_count (
		layer_index_type            layer,
		glyph_count_type::base_type num_attribs
	) const noexcept {
		return glyph_count_type{num_attribs / _glyph_sizes[layer]};
	}

	/// Calculate the number of attributes required to define a given number of glyphs on a
	/// given layer.
	[[nodiscard]] constexpr glyph_count_type::base_type glyph_to_attrib_count (
		layer_index_type layer,
		glyph_count_type num_glyphs
	) const noexcept {
		return num_glyphs.value * _glyph_sizes[layer];
	}


	/// Initialize a glyph layer to hold up to `capacity` glyphs of `glyph_size` floats each.
	/// Return a read-only view of the allocated memory.
	[[nodiscard]] gpumem::span<const float> create_glyph_layer (
		layer_index_type layer,
		glyph_size_type  glyph_size,
		glyph_count_type capacity
	);

	/// Extend the trajectory by one node at the end.
	/// A new segment is created between the new node and the previous one, with the arc length
	/// parametrization `t_to_s`.
	/// If the trajectory is empty, no segment is created and `t_to_s` is ignored.
	void append_node (const node_attribs &node, const cgv::mat4 *t_to_s);

	/// Copy glyph attributes to a host-side buffer, from whence they will be added to the render
	/// buffer on the next frame.
	template <class Iter>
	void enqueue_glyphs (layer_index_type layer, ro_range<Iter> glyphs)
	{
		// Check that only complete glyphs are given.
		assert(glyphs.length() % _glyph_sizes[layer] == 0);

		// Copy attributes to host buffer.
		_layers[layer].attrib_queue.push_back(glyphs);
	}

	/// Add glyph attibutes from the queue to the render buffer.
	void append_glyphs ();

	/// Forget about the latest `num_glyphs` glyphs, allowing the memory they occupy to be
	/// reused once all draw calls using them are complete.
	/// Users are responsible for ensuring that the dropped glyphs are not referenced by any
	/// glyph ranges.
	void drop_glyphs (layer_index_type layer, glyph_count_type num_glyphs)
	{
		_layers[layer].glyph_attribs.pop_front(glyph_to_attrib_count(layer, num_glyphs));
	}

	/// Logically delete old glyphs from the render buffer to make room for the new glyphs currently
	/// in the glyphs attibute queue's read buffer.
	/// WARNING: Must not be called during deferred shading.
	void trim_glyphs ();

	/// Must be called when and only when the last node on this trajectory is removed from the
	/// render buffer to reset internal state.
	/// Callers are responsible for ensuring that this precondition is met.
	void mark_empty () noexcept;

	/// Synchronize newly added glyphs with the GPU.
	[[nodiscard]] bool flush_glyph_attribs ();

	/// Update GPU sync guard indices once rendering has finished.
	void frame_completed ();

private:
	struct layer_data {
		/// Compiled glyphs shown on the trajectory.
		gpumem::ring_buffer<float, gpumem::memory_pool_ptr> glyph_attribs;
		/// Glyph attributes received from the client that have not been entered into the render
		/// buffer yet.
		dbuf_queue<float> attrib_queue;
		/// Absolute index of the newest segment containing a glyph on this layer.
		gpumem::index_type last_glyph_seg {nil};
	};


	/// Index value indicating the lack of an object.
	static constexpr gpumem::index_type nil {-1};


	/// Data specific to each glyph layer.
	per_layer<layer_data> _layers;
	/// Rendering data independent of any specific trajectories.
	render_state &_render;
	/// The absolute index of the last entry in the node buffer belonging to this trajectory.
	gpumem::index_type _last_node_idx {nil};
	/// Uniquely identifies this trajectory.
	id_type _id;
	/// The number of 32-bit float values used to define one glyph instance on each layer.
	per_layer<glyph_size_type> _glyph_sizes;
};

} // namespace otv
