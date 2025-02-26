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
	using id_type = unsigned int;


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
	/// If the layer is in use already it must first be freed by `destroy_glyph_layer`.
	/// Return a read-only view of the allocated memory.
	[[nodiscard]] gpumem::span<const float> create_glyph_layer (
		layer_index_type layer,
		glyph_size_type  glyph_size,
		glyph_count_type capacity
	);

	/// Release all resources associated with a glyph layer.
	void destroy_glyph_layer (layer_index_type idx)
	{
		_layers[idx] = {};
	}

	/// Extend the trajectory by one node at the end.
	/// A new segment is created between the new node and the previous one, with the arc length
	/// parametrization `t_to_s`.
	/// If the trajectory is empty, no segment is created and `t_to_s` is ignored.
	void append_node (const node_attribs &node, const cgv::mat4 *t_to_s);

	/// report the current logical arclength of the whole trajectory. @a Logical here means that this value represents
	/// the arclength since the very first node that was ever inserted, even if this node has long since been popped
	/// off the tail end of the buffer.
	float arclength (void) const;

	/// Copy glyph attributes to a host-side buffer, from whence they will be added to the render
	/// buffer on the next frame.
	template <class Iter>
	void enqueue_glyphs (layer_index_type layer, ro_range<Iter> glyphs)
	{
		// Check that only complete glyphs are given.
		assert(glyphs.length() % _glyph_sizes[layer] == 0);

		auto &queue {_layers[layer].attrib_queue};

		// If the queue was empty before, schedule an update to upload the new data.
		// If there are glyphs queued already, we have to wait for new geometry.
		_needs_glyph_update |= (queue.length() == 0) << layer;

		// Copy attributes to host buffer.
		queue.push_back(glyphs);
	}

	/// Add newly visible glyph attibutes from the host queue to the render buffer.
	void update_glyphs ();

	/// Must be called when a node belonging to this trajectory is removed from the render buffer.
	void on_delete_node (gpumem::index_type node_idx) noexcept
	{
		if (node_idx == _last_node_idx) {
			_last_node_idx = nil;
		}
	}

	/// Must be called when the geometry of a segment belonging to this trajectory is deleted.
	/// Frees all glyphs on the segment.
	void on_delete_segment (gpumem::index_type seg_idx);

	/// Logically delete old glyphs from the render buffer to make room for the new glyphs currently
	/// in the glyphs attibute queue's read buffer.
	/// WARNING: Must not be called during deferred shading.
	void trim_glyphs ();

	/// Synchronize newly added glyphs with the GPU.
	[[nodiscard]] bool flush_glyph_attribs ();

	/// Update GPU sync guard indices once rendering has finished.
	void on_frame_done ();

private:
	struct layer_data {
		/// Compiled glyphs shown on the trajectory.
		gpumem::ring_buffer<float, gpumem::memory_pool_ptr> glyph_attribs {};
		/// Glyph attributes received from the client that have not been entered into the render
		/// buffer yet.
		dbuf_queue<float> attrib_queue;
		/// Absolute index of the segment glyphs are currently being placed on.
		gpumem::index_type current_segment {nil};
		/// The initial range of glyphs on the segment that will next have glyphs added, including
		/// potential overlap (i.e. glyphs on multiple segments).
		index_range<glyph_count_type> next_segment_range {};
		/// The size, not capacity, of this layer's glyph buffer.
		glyph_count_type buffer_size;
		/// Indicates whether `current_segment` has changed since the last call to `update_glyphs`.
		bool segment_is_new;
	};


	/// Index value indicating the lack of an object.
	static constexpr gpumem::index_type nil {-1};


	/// Data specific to each glyph layer.
	per_layer<layer_data> _layers;
	/// Rendering data independent of any specific trajectories.
	render_state &_render;
	/// The absolute index of the last entry in the node buffer belonging to this trajectory.
	gpumem::index_type _last_node_idx {nil};
	/// Absolute index of the first, i.e. oldest, entry in the segment buffer belonging to this
	/// trajectory.
	gpumem::index_type _first_segment_idx {nil};
	/// Absolute index of the last entry in the segment buffer belonging to this trajectory.
	gpumem::index_type _last_segment_idx {nil};
	/// Uniquely identifies this trajectory.
	id_type _id;
	/// The number of 32-bit float values used to define one glyph instance on each layer.
	per_layer<glyph_size_type> _glyph_sizes;
	/// Bits indicate whether a layer has new potentially visible glyphs that should be uploaded to
	/// the GPU.
	uint8_t _needs_glyph_update {0};
	static_assert(std::numeric_limits<decltype(_needs_glyph_update)>::digits >= max_glyph_layers);
};

} // namespace otv
