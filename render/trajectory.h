#pragma once

#include <cgv/math/fmat.h>

#include "common.h"
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
		return _last_node_idx == nil_node;
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

	/// Extend the trajectory by one node at the end, potentially creating a new segment.
	void append_node (const node_attribs &node, const cgv::mat4 *t_to_s);

	/// Add glyphs past the end of the trajectory that will appear on future segments.
	template <class Iter>
	void append_glyphs (layer_index_type layer, ro_range<Iter> data)
	{
		_glyph_attribs[layer].push_back(data);
	}

	/// Forget about the latest `num_glyphs` glyphs, allowing the memory they occupy to be
	/// reused once all draw calls using them are complete.
	/// Users are responsible for ensuring that the dropped glyphs are not referenced by any
	/// glyph ranges.
	void drop_glyphs (layer_index_type layer, glyph_count_type num_glyphs)
	{
		_glyph_attribs[layer].pop_front(glyph_to_attrib_count(layer, num_glyphs));
	}

	/// Allow the oldest glyphs to be overwritten, so that at least `_render.reserve_glyphs`
	/// new glyphs can be added.
	/// Must not be called during deferred shading.
	void trim_glyphs ();

	void mark_empty () noexcept
	{
		_last_node_idx = nil_node;
	}

	/// Synchronize newly added glyphs with the GPU.
	[[nodiscard]] bool flush_glyph_attribs ();

	/// Update GPU sync guard indices once rendering has finished.
	void frame_completed ();

private:
	// friend class on_tube_vis;

	/// Index value indicating the lack of a node.
	static constexpr gpumem::index_type nil_node {-1};

	/// Rendering data independent of any specific trajectories.
	render_state &_render;
	/// Compiled glyphs shown on the trajectory.
	per_layer<gpumem::ring_buffer<float, gpumem::memory_pool_ptr>> _glyph_attribs;
	/// Indices of the first glyph in each layer to be included on the next segment.
	per_layer<gpumem::index_type> _first_attribs_on_seg;
	/// The absolute index of the last entry in the node buffer belonging to this trajectory.
	gpumem::index_type _last_node_idx {nil_node};
	/// Uniquely identifies this trajectory.
	id_type _id;
	/// The number of 32-bit float values used to define one glyph instance on each layer.
	per_layer<glyph_size_type> _glyph_sizes;
};

} // namespace otv
