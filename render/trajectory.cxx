#include "render/trajectory.h"

#include "gpumem/memory_pool.inl"
#include "gpumem/ring_buffer.inl"
#include "render/state.h"


namespace otv {

trajectory::trajectory(id_type id, render_state &render)
	: _render {render}
	, _id     {id}
{
	// All glyph layers are initially inactive with no attribute memory.
	// If a layer is activated later on (`create_glyph_layer`), an attribute buffer is allocated
	// from that layer's pool.
	for (layer_index_type layer {0}; layer < max_glyph_layers; ++layer) {
		_glyph_attribs[layer]        = gpumem::memory_pool_ptr{_render.glyphs[layer].attribs};
		_first_attribs_on_seg[layer] = 0;
	}
}

gpumem::span<const float> trajectory::create_glyph_layer (
	layer_index_type layer,
	glyph_size_type  glyph_size,
	glyph_count_type capacity
) {
	_glyph_sizes[layer] = glyph_size;

	// The ring buffer requires backing memory for one additional element, but its length in floats
	// must be a multiple of `glyph_size` to prevent glyphs from being split by wrap-around.
	// Therefore, request memmory for one additional glyph, minus the single float automatically
	// added by `ring_buffer`.
	++capacity.value;

	if (! _glyph_attribs[layer].create(glyph_to_attrib_count(layer, capacity) - 1))  {
		return {};
	}

	// Store the beginning of the first range.
	_first_attribs_on_seg[layer] = _glyph_attribs[layer].back();
	return _glyph_attribs[layer].as_span().as_const();
}

void trajectory::append_node (const node_attribs &node, const cgv::mat4 *t_to_s)
{
	const auto prev_node_idx {_last_node_idx};

	// Add the node to the GPU buffer, storing the index at which it is placed.
	_last_node_idx = _render.node_buffer.back();
	_render.node_buffer.push_back(node);

	// If this node is the first one on the trajectory, there is nothing more to do.
	if (prev_node_idx == nil_node) {
		return;
	}

	// Otherwise, create a new segment between the previous node and the new one.
	auto new_seg_idx = _render.segment_buffer.back();
	_render.segment_buffer.push_back({
		static_cast<unsigned>(prev_node_idx),
		static_cast<unsigned>(_last_node_idx)
	});

	// Mark the segment as belonging to the trajectory.
	_render.seg_to_traj[new_seg_idx] = _id;

	// Store the segment's arclength parametrization at the corresponding index.
	_render.t_to_s[new_seg_idx] = *t_to_s;

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		const auto &attribs {_glyph_attribs[layer_idx]};

		// Set the range of glyphs for the newly created segment.
		layer.ranges[new_seg_idx] = {
			attrib_to_glyph_count(layer_idx, _first_attribs_on_seg[layer_idx]),
			attrib_to_glyph_count(layer_idx, attribs.distance(
				_first_attribs_on_seg[layer_idx],
				attribs.back()
			))
		};

		// The next segment will continue where this one ends.
		_first_attribs_on_seg[layer_idx] = attribs.back();
	});
}

void trajectory::trim_glyphs ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		auto       &attribs      {_glyph_attribs[layer_idx]};
		const auto capacity      {attribs.as_span().length() - _glyph_sizes[layer_idx]};
		const auto free_capacity {capacity - attribs.length()};

		auto reserve_attribs = std::min(_render.reserve_glyph_attribs, capacity);

		// If the glyph buffer's remaining capacity is lower than configured, delete as many glyphs
		// as necessary and update glyph ranges to no longer reference them.
		if (free_capacity < reserve_attribs) {
			const auto attribs_to_delete {reserve_attribs - free_capacity};
			attribs.pop_front(attribs_to_delete);

			auto glyphs_to_delete {attrib_to_glyph_count(layer_idx, attribs_to_delete)};

			for (
				auto segment {_render.seg_to_traj.wrapping_iterator(
					_render.segment_buffer.front()
				)};
				&*segment != &_render.seg_to_traj[_render.segment_buffer.back()];
				++segment
			) {
				// Find a segment belonging to this trajectory.
				if (*segment == _id) {
					auto &range = layer.ranges[segment.index()];

					// If the segment contains fewer glyphs than are to be deleted, remove all
					// glyphs from the segment and search for the next one.
					if (range.n.value < glyphs_to_delete.value) {
						glyphs_to_delete.value -= range.n.value;
						range.n                 = glyph_count_type{0};
					}
					// Otherwise remove only as many glyphs as required from the front of the
					// segment and exit the loop.
					else {
						range.i0.value += glyphs_to_delete.value;
						range.n.value  -= glyphs_to_delete.value;
						break;
					}
				}
			}
		}
	});
}

bool trajectory::flush_glyph_attribs ()
{
	auto ok {true};

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		ok &= _glyph_attribs[layer_idx].flush();
	});

	return ok;
}

void trajectory::frame_completed ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		_glyph_attribs[layer_idx].set_gpu_front(_glyph_attribs[layer_idx].front());
	});
}

} // namespace otv
