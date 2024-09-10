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
	for (layer_index_type i {0}; i < max_glyph_layers; ++i) {
		_layers[i].glyph_attribs = gpumem::memory_pool_ptr{_render.glyphs[i].attribs};
	}
}

gpumem::span<const float> trajectory::create_glyph_layer (
	layer_index_type layer_idx,
	glyph_size_type  glyph_size,
	glyph_count_type capacity
) {
	_glyph_sizes[layer_idx] = glyph_size;

	// The ring buffer requires backing memory for one additional element, but its length in floats
	// must be a multiple of `glyph_size` to prevent glyphs from being split by wrap-around.
	// Therefore, request memmory for one additional glyph, minus the single float automatically
	// added by `ring_buffer`.
	capacity += glyph_count_type{1};

	auto &layer {_layers[layer_idx]};

	// Try to allocate memory for the layer's glyph attributes.
	if (! layer.glyph_attribs.create(glyph_to_attrib_count(layer_idx, capacity) - 1)) {
		return {};
	}

	return layer.glyph_attribs.as_span().as_const();
}

void trajectory::append_node (const node_attribs &node, const cgv::mat4 *t_to_s)
{
	const auto prev_node_idx {_last_node_idx};

	// Add the node to the GPU buffer, storing the index at which it is placed.
	_last_node_idx = _render.node_buffer.back();
	_render.node_buffer.push_back(node);

	// If this node is the first one on the trajectory, there is nothing more to do.
	if (prev_node_idx == nil) {
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

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		// Initialize the number of glyphs on the segment to zero.
		shared_layer.ranges[new_seg_idx].n = glyph_count_type{0};

		// If this segment is the first one to be added after a glyph layer was created, that layer
		// may now add glyphs starting at the new segment.
		auto &layer {_layers[layer_idx]};

		if (layer.last_glyph_seg == nil) {
			layer.last_glyph_seg = new_seg_idx;
		}
	});
}

void trajectory::append_glyphs ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		auto &layer {_layers[layer_idx]};

		// If there are no new attributes, all that has to be done is to prepare the queue for the
		// next frame.
		if (layer.attrib_queue.length() == 0) {
			layer.attrib_queue.flush();
			return;
		}

		// If there is no segment yet, keep buffering glpyhs until a segment is created.
		auto seg_idx {layer.last_glyph_seg};

		if (seg_idx == nil) {
			return;
		}

		// Check whether the GPU buffer can fit all glyphs in the read buffer.
		auto const capacity {layer.glyph_attribs.free_capacity()};

		if (capacity < layer.attrib_queue.length()) {
			// If not, discard the oldest nodes that don't fit.
			// Since `trim_glyphs` attempts to make room for all glyphs in the buffer, this can only
			// happen if there are more glyphs at once than the entire buffer can hold.
			assert(
				layer.attrib_queue.length() > layer.glyph_attribs.capacity()
				&& layer.glyph_attribs.is_empty()
			);
			layer.attrib_queue.pop(layer.attrib_queue.length() - capacity);
		}

		// Iterate over all segments, starting at the one holding the last glyph.
		auto seg_to_traj {_render.seg_to_traj.as_span().wrapping_iterator(seg_idx)};

		while (true) {
			// Retreive the arclength range of the current segment.
			const auto &t_to_s    {_render.t_to_s[seg_idx]};
			const auto min_arclen {t_to_s[0]};
			const auto max_arclen {t_to_s[15]};

			// Skip glyphs that lie before the current segment.
			auto first_glyph {layer.attrib_queue.begin()};

			while (true) {
				if (first_glyph == layer.attrib_queue.end()) {
					// All glyphs are too old, nothing more to do.
					layer.attrib_queue.flush();
					return;
				} else if (*first_glyph >= min_arclen) {
					// Found a glyph on the segment.
					break;
				}

				first_glyph += _glyph_sizes[layer_idx];
			}

			// Now that the first glyph on the segment is known, find the last one as well.
			// Count the number of glyphs alongside.
			ro_range         seg_attribs {first_glyph, first_glyph};
			glyph_count_type num_glyphs  {0};

			while (seg_attribs.end != layer.attrib_queue.end() && *seg_attribs.end <= max_arclen) {
				seg_attribs.end += _glyph_sizes[layer_idx];
				num_glyphs += glyph_count_type{1};
			}

			// Update the glyph range for the current segment.
			auto &range {shared_layer.ranges[seg_idx]};

			if (range.n == glyph_count_type{0}) {
				range.i0 = attrib_to_glyph_count(layer_idx, layer.glyph_attribs.back());
			}

			range.n += num_glyphs;

			// Move attributes from queue to render buffer.
			layer.glyph_attribs.push_back(seg_attribs);
			layer.attrib_queue.pop(seg_attribs.length());

			// The new latest glyph is on the current segment.
			layer.last_glyph_seg = seg_idx;

			// If the queue is empty, we are done.
			if (layer.attrib_queue.length() == 0){
				layer.attrib_queue.flush();
				break;
			}

			// Otherwise find the next segment belonging to this trajectory.
			while (true) {
				++seg_to_traj;

				if (seg_to_traj.index() == _render.segment_buffer.back()) {
					// All segments have been checked, the remaining glyphs will be placed on future
					// segments.
					return;
				} else if (*seg_to_traj == _id) {
					break;
				}
			}

			seg_idx = seg_to_traj.index();
		}
	});
}

void trajectory::trim_glyphs ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		auto &layer {_layers[layer_idx]};

		// For wrap-around to work, the attribute buffer has capacity for one additional glyph minus
		// one attribute, which is automatically added by the ring buffer implementation.
		// This partial padding glyph is subtracted so that all calculations are done for complete
		// glyphs.
		auto free_capacity {
			layer.glyph_attribs.free_capacity() - _glyph_sizes[layer_idx] + 1
		};
		const auto target_capacity {std::min(
			static_cast<gpumem::size_type>(layer.attrib_queue.length()),
			layer.glyph_attribs.capacity() - _glyph_sizes[layer_idx] + 1
		)};
		assert(free_capacity >= 0 && target_capacity >= 0);

		// Nothing to do if the remaining capacity is already sufficient.
		if (free_capacity >= target_capacity) {
			return;
		}

		// Logically delete the glyphs.
		layer.glyph_attribs.pop_front(target_capacity - free_capacity);

		// Calculate the index of the oldest remaining glyph.
		const auto oldest_glyph_idx {attrib_to_glyph_count(layer_idx, layer.glyph_attribs.front())};
		const auto buffer_size {
			attrib_to_glyph_count(layer_idx, layer.glyph_attribs.as_span().length())
		};

		// Remove the deleted glyphs from any segment ranges that reference them.
		for (
			auto segment {_render.seg_to_traj.wrapping_iterator(
				_render.segment_buffer.front()
			)};
			&*segment != &_render.seg_to_traj[_render.segment_buffer.back()];
			++segment
		) {
			// Only consider segments belonging to this trajectory.
			if (*segment != _id) {
				continue;
			}

			auto &range = shared_layer.ranges[segment.index()];

			// Calculate the index of the oldest remaining glyph relative to the segment, accounting
			// for wrap-around.
			const auto offset {oldest_glyph_idx >= range.i0
				? oldest_glyph_idx - range.i0
				: oldest_glyph_idx + buffer_size - range.i0
			};

			if (offset < range.n) {
				// If the current segment contains the oldest glyph, remove all preceding glyphs
				// from its range.
				range.i0 += offset;
				range.n  -= offset;
				break;
			} else {
				// If the oldest remaining glyph has not been found yet, the current segment must
				// be earlier, so all of its glyphs have been deleted.
				range.n = glyph_count_type{0};
			}
		}
	});
}

void trajectory::mark_empty () noexcept
{
	_last_node_idx = nil;

	for (auto &layer : _layers) {
		layer.last_glyph_seg = nil;
	}
}

bool trajectory::flush_glyph_attribs ()
{
	auto ok {true};

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		ok &= _layers[layer_idx].glyph_attribs.flush();
	});

	return ok;
}

void trajectory::frame_completed ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		auto &attribs {_layers[layer_idx].glyph_attribs};
		attribs.set_gpu_front(attribs.front());
	});
}

} // namespace otv
