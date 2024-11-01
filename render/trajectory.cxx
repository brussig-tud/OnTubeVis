#include "render/trajectory.h"

#include "gpumem/memory_pool.inl"
#include "gpumem/ring_buffer.inl"
#include "render/state.h"


namespace otv {

trajectory::trajectory(id_type id, render_state &render)
	: _render {render}
	, _id     {id}
{}

gpumem::span<const float> trajectory::create_glyph_layer (
	layer_index_type layer_idx,
	glyph_size_type  glyph_size,
	glyph_count_type capacity
) {
	auto &layer {_layers[layer_idx]};

	// Ensure that the layer is not currently in use.
	assert(! layer.glyph_attribs.as_span().data());

	// Store the number of floats per glyph.
	_glyph_sizes[layer_idx] = glyph_size;

	// The ring buffer requires backing memory for one additional element, but its length in floats
	// must be a multiple of `glyph_size` to prevent glyphs from being split by wrap-around.
	// Therefore, request memmory for one additional glyph, minus the single float automatically
	// added by `ring_buffer`.
	layer.buffer_size = capacity + glyph_count_type{1};

	// Select the glyph memory allocator.
	layer.glyph_attribs = gpumem::ring_buffer<float, gpumem::memory_pool_ptr>{
			gpumem::memory_pool_ptr{_render.glyphs[layer_idx].attribs}};

	// Try to allocate memory for the layer's glyph attributes.
	if (! layer.glyph_attribs.create(glyph_to_attrib_count(layer_idx, layer.buffer_size) - 1)) {
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
	const auto prev_seg_idx {_last_segment_idx};

	// Add the segment to the GPU buffer, storing the index at which it is placed.
	_last_segment_idx = _render.segment_buffer.back();
	_render.segment_buffer.push_back({
		static_cast<unsigned>(prev_node_idx),
		static_cast<unsigned>(_last_node_idx)
	});

	// If this is the first segment on the trajectory, set it as the linked list's head.
	if (_first_segment_idx == nil) {
		_first_segment_idx = _last_segment_idx;
	}

	// Link the previous segment to the new one.
	if (prev_seg_idx != nil) {
		_render._next_segment[prev_seg_idx] = _last_segment_idx;
	}

	// The new segment has no successor yet.
	_render._next_segment[_last_segment_idx] = nil;

	// Mark the segment as belonging to the trajectory.
	_render.seg_to_traj[_last_segment_idx] = _id;

	// Store the segment's arclength parametrization at the corresponding index.
	_render.t_to_s[_last_segment_idx] = *t_to_s;

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		// Initialize the number of glyphs on the segment to zero.
		shared_layer.ranges[_last_segment_idx].n = glyph_count_type{0};

		// If this segment is the first one to be added after a glyph layer was created, that layer
		// may now add glyphs starting at the new segment.
		auto &layer {_layers[layer_idx]};

		if (layer.last_glyph_seg == nil) {
			layer.last_glyph_seg = _last_segment_idx;
		}

		// Queue an update so glyphs can appear on the new segment.
		_needs_glyph_update |= 1 << layer_idx;
	});
}

void trajectory::update_glyphs ()
{
	// Nothing to do if no layers changed.
	if (_needs_glyph_update == 0) {
		return;
	}

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		// Update only the layers that have changed.
		if (! (_needs_glyph_update & 1 << layer_idx)) {
			return;
		}
		auto &layer {_layers[layer_idx]};

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

		// Iterate over all segments in this trajectory, starting at the one holding the last glyph.
		auto seg_idx {layer.last_glyph_seg};

		while (seg_idx != nil) {
			auto &range {shared_layer.ranges[seg_idx]};

			// If there have not yet been any glyphs starting on the current segment, initialize its
			// range.
			if (seg_idx != layer.last_glyph_seg) {
				range = layer.next_segment_range;
			}

			// Retreive the arclength range of the current segment.
			const auto &t_to_s   {_render.t_to_s[seg_idx]};
			const auto seg_s_min {t_to_s[0]};
			const auto seg_s_max {t_to_s[15]};

			// Skip new glyphs that lie before the current segment, which can only occur for the
			// first segment.
			ro_range seg_attribs {layer.attrib_queue.begin(), layer.attrib_queue.begin()};

			if (seg_idx == layer.last_glyph_seg) while (true) {
				if (seg_attribs.begin == layer.attrib_queue.end()) {
					// All glyphs are too old, nothing more to do.
					layer.attrib_queue.flush();
					goto done;
				}

				// Calculate the glyph's front edge.
				const auto glyph_s_max {
					/* center */ *seg_attribs.begin
					/* radius */ + 0.5f * _render.glyph_length(layer_idx, &*seg_attribs.begin + 2)
				};

				// Found a glyph potentially on the segment.
				if (glyph_s_max >= seg_s_min) {
					break;
				}

				seg_attribs.begin += _glyph_sizes[layer_idx];
			}

			// Now that the start of the glyph range is known, find its end.
			// Count the number of glyphs along the way.
			seg_attribs.end = seg_attribs.begin;

			while (seg_attribs.end != layer.attrib_queue.end()) {
				float glyph_center {*seg_attribs.end};
				float glyph_radius {0.5f * _render.glyph_length(layer_idx, &*seg_attribs.end + 2)};

				// If the glyph lies fully beyond the current segment, the segment is complete.
				if (glyph_center - glyph_radius > seg_s_max) {
					break;
				}

				// Otherwise count the glyph.
				range.n += glyph_count_type{1};

				// If the glyph also overlaps the next segment, count it there as well.
				// Otherwise the next range is cleared after its previous use for the current
				// segment.
				if (glyph_center + glyph_radius > seg_s_max) {
					layer.next_segment_range.n += glyph_count_type{1};
				} else {
					layer.next_segment_range.n = glyph_count_type{0};
				}

				// Next glyph.
				seg_attribs.end += _glyph_sizes[layer_idx];
			}

			// Calculate the initial glyph range of this trajectory's next segment, consisting of
			// all glyphs shared with the current segment.
			layer.next_segment_range.i0  = range.end() - layer.next_segment_range.n;
			layer.next_segment_range.i0 -= layer.next_segment_range.i0 >= layer.buffer_size
				? layer.buffer_size
				: glyph_count_type{0};

			// Move attributes from queue to render buffer.
			layer.glyph_attribs.push_back(seg_attribs);
			layer.attrib_queue.pop(seg_attribs.length());

			// The new latest glyph is on the current segment.
			layer.last_glyph_seg = seg_idx;

			// If the queue is empty, we are done.
			if (layer.attrib_queue.length() == 0){
				layer.attrib_queue.flush();
				goto done;
			}

			// Move on to the next segment on this trajectory.
			seg_idx = _render._next_segment[seg_idx];
		}
		done:;

		// If all glyphs have been uploaded, no further update is required.
		if (layer.attrib_queue.length() == 0) {
			_needs_glyph_update ^= 1 << layer_idx;
		}
	});
}

void trajectory::on_delete_segment (gpumem::index_type seg_idx)
{
	// The trajectory now starts at the next segment.
	_first_segment_idx = _render._next_segment[seg_idx];

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		auto                          &layer   {_layers[layer_idx]};
		index_range<glyph_count_type> next_seg;

		// Reset indices when the last segment is deleted.
		if (_first_segment_idx == nil) {
			_last_segment_idx    = nil;
			layer.last_glyph_seg = nil;

			next_seg = layer.next_segment_range;
		} else {
			next_seg = shared_layer.ranges[_first_segment_idx];
		}

		// Discard all glyphs ending on the deleted segment, i.e. not overlapping onto the next one.
		layer.glyph_attribs.set_front(glyph_to_attrib_count(layer_idx, next_seg.i0));
	});
}

void trajectory::trim_glyphs ()
{
	// If there are no glyphs to be uploaded, there is no need to make space.
	if (_needs_glyph_update == 0) {
		return;
	}

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &shared_layer) {
		// Only check layers that actually have new glyphs ready to be uploaded.
		if (! (_needs_glyph_update & 1 << layer_idx)) {
			return;
		}

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

		// Remove the deleted glyphs from any segment ranges that reference them.
		bool found_oldest_glpyh {false};

		for (
			auto seg_idx {_first_segment_idx};
			seg_idx != nil;
			seg_idx = _render._next_segment[seg_idx]
		) {
			auto &range = shared_layer.ranges[seg_idx];

			// Calculate the index of the oldest remaining glyph relative to the segment, accounting
			// for wrap-around.
			const auto offset {oldest_glyph_idx >= range.i0
				? oldest_glyph_idx - range.i0
				: oldest_glyph_idx + layer.buffer_size - range.i0
			};

			if (offset < range.n) {
				// If the current segment contains the oldest glyph, remove all preceding glyphs
				// from its range.
				range.n           -= offset;
				range.i0          += offset;
				found_oldest_glpyh = true;

				// We are not done yet, removed glyphs could also overlap the next segment.
			} else {
				// Only wehen the oldest glyph was found and it does not overlap the current segment
				// are we done.
				if (found_oldest_glpyh) {
					return;
				}

				// If the oldest remaining glyph has not been found yet, the current segment must
				// be earlier, so all of its glyphs have been deleted.
				range.n = glyph_count_type{0};
			}
		}
	});
}

bool trajectory::flush_glyph_attribs ()
{
	auto ok {true};

	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		ok &= _layers[layer_idx].glyph_attribs.flush();
	});

	return ok;
}

void trajectory::on_frame_done ()
{
	_render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		auto &attribs {_layers[layer_idx].glyph_attribs};
		attribs.set_gpu_front(attribs.front());
	});
}

} // namespace otv
