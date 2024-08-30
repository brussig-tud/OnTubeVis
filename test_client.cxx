#include "test_client.h"

#include "gpumem/ring_buffer.inl"
#include "render/state.h"


namespace otv {

void test_client::update ()
{
	// Extend all trajectories based on the animation time, emulating streaming.
	for (auto &traj : trajectories) {
		// Add new nodes, and thereby new segments.
		for (
			auto &node_idx = traj.node_idcs.begin;
			node_idx < traj.node_idcs.end;
			++node_idx
		) {
			// Only add data up to the current playback time.
			if (data->timestamps[node_idx] > render.style.max_t) {
				break;
			}

			auto &render_traj {*render.try_get_trajectory(traj.id)};

			// The first node of each trajectory does not create a segment, so the index should not
			// be advanced.
			const cgv::mat4 *t_to_s {nullptr};

			if (! render_traj.is_empty()) {
				t_to_s = &arclen_data.t_to_s[traj.segment_idx];
				++traj.segment_idx;
			}

			// Append a node, potentially creating a new segment.
			auto col = data->colors[node_idx];
			render.enqueue_node(
				traj.id,
				{
					{data->positions[node_idx], data->radii[node_idx]},
					{col.R(), col.G(), col.B(), 1},
					data->tangents[node_idx],
					{data->timestamps[node_idx], 0, 0, 0}
				},
				t_to_s
			);

			// Add all glyphs on the next segment.
			render.foreach_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
				const auto data  {glyphs[layer_idx].attribs.data.begin()};
				const auto begin {glyphs[layer_idx].ranges[traj.segment_idx].i0};
				const auto end   {glyphs[layer_idx].ranges[traj.segment_idx].end()};

				render_traj.append_glyphs(layer_idx, ro_range{
					data + render_traj.glyph_to_attrib_count(layer_idx, begin),
					data + render_traj.glyph_to_attrib_count(layer_idx, end)
				});
			});
		}
	}
}

} // namespace otv
