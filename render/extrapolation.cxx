
// local includes
#include "arclen_helper.h"
#include "render/extrapolation.h"

#include <numeric>

#include "state/core.h"


namespace otv
{
namespace extrapol
{

void compute_path (
	std::vector<extrapol::node> &out, const unsigned num, const node_attribs &ref_node0,
	const node_attribs &ref_node1, const cgv::mat4 &ref_t_to_s
){
	// Compute extrapolation parameters
	const float r = ref_node0.pos_rad.w();
	const auto p1 = cgv::vec3(ref_node1.pos_rad);
	const auto m1 = cgv::vec3(ref_node1.tangent);
	const auto m1_len = m1.length();
	const float t1 = ref_node1.t.x();
	const float dt = t1 - ref_node0.t.x();

	// Extrapolate
	const extrapol::node prev_extrapol_values = {
		ref_node1, ref_t_to_s
	};
	const extrapol::node *prev_extrapol = &prev_extrapol_values;
	for (unsigned i=1; i<=num; i++)
	{
		const node_attribs new_node {
			cgv::vec4(p1 + float(i)*m1, r), ref_node1.color, cgv::vec4(m1, 0),
			cgv::vec4(t1 + float(i)*dt, 0, 0, 0)
		};
		const extrapol::node new_extrapol {
			new_node, arclen::single_linear_t_to_s(m1_len, /*sigma: */ prev_extrapol->t_to_s[15])
		};
		prev_extrapol = &out.emplace_back(new_extrapol);
	}
}

} // namespace extrapol

void extrapolation_manager::clear(void)
{
	trajectories.clear();
	glyphs.glyph_attribs_arena.destroy();
	geom.ranges_arena.destroy();
	std::fill(setup.glyph_attrib_counts.begin(), setup.glyph_attrib_counts.end(), 0);
	geom.t_to_s_arena.destroy();
	geom.nodes_arena.destroy();
}



bool extrapolation_manager::create_geom_buffers (unsigned num_trajectories, unsigned num_segments)
{
	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer(std::bind(&extrapolation_manager::clear, this));

	// Clean up previous contents, if any
	_.f(); // re-use finalizer to perform cleanup of GPU memory arenas

	// Create GPU ring buffer arenas
	const unsigned num_nodes = num_segments+1;
	const bool success
	         = geom.nodes_arena.create(num_trajectories, num_nodes)
	        && geom.t_to_s_arena.create(num_trajectories, num_segments);

	// Distribute per-trajectory ringbuffers and create indices in the process
	trajectories.reserve(num_trajectories);
	geom.node_indices.reserve(num_trajectories * num_segments);
	for (unsigned t=0; t<num_trajectories; t++) {
		trajectories.emplace_back(geom.nodes_arena.buffer(t), geom.t_to_s_arena.buffer(t));
		uint32_t start_index = t*num_nodes;
		for (unsigned i=0; i<num_segments; i++)
			geom.node_indices.emplace_back(start_index+i, start_index+i+1);
	}

	// Done!
	return _.disarm(success);
}

bool extrapolation_manager::create_glyph_and_per_layer_buffers (unsigned per_layer_min_glyphs_capacity)
{
	// Sanity checks
	assert(
		!geom.nodes_arena.ring_buffers.empty() &&
		"extrapolation_manager: create_glyph_and_per_layer_buffers() needs pre-created geometry buffers"
	);
	assert(
		geom.ranges_arena.ring_buffers.empty() &&
		"extrapolation_manager: create_glyph_and_per_layer_buffers() must not be called on a fully constructed setup"
	);

	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer([this] {
		geom.ranges_arena.destroy();
		glyphs.glyph_attribs_arena.destroy();
		std::fill(setup.glyph_attrib_counts.begin(), setup.glyph_attrib_counts.end(), 0);
	});

	// Commit setup and find out actual number of float elements we need in our glyph attribute ring buffers. We need
	// to use the least-common multiple for the single-glyph attribute counts from each layer in order to be able to
	// advance the GPU ring buffers in a consistent way.
	const unsigned num_layers = render.visualizations[0].config.layer_configs.size();
	const auto &some_traj = render.trajectories.front();
	setup.glyph_attrib_counts.front() = some_traj.glyph_to_attrib_count(0, glyph_count_type{1});
	unsigned glyph_attribs_lcm = setup.glyph_attrib_counts.front();
	for (unsigned l=1; l<num_layers; l++) {
		setup.glyph_attrib_counts[l] = some_traj.glyph_to_attrib_count(l, glyph_count_type{1});
		glyph_attribs_lcm = std::lcm(glyph_attribs_lcm, setup.glyph_attrib_counts[l]);
	}

	// Create GPU ring buffer arenas
	// - glyph index mapping ranges
	const auto num_trajectories = geom.t_to_s_arena.num_ring_buffers();
	const auto num_segments = geom.t_to_s_arena.ring_buffer_capacity();
	const auto num_buffers = num_trajectories*num_layers;
	bool success = geom.ranges_arena.create(num_buffers, num_segments);
	// - glyph instances
	const auto num_attribs = glyph_attribs_lcm*per_layer_min_glyphs_capacity;
	success &= glyphs.glyph_attribs_arena.create(num_buffers, num_attribs);

	// Assign range ring buffers from arena to corresponding layer on each trajectory
	for (unsigned t=0; t<num_trajectories; t++) {
		auto &traj = trajectories[t];
		traj.layers.reserve(num_layers);
		for (unsigned l=0; l<num_layers; l++) {
			const unsigned per_layer_buffer_id = t*num_layers + l;
			traj.layers.emplace_back(
				geom.ranges_arena.buffer(per_layer_buffer_id),
				glyphs.glyph_attribs_arena.buffer(per_layer_buffer_id)
			);
		}
	}

	// Done!
	return _.disarm(success);
}

void extrapolation_manager::replace_extrapolation (
	unsigned traj_id, const node_attribs &last_measured_node, const std::vector<extrapol::node> &extrapolation
){
	#define DEBUG_OUTPUT 0

	// Retrieve trajectory
	auto &traj = trajectories[traj_id];

	// Replace geometry buffer contents
	#if DEBUG_OUTPUT
		std::clog << "### UPDATING EXTRAPOLATED NODES ##############################\n";
	#endif
	const auto num_segments = traj.t_to_s.capacity();
	assert(num_segments == (unsigned)extrapolation.size());
	traj.nodes.contents[0] = last_measured_node;
	for (unsigned i=0; i<num_segments; i++) {
		traj.nodes.contents[i+1] = extrapolation[i].hnode;
		traj.t_to_s.contents[i] = extrapolation[i].t_to_s;
	}

	// Update per-layer range maps for current set of glyphs
	const auto num_layers = (unsigned)traj.layers.size();
	for (unsigned l=0; l<num_layers; l++)
	{
		// Convenience shorthand
		auto &layer = traj.layers[l];

		// ------------- DEBUG TESET ---
		/*layer.glyphs_deque.clear();
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-3][0]-.75f);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-3][0]-.001f);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-3][7]);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-3][15]);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-2][15]);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-1][6]);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);
		layer.glyphs_deque.emplace_back(traj.t_to_s.contents[num_segments-1][14]);
		for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
			layer.glyphs_deque.emplace_back(0);*/
		// --- END ---------------------

		// Nothing to do if there are no glyphs here
		if (layer.glyphs_deque.empty())
			continue;

		// First, retrieve general per-layer information
		const unsigned stride = setup.glyph_attrib_counts[l];

		// Next, differentiate between glyph and plot layers
		// TODO: The code in both cases looks very similar and can probably be templated to avoid duplication
		const auto glyph_geometry = render.glyph_extents(l, &*(layer.glyphs_deque.begin()+2));
		if (glyph_geometry.has_value()) // it's a glyph
		{
			////
			// 1 - Eliminate all but the first relevant glyph

			const auto &alen = traj.t_to_s.contents[0]; // arc length of first segment
			ro_range cur_glyph {layer.glyphs_deque.begin(), layer.glyphs_deque.begin()+stride};
			assert(cur_glyph.end <= layer.glyphs_deque.end());

			do {
				// Determine arc length range covered by glyph
				const auto s_range = render.glyph_range(l, &*cur_glyph.begin).value();

				// Check if glyph is on the extrapolation
				if (s_range.y() >= alen[0])
					// We found the first glyph that actually lies on the extrapolation
					break;

				// Glyph didn't pass the check, remove
				layer.glyphs_deque.erase(cur_glyph.begin, cur_glyph.end);
				cur_glyph = {layer.glyphs_deque.begin(), layer.glyphs_deque.begin()+stride};
				assert(cur_glyph.end <= layer.glyphs_deque.end());
			}
			while (!layer.glyphs_deque.empty());


			////
			// 2 - Iterate through remaining gylphs to assign them to the extrapolated segments

			// Outer loop is over extrapolated segments
			const auto num_glyphs = (unsigned)render.trajectories.front().attrib_to_glyph_count(
				l, (int)layer.glyphs_deque.size()
			).value;
			unsigned cur_glyph_idx = 0;
			for (unsigned seg=0; seg<num_segments; seg++)
			{
				// Obtain relevant quantities and references
				const auto &alen = traj.t_to_s.contents[seg];
				const auto s_max = alen[15];
				auto &cur_range = layer.ranges.contents[seg];
				cur_range.i0 = (int)cur_glyph_idx; // <- this can already be out-of-range, but it doesn't matter since
				cur_range.n = 0;                   //    in that case n will remain 0

				// Check if we have any glyphs remaining
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					// While we're actually fully done, we need to ensure the remaining segments get initialized too
					continue;

				// Inner loop is over the glyphs that overlap the current segment
				auto s_range = render.glyph_range(l, &*cur_glyph.begin).value(), s_range_prev = s_range;
				while (s_range.x() <= s_max) {
					cur_range.n++;
					cur_glyph_idx++;
					cur_glyph += stride;
					assert(cur_glyph_idx <= num_glyphs);
					if (cur_glyph_idx == num_glyphs)
						break; // all glyphs exhausted
					s_range_prev = s_range;
					s_range = render.glyph_range(l, &*cur_glyph.begin).value();
				}

				// Finally, move the cursor back one glyph (if it's not already the first) in case it also overlaps the
				// next segment.
				if (cur_glyph_idx > 0 && s_range_prev.y() >= s_max) {
					cur_glyph_idx--;
					cur_glyph -= stride;
				}
			}
		}
		else // it's a plot
		{
			////
			// 1 - Eliminate all but the first relevant plot control point

			// For this, we loop through all glyphs front to back, looking at the oldest _two_ points. We are looking
			// for either (a) two subsequent control points that bracket the start arc length of the first segment, or
			// (b) the very last control point we know about (which will then extend over all extrapolated segments).

			// Loop front to back until (a) or (b) is satisfied
			const auto &alen = traj.t_to_s.contents[0]; // arc length of first segment
			ro_range cur_glyph {layer.glyphs_deque.begin(), layer.glyphs_deque.begin()+stride};
			assert(cur_glyph.end <= layer.glyphs_deque.end());
			do {
				const float s = get_glyph_pos(cur_glyph);
				if (s < alen[0])
				{
					// Check condition (b) first:
					if (cur_glyph.end == layer.glyphs_deque.end())
						// We arrived at the last control point, and it still precedes the first extrapolated segment.
						// The search for the first control point to include is thus complete (we threw out all but the
						// last one).
						break;

					// Now check condition (a):
					const auto next_glyph = cur_glyph + stride;
					const float next_s = get_glyph_pos(next_glyph);
					if (next_s > alen[0])
						// The current control point is the last one to precede the first extrapolated segment, so we
						// must include it for proper interpolation, having thrown out all that came before. The search
						// for the first control point to include is thus complete.
						break;
				}
				else
					// If we get here, the very first of the currently considered control points falls already within
					// the arc length range of the extrapolation. We stop right here and now, not throwing out any.
					// (we might end up with some part of the beginning of the extrapolation not covered by the plot,
					// which would then have been the client's fault for not telling us to consider all potentially
					// relevant control points)
					break;

				// If we arrived here, then the current control point does not contribute to the first segment, so
				// we throw it out for good.
				layer.glyphs_deque.erase(cur_glyph.begin, cur_glyph.end);
				cur_glyph = {layer.glyphs_deque.begin(), layer.glyphs_deque.begin()+stride};
				assert(cur_glyph.end <= layer.glyphs_deque.end());
			}
			while (true/*cur_glyph.begin < layer.glyphs_deque.end()*/);


			////
			// 2 - Iterate through remaining control points to assign them to the extrapolated segments

			// Outer loop is over extrapolated segments
			const auto num_glyphs = (unsigned)render.trajectories.front().attrib_to_glyph_count(
				l, (int)layer.glyphs_deque.size()
			).value;
			unsigned cur_glyph_idx = 0;
			for (unsigned seg=0; seg<num_segments; seg++)
			{
				// Obtain relevant quantities and references
				const auto &alen = traj.t_to_s.contents[seg];
				const auto s_max = alen[15];
				auto &cur_range = layer.ranges.contents[seg];
				cur_range.i0 = (int)cur_glyph_idx; // <- this can already be out-of-range, but it doesn't matter since
				cur_range.n = 0;                   //    in that case n will remain 0

				// Check if we have any control points remaining
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					// While we're actually fully done, we need to ensure the remaining segments get initialized too
					continue;

				// Inner loop is over control points that fall _before_ the end of the segment
				auto s = get_glyph_pos(cur_glyph);
				while (s < s_max) {
					cur_range.n++;
					cur_glyph_idx++;
					cur_glyph += stride;
					assert(cur_glyph_idx <= num_glyphs);
					if (cur_glyph_idx == num_glyphs)
						break; // all glyphs exhausted
					s = get_glyph_pos(cur_glyph);
				}

				// Now, include one additional (if it exists) control point to ensure correct inter-segment
				// interpolation on the _current_ segment.
				if (cur_glyph_idx < num_glyphs)
					cur_range.n++;

				// Finally, move the cursor back one control point (if it's not already the first) to make sure the last
				// control point considered to be _on_ the current segment will be included in the next segment also, to
				// ensure correct inter-segment interpolation on the _next_ segment. In case the current control point
				// lies _exactly_ on the segment boundary (2nd condition), this is not necessary.
				if (cur_glyph_idx > 0 && s!=s_max) {
					cur_glyph_idx--;
					cur_glyph -= stride;
				}
			}
		}
	}

	// Flush nodes buffer
	//const auto flush_result = traj.nodes.contents_mem.flush();
	#if DEBUG_OUTPUT
		for (unsigned i=0; i<num; i++) {
			std::clog << "Elem["<<i<<"]:\n"
			          << " .pos_rad = "<<traj.nodes.contents[i].pos_rad<<"\n"
			          << " .color = "<<traj.nodes.contents[i].color<<"\n"
			          << " .tangent = "<<traj.nodes.contents[i].tangent<<"\n"
			          << " .t = "<<traj.nodes.contents[i].t<<"\n";
		}
		//std::clog << "flush_result: "<<flush_result << std::endl;
		//std::clog.flush();
	#endif
}

bool extrapolation_manager::flush_changes(void) const {
	bool result = geom.nodes_arena.flush_all();
	result &= geom.t_to_s_arena.flush_all();
	result &= geom.ranges_arena.flush_all();
	result &= glyphs.glyph_attribs_arena.flush_all();
	return result;
}


} // namespace otv
