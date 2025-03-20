
// C++ STL
#include <numeric>

// Local includes
#include "arclen_helper.h"
#include "render/extrapolation.h"
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
	// Glyph-related
	trajectories.clear();
	glyphs.glyph_attribs_arena.destroy();
	geom.ranges_arena.destroy();
	setup.reset();

	// Purely geometry-related
	geom.t_to_s_arena.destroy();
	geom.nodes_arena.destroy();
}



bool extrapolation_manager::create_geom_buffers (unsigned num_trajectories, unsigned num_segments)
{
	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer([this] {
		this->clear();
		std::cerr << "extrapolation_manager::create_geom_buffers(): failed to create GPU resources!"
		          << std::endl<<std::endl;
	});

	// Clean up previous contents, if any
	clear();

	// Create GPU ring buffer arenas
	const unsigned num_nodes = num_segments+1;
	const bool success =    geom.nodes_arena.create(num_trajectories, num_nodes)
	                     && geom.t_to_s_arena.create(num_trajectories, num_segments);

	// Distribute per-trajectory ringbuffers and create indices in the process
	trajectories.reserve(num_trajectories);
	geom.node_indices.reserve(num_trajectories * num_segments);
	for (unsigned t=0; t<num_trajectories; t++) {
		auto &nodes_buf = geom.nodes_arena.buffer(t); {
			const auto res = nodes_buf.push_uninit(num_nodes);
			assert(res.num_overwritten==0 && res.num_skipped==0);
		}
		auto &alen_buf = geom.t_to_s_arena.buffer(t); {
			const auto res = alen_buf.push_uninit(num_segments);
			assert(res.num_overwritten==0 && res.num_skipped==0);
		}
		trajectories.emplace_back(nodes_buf, alen_buf);
		uint32_t start_index = t*num_nodes;
		for (unsigned i=0; i<num_segments; i++)
			geom.node_indices.emplace_back(start_index+i, start_index+i+1);
	}

	// Done!
	return _.disarm(success);
}

void extrapolation_manager::reinit_layer_config (void)
{
	// Commit setup and find out actual number of float elements we need in our glyph attribute ring buffers. We need
	// to use the least-common multiple for the single-glyph attribute counts from each layer in order to be able to
	// advance the GPU ring buffers in a consistent way.
	setup.num_layers = render.visualizations[0].config.layer_configs.size();
	const auto &some_traj = render.trajectories.front();
	setup.glyph_attrib_counts.front() = some_traj.glyph_to_attrib_count(0, glyph_count_type{1});
	setup.glyph_attribs_lcm = setup.glyph_attrib_counts.front();
	for (unsigned l=1; l<setup.num_layers; l++) {
		setup.glyph_attrib_counts[l] = some_traj.glyph_to_attrib_count(l, glyph_count_type{1});
		setup.glyph_attribs_lcm = std::lcm(setup.glyph_attribs_lcm, setup.glyph_attrib_counts[l]);
	}
}

bool extrapolation_manager::create_glyph_and_per_layer_buffers (unsigned per_layer_min_glyphs_capacity)
{
	// Sanity checks
	assert(
		!geom.nodes_arena.ring_buffers.empty() &&
		"extrapolation_manager::create_glyph_and_per_layer_buffers() needs pre-created geometry buffers"
	);
	assert(
		geom.ranges_arena.ring_buffers.empty() &&
		"extrapolation_manager::create_glyph_and_per_layer_buffers() must not be called on a fully constructed setup"
	);

	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer([this] {
		geom.ranges_arena.destroy();
		glyphs.glyph_attribs_arena.destroy();
		std::fill(setup.glyph_attrib_counts.begin(), setup.glyph_attrib_counts.end(), 0);
		std::cerr << "extrapolation_manager::create_glyph_and_per_layer_buffers(): failed to create GPU resources!"
		          << std::endl<<std::endl;
	});

	// Create GPU ring buffer arenas
	// - glyph index mapping ranges
	const auto num_trajectories = geom.t_to_s_arena.num_ring_buffers();
	const auto num_segments = geom.t_to_s_arena.ring_buffer_capacity();
	const auto num_buffers = num_trajectories*setup.num_layers;
	bool success = geom.ranges_arena.create(num_buffers, num_segments);
	// - glyph instances
	const auto num_attribs = setup.glyph_attribs_lcm*per_layer_min_glyphs_capacity;
	success &= glyphs.glyph_attribs_arena.create(num_buffers, num_attribs);

	// Assign range ring buffers from arena to corresponding layer on each trajectory
	for (unsigned t=0; t<num_trajectories; t++) {
		auto &traj = trajectories[t];
		traj.layers.reserve(setup.num_layers);
		for (unsigned l=0; l<setup.num_layers; l++) {
			const unsigned per_layer_buffer_id = t*setup.num_layers + l;
			auto &ranges_buf = geom.ranges_arena.buffer(per_layer_buffer_id); {
				const auto res = ranges_buf.push_uninit(num_segments);
				assert(res.num_overwritten==0 && res.num_skipped==0);
			}
			traj.layers.emplace_back(
				ranges_buf, glyphs.glyph_attribs_arena.buffer(per_layer_buffer_id)
			);
		}
	}

	// Done!
	return _.disarm(success);
}

void extrapolation_manager::replace_extrapolation (
	unsigned traj_id, const node_attribs &last_measured_node, const std::vector<extrapol::node> &extrapolation
){
	// Retrieve trajectory
	auto &traj = trajectories[traj_id];

	// Replace geometry buffer contents
	const auto num_segments = traj.t_to_s.capacity();
	assert(num_segments == (unsigned)extrapolation.size());
	traj.nodes.contents[0] = last_measured_node;
	for (unsigned i=0; i<num_segments; i++) {
		traj.nodes.contents[i+1] = extrapolation[i].hnode;
		traj.t_to_s.contents[i] = extrapolation[i].t_to_s;
	}

	// Update per-layer range maps for current set of glyphs
	for (unsigned l=0; l<setup.num_layers; l++)
	{
		// Convenience shorthand
		auto &layer = traj.layers[l];

		/* --- DEBUG TEST ------------- */ /*{
			const unsigned glyph_attrib_count = setup.glyph_attrib_counts[l];
			layer.glyph_attribs.clear();
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-3][0]-.75f);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			unsigned num_glyphs = 1;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-3][0]-.5f);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-3][7]);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-3][15]);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-2][15]);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-1][6]);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-1][14]);
			for (unsigned i=1; i<setup.glyph_attrib_counts[l]; i++)
				layer.glyph_attribs.push_back(.0f);
			num_glyphs++;
			const unsigned max_glyphs = layer.glyph_attribs.capacity()/glyph_attrib_count;
			for (unsigned g=num_glyphs; g<max_glyphs; ++g) {
				layer.glyph_attribs.push_back(traj.t_to_s.contents[num_segments-1][15] + float(g)*.5f);
				for (unsigned i = 1; i < setup.glyph_attrib_counts[l]; i++)
					layer.glyph_attribs.push_back(.0f);
				num_glyphs++;
			}
			assert(num_glyphs == max_glyphs);
			const auto test_range = ro_range{layer.glyph_attribs.begin(), layer.glyph_attribs.end()};
			const auto test_range_len = test_range.length();
			assert(test_range_len == layer.glyph_attribs.size());
			assert(test_range_len % glyph_attrib_count == 0);
			assert(test_range_len == num_glyphs*glyph_attrib_count);
			for (unsigned i=0; i<test_range_len; i++) {
				const auto it = test_range.begin+i;
				std::clog << "glyph_attrib["<<it.idx<<"] = "<<*it << "\n";
			}
		}*/

		// Retrieve general per-layer information
		const unsigned stride = setup.glyph_attrib_counts[l];

		// First, find the oldest glyph we need to still include
		auto on_extrapol = skip_glyphs_before(
			l, extrapolation.front().t_to_s[0], ro_range{layer.glyph_attribs.begin(), layer.glyph_attribs.end()}
		);
		// ...and erase those that fell off the extrapolation
		if (on_extrapol.begin > layer.glyph_attribs.begin()) {
			const auto erase_range = ro_range{layer.glyph_attribs.begin(), on_extrapol.begin};
			const auto erase_count = on_extrapol.begin - layer.glyph_attribs.begin();
			assert(erase_range.length() == erase_count);
			const auto old_num = layer.glyph_attribs.size();
			layer.glyph_attribs.pop_front(erase_count);
			const auto new_num = layer.glyph_attribs.size();
			assert(new_num % stride == 0);
			assert(new_num == old_num-erase_count);
			// Refresh on_extrapol range since iterators are _theoretically_ invalidated by pop_front()
			// ToDo: investigate whether we can get away with not refreshing in all cases (it definitely works if the
			//       range was empty before)
			on_extrapol = ro_range{layer.glyph_attribs.begin(), layer.glyph_attribs.end()};
		}

		// Nothing to do if there are no in-range glyphs
		if (layer.glyph_attribs.empty())
			continue;

		// Assign remaining in-range glyphs
		assign_glyphs(
			ro_range{layer.ranges.begin(), layer.ranges.end()}, ro_range{traj.t_to_s.begin(), traj.t_to_s.end()},
			l, on_extrapol
		);
	}

	// Done!
	return;
}

template <class Iter>
ro_range<Iter> extrapolation_manager::consider_glyphs (
	unsigned traj_id, unsigned l, const ro_range<Iter> &glyph_attribs
){
	// Nothing to do if there are no glyphs
	if (glyph_attribs.is_empty())
		return glyph_attribs;

	// Convenience shorthands
	auto &traj = trajectories[traj_id];
	auto &layer = traj.layers[l];

	// First, retrieve general information about the layer
	const unsigned stride = setup.glyph_attrib_counts[l];
	const auto num_glyphs = render.trajectories[traj_id].attrib_to_glyph_count(
		l, glyph_attribs.length()
	).value;

	// Trim glyphs that precede the range covered by the extrapolation (this can happen due to network issues or if they
	// were submitted late by the client for whatever reason)
	const auto on_extrapol = skip_glyphs_before(l, traj.t_to_s.contents[0][0], glyph_attribs);
	assert(on_extrapol.end == glyph_attribs.end);

	// Add to displayed glyphs
	assert([&]{
		if (layer.glyph_attribs.empty())
			return true;
		// Sanity check
		const float old_max_s = get_glyph_pos(
			ro_range{layer.glyph_attribs.end()-stride, layer.glyph_attribs.end()}
		);
		const float first_glyph_s = get_glyph_pos(on_extrapol);
		return first_glyph_s >= old_max_s;
	}() && "extrapolation_manager::consider_glyphs(): new glyphs must never precede previously submitted glyphs!");

	//DEBUG TEST:
	layer.ranges.contents[0].i0 = 0;
	layer.ranges.contents[0].n = 4;
	layer.ranges.contents[1].i0 = 2;
	layer.ranges.contents[1].n = 6;
	layer.ranges.contents[2].i0 = 6;
	layer.ranges.contents[2].n = 5;

	layer.glyph_attribs.push_uninit(6);
	const auto discarded = layer.glyph_attribs.push_uninit(layer.glyph_attribs.capacity()+stride); // layer.glyph_attribs.push_range(on_extrapol)
	// make sure we don't include the number of discarded elements from the _input_ range in the count
	const int num_discarded = discarded.num_overwritten;
	assert(num_discarded <= discarded.total());
	assert(layer.glyph_attribs.size() % stride == 0); // sanity check

	// Unlink discarded glyphs, if any
	for (auto range_it=layer.ranges.begin(); range_it<layer.ranges.end(); ++range_it)
	{
		auto &range = *range_it;
		int num_affected = std::max(num_discarded-range.i0, 0);
		if (num_affected > 0) {
			// This segment now has some or all of its glyphs missing, which we can resolve solely on the basis of its
			// mapping range start index
			range.i0 = 0;
			num_affected = std::min<unsigned>(num_affected, range.n);
			range.n -= num_affected;
		}
		else
			// All glyphs mapped to this segment are still there, they just moved tailwards, so adjust start index
			// accordingly
			range.i0 -= num_discarded;
	}

	// Assign to segments
	assign_glyphs( // FIXME: start from most recently covered-by-glyphs segment
		ro_range{layer.ranges.begin(), layer.ranges.end()}, ro_range{traj.t_to_s.begin(), traj.t_to_s.end()},
		l, on_extrapol
	);

	// Done!
	return ro_range{on_extrapol.begin, glyph_attribs.end};
}
template ro_range<std_vector_float_iter> extrapolation_manager::consider_glyphs (
	unsigned, unsigned, const ro_range<std_vector_float_iter>&
);
template ro_range<std_deque_float_iter> extrapolation_manager::consider_glyphs (
	unsigned, unsigned, const ro_range<std_deque_float_iter>&
);
template ro_range<float*> extrapolation_manager::consider_glyphs (unsigned, unsigned, const ro_range<float*>&);

template <class Iter>
ro_range<Iter> extrapolation_manager::skip_glyphs_before (
	const unsigned layer, const float s_min, const ro_range<Iter> &glyph_attribs
){
	// Nothing to do if there are no glyphs
	if (glyph_attribs.is_empty())
		return glyph_attribs;

	// Init glyph cursor
	const unsigned stride = setup.glyph_attrib_counts[layer];
	ro_range cur_glyph {glyph_attribs.begin, glyph_attribs.begin+stride};
	assert(cur_glyph.end <= glyph_attribs.end);

	// Next, differentiate between glyph and plot layers (the required logic is slightly different)
	const auto glyph_geometry = render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Find first glyph that overlaps with the area behind the start of the extrapolation
		do {
			// Determine arc length range covered by glyph
			const auto s_range = render.glyph_range(layer, &*cur_glyph.begin).value();

			// Check if glyph is on the extrapolation
			if (s_range.y() >= s_min)
				// We found the first glyph that actually lies on the extrapolation
				break;

			// Glyph didn't pass the check, remove from consideration
			cur_glyph.safe_advance(stride, glyph_attribs.end);
			assert(cur_glyph.begin <= glyph_attribs.end);
		}
		while (cur_glyph.begin < glyph_attribs.end);
	}
	else // it's a plot
	{
		// Find first plot control point that would need to be included on the extrapolation.
		//
		// For this, we loop through all control points front to back, looking at the oldest _two_ points. We are
		// looking for either (a) two subsequent control points that bracket the start arc length of the first segment,
		// or (b) the very last control point we know about (which will then extend over all extrapolated segments).

		// Loop front to back until (a) or (b) is satisfied
		do {
			const float s = get_glyph_pos(cur_glyph);
			if (s < s_min)
			{
				// Check condition (b) first:
				if (cur_glyph.end == glyph_attribs.end)
					// We arrived at the last control point, and it still precedes the first extrapolated segment. The
					// search for the first control point to include is thus complete (we threw out all but the last
					// one).
					break;

				// Now check condition (a):
				const auto next_glyph = cur_glyph + stride;
				const float next_s = get_glyph_pos(next_glyph);
				if (next_s > s_min)
					// The current control point is the last one to precede the first extrapolated segment, so we must
					// include it for proper interpolation, having thrown out all that came before. The search for the
					// first control point to include is thus complete.
					break;
			}
			else
				// If we get here, the very first of the currently considered control points falls already within the
				// arc length range of the extrapolation. We stop right here and now, not throwing out any points.
				// (we might end up with some part of the beginning of the extrapolation not covered by the plot, which
				// would then have been the client's fault for not telling us to consider all potentially relevant
				// control points)
				break;

			// If we arrived here, then the current control point does not contribute to the first segment, so we throw
			// it out for good.
			cur_glyph.safe_advance(stride, glyph_attribs.end);
			assert(cur_glyph.end <= glyph_attribs.end);
		}
		while (true/*cur_glyph.begin < glyph_attribs.end*/);
	}

	// Done!
	return ro_range{cur_glyph.begin, glyph_attribs.end};
}
template ro_range<std_vector_float_iter> extrapolation_manager::skip_glyphs_before (
	unsigned layer, float s_min, const ro_range<std_vector_float_iter>&
);
template ro_range<std_deque_float_iter> extrapolation_manager::skip_glyphs_before (
	unsigned layer, float s_min, const ro_range<std_deque_float_iter>&
);
template ro_range<float*> extrapolation_manager::skip_glyphs_before (
	unsigned layer, float s_min, const ro_range<float*>&
);

template <class Iter>
ro_range<Iter> extrapolation_manager::keep_glyphs_after_including (
	const unsigned layer, const float s_min, const ro_range<Iter> &glyph_attribs
){
	// Nothing to do if there are no glyphs
	if (glyph_attribs.is_empty())
		return glyph_attribs;

	// Init glyph cursor
	const unsigned stride = setup.glyph_attrib_counts[layer];
	ro_range cur_glyph {glyph_attribs.end-stride, glyph_attribs.end};
	assert(cur_glyph.begin >= glyph_attribs.begin);

	// Next, differentiate between glyph and plot layers (the required logic is slightly different)
	const auto glyph_geometry = render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Find first glyph that doesn't overlap with the area behind the start of the extrapolation
		do {
			// Determine arc length range covered by glyph
			const auto s_range = render.glyph_range(layer, &*cur_glyph.begin).value();

			// Check if glyph is on the extrapolation
			if (s_range.y() < s_min)
				// We found the first glyph that lies completely outside the extrapolation, don't include it and all
				// earlier ones
				break;

			// Glyph passed the check, keep it
			cur_glyph.safe_retreat(stride, glyph_attribs.begin);
			assert(cur_glyph.end >= glyph_attribs.begin);
		}
		while (cur_glyph.end > glyph_attribs.begin);

		// We stopped at the first glyph completely preceding the extrapolation, so we have to advance forward one glyph
		// again to obtain the range of glyphs _on_ the extrapolation. If the very first glyph we considered already
		// precedes the extrapolation, then the curser will point at the first glyph _after_ the whole input range, so
		// the reported sub range of glyphs on the extrapolation will be empty.
		cur_glyph.safe_advance(stride, glyph_attribs.end);
	}
	else // it's a plot
	{
		// Find first plot control point that can be excluded from the extrapolation.
		//
		// For this, we loop through all control points back to front, looking at the last _two_ points. We are looking
		// for either (a) two subsequent control points that bracket the start arc length of the first segment,
		// or (b) the oldest control point we know about.

		// Loop front to back until (a) or (b) is satisfied
		do {
			const float s = get_glyph_pos(cur_glyph);
			if (s > s_min)
			{
				// Check condition (b) first:
				if (cur_glyph.begin == glyph_attribs.begin)
					// We arrived at the oldest control point, and it still doesn't precede the first extrapolated
					// segment. The search for the first control point to include is thus complete. This means part of
					// the beginning of the extrapolation will not be covered by the plot, but that would then have been
					// the client's fault for not submitting all relevant control points.
					break;

				// Now check condition (a):
				const auto prev_glyph = cur_glyph - stride;
				const float prev_s = get_glyph_pos(prev_glyph);
				if (prev_s <= s_min) {
					// The previous control point is the most recent one not situated on the extrapolated segment (or
					// exactly on the beginning), so we must include it for proper interpolation, but will throw out all
					// older ones.
					cur_glyph = prev_glyph;
					break;
				}
			}
			else {
				// If we get here, the newest of the currently considered control points falls already outside the arc
				// length range of the extrapolation. So we stop right here and now, keeping it to make sure it spreads
				// out over the whole extrapolation.
				break;
			}

			// If we arrived here, then the current control point does not contribute to the first segment, so we throw
			// it out for good.
			cur_glyph.safe_retreat(stride, glyph_attribs.begin);
			assert(cur_glyph.end >= glyph_attribs.begin);
		}
		while (true/*cur_glyph.end > glyph_attribs.begin*/);
	}

	// Done!
	return ro_range{cur_glyph.begin, glyph_attribs.end};
}
template ro_range<std_vector_float_iter> extrapolation_manager::keep_glyphs_after_including (
	unsigned layer, float s_min, const ro_range<std_vector_float_iter>&
);
template ro_range<std_deque_float_iter> extrapolation_manager::keep_glyphs_after_including (
	unsigned layer, float s_min, const ro_range<std_deque_float_iter>&
);
template ro_range<float*> extrapolation_manager::keep_glyphs_after_including (
	unsigned layer, float s_min, const ro_range<float*>&
);

template <class IRangeIter, class AlenIter, class GlyphAttribsIter>
void extrapolation_manager::assign_glyphs (
	ro_range<IRangeIter> &&ranges_out, const ro_range<AlenIter> &t_to_s, unsigned layer,
	const ro_range<GlyphAttribsIter> &glyph_attribs, const unsigned idx_offset
){
	// Prelude
	const unsigned num_segments = ranges_out.length();
	const auto num_glyphs = (unsigned)render.trajectories.front().attrib_to_glyph_count(
		layer, (int)glyph_attribs.length()
	).value + idx_offset;
	const auto stride = setup.glyph_attrib_counts[layer];
	ro_range cur_glyph {glyph_attribs.begin, glyph_attribs.begin+stride};
	unsigned cur_glyph_idx = idx_offset;


	/////
	// Iterate through input glyph attribute range to assign them to the extrapolated segments

	// We need to differentiate between glyph and plot layers.
	// TODO: The code in both cases looks very similar and can probably be templated – at least partially – to minimize
	// duplication.
	const auto glyph_geometry = render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Outer loop is over extrapolated segments
		for (unsigned seg=0; seg<num_segments; seg++)
		{
			// Obtain relevant quantities and references
			const auto s_max = (*(t_to_s.begin+seg))[15];
			auto &cur_range = *(ranges_out.begin+seg);
			cur_range.i0 = (int)cur_glyph_idx; // <- this can already be out-of-range, but it doesn't matter since
			cur_range.n = 0;                   //    in that case n will remain 0

			// Check if we have any glyphs remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over the glyphs that overlap the current segment
			auto s_range = render.glyph_range(layer, &*cur_glyph.begin).value(), s_range_prev = s_range;
			while (s_range.x() <= s_max)
			{
				cur_range.n++;
				cur_glyph_idx++;
				cur_glyph.safe_advance(stride, glyph_attribs.end);
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					break; // all glyphs exhausted
				s_range_prev = s_range;
				s_range = render.glyph_range(layer, &*cur_glyph.begin).value();
			}

			// Finally, move the cursor back one glyph (if it's not already the first) in case it also overlaps the
			// next segment.
			if (cur_glyph_idx > 0 && s_range_prev.y() >= s_max) {
				cur_glyph_idx--;
				cur_glyph.safe_unadvance(stride);
			}
		}
	}
	else // it's a plot
	{
		// Outer loop is over extrapolated segments
		for (unsigned seg=0; seg<num_segments; seg++)
		{
			// Obtain relevant quantities and references
			const auto s_max = (*(t_to_s.begin+seg))[15];
			auto &cur_range = *(ranges_out.begin+seg);
			cur_range.i0 = (int)cur_glyph_idx; // <- this can already be out-of-range, but it doesn't matter since
			cur_range.n = 0;                   //    in that case n will remain 0

			// Check if we have any control points remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over control points that fall _before_ the end of the segment
			auto s = get_glyph_pos(cur_glyph);
			while (s < s_max)
			{
				cur_range.n++;
				cur_glyph_idx++;
				cur_glyph.safe_advance(stride, glyph_attribs.end);
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
				cur_glyph.safe_unadvance(stride);
			}
		}
	}
}

bool extrapolation_manager::flush_changes (void) const {
	bool result = geom.nodes_arena.flush_all();
	result &= geom.t_to_s_arena.flush_all();
	result &= geom.ranges_arena.flush_all();
	result &= glyphs.glyph_attribs_arena.flush_all();
	return result;
}


} // namespace otv
