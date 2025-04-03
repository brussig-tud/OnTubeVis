
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
	for (unsigned l=0; l<4; ++l) {
		glyphs.glyph_attribs_arena[l].destroy();
		glyphs.ranges_arena[l].destroy();
	}
	glyphs.traj_glyph_mem_arena.destroy();
	setup.reset();

	// Purely geometry-related
	geom.seg_to_traj_arena.destroy();
	geom.t_to_s_arena.destroy();
	geom.nodes_arena.destroy();
	geom.node_indices.clear();
	//geom.segment_indices.clear();
}



bool extrapolation_manager::create_geom_buffers (
	cgv::render::context &ctx, unsigned num_trajectories, unsigned num_segments
){
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
	                     && geom.t_to_s_arena.create(num_trajectories, num_segments)
	                     && geom.seg_to_traj_arena.create(num_trajectories, num_segments);

	// Distribute per-trajectory ringbuffers and create indices in the process
	trajectories.reserve(num_trajectories);
	geom.node_indices.reserve(num_trajectories * num_segments);
	std::vector<unsigned> segment_indices;
	/*geom.*/segment_indices.reserve(geom.node_indices.size());
	for (unsigned idx=-1, t=0; t<num_trajectories; t++)
	{
		// Resize nodes buffer
		auto &nodes_buf = geom.nodes_arena.buffer(t); {
			const auto res = nodes_buf.push_uninit(num_nodes);
			assert(res.num_overwritten==0 && res.num_skipped==0);
		}
		// Resize t_to_s buffer
		auto &alen_buf = geom.t_to_s_arena.buffer(t); {
			const auto res = alen_buf.push_uninit(num_segments);
			assert(res.num_overwritten==0 && res.num_skipped==0);
		}
		// Set segment-to-trajectory mapping buffer
		auto &seg_to_traj_buf = geom.seg_to_traj_arena.buffer(t); {
			const auto res = seg_to_traj_buf.push_uninit(num_segments);
			assert(res.num_overwritten==0 && res.num_skipped==0);
			assert(seg_to_traj_buf.num_elems == num_segments);
			std::fill(seg_to_traj_buf.begin(), seg_to_traj_buf.end(), t);
		}
		trajectories.emplace_back(nodes_buf, alen_buf);
		uint32_t start_index = t*num_nodes;
		for (unsigned i=0; i<num_segments; i++) {
			geom.node_indices.emplace_back(start_index+i, start_index+i+1);
			/*geom.*/segment_indices.emplace_back(++idx);
		}
	}
	assert(geom.node_indices.size() == num_trajectories*num_segments);
	assert(geom.node_indices.size() == /*geom.*/segment_indices.size());

	// Set up renderer
	if (!render.aam.is_created()) {
		if (!render.tstr.init(ctx))
			ctx.error(std::string("unable to initialize textured spline tube renderer for extrapolations"));
		render.tstr.set_render_style(render.style);
		if (!render.aam.init(ctx))
			ctx.error(std::string("unable to initialize attribute array manager for extrapolations"));
	}
	render.tstr.enable_attribute_array_manager(ctx, render.aam);
	render.tstr.set_node_id_array(
		ctx, reinterpret_cast<const cgv::uvec2*>(geom.node_indices.data()),
		num_trajectories*num_segments,
		sizeof(cgv::uvec2)
	);
	render.tstr.set_indices(ctx, /*geom.*/segment_indices);
	render.tstr.disable_attribute_array_manager(ctx, render.aam);

	/*geom.test_nodes.create(1, 2);
	geom.test_alens.create(1, 1);*/

	// Done!
	return _.disarm(success);
}

void extrapolation_manager::reinit_layer_config (void)
{
	// Commit setup and find out actual number of float elements we need in our glyph attribute ring buffers. We need
	// to use the least-common multiple for the single-glyph attribute counts from each layer in order to be able to
	// advance the GPU ring buffers in a consistent way.
	setup.num_layers = otv_render.visualizations[0].config.layer_configs.size();
	const auto &some_traj = otv_render.trajectories.front();
	setup.glyph_attrib_counts.front() = some_traj.glyph_to_attrib_count(0, glyph_count_type{1});
	setup.glyph_attribs_lcm = setup.glyph_attrib_counts.front();
	for (unsigned l=1; l<setup.num_layers; l++) {
		setup.glyph_attrib_counts[l] = some_traj.glyph_to_attrib_count(l, glyph_count_type{1});
		setup.glyph_attribs_lcm = std::lcm(setup.glyph_attribs_lcm, setup.glyph_attrib_counts[l]);
	}
}

bool extrapolation_manager::create_glyph_and_per_layer_buffers (
	cgv::render::context &ctx, const tube_shading_settings &tube_shading,
	const glyph_layer_manager::configuration &layer_config, unsigned per_layer_min_glyphs_capacity
){
	// Sanity checks
	assert(
		!geom.nodes_arena.ring_buffers.empty() &&
		"extrapolation_manager::create_glyph_and_per_layer_buffers() needs pre-created geometry buffers"
	);
	assert(
		glyphs.ranges_arena[0].ring_buffers.empty() && glyphs.ranges_arena[1].ring_buffers.empty() &&
		glyphs.ranges_arena[2].ring_buffers.empty() && glyphs.ranges_arena[3].ring_buffers.empty() &&
		"extrapolation_manager::create_glyph_and_per_layer_buffers() must not be called on a fully constructed setup"
	);

	// Make sure we don't leak resources when any single one of the steps fails
	auto _ = finalizer([this] {
	    glyphs.traj_glyph_mem_arena.destroy();
		glyphs.ranges_arena[3].destroy(); glyphs.ranges_arena[2].destroy();
		glyphs.ranges_arena[1].destroy(); glyphs.ranges_arena[0].destroy();
		glyphs.glyph_attribs_arena[3].destroy(); glyphs.glyph_attribs_arena[2].destroy();
		glyphs.glyph_attribs_arena[1].destroy(); glyphs.glyph_attribs_arena[0].destroy();
		std::fill(setup.glyph_attrib_counts.begin(), setup.glyph_attrib_counts.end(), 0);
		std::cerr << "extrapolation_manager::create_glyph_and_per_layer_buffers(): failed to create GPU resources!"
		          << std::endl<<std::endl;
	});

	// Create GPU ring buffer arenas
	// - glyph index mapping ranges
	const auto num_trajectories = geom.t_to_s_arena.num_ring_buffers();
	const auto num_segments = geom.t_to_s_arena.ring_buffer_capacity();
	//const auto num_buffers = num_trajectories*setup.num_layers;
	bool success = glyphs.ranges_arena[0].create(num_trajectories, num_segments)
	            && glyphs.ranges_arena[1].create(num_trajectories, num_segments)
	            && glyphs.ranges_arena[2].create(num_trajectories, num_segments)
	            && glyphs.ranges_arena[3].create(num_trajectories, num_segments);
	// - glyph instances
	const auto num_attribs = setup.glyph_attribs_lcm*per_layer_min_glyphs_capacity;
	success &= glyphs.glyph_attribs_arena[0].create(num_trajectories, num_attribs)
	        && glyphs.glyph_attribs_arena[1].create(num_trajectories, num_attribs)
	        && glyphs.glyph_attribs_arena[2].create(num_trajectories, num_attribs)
	        && glyphs.glyph_attribs_arena[3].create(num_trajectories, num_attribs);
	// - ring buffer delimiting ranges (must not pack tightly by allocating less when less than 4 layers are configured)
	success &= glyphs.traj_glyph_mem_arena.create(num_trajectories, 4);

	// Assign range ring buffers from arena to corresponding layer on each trajectory
	for (unsigned t=0; t<num_trajectories; t++)
	{
		auto &traj = trajectories[t];
		traj.layers.reserve(setup.num_layers);
		auto &traj_glyph_mem_buf = glyphs.traj_glyph_mem_arena.buffer(t); {
			const auto res = traj_glyph_mem_buf.push_uninit(setup.num_layers);
			assert(res.num_overwritten==0 && res.num_skipped==0);
			assert(traj_glyph_mem_buf.num_elems == setup.num_layers);
		}
		auto traj_glyph_mem = &*traj_glyph_mem_buf.begin();
		for (unsigned l=0; l<setup.num_layers; l++)
		{
			const unsigned per_layer_buffer_id = t/* *setup.num_layers + l*/;
			auto &ranges_buf = glyphs.ranges_arena[l].buffer(per_layer_buffer_id); {
				const auto res = ranges_buf.push_uninit(num_segments);
				assert(res.num_overwritten==0 && res.num_skipped==0);
			}
			traj.layers.emplace_back(
				ranges_buf, glyphs.glyph_attribs_arena[l].buffer(per_layer_buffer_id)
			);
			traj_glyph_mem[l] = irange{int((t/* *setup.num_layers + l*/)*num_attribs), (int)num_attribs};
		}
	}

	// Copy tube shading configuration form on_tube_vis as basis and modify for extrapolation
	render.tube_shading = tube_shading;
	const auto tube_shading_defines = render.tube_shading.build_tube_shading_defines(
		layer_config, false
	);
	render.tstr.set_additional_defines(tube_shading_defines);
	render.tstr.enable(ctx);
	render.tstr.disable(ctx);

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

	// Indicate that flush of geometry-related arenas will be required
	state.dirty_flags = state.dirty_flags | state.SEGMENTS_DIRTY;

	// Update per-layer range maps for current set of glyphs
	for (unsigned l=0; l<setup.num_layers; l++)
	{
		// Convenience shorthand
		auto &layer = traj.layers[l];

		// Retrieve general per-layer information
		const unsigned stride = setup.glyph_attrib_counts[l];

		// First, find the oldest glyph we need to still include
		auto on_extrapol = skip_glyphs_before(
			l, extrapolation.front().t_to_s[0], ro_range{layer.glyph_attribs.begin(), layer.glyph_attribs.end()}
		);
		// ...and erase those that fell off the extrapolation
		if (on_extrapol.begin > layer.glyph_attribs.begin())
		{
			// Pop old glyphs from the ring buffer
			const auto erase_range = ro_range{layer.glyph_attribs.begin(), on_extrapol.begin};
			const auto erase_count = on_extrapol.begin - layer.glyph_attribs.begin();
			assert(erase_range.length() == erase_count);
			const auto old_num = layer.glyph_attribs.size();
			layer.glyph_attribs.pop_front(erase_count);
			const auto new_num = layer.glyph_attribs.size();
			assert(new_num % stride == 0);
			assert(new_num == old_num-erase_count);

			// Indicate that flush of glyph-related arenas will be required
			state.dirty_flags = state.dirty_flags | state.glyphs_dirty_flag[l];

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

	// Trim glyphs that precede the range covered by the extrapolation (this can happen due to network issues or if they
	// were submitted late by the client for whatever reason)
	const auto on_extrapol = skip_glyphs_before(l, traj.t_to_s.contents[0][0], glyph_attribs);
	assert(on_extrapol.end == glyph_attribs.end);
	if (on_extrapol.is_empty())
		// Nothing to do if the client only submitted very old glyphs that don't affect the extrapolation
		return on_extrapol;

	// Monotonicity check
	#ifndef NDEBUG
		float old_max_s;
		float first_glyph_s;
		if (!layer.glyph_attribs.empty()) {
			// Sanity check
			old_max_s = get_glyph_pos(
				ro_range{layer.glyph_attribs.end()-stride, layer.glyph_attribs.end()}
			);
			first_glyph_s = get_glyph_pos(on_extrapol);
			assert(
				first_glyph_s >= old_max_s &&
				"extrapolation_manager::consider_glyphs(): new glyphs must never precede previously submitted glyphs!"
			);
		}
	#endif

	// Add to displayed glyphs
	const auto num_glyphs_before =  layer.glyph_attribs.size()/stride;
	#ifndef NDEBUG
		const unsigned num_attribs_before =  layer.glyph_attribs.size();
		const unsigned num_glyphs_before_check = otv_render.trajectories[traj_id].attrib_to_glyph_count(
			l, static_cast<int>(num_attribs_before)
		).value;
	#endif
	assert(num_glyphs_before == num_glyphs_before_check);
	const auto discarded = layer.glyph_attribs.push_range(on_extrapol);
	assert(layer.glyph_attribs.size() % stride == 0); // sanity check
	const auto actually_on_extrapol = ro_range{on_extrapol.begin+discarded.num_skipped, on_extrapol.end};
	assert(actually_on_extrapol.end == on_extrapol.end);
	#ifndef NDEBUG
		const unsigned actually_on_extrapol_len = actually_on_extrapol.length();
		assert(actually_on_extrapol_len == on_extrapol.length()-discarded.num_skipped);
		const unsigned num_new_glyphs = otv_render.trajectories[traj_id].attrib_to_glyph_count(
			l, actually_on_extrapol_len
		).value;
		assert(num_new_glyphs == actually_on_extrapol_len/stride);
		if (discarded.num_overwritten > 0) {
			assert(num_new_glyphs > 0);
			std::clog.flush();
		}
	#endif

	// Indicate that flush of glyph-related arenas will be required
	state.dirty_flags = state.dirty_flags | state.glyphs_dirty_flag[l];

	// Unlink discarded glyphs, if any, and search for the newest segment that has any glyphs at the same time
	unsigned newest_seg_with_glyphs=0, seg=0;
	for (auto range_it=layer.ranges.begin(); range_it<layer.ranges.end(); ++range_it, ++seg)
	{
		auto &range = *range_it;
		int num_affected = std::max(static_cast<int>(discarded.num_overwritten) - range.i0, 0);
		if (num_affected > 0) {
			// This segment now has some or all of its glyphs missing, which we can resolve solely on the basis of its
			// mapped range start index
			range.i0 = 0; // <- glyphs can only ever be missing at the tail end
			num_affected = std::min<unsigned>(num_affected, range.n);
			range.n -= num_affected;
			newest_seg_with_glyphs = seg;
		}
		else if (range.n) {
			// All glyphs mapped to this segment are still there, they just moved tailwards, so adjust start index
			// accordingly
			const int new_i0 = range.i0 - discarded.num_overwritten;
			assert(new_i0 >= 0);
			range.i0 = new_i0;
			newest_seg_with_glyphs = seg;
		}
	}

	// Assign to segments
	if (newest_seg_with_glyphs>0)
		std::clog.flush();
	assign_glyphs(
		ro_range{layer.ranges.begin()+newest_seg_with_glyphs, layer.ranges.end()},
		ro_range{traj.t_to_s.begin()+newest_seg_with_glyphs, traj.t_to_s.end()},
		l, actually_on_extrapol, num_glyphs_before
	);

	#ifndef NDEBUG
	/* final sanity checks */ {
		const auto newest_input = actually_on_extrapol.end-1;
		const auto newest_committed = layer.glyph_attribs.end()-1;
		assert(*newest_input == *newest_committed);
		const auto oldest_input = actually_on_extrapol.begin;
		const auto oldest_committed = newest_committed - (actually_on_extrapol.length()-1);
		const auto input_range = ro_range{oldest_input, newest_input};
		const auto committed_range = ro_range{oldest_committed, newest_committed};
		const auto input_range_len = input_range.length(), committed_range_len = committed_range.length();
		assert(input_range_len == committed_range_len);
		assert(*input_range.begin == *committed_range.begin);
	}
	#endif

	// Done!
	return ro_range{actually_on_extrapol.begin, actually_on_extrapol.end};
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
	const auto glyph_geometry = otv_render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Find first glyph that overlaps with the area behind the start of the extrapolation
		do {
			// Determine arc length range covered by glyph
			const auto s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value();

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
	const auto glyph_geometry = otv_render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Find first glyph that doesn't overlap with the area behind the start of the extrapolation
		do {
			// Determine arc length range covered by glyph
			const auto s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value();

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
	////
	// Prelude

	const unsigned num_segments = ranges_out.length();
	const auto num_glyphs = (unsigned)otv_render.trajectories.front().attrib_to_glyph_count(
		layer, (int)glyph_attribs.length()
	).value + idx_offset;
	const auto stride = setup.glyph_attrib_counts[layer];
	ro_range cur_glyph {glyph_attribs.begin, glyph_attribs.begin+stride};
	unsigned cur_glyph_idx = idx_offset;
	const bool replace = !idx_offset; // make it an extra variable for code readability
	#ifndef NDEBUG
		if (!replace)
			std::clog.flush();
	#endif

	// If we're replacing, then we will need to make sure the dirty flag gets set even if none of the conditions further
	// down get triggered (e.g. because no glyphs actually end up being assigned)
	state.dirty_flags = state.dirty_flags | (replace*state.ranges_dirty_flag[layer]);


	/////
	// Iterate through input glyph attribute range to assign them to the extrapolated segments

	// We need to differentiate between glyph and plot layers.
	// TODO: The code in both cases looks very similar and can probably be templated – at least partially – to minimize
	//       duplication.
	// FIXME: The below algorithm currently DOES NOT unlink plot control points that get superseded by a new one passed
	//        to this function. Such a replacement would be possible if both old and new control points precede the
	//        start of a segment. However, the potential runtime gain for the bisection search in the fragment shader is
	//        likely negligible and thus not worth the hassle.
	const auto glyph_geometry = otv_render.glyph_extents(layer, &*(glyph_attribs.begin+2));
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Outer loop is over extrapolated segments
		for (unsigned seg=0; seg<num_segments; seg++)
		{
			// Obtain relevant quantities and references
			const auto s_max = (*(t_to_s.begin+seg))[15];
			auto &cur_range = *(ranges_out.begin+seg);
			cur_range.i0 = replace ? (int)cur_glyph_idx : cur_range.i0;
			cur_range.n = replace ? 0 : cur_range.n;

			// Check if we have any glyphs remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over the glyphs that overlap the current segment
			auto s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value(), s_range_prev = s_range;
			while (s_range.x() <= s_max)
			{
				// Update the range
				cur_range.n++;
				cur_glyph_idx++;
				cur_glyph.safe_advance(stride, glyph_attribs.end);

				// Indicate that flush of the glyph mapping ranges arena will be required
				state.dirty_flags = state.dirty_flags | state.ranges_dirty_flag[layer];

				// Advance glyph cursor
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					break; // all glyphs exhausted
				s_range_prev = s_range;
				s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value();
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
			cur_range.i0 = replace ? (int)cur_glyph_idx : cur_range.i0;
			cur_range.n = replace ? 0 : cur_range.n;

			// Check if we have any control points remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over control points that fall _before_ the end of the segment
			auto s = get_glyph_pos(cur_glyph);
			while (s < s_max)
			{
				// Update the range
				cur_range.n++;
				cur_glyph_idx++;

				// Indicate that flush of the glyph mapping ranges arena will be required
				state.dirty_flags = state.dirty_flags | state.ranges_dirty_flag[layer];

				// Advance glyph cursor
				cur_glyph.safe_advance(stride, glyph_attribs.end);
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					break; // all glyphs exhausted
				s = get_glyph_pos(cur_glyph);
			}

			// Now, include one additional (if it exists) control point to ensure correct inter-segment
			// interpolation on the _current_ segment.
			if (cur_glyph_idx < num_glyphs) {
				// Update the range
				cur_range.n++;

				// Indicate that flush of the glyph mapping ranges arena will be required
				state.dirty_flags = state.dirty_flags | state.ranges_dirty_flag[layer];
			}

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

bool extrapolation_manager::flush_changes (void)
{
	// Determine the arenas we need to flush
	const bool flush_nodes = state.dirty_flags & state.SEGMENTS_DIRTY,
	           flush_t_to_s = state.dirty_flags & state.SEGMENTS_DIRTY,
	           flush_ranges[4] = {
	           	bool(state.dirty_flags & state.RANGES0_DIRTY), bool(state.dirty_flags & state.RANGES1_DIRTY),
	           	bool(state.dirty_flags & state.RANGES2_DIRTY), bool(state.dirty_flags & state.RANGES3_DIRTY)
	           },
	           flush_glyphs[4] = {
	           	bool(state.dirty_flags & state.GLYPHS0_DIRTY), bool(state.dirty_flags & state.GLYPHS1_DIRTY),
	           	bool(state.dirty_flags & state.GLYPHS2_DIRTY), bool(state.dirty_flags & state.GLYPHS3_DIRTY)
	           };

	// Perform flushes
	bool result = true;
	if (flush_nodes)
		result &= geom.nodes_arena.flush_all();
	if (flush_t_to_s)
		result &= geom.t_to_s_arena.flush_all();
	for (unsigned l=0; l<4; l++) {
		if (flush_ranges[l])
			result &= glyphs.ranges_arena[l].flush_all();
		if (flush_glyphs[l])
			result &= glyphs.glyph_attribs_arena[l].flush_all();
	}

	// Update state
	state.last_frame_timepoint = state.update_needed ?
		  state.last_frame_timepoint
		: std::chrono::high_resolution_clock::now();
	state.update_needed |= state.dirty_flags;
	state.dirty_flags = 0;

	// Done!
	return result;
}

bool extrapolation_manager::update_needed (void) const {
	return state.update_needed;
}

void extrapolation_manager::update (void) {
	state.update_needed = false;
}

void extrapolation_manager::draw_extrapolations(cgv::render::context &ctx, const cgv::vec3 &eye_pos)
{
	// Do nothing if no data
	if (geom.node_indices.empty())
		return;

	// Pull in up-to-date render style settings
	// FIXME: this should be push rather than pull
	update_render_style(otv_render.style);

	// Set up draw call
	/*auto nodes = geom.test_nodes.data_memory.data();
	nodes[0].color.set(0, 1, 0, 1);
	nodes[0].pos_rad.set(0, 0, -1, 0.25);
	nodes[0].tangent.set(0, 0, 1, 0);
	nodes[0].t.x() = 0;
	nodes[1].color.set(1, 0, 1, 1);
	nodes[1].pos_rad.set(0, 0, 0, 0.25);
	nodes[1].tangent.set(0, 0, 1, 0);
	nodes[1].t.x() = 1;
	assert(geom.nodes_arena.data_memory.flush());
	auto alens = geom.test_alens.data_memory.data();
	alens[0] = arclen::compute_single_t_to_s(
		cgv::vec3(nodes[0].pos_rad), cgv::vec3(nodes[0].tangent),
		cgv::vec3(nodes[1].pos_rad), cgv::vec3(nodes[1].tangent), .0f
	);
	assert(geom.test_alens.data_memory.flush());
	std::vector<cgv::uvec2> nids;
	nids.emplace_back(0, 1);
	std::vector<unsigned> indices;
	indices.emplace_back(0);*/

	render.tstr.set_render_style(render.style);
	render.tstr.set_cyclopic_eye(eye_pos);
	render.tstr.enable_attribute_array_manager(ctx, render.aam);
	/*render.tstr.set_node_id_array(
		ctx, nids.data(), 1, sizeof(cgv::uvec2)
	);
	render.tstr.set_indices(ctx, indices);
	glFlush();*/
	const auto node_id_buf = render.tstr.get_vertex_buffer_ptr(ctx, render.aam, "node_ids");
	GLuint buffer_handles[13] = {
		geom.nodes_arena.data_memory.handle(),
		geom.t_to_s_arena.data_memory.handle(),/*
		geom.test_nodes.data_memory.handle(),
		geom.test_alens.data_memory.handle(),*/
		cgv::render::gl::get_gl_id(node_id_buf->handle), // <- required since we're drawing attribute-less
	};
	for (unsigned l=0; l<setup.num_layers; ++l) {
		buffer_handles[3 + l*2] = glyphs.glyph_attribs_arena[l].data_memory.handle();
		buffer_handles[4 + l*2] = glyphs.ranges_arena[l].data_memory.handle();
	}
	buffer_handles[3 + setup.num_layers*2] = geom.seg_to_traj_arena.data_memory.handle();
	buffer_handles[4 + setup.num_layers*2] = glyphs.traj_glyph_mem_arena.data_memory.handle();
	glBindBuffersBase(
		GL_SHADER_STORAGE_BUFFER, 0, 3+2 + setup.num_layers*2, buffer_handles
	);

	// Set uniforms
	render.tube_shading.set_uniforms(
		ctx, render.tstr.ref_prog(), render.style, otv_render.visualizations[0].config
	);

	// Draw and clean up
	render.tstr.render(ctx, 0, geom.node_indices.size());
	render.tstr.disable_attribute_array_manager(ctx, render.aam);
}


} // namespace otv
