
// C++ STL
#include <numeric>

// Local includes
#include "arclen/main.h"
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
	const cgv::vec4 color(cgv::vec3(ref_node1.color), .5f);
	const extrapol::node prev_extrapol_values = {
		ref_node1, ref_t_to_s
	};
	const extrapol::node *prev_extrapol = &prev_extrapol_values;
	for (unsigned i=1; i<=num; i++)
	{
		const node_attribs new_node {
			cgv::vec4(p1 + float(i)*m1, r), color, cgv::vec4(m1, 0),
			cgv::vec4(t1 + float(i)*dt, 0, 0, 0)
		};
		const extrapol::node new_extrapol {
			new_node, arclen::single_linear_t_to_s(m1_len, /*sigma: */ prev_extrapol->t_to_s[15])
		};
		prev_extrapol = &out.emplace_back(new_extrapol);
	}
}

} // namespace extrapol


extrapolation_manager::extrapolation_manager (render_state &otv_render_state) : otv_render(otv_render_state)
{
	// Apply render style
	update_render_style(otv_render.style);
}

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

	// Reset stats
	stats.reset();
}

bool extrapolation_manager::create_geom_buffers (
	cgv::render::context &ctx, unsigned num_trajectories, unsigned num_segments
){
	// Make sure the timer objects are created
	if (!render.sort_time_query.is_initialized())
		render.sort_time_query.init(ctx);
	if (!render.draw_time_query.is_initialized())
		render.draw_time_query.init(ctx);
	if (!render.flush_time_query.is_initialized())
		render.flush_time_query.init(ctx);

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
	bool success =    geom.nodes_arena.create(num_trajectories, num_nodes)
	               && geom.t_to_s_arena.create(num_trajectories, num_segments)
	               && geom.seg_to_traj_arena.create(num_trajectories, num_segments);

	// Distribute per-trajectory ringbuffers and create indices in the process
	trajectories.reserve(num_trajectories);
	geom.node_indices.reserve(num_trajectories * num_segments);
	std::vector<unsigned> segment_indices;
	segment_indices.reserve(geom.node_indices.size());
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
			for (unsigned seg=0; seg<num_segments; ++seg)
				seg_to_traj_buf.contents[seg] = topology_info{t, t*num_segments + seg};
			seg_to_traj_buf.contents[num_segments-1].next = -1;
		}
		trajectories.emplace_back(nodes_buf, alen_buf);
		uint32_t start_index = t*num_nodes;
		for (unsigned i=0; i<num_segments; i++) {
			geom.node_indices.emplace_back(start_index+i, start_index+i+1);
			segment_indices.emplace_back(++idx);
		}
	}
	assert(geom.node_indices.size() == num_trajectories*num_segments);
	assert(geom.node_indices.size() == segment_indices.size());

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
	render.tstr.set_indices(ctx, segment_indices);
	render.tstr.disable_attribute_array_manager(ctx, render.aam);

	// Set up and initialize GPU visibility sorter or resize if it is already initialized
	if(render.sorter.is_initialized()) {
		render.sorter.resize(ctx, geom.node_indices.size());
	} else {
		// Define the data type of a segment node
		sl::data_type node_type = { "node_type", {
			{ sl::Type::kVec4, "pos_rad" },
			{ sl::Type::kVec4, "color" },
			{ sl::Type::kVec4, "tangent" },
			{ sl::Type::kVec4, "t" },
		} };

		// Define an array of type uvec2 with variable size to use in the node indices buffer
		sl::named_variable node_indices = { sl::Type::kUVec2, "node_indices", sl::varsize };

		// Define arguments used for the key generation step of the sorting
		cgv::gpgpu::argument_definitions key_arguments = {
			{ sl::Type::kVec3, "u_eye_pos" },
			{ sl::Type::kVec3, "u_view_dir" },
			{ sl::tag::buffer{}, node_indices, "node_index_buffer" , { sl::MemoryQualifier::kReadOnly } }
		};

		// Define a function that takes in node pairs and returns distances based on the passed arguments
		static const std::string key_transform =
			R"(uvec2 indices = node_indices[index];
			node_type a = data_in[indices.x];
			node_type b = data_in[indices.y];
			vec3 pa = a.pos_rad.xyz;
			vec3 pb = b.pos_rad.xyz;

			vec3 x = 0.5*(pa + pb);
			vec3 eye_to_pos = x - u_eye_pos;
			float ddv = dot(eye_to_pos, u_view_dir);
			return (ddv < 0.0 ? -1.0 : 1.0) * dot(eye_to_pos, eye_to_pos);)";

		// Initialize the sorter with the given data types and key arguments/transform. Use descending order to sort most distant elements last,
		// which is needed for back-to-front rendering.
		success &= render.sorter.init(ctx, node_type, sl::Type::kUInt, geom.node_indices.size(), key_arguments, key_transform, cgv::gpgpu::SortOrder::kDescending);
	}

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
			const unsigned per_layer_buffer_id = t/**setup.num_layers + l*/;
			auto &ranges_buf = glyphs.ranges_arena[l].buffer(per_layer_buffer_id); {
				const auto res = ranges_buf.push_uninit(num_segments);
				assert(res.num_overwritten==0 && res.num_skipped==0);
			}
			traj.layers.emplace_back(
				ranges_buf, glyphs.glyph_attribs_arena[l].buffer(per_layer_buffer_id)
			);
			assert(num_attribs%setup.glyph_attrib_counts[l] == 0);
			const unsigned layer_glyph_capacity = num_attribs / setup.glyph_attrib_counts[l];
			assert(layer_glyph_capacity*setup.glyph_attrib_counts[l] == num_attribs);
			traj_glyph_mem[l] = irange{int(t*layer_glyph_capacity/* + l*/), (int)layer_glyph_capacity};
		}
	}

	// Copy tube shading configuration form on_tube_vis as basis and modify for extrapolation
	update_tube_shading(tube_shading, layer_config);
	render.tstr.enable(ctx);  // make sure the shader program gets recompiled
	render.tstr.disable(ctx);

	// Done!
	return _.disarm(success);
}

hires_duration_type extrapolation_manager::replace_extrapolation (
	unsigned traj_id, const node_attribs &last_measured_node, const std::vector<extrapol::node> &extrapolation
){
	// Start timing the CPU duration of the replacement
	const auto start_time = std::chrono::high_resolution_clock::now();

	// Retrieve trajectory
	auto &traj = trajectories[traj_id];

	// Replace geometry buffer contents
	const auto num_segments = traj.t_to_s.capacity();
	assert(num_segments == (unsigned)extrapolation.size());
	traj.nodes.contents[0] = last_measured_node;
	traj.nodes.contents[0].color.w() = extrapolation.front().hnode.color.w();
	for (unsigned i=0; i<num_segments; i++) {
		traj.nodes.contents[i+1] = extrapolation[i].hnode;
		traj.t_to_s.contents[i] = extrapolation[i].t_to_s;
	}

	// Indicate that flush of geometry-related arenas will be required
	state.dirty_flags = state.dirty_flags | state.SEGMENTS_DIRTY;

	// Update per-layer range maps for current set of glyphs
	unsigned total_relinked_glyphs = 0;
	for (unsigned l=0; l<setup.num_layers; l++)
	{
		// Convenience shorthand
		auto &layer = traj.layers[l];

		// Retrieve general per-layer information
		const unsigned stride = setup.glyph_attrib_counts[l];

		// First, find the oldest glyph we need to still include
		total_relinked_glyphs += layer.glyph_attribs.size()/stride;
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
		if (layer.glyph_attribs.empty()) {
			layer.newest_seg_with_glyphs = 0;
			continue;
		}

		// Assign remaining in-range glyphs
		layer.newest_seg_with_glyphs = assign_glyphs(
			ro_range{layer.ranges.begin(), layer.ranges.end()}, ro_range{traj.t_to_s.begin(), traj.t_to_s.end()},
			traj_id, l, on_extrapol
		);
	}

	// Update stats and return
	stats.glyphs_per_replace.add_measurement(total_relinked_glyphs);
	++stats.num_replacements;
	const auto time = std::chrono::high_resolution_clock::now() - start_time;
	stats.replace_times.add_measurement(time);
	return time;
}

template <class Iter>
ro_range<Iter> extrapolation_manager::consider_glyphs (
	unsigned traj_id, unsigned l, const ro_range<Iter> &glyph_attribs, hires_duration_type &consumed_time
){
	// Nothing to do if there are no glyphs
	if (glyph_attribs.is_empty())
		return glyph_attribs;

	// Start timing the CPU duration of the placement
	const auto start_time = std::chrono::high_resolution_clock::now();

	// Convenience shorthands
	auto &traj = trajectories[traj_id];
	auto &layer = traj.layers[l];

	// First, retrieve general information about the layer
	const unsigned stride = setup.glyph_attrib_counts[l];

	// Count pushed glyphs for the statistics
	const unsigned num_pushed_glyphs = glyph_attribs.length()/stride;
	stats.glyphs_per_push.add_measurement(num_pushed_glyphs);
	stats.num_glyphs_pushed += num_pushed_glyphs;
	stats.num_multi_glyph_pushes += num_pushed_glyphs > 1;
	stats.num_single_glyph_pushes += num_pushed_glyphs == 1;

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

	// Store current logical start indices from each segment before actually adding any glyphs to the ring buffer. This
	// should be done before adding the new glyphs to the ring buffer as on-the-fly computation from real indices can
	// yield wildly wrong values when adding the new glyphs caused some old ones being popped off the buffer. The reason
	// for this is that the stored real indices can slip in front of the tail element when old elements are popped.
	// ORIGINAL WITH LOGICAL IDCS IN RANGE:
	/* nothing at all */
	int logical_i0 [16]; // <- we'll assume there won't ever be more than 16 segments of extrapolation per traj
	const auto range0 = layer.ranges.begin();
	for (unsigned seg=0; seg<layer.ranges.size(); seg++)
		logical_i0[seg] = layer.logical_glyph_idx_from_real(stride, (range0+seg)->i0);

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
	const unsigned actually_on_extrapol_len = actually_on_extrapol.length();
	assert(actually_on_extrapol_len == on_extrapol.length()-discarded.num_skipped);
	const unsigned num_new_glyphs = actually_on_extrapol_len/stride;
	#ifndef NDEBUG
		assert(
			num_new_glyphs == otv_render.trajectories[traj_id].attrib_to_glyph_count(l, actually_on_extrapol_len).value
		);
		if (discarded.num_overwritten > 0)
			assert(num_new_glyphs > 0);
	#endif

	// Indicate that flush of glyph-related arenas will be required
	state.dirty_flags = state.dirty_flags | state.glyphs_dirty_flag[l];

	// Unlink discarded glyphs
	for (auto range_it=layer.ranges.begin(); range_it<layer.ranges.end(); ++range_it)
	{
		auto &range = *range_it;
		// ORIGINAL WITH LOGICAL IDCS IN RANGE:
		/*int num_affected = std::max(static_cast<int>(discarded.num_overwritten) - range.i0, 0);
		if (num_affected > 0) {
			// This segment now has some or all of its glyphs missing, which we can resolve solely on the basis of its
			// mapped range start index
			range.i0 = 0; // <- glyphs can only ever be missing at the tail end
			num_affected = std::min<unsigned>(num_affected, range.n);
			range.n -= num_affected;
		}
		else if (range.n) {
			// All glyphs mapped to this segment are still there, they just moved tailwards, so adjust start index
			// accordingly
			#ifndef NDEBUG
				if (discarded.num_overwritten > 0)
					std::clog.flush();
			#endif
			const int new_i0 = range.i0 - discarded.num_overwritten;
			assert(new_i0 >= 0);
			range.i0 = new_i0;
		}*/
		const auto seg = std::distance(range0, range_it);
		int num_affected = std::max(static_cast<int>(discarded.num_overwritten) - logical_i0[seg], 0);
		if (num_affected > 0) {
			// This segment now has some or all of its glyphs missing, which we can resolve solely on the basis of its
			// mapped range start index
			logical_i0[seg] = 0; // <- glyphs can only ever be missing at the tail end
			num_affected = std::min<unsigned>(num_affected, range.n);
			range.n -= num_affected;
		}
		else if (range.n) {
			// All glyphs mapped to this segment are still there, they just moved tailwards, so adjust start index
			// accordingly
			#ifndef NDEBUG
				if (discarded.num_overwritten > 0)
					std::clog.flush();
			#endif
			const int new_i0 = logical_i0[seg] - discarded.num_overwritten;
			assert(new_i0 >= 0);
			logical_i0[seg] = new_i0;
		}
		range.i0 = layer.real_glyph_idx_from_logical(stride, logical_i0[seg]); // commit changes to range i0
	}

	// Assign to segments
	layer.newest_seg_with_glyphs += assign_glyphs(
		ro_range{layer.ranges.begin()+layer.newest_seg_with_glyphs, layer.ranges.end()},
		ro_range{traj.t_to_s.begin()+layer.newest_seg_with_glyphs, traj.t_to_s.end()},
		traj_id, l, actually_on_extrapol, num_glyphs_before
	);

	// Update stats about actually committed glyphs
	stats.num_glyphs_comitted += num_new_glyphs;

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

	// Done! Update stats and return
	consumed_time = std::chrono::high_resolution_clock::now() - start_time;
	stats.glyph_push_times.add_measurement(consumed_time);
	return ro_range{actually_on_extrapol.begin, actually_on_extrapol.end};
}
template ro_range<std_vector_float_iter> extrapolation_manager::consider_glyphs (
	unsigned, unsigned, const ro_range<std_vector_float_iter>&, hires_duration_type&
);
template ro_range<std_deque_float_iter> extrapolation_manager::consider_glyphs (
	unsigned, unsigned, const ro_range<std_deque_float_iter>&, hires_duration_type&
);
template ro_range<float*> extrapolation_manager::consider_glyphs (
	unsigned, unsigned, const ro_range<float*>&, hires_duration_type&
);

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
unsigned extrapolation_manager::assign_glyphs (
	ro_range<IRangeIter> &&ranges_out, const ro_range<AlenIter> &t_to_s, const unsigned traj_id, const unsigned layer,
	const ro_range<GlyphAttribsIter> &glyph_attribs, const unsigned idx_offset
){
	////
	// Prelude

	// Determine counts, strides, indices etc.
	const auto &layer_ref = trajectories[traj_id].layers[layer];
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
	unsigned last_seg_with_glyph_within = 0;
	if (glyph_geometry.has_value()) // it's a glyph
	{
		// Outer loop is over extrapolated segments
		for (unsigned seg=0; seg<num_segments; seg++)
		{
			// Obtain relevant quantities and references
			const auto s_max = (*(t_to_s.begin+seg))[15];
			auto &cur_range = *(ranges_out.begin+seg);
			// ORIGINAL WITH LOGICAL IDCS IN RANGE:
			/*cur_range.i0 = (replace || !cur_range.n) ? (int)cur_glyph_idx : cur_range.i0;*/
			cur_range.i0 = (replace || !cur_range.n) ?
				(int)layer_ref.real_glyph_idx_from_logical(stride, cur_glyph_idx) : cur_range.i0;
			cur_range.n = replace ? 0 : cur_range.n;

			// Check if we have any glyphs remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over the glyphs that overlap the current segment
			auto s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value(), s_range_prev = s_range;
			bool glyphs_added = false;
			while (s_range.x() <= s_max)
			{
				// Update the range
				cur_range.n++;
				cur_glyph_idx++;
				cur_glyph.safe_advance(stride, glyph_attribs.end);
				glyphs_added = true;
				last_seg_with_glyph_within = seg;

				// Indicate that flush of the glyph mapping ranges arena will be required
				state.dirty_flags = state.dirty_flags | state.ranges_dirty_flag[layer];

				// Advance glyph cursor
				assert(cur_glyph_idx <= num_glyphs);
				if (cur_glyph_idx == num_glyphs)
					break; // all glyphs exhausted
				s_range_prev = s_range;
				s_range = otv_render.glyph_range(layer, &*cur_glyph.begin).value();
			}

			// Finally, in case we did indeed add a glyph to this segment, move the cursor back one glyph (if it's not
			// already the first) in case the last of the added glyphs also overlaps the next segment.
			if (glyphs_added && cur_glyph_idx > 0 && s_range_prev.y() >= s_max) {
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
			const auto s_min = (*(t_to_s.begin+seg))[0], s_max = (*(t_to_s.begin+seg))[15];
			auto &cur_range = *(ranges_out.begin+seg);
			// ORIGINAL WITH LOGICAL IDCS IN RANGE:
			/*cur_range.i0 = (replace || !cur_range.n) ? (int)cur_glyph_idx : cur_range.i0;*/
			cur_range.i0 = (replace || !cur_range.n) ?
				(int)layer_ref.real_glyph_idx_from_logical(stride, cur_glyph_idx) : cur_range.i0;
			cur_range.n = replace ? 0 : cur_range.n;

			// Check if we have any control points remaining
			assert(cur_glyph_idx <= num_glyphs);
			if (cur_glyph_idx == num_glyphs)
				// While we're actually fully done, we need to ensure the remaining segments get initialized too
				continue;

			// Inner loop is over control points that fall _before_ the end of the segment
			auto s = get_glyph_pos(cur_glyph);
			bool points_before_seg_end = false;
			while (s < s_max)
			{
				// Update the range
				cur_range.n++;
				cur_glyph_idx++;
				cur_glyph.safe_advance(stride, glyph_attribs.end);
				points_before_seg_end = true;
				last_seg_with_glyph_within = s < s_min ? last_seg_with_glyph_within : seg;

				// Indicate that flush of the glyph mapping ranges arena will be required
				state.dirty_flags = state.dirty_flags | state.ranges_dirty_flag[layer];

				// Advance glyph cursor
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

			// Finally, if we added control points because they are _on_ the current segment, move the cursor back one
			// control point (if it's not already the first) to make sure the last such control point will be included
			// on the next segment also, to ensure correct inter-segment interpolation on the _next_ segment. In case
			// the current control point lies _exactly_ on the segment boundary (2nd condition), this is not necessary.
			if (points_before_seg_end && cur_glyph_idx > 0 && s!=s_max) {
				cur_glyph_idx--;
				cur_glyph.safe_unadvance(stride);
			}
		}
	}

	// Done!
	return last_seg_with_glyph_within;
}

bool extrapolation_manager::flush_changes (void)
{
	// Determine the arenas we need to flush
	state.flushed_something = false;
	const bool
		flush_nodes = state.dirty_flags & state.SEGMENTS_DIRTY,
		flush_t_to_s = state.dirty_flags & state.SEGMENTS_DIRTY,
		flush_ranges[4] = {
			bool(state.dirty_flags & state.RANGES0_DIRTY), bool(state.dirty_flags & state.RANGES1_DIRTY),
			bool(state.dirty_flags & state.RANGES2_DIRTY), bool(state.dirty_flags & state.RANGES3_DIRTY)
		},
		flush_glyphs[4] = {
			bool(state.dirty_flags & state.GLYPHS0_DIRTY), bool(state.dirty_flags & state.GLYPHS1_DIRTY),
			bool(state.dirty_flags & state.GLYPHS2_DIRTY), bool(state.dirty_flags & state.GLYPHS3_DIRTY)
		};

	// Wait for extrapolation rendering to complete
	/*if (render.draw_fence) {
		*//*auto wait_result = *//*glClientWaitSync(render.draw_fence, 0, -1);
		glDeleteSync(render.draw_fence);
		render.draw_fence = nullptr;
	}*/

	// Perform flushes
	bool result = true;
	if (state.dirty_flags)
	{
		glFlush();  // Flushing the pipeline is the only way to
		glFinish(); // time buffer uploads unfortunately :/
		render.flush_time_query.begin_scope();
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
		glFlush();
		glFinish();
		render.flush_time_query.end_scope();
		state.flushed_something = true;
	}

	// Update state
	state.last_frame_timepoint = state.update_needed ?
		  state.last_frame_timepoint
		: std::chrono::high_resolution_clock::now();
	state.update_needed = state.update_needed || state.dirty_flags;
	state.dirty_flags = 0;

	// Done!
	return result;
}

bool extrapolation_manager::update_needed (void) const {
	return state.update_needed;
}

void extrapolation_manager::update (const float playback_t) {
	render.tube_shading.playback_t = playback_t;
	state.update_needed = false;
}

void extrapolation_manager::draw_extrapolations(
	cgv::render::context &ctx, const cgv::vec3 &eye_pos, const cgv::vec3 &view_dir
){
	// Do nothing if no data
	if (geom.node_indices.empty())
		return;

	// Pull in up-to-date render style settings -- FIXME: this should be push rather than pull
	update_render_style(otv_render.style);

	// Obtain handles to buffers we don't directly manage
	render.sort_time_query.begin_scope();
	render.tstr.enable_attribute_array_manager(ctx, render.aam);
	geom.segment_idx_buf_ptr = render.tstr.get_index_buffer_ptr(render.aam);
	const cgv::render::vertex_buffer *node_id_buf_ptr = render.tstr.get_vertex_buffer_ptr(
		ctx, render.aam, "node_ids"
	);

	// Bind the actual parameters to arguments. Argument names must match the ones used to initialize the sorter.
	cgv::gpgpu::argument_binding_list sort_arguments;
	sort_arguments.bind_uniform("u_eye_pos", eye_pos);
	sort_arguments.bind_uniform("u_view_dir", view_dir);
	sort_arguments.bind_buffer("node_index_buffer", *node_id_buf_ptr);

	render.sorter.execute(ctx, geom.nodes_arena.as_vertex_buffer(), geom.node_indices.size(), *geom.segment_idx_buf_ptr, sort_arguments);

	render.sort_time_query.end_scope();

	// Set up draw call
	render.draw_time_query.begin_scope();
	render.tstr.set_render_style(render.style);
	render.tstr.set_cyclopic_eye(eye_pos);
	GLuint buffer_handles[13] = {
		geom.nodes_arena.data_memory.handle(),
		geom.t_to_s_arena.data_memory.handle(),
		cgv::render::gl::get_gl_id(node_id_buf_ptr->handle), // <- required since we're drawing attribute-less
		geom.seg_to_traj_arena.data_memory.handle()
	};
	for (unsigned l=0; l<setup.num_layers; ++l) {
		buffer_handles[4 + l*2] = glyphs.glyph_attribs_arena[l].data_memory.handle();
		buffer_handles[5 + l*2] = glyphs.ranges_arena[l].data_memory.handle();
	}
	for (unsigned nl=setup.num_layers; nl<4; ++nl)
		buffer_handles[5 + nl*2] = buffer_handles[4 + nl*2] = 0;
	buffer_handles[4 + 8] = glyphs.traj_glyph_mem_arena.data_memory.handle();
	glBindBuffersBase(GL_SHADER_STORAGE_BUFFER, 0, 13, buffer_handles);

	// Set uniforms
	render.tube_shading.set_uniforms(
		ctx, render.tstr.ref_prog(), render.style, otv_render.visualizations[0].config
	);

	// Draw
	render.tstr.render(ctx, 0, geom.node_indices.size());
	render.tstr.disable_attribute_array_manager(ctx, render.aam);
	render.draw_time_query.end_scope();

	// Insert sync point for potential buffer flushes
	/*if (render.draw_fence) {
		*//*auto wait_result = *//*glClientWaitSync(render.draw_fence, 0, -1);
		glDeleteSync(render.draw_fence);
	}
	render.draw_fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);*/

	state.rendered_something = true;
}

void extrapolation_manager::collect_timer_queries (void)
{
	// take flush time
	if (state.flushed_something) {
		const auto time_ns = duration_ns(render.flush_time_query.collect());
		stats.flush_times.add_measurement(time_ns);
	}
	if (state.rendered_something)
	{
		/* take sort time */ {
			const auto time_ns = duration_ns(render.sort_time_query.collect());
			stats.sort_times.add_measurement(time_ns);
		}
		/* take draw time */ {
			const auto time_ns = duration_ns(render.draw_time_query.collect());
			stats.draw_times.add_measurement(time_ns);
		}
		/* infer total render time */ {
			const auto time =
				stats.sort_times.measurements.back() + stats.draw_times.measurements.back();
			stats.render_times.add_measurement(time);
		}
		state.rendered_something = false;
	}
}


} // namespace otv
