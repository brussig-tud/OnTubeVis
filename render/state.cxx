// local includes
#include "gpumem/memory_pool.inl"
#include "gpumem/ring_buffer.inl"

// implemented header
#include "render/state.h"


namespace otv {

void render_state::update ()
{
	// Upload nodes from the host queue to the render buffer
	flushed_something = append_nodes();

	// Upload glyphs from the host queue to the render buffer
	const auto start_time = std::chrono::high_resolution_clock::now();
	unsigned glyphs_processed=0, glyphs_discarded=0; {
		for (auto &trajectory : trajectories) {
			const auto [_glyphs_processed, _glyphs_discarded] = trajectory.update_glyphs();
			glyphs_processed += _glyphs_processed;
			glyphs_discarded += _glyphs_discarded;
		}
	}
	flushed_something |= glyphs_processed;

	// Log glyph management stats
	if (glyphs_processed) {
		const auto time = std::chrono::high_resolution_clock::now() - start_time;
		stats.glyph_commit_times.add_measurement(time);
		stats.glyphs_per_push.add_measurement((float)glyphs_processed);
		const unsigned num_glyphs_comitted = glyphs_processed - glyphs_discarded;
		stats.num_glyphs_pushed += glyphs_processed;
		stats.num_glyphs_committed += num_glyphs_comitted;
	}

	// Logically delete old nodes from the render buffer
	// Since old data may still be in use by a draw call, it cannot be overwritten yet
	trim_trajectories();

	// Flush geometry buffers
	auto new_segments = segment_buffer.flush_range();
	glFlush();  // Flushing the pipeline is the only way to
	glFinish(); // time buffer uploads unfortunately :/
	flush_time_query.begin_scope();
	std::ignore = node_buffer.flush();
	std::ignore = segment_buffer.flush();
	std::ignore = seg_to_traj.flush_wrapping(new_segments);
	std::ignore = t_to_s.flush_wrapping(new_segments);

	// Flush glyph buffers
	for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
		std::ignore = layer.ranges.flush_wrapping(new_segments);
	});
	for (auto &trajectory : trajectories)
		std::ignore = trajectory.flush_glyph_attribs();
	glFlush();
	glFinish();
	flush_time_query.end_scope();
	stats.num_updates += flushed_something;
}

bool render_state::append_nodes ()
{
	// During the previous frame, `trim_trajectories` should have freed the configured amount of
	// capacity.
	assert(node_buffer.free_capacity() >= reserve_nodes);

	// Start measuring time
	const auto start_time = std::chrono::high_resolution_clock::now();

	// Check whether the GPU buffer can now fit all nodes that could not be pushed on the previous
	// frame.
	auto capacity {node_buffer.free_capacity() - reserve_nodes};

	if (capacity < _node_queue.length()) {
		// If not, discard the oldest nodes that don't fit.
		// Since `trim_trajectories` makes room for the entire backlog, this can only happen when
		// appending more nodes at once than the buffer's maximum capacity, in which case the
		// previous call to `trim_trajectories` has cleared the entire buffer.
		// Because of this, nodes can be safely discarded without the risk of creating wrong
		// segments by skipping nodes.
		assert(_node_queue.length() > node_buffer.capacity() && node_buffer.is_empty());
		_node_queue.pop(_node_queue.length() - capacity);
	}

	// Push nodes to the GPU buffer, creating segments where applicable.
	bool did_something = false;

	// All nodes in the backlog have been processed, remove them from the queue and swap buffers.
	_node_queue.flush();

	// Push as many new nodes as fit into the GPU buffer.
	// The rest will be added next frame, after old nodes have been deleted to make room.
	const auto end {_node_queue.begin() + std::min(
		_node_queue.length(),
		static_cast<std::size_t>(node_buffer.free_capacity())
	)};

	unsigned nodes_comitted = 0;
	for (auto node {_node_queue.begin()}; node != end; ++node) {
		auto& traj = *try_get_trajectory(node->trajectory);

		// Save the current end of the trajectory.
		auto const prev_node     = traj.most_recent_node();
		auto const prev_node_idx = traj.last_node_idx();

		// Add the new node.
		traj.append_node(node->node, &node->t_to_s);

		// If this created a new segment, insert it into the hash grid.
		if (traj.last_segment_idx() != trajectory::nil)
			traj_grid.add_segment(
				prev_node,
				traj.most_recent_node(),
				{static_cast<uint32_t>(prev_node_idx), static_cast<uint32_t>(traj.last_node_idx())},
				node->t_to_s
			);

		_node_queue.pop();
		++nodes_comitted;
		did_something = true;
	}

	// Done! Update stats and return
	if (did_something) {
		const auto time = std::chrono::high_resolution_clock::now() - start_time;
		stats.node_push_times.add_measurement(time);
		stats.num_nodes_pushed += nodes_comitted;
		stats.nodes_per_push.add_measurement((float)nodes_comitted);
	}

	return did_something;
}

void render_state::trim_trajectories ()
{
	// Timestamp marking nodes whose segment has been deleted.
	constexpr auto unlinked_node {std::numeric_limits<float>::infinity()};

	// Delete nodes and their associated segments until there is sufficient capacity for all new
	// nodes that could not be pushed this frame plus the configured number of nodes reserved for
	// next frame.
	auto       free_capacity   {node_buffer.free_capacity()};
	const auto target_capacity {std::min(
		reserve_nodes + static_cast<gpumem::size_type>(_node_queue.length()),
		node_buffer.capacity()
	)};

	const auto start_time = std::chrono::high_resolution_clock::now();
	const bool trimmed = free_capacity < target_capacity;
	while (free_capacity < target_capacity)
	{
		// Draw calls are over a contiguous (except for wrap-around) range of segments, so segments
		// must be deleted in order.
		auto *segment = segment_buffer.try_first();

		if (segment) {
			// By construction, a segment's first node is always the older one.
			auto &node = node_buffer.as_span()[segment->x()];

			// The node no longer starts a segment, so it may be deleted.
			_node_starts_segment[segment->x()] = false;

			// Notify the trajectory to which the segment belonged.
			trajectories[seg_to_traj[segment_buffer.front()].traj]
				.on_delete_segment(segment_buffer.front());

			// Remove the segment.
			segment_buffer.pop_front();
		}

		// Remove all nodes no longer used by a segment from the front of the buffer.
		// Depending on order, some unused nodes could be kept for now, but this is fine; they will
		// be removed by one of the next trimmings.
		while (auto *node = node_buffer.try_first())
		{
			if (_node_starts_segment[node_buffer.front()]) {
				assert(! segment_buffer.is_empty());
				break;
			}

			// Notify the trajectory that the node is being deleted.
			trajectories[_node_to_traj[node_buffer.front()]].on_delete_node(node_buffer.front());

			// Delete the node.
			node_buffer.pop_front();
			++free_capacity;
		}
	}

	// Done! Update stats and return
	if (trimmed) {
		const auto time = std::chrono::high_resolution_clock::now() - start_time;
		stats.traj_trim_times.add_measurement(time);
		++stats.num_trims;
	}
}

bool render_state::create_geom_buffers (
	cgv::render::context &ctx, gpumem::size_type max_nodes, gpumem::size_type reserve_nodes
){
	// Make sure the timer objects are created
	if (!flush_time_query.is_initialized())
		flush_time_query.init(ctx);
	if (!render_time_query.is_initialized())
		render_time_query.init(ctx);

	// Store desired margin.
	this->reserve_nodes = reserve_nodes;

	// Allocate memory for rendered and reserved nodes.
	auto capacity {max_nodes + reserve_nodes};

	if (!node_buffer.create(capacity))
		return false;

	_node_starts_segment = std::vector<bool>(node_buffer.as_span().length());

	// Each trajectory has one fewer segments than nodes.
	const auto ok {segment_buffer.create(capacity - 1)
		// The length of these buffers must match the segment buffer's backing memory, not its
		// capacity!
		&& seg_to_traj.create(segment_buffer.as_span().length())
		&& t_to_s.create(segment_buffer.as_span().length())
	};

	// Allocate CPU buffers.
	_node_to_traj = std::make_unique<trajectory::id_type[]>(node_buffer.as_span().length());
	_next_segment = std::make_unique<gpumem::index_type[]>(segment_buffer.as_span().length());
	return ok;
}

bool render_state::create_glyph_layer (
	layer_index_type  layer,
	glyph_size_type   glyph_size,
	gpumem::size_type num_trajectories,
	glyph_count_type  glyphs_per_trajectory
) {
	// Return the memory used by each trajectory for the layer's glyphs to the pool.
	// Only after all memory has been returned can the pool be destroyed/recreated.
	for (auto &traj : trajectories) {
		traj.destroy_glyph_layer(layer);
	}

	// Allocate the memory pool for glyph attributes that is shared between trajectories.
	// The ring buffer implementation requires room for one additional glyph per trajectory.
	if (! glyphs[layer].attribs.create(
		num_trajectories,
		(glyphs_per_trajectory.value + 1) * glyph_size * gpumem::memsize<float>,
		alignof(float)
	)) {
		return false;
	}

	// Initialize each trajectory's glyph attribute buffer.
	for (auto &trajectory : trajectories) {
		auto &traj     {trajectory};
		const auto mem {traj.create_glyph_layer(layer, glyph_size, glyphs_per_trajectory)};

		if (! mem.data()) {
			return false;
		}

		// Store the allocated memory range as offsets into the GPU buffer for access in the shader.
		const auto offset {static_cast<irange::index_type>(
			mem.data() - reinterpret_cast<float*>(glyphs[layer].attribs.as_span().data())
		)};

		traj_glyph_mem[traj.id() * max_glyph_layers + layer] = {
			offset / traj.glyph_sizes()[layer],
			static_cast<irange::index_type>(mem.length()) / traj.glyph_sizes()[layer]
		};
	}

	// Allocate memory for each segment's glyph range.
	return glyphs[layer].ranges.create(segment_buffer.as_span().length());
}

void render_state::create_traj_grid (cgv::vec4 cell_size)
{
	// Allocate a coherently mapped 1 GiB buffer.
	grid_mem = {1 << 30, gpumem::heap::sync_mode::coherent};
	// Initialize the grid with 2^10 buckets.
	traj_grid = {&grid_mem, cell_size, 10};
}

void render_state::collect_timer_queries (const bool collect_render)
{
	// take flush time
	if (flushed_something) {
		const auto time_ns = duration_ns(flush_time_query.collect());
		stats.buffer_flush_times.add_measurement(time_ns);
	}

	// take render time
	if (collect_render) {
		auto time_ns = duration_ns(render_time_query.collect());
		stats.render_times.add_measurement(time_ns);
	}
}

} // namespace otv
