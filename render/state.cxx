#include "render/state.h"

#include "gpumem/memory_pool.inl"
#include "gpumem/ring_buffer.inl"


namespace otv {

void render_state::append_nodes ()
{
	// During the previous frame, `trim_trajectories` should have freed the configured amount of
	// capacity.
	assert(node_buffer.free_capacity() >= reserve_nodes);

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
	for (const auto &node : _node_queue) {
		try_get_trajectory(node.trajectory)->append_node(node.node, &node.t_to_s);
	}

	// All nodes in the backlog have been processed, remove them from the queue and swap buffers.
	_node_queue.flush();

	// Push as many new nodes as fit into the GPU buffer.
	// The rest will be added next frame, after old nodes have been deleted to make room.
	const auto end {_node_queue.begin() + std::min(
		_node_queue.length(),
		static_cast<std::size_t>(node_buffer.free_capacity())
	)};

	for (auto node {_node_queue.begin()}; node != end; ++node) {
		try_get_trajectory(node->trajectory)->append_node(node->node, &node->t_to_s);
		_node_queue.pop();
	}
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

	while (free_capacity < target_capacity) {
		// Draw calls are over a contiguous (except for wrap-around) range of segments, so segments
		// must be deleted in order.
		auto *segment = segment_buffer.try_first();

		if (segment) {
			// By construction, a segment's first node is always the older one.
			auto &node = node_buffer.as_span()[segment->x()];

			// Mark the node for deletion.
			// NOTE: This could mean writing to storage currently read by a draw call, which is UB.
			// Chances and severity of actual problems, however, are low, so this simple approach is
			// used for now.
			node.t[0] = unlinked_node;

			// Allow the glyphs on the removed segment to be overwritten.
			for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
				const auto traj_id = seg_to_traj[segment_buffer.front()];
				const auto range   = layer.ranges[segment_buffer.front()];
				trajectories[traj_id].drop_glyphs(layer_idx, range.n);
			});

			// Remove the segment.
			segment_buffer.pop_front();
		} else {
			// If there are no more segments, yet the target capacity has not been reached, that
			// means there must be nodes which do not belong to a segment because they are the only
			// ones in their respective trajectories.
			// There must be at least one such node, or we would have already reached our target
			// capacity.
			auto &node = *node_buffer.try_first();
			// The node cannot have started a segment, or it would have been deleted already.
			assert(node.t[0] != unlinked_node);

			// Instead, the node has to be newest one on some trajectory.
			const auto traj {std::find_if(trajectories.begin(), trajectories.end(), [&](auto &t) {
				return t.last_node_idx() == node_buffer.front();
			})};
			assert(traj != trajectories.end());

			// Mark the trajectory as empty, so appending a new node will not create a segment with
			// the deleted one.
			traj->mark_empty();

			// Remove the node.
			node_buffer.pop_front();
			++free_capacity;
		}

		// Remove all nodes no longer used by a segment from the front of the buffer.
		// Depending on order, some unused nodes could be kept for now, but this is fine; they will
		// be removed by one of the next trimmings.
		while (auto *node = node_buffer.try_first()) {
			if (node->t[0] != unlinked_node) {
				break;
			}

			node_buffer.pop_front();
			++free_capacity;
		}
	}
}

bool render_state::create_geom_buffers (
	gpumem::size_type max_nodes,
	gpumem::size_type reserve_nodes
) {
	// Store desired margin.
	this->reserve_nodes = reserve_nodes;

	// Allocate memory for rendered and reserved nodes.
	auto capacity {max_nodes + reserve_nodes};

	if (! node_buffer.create(capacity)) {
		return false;
	}

	// Each trajectory has one fewer segments than nodes.
	--capacity;

	return segment_buffer.create(capacity)
		&& seg_to_traj.create(capacity)
		&& t_to_s.create(capacity);
}

bool render_state::create_glyph_layer (
	layer_index_type  layer,
	glyph_size_type   glyph_size,
	gpumem::size_type num_trajectories,
	glyph_count_type  glyphs_per_trajectory
) {

	// Allocate the memory pool for glyph attributes that is shared between trajectories.
	// The ring buffer implementation requires room for one additional glyph per trajectory.
	auto ok {glyphs[layer].attribs.create(
		num_trajectories,
		(glyphs_per_trajectory.value + 1) * glyph_size * gpumem::memsize<float>,
		alignof(float)
	)};

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

} // namespace otv
