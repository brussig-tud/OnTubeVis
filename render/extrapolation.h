
// C++ STL
#include <optional>

// local includes
#include "dbuf_queue.h"
#include "glyph_layer_manager.h"
#include "textured_spline_tube_renderer.h"
#include "render/common.h"
#include "render/trajectory.h"
#include "render/state.h"
#include "gpumem/alloc.h"
#include "gpumem/array.h"
#include "gpumem/memory_pool.h"
#include "gpumem/span.h"
#include "gpumem/ring_buffer_alt.h"


namespace otv
{

namespace extrapol
{
	struct per_trajectory {
		/// The ring buffer containing the trajectory nodes.
		gpumem::ring_buffer_alt<node_attribs> &nodes;

		/// The ring buffer containing the arc length re-parametrization for each segment.
		gpumem::ring_buffer_alt<cgv::mat4> &t_to_s;

		/// The ring buffer containing the per-layer glyph index range for each segment.
		std::vector<gpumem::ring_buffer_alt<irange>*> ranges;

		/// Build with the given ring buffers as backing.
		[[nodiscard]] explicit per_trajectory (
			gpumem::ring_buffer_alt<node_attribs> &nodes, gpumem::ring_buffer_alt<cgv::mat4> &t_to_s
		)
			: nodes(nodes), t_to_s(t_to_s)
		{}
	};

	struct per_layer {
		/// The ring buffer containing the glyph index ranges mapped to each segment for this layer. The capacity of each
		/// ringbuffer should match up with the correpsonding per-segment ring buffers in @ref otv::per_trajectory .
		gpumem::ring_buffer_alt<node_attribs> ranges;
	};
} // namespace extrapol

struct extrapolation_manager
{
	/// Per-trajectory extrapolated node buffers.
	std::vector<extrapol::per_trajectory> trajectories;

	/// Members related to the extrapolated trajectories
	struct {
		/// The GPU arena housing the per-trajectory node ring buffers.
		gpumem::ring_buffer_arena<node_attribs> nodes_arena;

		/// The GPU arena housing the per-trajectory t_to_s ring buffers.
		gpumem::ring_buffer_arena<cgv::mat4> t_to_s_arena;

		/// The GPU arena housing the per-trajectory and per layer glyph index ranges ring buffers.
		gpumem::ring_buffer_arena<irange> ranges_arena;

		/// The static (except for visibility sorting) index buffer
		std::vector<std::pair<uint32_t, uint32_t>> node_indices;
	} geom;

	~extrapolation_manager (void) {
		clear();
	}

	void clear (void) {
		trajectories.clear();
		geom.ranges_arena.destroy();
		geom.t_to_s_arena.destroy();
		geom.nodes_arena.destroy();
	}

	/// Create all buffers relating to pure geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_geom_buffers (unsigned num_trajectories, unsigned num_segments)
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

	/// Create all buffers relating to purely geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_glyph_and_per_layer_buffers (unsigned num_layers)
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
		});

		// Create GPU ring buffer arenas
		const auto num_trajectories = geom.t_to_s_arena.num_ring_buffers();
		const auto num_segments = geom.t_to_s_arena.ring_buffer_capacity();
		const auto num_range_buffers = num_trajectories*num_layers;
		const bool success = geom.ranges_arena.create(num_range_buffers, num_segments);

		// Assign range ring buffers from arena to corresponding layer on each trajectory
		for (unsigned t=0; t<num_trajectories; t++) {
			auto &traj = trajectories[t];
			traj.ranges.reserve(num_layers);
			for (unsigned l=0; l<num_layers; l++) {
				const unsigned per_layer_buffer_id = t*num_layers + l;
				traj.ranges.emplace_back(&geom.ranges_arena.buffer(per_layer_buffer_id));
			}
		}

		// Done!
		return _.disarm(success);
	}
};

} // namespace otv
