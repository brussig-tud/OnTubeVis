
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
		gpumem::ring_buffer_alt<node_attribs> nodes;

		/// The ring buffer containing the arc length re-parametrization for each segment.
		gpumem::ring_buffer_alt<cgv::mat4> t_to_s;

		/// construct for the given number of segments
		/*[[nodiscard]] explicit per_trajectory (
			gpumem::array<node_attribs> &nodes_mem, gpumem::array<cgv::mat4> &t_to_s_mem, unsigned num_segments
		){
			nodes.create(nodes_mem, num_segments+1);
			t_to_s.create(t_to_s_mem, num_segments);
		}*/
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

	/// The provider of GPU memory.
	gpumem::memory_pool<> memory;

	/// Members related to the extrapolated trajectories
	struct {
		/// The GPU memory arena backing the node ring buffers.
		gpumem::ring_buffer_arena<node_attribs> nodes_arena;

		/// The GPU memory arena backing the per-segment t_to_s ring buffers.
		gpumem::ring_buffer_arena<cgv::mat4> t_to_s_arena;
	} extrapol;

	/// The contiguous block of memory housing the per-trajectory segment and layer glyph index ranges, sourced from
	/// the @ref glyph_memory pool.
	//gpumem::array<irange> ranges_buffer;

	/// Create all buffers relating to purely geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_geom_buffers (unsigned num_trajectories, unsigned num_segments)
	{
		// Make sure we don't leak resources when any single one of the steps fails
		auto _ = finalizer([this] {
			this->extrapol.t_to_s_arena.destroy();
			this->extrapol.nodes_arena.destroy();
			this->memory.destroy();
		});

		// The number of blocks in the pool follows from the above.
		/*const size_t num_segments_total = num_trajectories*num_segments;
		const size_t num_nodes_total = num_segments_total + num_trajectories;
		success &= node_buffer.create(num_nodes_total, alignement);
		success &= t_to_s_buffer.create(num_segments_total, alignement);*/
		bool success = extrapol.nodes_arena.create(num_trajectories, num_segments+1);
		success &= extrapol.t_to_s_arena.create(num_trajectories, num_segments);

		// Create per-trajectory ringbuffers
		trajectories.reserve(num_trajectories);
		for (unsigned i=0; i<num_trajectories; i++) {
			//auto &new_traj = trajectories.emplace_back();
		}

		// Done!
		return _.disarm(success);
	}
};

} // namespace otv
