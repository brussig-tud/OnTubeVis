
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

	struct node
	{
		/// The Hermite node closing the extrapolated segment.
		node_attribs hnode;

		/// The arclength re-parameterization of the extrapolated seegment.
		cgv::mat4 t_to_s;

		inline OTV_Extrapolation into_api_extrapol (void) const {
			return {
				hnode.into_api_node(), *(const OTV_SegmentArclen*)&t_to_s
			};
		}
	};

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

	void compute_path (
		std::vector<extrapol::node> &out, const unsigned num, const node_attribs &ref_node0,
		const node_attribs &ref_node1, const cgv::mat4 &ref_t_to_s
	);

	inline std::vector<extrapol::node> compute_path (
		const unsigned num, const node_attribs &ref_node0, const node_attribs &ref_node1, const cgv::mat4 &ref_t_to_s
	){
		std::vector<extrapol::node> out;
		out.reserve(num);
		compute_path(out, num, ref_node0, ref_node1, ref_t_to_s);
		return out;
	}
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
	bool create_geom_buffers (unsigned num_trajectories, unsigned num_segments);

	/// Create all buffers relating to purely geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_glyph_and_per_layer_buffers (unsigned num_layers);

	///
	void replace_extrapolation (unsigned id, const std::vector<extrapol::node> &extrapolation);
};

} // namespace otv
