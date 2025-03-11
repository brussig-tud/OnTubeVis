
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
	/// The geometry of an extrapolated node forming an extrapolated segment. The segment is being formed either with
	/// a previous extrapolated node, or, in case of the first node in a series, with the most recent measured point.
	struct node
	{
		/// The Hermite node closing the extrapolated segment.
		node_attribs hnode;

		/// The arclength re-parameterization of the extrapolated seegment.
		cgv::mat4 t_to_s;

		/// Construct an instance of the API equivalent struct @c OTV_Extrapolation from the contents of this
		/// extrapolation node.
		inline OTV_Extrapolation into_api_extrapol (void) const {
			return {
				hnode.into_api_node(), *(const OTV_SegmentArclen*)&t_to_s
			};
		}
	};

	struct per_layer {
		/// Reference to the ring buffer to use for storing the glyph index ranges mapped to each segment for this
		/// layer. The capacity of each ringbuffer should match up with the corresponding per-segment ring buffers in
		/// @ref otv::per_trajectory .
		gpumem::ring_buffer_alt<irange> &ranges;

		/// Reference to the ring buffer to use for storing the actual glyph instances on the layer.
		gpumem::ring_buffer_alt<float> &glyph_attribs;

		[[nodiscard]] inline explicit per_layer (
			gpumem::ring_buffer_alt<irange> &ranges_buffer, gpumem::ring_buffer_alt<float> &glyphs_buffer
		)
			: ranges(ranges_buffer), glyph_attribs(glyphs_buffer)
		{}
	};

	struct per_trajectory {
		/// The ring buffer containing the trajectory nodes.
		gpumem::ring_buffer_alt<node_attribs> &nodes;

		/// The ring buffer containing the arc length re-parametrization for each segment.
		gpumem::ring_buffer_alt<cgv::mat4> &t_to_s;

		/// Per-layer data, including the ring buffer of mapped glyph ranges.
		std::vector<per_layer> layers;

		/// Build with the given ring buffers as backing.
		[[nodiscard]] explicit per_trajectory (
			gpumem::ring_buffer_alt<node_attribs> &nodes, gpumem::ring_buffer_alt<cgv::mat4> &t_to_s
		)
			: nodes(nodes), t_to_s(t_to_s)
		{}
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

	/// Members related to the geometry of extrapolated trajectories
	struct {
		/// The GPU arena housing the per-trajectory node ring buffers.
		gpumem::ring_buffer_arena<node_attribs> nodes_arena;

		/// The GPU arena housing the per-trajectory t_to_s ring buffers.
		gpumem::ring_buffer_arena<cgv::mat4> t_to_s_arena;

		/// The GPU arena housing the per-trajectory and per-layer glyph index ranges ring buffers.
		gpumem::ring_buffer_arena<irange> ranges_arena;

		/// The static (except for visibility sorting) index buffer
		std::vector<std::pair<uint32_t, uint32_t>> node_indices;
	} geom;

	/// Members related to the glyph data on extrapolated trajectories
	struct {
		/// The GPU arena housing the per-trajectory and per-layer glyph instances.
		gpumem::ring_buffer_arena<float> glyph_attribs_arena;
	} glyphs;

	~extrapolation_manager (void) {
		clear();
	}

	void clear (void) {
		trajectories.clear();
		glyphs.glyph_attribs_arena.destroy();
		geom.ranges_arena.destroy();
		geom.t_to_s_arena.destroy();
		geom.nodes_arena.destroy();
	}

	/// Create all buffers relating to pure geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_geom_buffers (unsigned num_trajectories, unsigned num_segments);

	/// Create all buffers relating to per-layer non-geometric information (glyph instances, mapping ranges).
	bool create_glyph_and_per_layer_buffers (
		unsigned num_layers, unsigned max_attribs_per_glyph=16, unsigned num_glyphs_capacity=128
	);

	/// Fully replace the current extrapolation for the given trajectory
	void replace_extrapolation (unsigned traj_id, const std::vector<extrapol::node> &extrapolation);

	/// Flush all changes to GPU buffers, making their current contents visible to the GPU. No guarantees are made that
	/// any changes (@ref replace_extrapolation() etc.) will ever become visible to the GPU unless this is called at
	/// some point. The manager will make an attempt at coalescing the flushes to individual buffers.
	///
	/// @todo Currently blanket-flushes whole arenas. Investigate actual coalescing of unit buffers at finer granularity
	bool flush_changes (void) const;
};

} // namespace otv
