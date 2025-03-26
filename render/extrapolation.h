
#ifndef __EXTRAPOLATION_H__
#define __EXTRAPOLATION_H__

// Local includes
#include "glyph_layer_manager.h"
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

		/// Construct from the contents of an instance of the API equivalent struct @c OTV_Extrapolation.
		[[nodiscard]] inline static node from_api_extrapol (const OTV_Extrapolation &api_extrapol) {
			return {
				node_attribs::from_api_node(api_extrapol.node, 0, {0,0,0,0}),
				*(const cgv::mat4*)&api_extrapol.arclen
			};
		}

		/// Construct an instance of the API equivalent struct @c OTV_Extrapolation from the contents of this
		/// extrapolation node.
		[[nodiscard]] inline OTV_Extrapolation into_api_extrapol (void) const {
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
	/// Reference to the OnTubeVis render state
	render_state &render;

	/// Per-trajectory extrapolated node buffers.
	std::vector<extrapol::per_trajectory> trajectories;

	/// Visualization setup meta data
	struct {
		/// The umber of configured layers.
		unsigned num_layers = 0;

		/// The least common multiple of all @ref glyph_attrib_counts.
		unsigned glyph_attribs_lcm = 0;

		/// The attribute counts of each layer.
		std::array<unsigned, 4> glyph_attrib_counts;

		/// Clear every field to 0.
		inline void reset (void) {
			std::fill(glyph_attrib_counts.begin(), glyph_attrib_counts.end(), 0);
			glyph_attribs_lcm = num_layers = 0;
		}
	} setup;

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

	extrapolation_manager(render_state &render) : render(render)
	{}

	~extrapolation_manager (void) {
		clear();
	}

	void clear (void);

	/// Create all buffers relating to pure geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_geom_buffers (unsigned num_trajectories, unsigned num_segments);

	/// Make sure internal knowledge about the layer configuration mirrors the one currently active in the @link render
	/// render state @endlink, invalidating all glyph-related buffers until @ref create_glyph_and_per_layer_buffers() is
	/// called.
	void reinit_layer_config (void);

	/// Create all buffers relating to per-layer non-geometric information (glyph instances, mapping ranges). Must not
	/// be called before @ref reinit_layer_config() has been called at least once!
	bool create_glyph_and_per_layer_buffers (unsigned per_layer_min_glyphs_capacity=128);

	/// Fully replace the current extrapolation for the given trajectory
	void replace_extrapolation (
		unsigned traj_id, const node_attribs &last_measured_node, const std::vector<extrapol::node> &extrapolation
	);

	/// Consider the given range of <b>new</b> glyphs for inclusion on the current extrapolation. The same requirements
	/// about monotonically increasing arc length positions of the provided glyphs apply; most notably that implies that
	/// clients <b>must never</b> feed glyphs to this function that were submitted to the manager for consideration
	/// before.
	///
	/// @return
	///		The sub range of the provided glyphs that were included for display on the extrapolation.
	///
	/// @note
	///		Typically, to-be-considered glyphs (unless clients are greatly behind with submitting them) will succeed the
	///		most recent real segment pending new position measurements, so the returned sub range should be empty ≥90%
	///		of the time. If not, clients should check their node/glyph submission logic.
	template <class Iter>
	ro_range<Iter> consider_glyphs (unsigned traj_id, unsigned layer, const ro_range<Iter> &glyph_attribs);

	/// Return the sub-range of the range @a glyph_attribs on @a layer that have influence after (and including)
	/// @a s_min, searching from the front.
	template <class Iter>
	ro_range<Iter> skip_glyphs_before (unsigned layer, float s_min, const ro_range<Iter> &glyph_attribs);

	/// Return the sub-range of the range @a glyph_attribs on @a layer that have influence after (and including)
	/// @a s_min, searching from the back.
	template <class Iter>
	ro_range<Iter> keep_glyphs_after_including (unsigned layer, float s_min, const ro_range<Iter> &glyph_attribs);

	/// Given a range of Hermite segments and their arc length parameterization, assign the given range of glyphs known
	/// to be from the given layer to these segments such that each segment only references the glyphs that influence
	/// it.
	///
	/// If the input range of glyph attributes is known to be a sub-range of some larger collection, @a idx_offset may
	/// be used offset each assigned glyph index by the given amount.
	template <class IRangeIter, class AlenIter, class GlyphAttribsIter>
	void assign_glyphs (
		ro_range<IRangeIter> &&ranges_out, const ro_range<AlenIter> &t_to_s, unsigned layer,
		const ro_range<GlyphAttribsIter> &glyph_attribs, unsigned idx_offset=0
	);

	/// Flush all changes to GPU buffers, making their current contents visible to the GPU. No guarantees are made that
	/// any changes (@ref replace_extrapolation() etc.) will ever become visible to the GPU unless this is called at
	/// some point. The manager will make an attempt at coalescing the flushes to individual buffers.
	///
	/// @todo Currently blanket-flushes whole arenas. Investigate actual coalescing of unit buffer updates
	bool flush_changes (void) const;
};

} // namespace otv


#endif // ifndef __EXTRAPOLATION_H__
