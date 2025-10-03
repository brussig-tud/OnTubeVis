
#ifndef __EXTRAPOLATION_H__
#define __EXTRAPOLATION_H__

// C++ STL
#include <chrono>

// CGV Framework
#include "libs/cgv_gl/gl/gl_time_query.h"

// Local includes
#include "glyph_layer_manager.h"
#include "cgv/utils/stopwatch.h"
#include "render/common.h"
#include "render/trajectory.h"
#include "render/state.h"
#include "gpumem/ring_buffer_alt.h"

// Forward declarations
class on_tube_vis;


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
				node_attribs::from_api_node(api_extrapol.node, 0, {0,0,0,0.5}),
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

	struct per_layer
	{
		/// Reference to the ring buffer to use for storing the glyph index ranges mapped to each segment for this
		/// layer. The capacity of each ringbuffer should match up with the corresponding per-segment ring buffers in
		/// @ref otv::per_trajectory .
		gpumem::ring_buffer_alt<irange> &ranges;

		/// Reference to the ring buffer to use for storing the actual glyph instances on the layer.
		gpumem::ring_buffer_alt<float> &glyph_attribs;

		/// Index of the last segment of the containing trajectory that has a glyph or control point assigned _within
		/// its start and end arclength_. This bookkeeping information is required because plots can have infinite
		/// influence, so for themthere is no way to extract this information on-the-fly reliably just by looking at the
		/// @ref ranges information (which can be done easily for glyphs and their finite influence).
		unsigned newest_seg_with_glyphs = 0;

		[[nodiscard]] inline explicit per_layer (
			gpumem::ring_buffer_alt<irange> &ranges_buffer, gpumem::ring_buffer_alt<float> &glyphs_buffer
		)
			: ranges(ranges_buffer), glyph_attribs(glyphs_buffer)
		{}

		inline unsigned real_glyph_idx_from_logical (const unsigned glyph_stride, const unsigned logical_idx) const {
			assert(glyph_attribs.capacity()%glyph_stride == 0);
			const unsigned logical_idx_buf = logical_idx*glyph_stride;
			const unsigned real_idx_buf = glyph_attribs.real_idx_from_logical(logical_idx_buf);
			const unsigned real_idx = real_idx_buf/glyph_stride;
			return real_idx;
		}
		inline unsigned logical_glyph_idx_from_real (const unsigned glyph_stride, const unsigned real_idx) const {
			assert(glyph_attribs.capacity()%glyph_stride == 0);
			const unsigned real_idx_buf = real_idx*glyph_stride;
			const unsigned logical_idx_buf = glyph_attribs.logical_idx_from_real(real_idx_buf);
			const unsigned logical_idx = logical_idx_buf/glyph_stride;
			return logical_idx;
		}
	};

	struct per_trajectory
	{
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
	/// Duration type used for nanosecond timings.
	typedef std::chrono::duration<double, std::nano> duration_ns;

	/// Duration type used for microsecond timings.
	typedef std::chrono::duration<double, std::micro> duration_us;

	/// Duration type used for millisecond timings.
	typedef std::chrono::duration<double, std::milli> duration_ms;

	/// Reference to the OnTubeVis render state
	render_state &otv_render;

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
		gpumem::ring_buffer_arena<node_attribs> nodes_arena/*, test_nodes*/;

		/// The GPU arena housing the per-trajectory t_to_s ring buffers.
		gpumem::ring_buffer_arena<cgv::mat4> t_to_s_arena/*, test_alens*/;

		/// The GPU arena housing the per-segment trajectory IDs to reverse-map from segment to trajectory for fetching
		/// glyph-related data. Technically not needed for extrapolation display, as this mapping could be computed
		/// statically. But we need to mirror the regular trajectories shading interface to keep shader code complexity
		/// down.
		gpumem::ring_buffer_arena<topology_info> seg_to_traj_arena;

		/// The static (except for visibility sorting) index buffer
		std::vector<std::pair<uint32_t, uint32_t>> node_indices;

		/// Pointer to the internally managed (by the t.s.t. renderer) index buffer, for visibility sorting
		const cgv::render::vertex_buffer *segment_idx_buf_ptr = nullptr;
	} geom;

	/// Members related to the glyph data on extrapolated trajectories
	struct {
		/// The GPU arena housing the per-trajectory and per-layer glyph instances.
		gpumem::ring_buffer_arena<float> glyph_attribs_arena [4];

		/// The GPU arena housing the per-trajectory and per-layer glyph index ranges ring buffers.
		gpumem::ring_buffer_arena<irange> ranges_arena [4];

		/// The GPU arena housing the per-segment and per-layer index range delimiting the corresponding glyph data
		/// ring buffer. Technically not needed for extrapolation display, as these ranges could be computed statically.
		/// But we need to mirror the regular trajectories shading interface to keep shader code complexity down.
		gpumem::ring_buffer_arena<irange> traj_glyph_mem_arena;
	} glyphs;

	/// Render state for the extrapolations.
	struct {
		/// Textured spline tube renderer.
		cgv::render::textured_spline_tube_renderer tstr;

		/// Render style for the textured spline tubes.
		cgv::render::textured_spline_tube_render_style style;

		/// The tube shading configuration for the forward renderer.
		tube_shading_settings tube_shading;

		/// Custom attribute array manager for binding the ring buffers to the renderer.
		cgv::render::attribute_array_manager aam;

		/// The GPU sorter for sorting the extrapolated segments back-to-front.
		cgv::gpgpu::visibility_sort sorter;

		/// The first time query object for benchmarking.
		cgv::render::gl::gl_time_query sort_time_query, draw_time_query, flush_time_query;

		/// OpenGL fence object to sync buffer flushes with rendering
		//GLsync draw_fence = nullptr;
	} render;

	/// Logical state.
	struct state_struct
	{
		static constexpr uint16_t SEGMENTS_DIRTY=1, RANGES0_DIRTY=2, RANGES1_DIRTY=4, RANGES2_DIRTY=8, RANGES3_DIRTY=16,
		                          GLYPHS0_DIRTY=32, GLYPHS1_DIRTY=64, GLYPHS2_DIRTY=128, GLYPHS3_DIRTY=256;
		static constexpr uint16_t ranges_dirty_flag[4] = {RANGES0_DIRTY, RANGES1_DIRTY, RANGES2_DIRTY, RANGES3_DIRTY},
		                          glyphs_dirty_flag[4] = {GLYPHS0_DIRTY, GLYPHS1_DIRTY, GLYPHS2_DIRTY, GLYPHS3_DIRTY};

		uint16_t dirty_flags = 0;
		bool update_needed = false;
		std::chrono::time_point<std::chrono::high_resolution_clock> last_frame_timepoint;

		// controls whether we need to fetch timer queries
		// we performed a buffer flush
		bool flushed_something = false;
		// we rendered something
		bool rendered_something = false;
	} state;

	/// Statistics container
	struct stats_struct {
		/// How many times an extrapolation was replaced (causing re-mapping of glyphs).
		unsigned num_replacements = 0;
		/// How many times a single glyph was pushed.
		unsigned num_single_glyph_pushes = 0;
		/// How many times a set of 2 or more glyphs were pushed.
		unsigned num_multi_glyph_pushes = 0;
		/// How many unique glyphs were pushed in total.
		unsigned num_glyphs_pushed = 0;
		/// How many unique glyphs were committed to an extrapolation in total.
		unsigned num_glyphs_comitted = 0;

		/// Segment sorting timings.
		stats_collector<duration_ms> sort_times, draw_times, render_times, replace_times, glyph_push_times;
		stats_collector<duration_us> flush_times;
		stats_collector<float> glyphs_per_replace, glyphs_per_push;

		/// Reset all counters to 0.
		void reset (void) {
			num_replacements = num_single_glyph_pushes = num_multi_glyph_pushes = num_glyphs_pushed
			= num_glyphs_comitted = 0;
		}

		/// Convenience method to trigger processing for all collected statistics at once.
		void process (void)
		{
			sort_times.process();
			draw_times.process();
			render_times.process();
			flush_times.process();
			replace_times.process();
			glyphs_per_replace.process();
			glyph_push_times.process();
			glyphs_per_push.process();
		}

		/// Convenience method for printing out the collacted stats (@ref #process() should have already been called).
		void print (std::ostream &os) const
		{
			os << std::endl << ">>> EXTRAPOLATION STATS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl
			   << "-- Timings ------------------------------" << std::endl;
			sort_times.print(os, "segment_sort");
			draw_times.print(os, "segment_draw");
			render_times.print(os, "total_render");
			flush_times.print(os, "buffer_flush_times");
			replace_times.print(os, "replace");
			glyphs_per_replace.print(os, "relinked_glyphs_per_replace");
			glyph_push_times.print(os, "glyph_push");
			glyphs_per_push.print(os, "glyphs_per_push");
			os << std::endl << "-- Counters -----------------------------" << std::endl;
			os << "       num_replacements: "<<num_replacements << "\n"
			   << "num_single_glyph_pushes: "<<num_single_glyph_pushes << "\n"
			   << " num_multi_glyph_pushes: "<<num_multi_glyph_pushes << "\n"
			   << "               == total: "<<num_single_glyph_pushes+num_multi_glyph_pushes << "\n"
			   << "      num_glyphs_pushed: "<<num_glyphs_pushed << "\n"
			   << "    num_glyphs_comitted: "<<num_glyphs_comitted << "\n";
			os << std::endl << "<<< \\END EXTRAPOLATION STATS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
		}
	} stats;

	extrapolation_manager(render_state &otv_render_state);

	~extrapolation_manager (void) {
		clear();
	}

	void update_render_style (const cgv::render::textured_spline_tube_render_style &rs) {
		render.style = otv_render.style;
		render.style.attrib_mode = cgv::render::textured_spline_tube_render_style::AM_ATTRIBLESS;
		render.style.forward = true; // <- can't use deferred shading with transparency
		render.tube_shading.ao_style.enable = false; // <- not supported when streaming
	}

	void update_tube_shading (
		const tube_shading_settings &tube_shading, const glyph_layer_manager::configuration &layer_config
	){
		render.tube_shading = tube_shading;
		render.tube_shading.alternative_ring_buffer = true;
		auto tube_shading_defines = render.tube_shading.build_tube_shading_defines(
			layer_config, false
		);
		cgv::render::shader_code::set_define(tube_shading_defines, "SHOW_CURSOR", true, false);
		render.tstr.set_additional_defines(tube_shading_defines);
	}

	void clear (void);

	/// Create all buffers relating to pure geometry (nodes, node indices and segment arclength re-parametrizations).
	bool create_geom_buffers (cgv::render::context &ctx, unsigned num_trajectories, unsigned num_segments);

	/// Make sure internal knowledge about the layer configuration mirrors the one currently active in the @link render
	/// render state @endlink, invalidating all glyph-related buffers until @ref create_glyph_and_per_layer_buffers() is
	/// called.
	void reinit_layer_config (void);

	/// Create all buffers relating to per-layer non-geometric information (glyph instances, mapping ranges). Must not
	/// be called before @ref reinit_layer_config() has been called at least once!
	bool create_glyph_and_per_layer_buffers (
		cgv::render::context &ctx, const tube_shading_settings &tube_shading,
		const glyph_layer_manager::configuration &layer_config, unsigned per_layer_min_glyphs_capacity=128
	);

	/// Fully replace the current extrapolation for the given trajectory
	hires_duration_type replace_extrapolation (
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
	ro_range<Iter> consider_glyphs (
		unsigned traj_id, unsigned layer, const ro_range<Iter> &glyph_attribs, hires_duration_type &consumed_time
	);

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
	///
	/// @return
	///		The index of the last segment from the input segment range that got a glyph assigned <em>within its
	///		borders</em>.
	template <class IRangeIter, class AlenIter, class GlyphAttribsIter>
	unsigned assign_glyphs (
		ro_range<IRangeIter> &&ranges_out, const ro_range<AlenIter> &t_to_s, unsigned traj_id, unsigned layer,
		const ro_range<GlyphAttribsIter> &glyph_attribs, unsigned idx_offset=0
	);

	/// Flush all changes to GPU buffers, making their current contents visible to the GPU. No guarantees are made that
	/// any changes (@ref replace_extrapolation() etc.) will ever become visible to the GPU unless this is called at
	/// some point. The manager will make an attempt at coalescing the flushes to individual buffers.
	///
	/// @todo Currently blanket-flushes whole arenas. Investigate actual coalescing of unit buffer updates
	bool flush_changes (void);

	/// Reports @c true if the manager needs to additional frames (e.g. to finish morphing animations), @c false
	/// otherwise. Notably, when @c false is  returned, the application is free to sleep and block (e.g. to safe energy
	/// while waiting for user interaction).
	bool update_needed (void) const;

	/// Update internal state required for rendering, e.g. to update animations.
	void update (const float playback_t);

	/// Draw the managed extrapolations in their current state.
	void draw_extrapolations (cgv::render::context &ctx, const cgv::vec3 &eye_pos, const cgv::vec3 &view_dir);

	/// Collect all timer queries that were inserted over the course of the current frame.
	void collect_timer_queries (void);
};


} // namespace otv


#endif // ifndef __EXTRAPOLATION_H__
