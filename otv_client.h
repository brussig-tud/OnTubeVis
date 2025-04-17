#pragma once

#include <mutex>
#include <condition_variable>

#include <OnTubeVis/OnTubeVis.h>

#include "arclen_helper.h"
#include "color_map_manager.h"
#include "traj_loader.h"
#include "util.h"
#include "render/trajectory.h"
#include "render/extrapolation.h"

#include "api/state/core.h"


// Forward declaration, actually defined in messy OnTubeVis internals
class visualization_variables_info;


namespace otv {

OTV_ColorMap colormap_name_to_api_enum (const std::string &name);
unsigned colormap_name_to_internal_id (const color_map_manager &colormap_mgr, const std::string &colormap_name);
unsigned colormap_api_enum_to_internal_id (const color_map_manager &colormap_mgr, const OTV_ColorMap &color_map);

OTV_InterpolationMode interpolation_attribval_to_api_enum (const float value);
float interpolation_api_enum_to_attribval (const OTV_InterpolationMode interpolation_mode);

/// Stores trajectory data that is known ahead of time in host memory, then gradually feeds it
/// to the renderer to simulate streaming.
/// NOTE: This type is only used for testing and not required to implement the OnTubeVis API.
struct otv_client
{
	/// Wrapper around a rendered trajectory that can be used to simulate streaming by gradually
	/// adding data from a buffer.
	struct trajectory
	{
		trajectory() = delete;
		trajectory(const unsigned i0, const unsigned n, unsigned first_segment, unsigned traj_id)
			: node_idcs{i0, i0+n}, segment_idx(first_segment), id(traj_id)
		{}

		/// The range of nodes in the client's buffer belonging to this trajectory.
		ro_range<unsigned> node_idcs;

		/// The index of the next segment in the client's buffer belonging to this trajectory.
		/// There is always one fewer segment than nodes, so there is no need to store the maximum
		/// index.
		unsigned segment_idx;

		/// The arclength position of the last glyph submitted for this trajectory on each layer. Required because the
		/// indexing into a trajectory glyph series is non-monotonous by nature due to potential overlap between
		/// segments. This is especially the case for plots, which almost always need inter-segment overlap to ensure
		/// smooth interpolation.
		/// Bookkeeping the most recently submitted s positions enables filtering out duplicated references and submit
		/// unique glyphs only, as the submission interface requires (this information will be properly recreated
		/// behind the interface, including re-routing to extrapolations).
		float last_s[4] = {
			-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
			-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity()
		};

		/// The ID under which this is managed by the render state.
		otv::trajectory::id_type id;
	};

	/// Host-side data for one glyph layer.
	struct glyph_layer
	{
		glyph_layer() = default;

		glyph_layer(const glyph_layer &) = delete;
		glyph_layer(glyph_layer &&) noexcept = default;

		glyph_layer &operator= (const glyph_layer &) = delete;
		glyph_layer &operator= (glyph_layer &&) noexcept = default;

		/// The ranges of glyphs in this layer on each trajectory.
		std::vector<index_range<glyph_count_type>> ranges;
		/// The attributes defining each glyph.
		glyph_attributes attribs;
		/// The timestamps of each glyph
		std::vector<float> timestamps;
	};

	/// Helper type for @ref #enqueue_glyph
	typedef ref_with_id<otv::trajectory> trajectory_ref;

	/// session control state - the whole thing is completely atomic (and thus thread-safe) on its public interface.
	class {
		OTV_VisSetupHandle pending_setup = nullptr;
		bool init_pending = true;
		std::mutex mtx;
		std::condition_variable cv;

	public:
		void reset (void) {
			std::lock_guard g(mtx);
			assert(
				!pending_setup && "INTERNAL LOGIC ERROR - otv_client: Must not reset session while setup is pending!"
			);
			init_pending = true;
		}
		void start_setup (OTV_VisSetupHandle setup) {
			std::lock_guard g(mtx);
			assert(
				!pending_setup &&
				"INTERNAL LOGIC ERROR - otv_client: Cannot start a new setup while another is still pending!"
			);
			pending_setup = setup;
		}
		OTV_VisSetupHandle finish_setup () {
			std::lock_guard g(mtx);
			assert(pending_setup && "INTERNAL LOGIC ERROR - otv_client: No pending setup to finish!");
			auto setup = pending_setup;
			pending_setup = nullptr;
			return setup;
		}
		bool is_setup_pending (void) {
			std::lock_guard g(mtx);
			return pending_setup;
		}
		bool is_init_pending (void) {
			std::lock_guard g(mtx);
			return init_pending;
		}
		void signal_init_done (void) {
			std::lock_guard g(mtx);
			assert(
				!pending_setup &&
				"INTERNAL LOGIC ERROR - otv_client: Cannot finish init while setup is pending!"
			);
			init_pending = false;
			cv.notify_all();
		}
		void wait_init_ready (void) {
			std::unique_lock l(mtx);
			assert(
				!pending_setup &&
				"INTERNAL LOGIC ERROR - otv_client: Attempting wait for session start while still in setup!"
			);
			while (init_pending)
				cv.wait(l);
		}
	} session;

	/// helper struct for statistics collection
	struct stats_struct {
		unsigned num_updates = 0;
		stats_collector<extrapolation_manager::duration_ms> overall_updates;

		/// Convenience method to trigger processing for all collected statistics at once.
		void process (void) {
			overall_updates.process();
		}

		/// Convenience method for printing out the collacted stats (@ref #process() should have already been called).
		void print (std::ostream &os) const
		{
			os << std::endl << ">>> CLIENT STATS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl
			   << "-- Timings ------------------------------" << std::endl;
			overall_updates.print(os, "per_frame_update_time");
			os << std::endl << "-- Counters -----------------------------" << std::endl;
			os << "       num_updates: "<<num_updates << "\n";
			os << std::endl << "<<< \\END CLIENT STATS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
		}
	} stats;


	/// the render "servers" (one each for real and extrapolated) used by the client.
	render_state &render;

	/// the playback time
	float playback_t = -std::numeric_limits<float>::infinity();

	/// session settings
	unsigned num_extrapol_segments = 3;
	bool use_natural_progression = false;

	/// the extrapolation manager
	extrapolation_manager extrapol_mgr;

	/// render data generated by the trajectory manager
	const traj_manager<float>::render_data *data;

	/// glyph data.
	per_layer<glyph_layer> glyphs;

	/// segment-wise arclength approximations (set of 4 cubic bezier curves returning global
	/// trajectory arclength at the segment, packed into the columns of a 4x4 matrix)
	arclen::parametrization arclen_data;

	/// pre-computed trajectory extrapolation for each Hermite node.
	std::vector<std::vector<extrapol::node>> extrapols;

	/// data specific to each trajectory.
	std::vector<trajectory> trajectories;


	/// construct with reference to trajectory render data
	otv_client(render_state &render)
		: render(render), extrapol_mgr(render)
	{}

	/// initialize a completely new session (i.e. a new dataset and layer configuration)
	void new_session (void);

	/// in case there is no ongoing setup, initialize a completely new session (i.e. a new dataset and layer
	/// configuration), otherwise don't do anything. Returns @c true if a new session was started, @c false otherwise.
	bool new_session_if_not_in_setup (void);

	/// transition the session into an ongoing-setup state
	void begin_setup (const std::string &name);

	/// start a pending session with the current setup
	void commit_session (void);

	/// pre-compute all extrapolations for the current dataset
	void precompute_extrapolations (void);

	/// append all data points up to the current timestamp to their respective trajectory.
	void update (void);

	/// Finds the trajectory with the given ID and returns a reference to it.
	[[nodiscard]] trajectory_ref find_trajectory (unsigned id) const {
		return { id, *render.try_get_trajectory(id) };
	}

	/// perform necessary logic to enqueue a node.
	void enqueue_node (
		trajectory_ref target, const node_attribs &node, const cgv::mat4 *t_to_s,
		const std::vector<extrapol::node> &extrapol
	);

	/// perform necessary logic to enqueue glyphs. Returns the sub-range of the input range that currently need to be
	/// displayed on an extrapolation.
	template <class Iter>
	ro_range<Iter> enqueue_glyphs (trajectory_ref traj, unsigned layer, const ro_range<Iter> &glyph_data);

	/// for service mode: enqueue a new hermite node to be uploaded to the GPU ring buffer of the indicated trajectory
	/// NOTE: argument `node` will be updated with the color and radius (plus derivative) of the selected trajectory, so
	/// the caller doesn't have to find that out itself, i.e. can just directly feed in the result of a call to
	/// otv_client::convert_api_node_to_internal()
	void service_push_spline_node (
		unsigned traj_id, node_attribs &&node, const cgv::mat4 *t_to_s, std::vector<extrapol::node> &&extrapol
	);

	/// for service mode: enqueue glyphs to the given layer of the given trajectory
	void service_push_glyphs (unsigned traj_id, unsigned layer, std::vector<float> &&glyph_data);

	/// convert the given streaming API representation of a Hermite node to the internal one used for rendering, leaving
	/// fields holding information about color and radius uninitialized (as the API does not support them)
	/// NOTE: otv_client::service_push_spline_node will look up the correct values for these from the dummy dataset!
	static node_attribs convert_api_node_to_internal (const OTV_HermiteNode &node);

	/// convert the given streaming API representation of a trajectory extrapolation to the internal one used for
	/// rendering.
	static std::vector<extrapol::node> convert_api_extrapol_to_internal (
		const std::vector<OTV_Extrapolation> &extrapol
	);

	/// convert the given streaming API representation of an array of glyphs <b>of the same type and configuration</b>
	/// to the internally used array of floats.
	static std::vector<float> convert_api_glyphs_to_internal (
		unsigned traj_id, unsigned layer, const std::vector<OTV_GlyphData> &glyphs
	);
};


} // namespace otv
