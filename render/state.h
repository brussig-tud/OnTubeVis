#pragma once

// C++ STL
#include <bitset>
#include <optional>

// CGV framework
#include <cgv/render/context.h>

// CGV framework OpenGL library
#include "libs/cgv_gl/gl/gl_time_query.h"

// CGV framework GPU algorithms
#include <cgv_gpgpu/visibility_sort.h>

// local includes
#include "dbuf_queue.h"
#include "glyph_layer_manager.h"
#include "gpumem/heap_buffer.h"
#include "gpumem/ring_buffer.h"
#include "render/common.h"
#include "render/hash_grid.h"
#include "render/trajectory.h"
#include "textured_spline_tube_renderer.h"


namespace otv {

struct on_tube_visualization {
	glyph_layer_manager::configuration config;
	glyph_layer_manager manager;
	std::shared_ptr<visualization_variables_info> variables;

	on_tube_visualization(cgv::base::base_ptr base_ptr) : manager(base_ptr) {
		variables = std::make_shared<visualization_variables_info>();
	}
};

struct render_state
{
	/// Duration type used for nanosecond timings.
	typedef std::chrono::duration<double, std::nano> duration_ns;

	/// Duration type used for microsecond timings.
	typedef std::chrono::duration<double, std::micro> duration_us;

	/// Duration type used for millisecond timings.
	typedef std::chrono::duration<double, std::milli> duration_ms;

	/// GPU data required to render a glyph layer.
	struct glyph_layer {
		/// The range of glyphs on each segment, relative to the base index of the trajectory's
		/// allocation.
		gpumem::array<index_range<glyph_count_type>> ranges {};
		/// The actual glyph instances.
		/// Conceptually, the element type of this buffer is `std::array<float, n>` for some layer-dependent n that can
		/// only be known at runtime.
		/// For the program to work correctly, the length of this buffer's allocation has to be a multiple of n; its
		/// capacity then has to be one less.
		gpumem::memory_pool<> attribs {};
	};

	/// helper struct for statistics collection
	struct stats_struct
	{
		unsigned num_updates = 0;
		unsigned num_nodes_pushed = 0;
		unsigned num_glyphs_pushed = 0;
		unsigned num_glyphs_committed = 0;
		unsigned num_trims = 0;

		stats_collector<float> nodes_per_push;
		stats_collector<duration_ms> node_push_times;
		stats_collector<float> glyphs_per_push;
		stats_collector<duration_ms> glyph_commit_times;
		stats_collector<duration_ms> traj_trim_times;
		stats_collector<duration_us> buffer_flush_times;
		stats_collector<duration_ms> render_times;

		/// Convenience method to trigger processing for all collected statistics at once.
		void process (void)
		{
			nodes_per_push.process();
			node_push_times.process();
			glyphs_per_push.process();
			glyph_commit_times.process();
			traj_trim_times.process();
			buffer_flush_times.process();
			render_times.process();
		}

		/// Convenience method for printing out the collacted stats (@ref #process() should have already been called).
		void print (std::ostream &os) const
		{
			os << std::endl << ">>> RENDER STATS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl
			   << "-- Timings ------------------------------" << std::endl;
			nodes_per_push.print(os, "nodes_per_push");
			node_push_times.print(os, "node_push_times");
			glyphs_per_push.print(os, "glyphs_per_push");
			glyph_commit_times.print(os, "glyph_commit_times");
			traj_trim_times.print(os, "traj_trim_times");
			buffer_flush_times.print(os, "buffer_flush_times");
			render_times.print(os, "render_times");
			os << std::endl << "-- Counters -----------------------------" << std::endl;
			os << "       num_updates: "<<num_updates << "\n";
			os << "   nodes_committed: "<<num_nodes_pushed << "\n";
			os << "     glyphs_pushed: "<<num_glyphs_pushed << "\n";
			os << "  glyphs_committed: "<<num_glyphs_committed << "\n";
			os << "         num_trims: "<<num_trims << "\n";
			os << std::endl << "<<< \\END RENDER STATS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
		}
	} stats;
	bool flushed_something = false;

	/// render style for the textured spline tubes
	cgv::render::textured_spline_tube_render_style style;

	/// the on-tube visualization layers for each loaded dataset
	std::vector<on_tube_visualization> visualizations;

	/// GPU ring buffer containing trajectory nodes.
	gpumem::ring_buffer<node_attribs> node_buffer;

	/// GPU ring buffer containing trajectory segments defined as pairs of absolute indices into
	/// #node_buffer.
	gpumem::ring_buffer<cgv::uvec2> segment_buffer;

	/// GPU buffer storing which trajectory each segment belongs to.
	/// Entries correspond to #segment_buffer.
	gpumem::array<topology_info> seg_to_traj;

	/// GPU buffer containing segment-wise arclength parametrization.
	/// Entries correspond to #segment_buffer.
	gpumem::array<cgv::mat4> t_to_s;

	/// GPU buffers specific to each glyph layer.
	per_layer<glyph_layer> glyphs;

	/// GPU buffer storing which index range of `glyphs[_].attribs` is used for each
	/// trajectory's glyph attribute buffer.
	/// The entry for trajectory id _t_ and layer _l_ is stored at index
	/// _t * max_glyph_layers + l_.
	gpumem::array<irange> traj_glyph_mem;

	/// shared attribute array manager used by both renderers
	cgv::render::attribute_array_manager aam;

	/// the gpu sorter used to reorder the indices according to their corresponding segment visibility order
	cgv::gpgpu::visibility_sort sorter;

	/// Render data and state specific to each trajectory.
	std::vector<trajectory> trajectories;

	/// GL buffer storing the hash grid.
	std::unique_ptr<gpumem::heap_buffer> grid_mem {};
	/// GPU-accessible 4D hash grid of trajectory intervals.
	/// Used to calculate relations between trajectories on the GPU.
	hash_grid hash_grid {};

	/// Fence placed directly after the last draw command for synchronization with the GPU.
	//GLsync draw_fence;

	/// The minimum number of node slots that will be vacant after each call to
	/// `trim_trajectories`, and thus the maximum number of nodes that can be added each frame
	/// without waiting for draw calls.
	gpumem::size_type reserve_nodes {0};

	/// Stores which glyph layers are updated and rendered.
	std::bitset<max_glyph_layers> active_glyph_layers = 0;

	/// GPU timer object for benchmarking
	cgv::render::gl::gl_time_query flush_time_query, render_time_query;


	render_state() = default;
	~render_state() noexcept;


	/// Create, register and return an empty trajectory.
	trajectory& add_trajectory() {
		trajectories.push_back({static_cast<trajectory::id_type>(trajectories.size()), *this});
		return trajectories.back();
	}

	/// Return the trajectory with a given ID, or `nullptr` if the ID is not in use.
	[[nodiscard]] trajectory *try_get_trajectory(trajectory::id_type id) {
		return id < trajectories.size() ? &trajectories[id] : nullptr;
	}

	/// Implements `otv__stream_spline_node`.
	void enqueue_node (trajectory::id_type trajectory, const node_attribs &node, const cgv::mat4 *t_to_s) {
		_node_queue.push_back({node, t_to_s ? *t_to_s : cgv::mat4{}, trajectory});
	}

	/// Upload new data from the host to the GPU and prune old geometry.
	/// Should be called each frame.
	/// May be executed during ongoing draw calls.
	void update ();

	/// Execute a callback for every active glyph layer.
	template <class Callback, class = std::enable_if_t<
		std::is_invocable_v<Callback, layer_index_type, const glyph_layer&>
	>>
	void for_each_active_glyph_layer (Callback callback)
	{
		for (layer_index_type idx = 0; idx < max_glyph_layers; ++idx) {
			if (active_glyph_layers[idx]) {
				callback(idx, glyphs[idx]);
			}
		}
	}

	/// Allocate memory for rendering up to `max_nodes` trajectory nodes, while adding up to
	/// `reserve_nodes` new ones each frame.
	[[nodiscard]] bool create_geom_buffers (
		cgv::render::context &ctx,
		gpumem::size_type max_nodes,
		gpumem::size_type reserve_nodes
	);

	/// Initialize a layer's glyph attribute containers with sufficient memory to hold the
	/// requested number of glyphs.
	[[nodiscard]] bool create_glyph_layer (
		layer_index_type  layer,
		glyph_size_type   glyph_size,
		gpumem::size_type num_trajectories,
		glyph_count_type  glyphs_per_trajectory
	);

	/// Create a new trajectory hash grid and insert all rendered segments.
	/// Any previous grid is replaced.
	void build_hash_grid (hash_grid::params const&);

	/// Calculate the extent of a glyph relative to its anchor point on the given layer along the trajectory, taking
	/// into account the configured scale. Plot control points have flexible extents which cannot be determined in
	/// isolation (they extend all the way to their neighboring control points), so in case of non-glyphs, a @a none
	/// @c std::optional will be returned.
	///
	/// ToDo: `glyph_data` must point to an array of attributes as they are stored in the render buffer, including
	/// attributes that do not affect the size of the glyph, but excluding arc length and debug info.
	[[nodiscard]] std::optional<cgv::vec2> glyph_extents (layer_index_type layer, const float *glyph_data) {
		// Streaming visualization only supports one dataset.
		const float length_scale_x2 = (style.length_scale+style.length_scale); // to get radius instead of diameter
		const float glyph_length = visualizations[0].config.layer_configs[layer].glyph_length(glyph_data);
		if (glyph_length < 0)
			return {};
		const float radius = glyph_length/length_scale_x2;
		return std::make_optional<cgv::vec2>(-radius, radius);
	}

	/// Calculate the absolute extents of a glyph in terms of arc length covered on the given layer along the
	/// trajectory, taking into account the configured scale. Plot control points have flexible extents which cannot be
	/// determined in isolation (they extend all the way to their neighboring control points), so in case of non-glyphs,
	/// a @a none @c std::optional will be returned.
	///
	/// ToDo: `glyph_data` must point to an array of attributes as they are stored in the render buffer, including
	/// attributes that do not affect the size of the glyph, most importantly the first two meta attributes (arc length
	/// position and debug flag).
	[[nodiscard]] std::optional<cgv::vec2> glyph_range (layer_index_type layer, const float *glyph_data) {
		const auto extents = glyph_extents(layer, glyph_data+2);
		return map_optional(extents, [glyph_data](const cgv::vec2 &extents) {
			return cgv::vec2(*glyph_data + extents.x(), *glyph_data + extents.y());
		});
	}

	/// Calculate the diameter of a glyph on the given layer along the trajectory, taking into account the configured
	/// scale.
	///
	/// ToDo: `glyph_data` must point to an array of attributes as they are stored in the render buffer, including
	/// attributes that do not affect the size of the glyph, but excluding arc length and debug info.
	[[nodiscard]] std::optional<float> glyph_diameter (layer_index_type layer, const float *glyph_data) {
		// Streaming visualization only supports one dataset.
		const auto extents = glyph_extents(layer, glyph_data);
		return map_optional(extents, [](const cgv::vec2 &extents) {
			return extents.y() - extents.x();
		});
	}

	/// Collect all timer queries that were inserted over the course of the current frame.
	void collect_timer_queries (const bool collect_render);

private:
	friend class trajectory;

	/// All information required to append a node to a trajectory.
	struct new_node
	{
		/// Attributes of the node itself.
		node_attribs node;

		/// Arclength parametrization of the segment completed by the node.
		/// May be arbitrary if the node is first in its trajectory.
		cgv::mat4 t_to_s;

		/// Identifies the trajectory to which the node will be added.
		trajectory::id_type trajectory;
	};

	/// Indicates for each node whether or not it is used as the start of a segment.
	std::vector<bool> _node_starts_segment;
	/// Newly added nodes, for which geometry data should be created.
	dbuf_queue<new_node> _node_queue;
	/// Stores which trajectory each node belongs to.
	std::unique_ptr<trajectory::id_type[]> _node_to_traj;
	/// For each segments stores the index of its successor in the trajectory.
	std::unique_ptr<gpumem::index_type[]> _next_segment;

	/// Fill the GPU buffer with nodes from the queue, creating segments where applicable.
	bool append_nodes ();

	/// Logically delete old nodes until the render buffer has capacity for at least `reserve_nodes`
	/// new nodes as well as any nodes that could not be added this frame.
	void trim_trajectories ();
};

} // namespace otv
