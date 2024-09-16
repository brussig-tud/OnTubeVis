#pragma once

// C++ STL
#include <bitset>

// CGV framework GPU algorithms
#include <cgv_gpgpu/visibility_sort.h>

// local includes
#include "dbuf_queue.h"
#include "glyph_layer_manager.h"
#include "textured_spline_tube_renderer.h"
#include "gpumem/ring_buffer.h"
#include "render/trajectory.h"
#include "render/common.h"


namespace otv {

struct on_tube_visualization {
	glyph_layer_manager::configuration config;
	glyph_layer_manager manager;
	std::shared_ptr<visualization_variables_info> variables;

	on_tube_visualization(cgv::base::base_ptr base_ptr) : manager(base_ptr) {
		variables = std::make_shared<visualization_variables_info>();
	}
};

struct render_state {
public:
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
	gpumem::array<uint> seg_to_traj;

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

	/// Fence placed directly after the last draw command for synchronization with the GPU.
	GLsync draw_fence;

	/// The minimum number of node slots that will be vacant after each call to
	/// `trim_trajectories`, and thus the maximum number of nodes that can be added each frame
	/// without waiting for draw calls.
	gpumem::size_type reserve_nodes {0};

	/// Stores which glyph layers are updated and rendered.
	std::bitset<max_glyph_layers> active_glyph_layers = 0;


	/// Create, register and return an empty trajectory.
	[[nodiscard]] trajectory &add_trajectory()
	{
		trajectories.push_back({static_cast<trajectory::id_type>(trajectories.size()), *this});
		return trajectories.back();
	}

	/// Return the trajectory with a given ID, or `nullptr` if the ID is not in use.
	[[nodiscard]] trajectory *try_get_trajectory(trajectory::id_type id)
	{
		return id < trajectories.size() ? &trajectories[id] : nullptr;
	}

	/// Implements `otv__stream_spline_node`.
	void enqueue_node (trajectory::id_type trajectory, const node_attribs &node, const cgv::mat4 *t_to_s)
	{
		_node_queue.push_back({node, t_to_s ? *t_to_s : cgv::mat4{}, trajectory});
	}

	/// Fill the GPU buffer with nodes from the queue, creating segments where applicable.
	void append_nodes ();

	/// Logically delete old nodes until the render buffer has capcity for at least `reserve_nodes`
	/// new nodes as well as any nodes that could not be added this frame.
	void trim_trajectories ();

	/// Execute a callback for every active glyph layer.
	template <class Callback, class = std::enable_if_t<
		std::is_invocable_v<Callback, layer_index_type, const glyph_layer&>>
	>
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

	/// Calculate the extent of a glyph on the given layer along the trajectory, taking into account
	/// the configured scale.
	/// `glyph_data` must point to an array of attributes as they are stored in the render buffer,
	/// including attributes that do not affect the size of the glyph, but excluding arc length and
	/// debug info.
	[[nodiscard]] float glyph_length(layer_index_type layer, const float *glyph_data) {
		// Streaming visualization only supports one dataset.
		return visualizations[0].config.layer_configs[layer].glyph_length(glyph_data)
			/ style.length_scale;
	}

private:
	/// All information required to append a node to a trajectory.
	struct new_node {
		/// Attributes of the node itself.
		node_attribs node;
		/// Arclength parametrization of the segment completed by the node.
		/// May be arbitrary if the node is first in its trajectory.
		cgv::mat4 t_to_s;
		/// Identifies the trajectory to which the node will be added.
		trajectory::id_type trajectory;
	};

	/// Newly added nodes, for which geometry data should be created.
	dbuf_queue<new_node> _node_queue;
};

} // namespace otv
