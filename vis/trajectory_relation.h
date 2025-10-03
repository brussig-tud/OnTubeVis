#pragma once

// C++ STL
#include <cstdint>

// CGV framework
#include <cgv/math/fvec.h>
#include <cgv/media/color.h>
#include <cgv/reflect/reflect_enum.h>
#include <cgv/render/shader_code.h>

// forward declarations
namespace cgv::gui {
	class provider;
}
namespace cgv::render {
	class context;
	class shader_program;
}


namespace otv::vis {

/// Parameters for on-the-fly calculation and visualization of relations between trajectories.
struct trajectory_relation {
	/// Color marking special trajectory parts.
	cgv::rgb highlight_color {1, 0, 1};
	/// Color of trajectories for which no relation value is calculated.
	cgv::rgb background_color {1.0f/3};
	/// Relations are calculated between points no further apart than `radius[0]` in space and
	/// `radius[1]` in time.
	cgv::vec2 radius {1};
	/// Relation values mapped onto the endpoints of the color scale.
	cgv::vec2 color_range {0, 1};
	enum class shading : uint32_t { // uint8_t causes problems with CGV GUI.
		forward,
		deferred,
	}
	/// Determines at which stage of the rendering pipeline relations are evaluated.
	shading {shading::deferred};
	enum class function : uint32_t { // uint8_t causes problems with CGV GUI.
		none,               // No visualization.
		distance,           // Euclidean spatial distance between trajectories.
		dbg_seg_t,          // Segment-local curve parameter.
		dbg_index_xyz,      // Spatial grid index.
		dbg_index_t,        // Temporal grid index.
		dbg_signature,      // Index hash signature.
		dbg_bucket_load,    // Hash bucket load factor.
		dbg_local_interval, // Curve parameter relative to local trajectory interval.
		dbg_skipped_cells,  // Number of cells in query AABB but outside query radius.
		dbg_num_cells,      // Number of cells within the query radius.
		dbg_num_intervals,  // Number of trajectory intervals within queried cells.
		dbg_num_samples,    // Number of sampled trajectory points.
		dbg_num_evals,      // Number of trajectory samples within the query radius.
	}
	/// The value to visualize.
	function {};
	enum class direction : uint32_t { // uint8_t causes problems with CGV GUI.
		/// Only evaluate the relation for the reference trajectory.
		ref_to_all,
		/// Evaluate the relation with the reference trajectory.
		all_to_ref,
		/// Evaluate the relation for every pair of trajectories.
		all_to_all,
	}
	/// Determines for which pairs of trajectories the relation is visualized.
	direction {direction::all_to_ref};
	/// Trajectory evaluations per unit of time to calculate relation.
	float sample_rate {1};
	/// Color scale used to visualize the relation value.
	struct {int32_t index;} color_map {12/*imola*/};
	/// ID of the "reference trajectory" whose meaning depends on `direction`.
	uint32_t reference_trajectory {0};
	/// Determines whether the relation is averaged (true) or accumulated (false) over time.
	bool normalize {true};
	/// Determines whether the relation is mapped onto the color scale linearly (false) or
	/// logarithmically (true).
	bool log_scale {false};

	/// Generate GUI elements to control member variables.
	void build_gui (
		cgv::gui::provider&,
		uint32_t  num_trajectories,
		cgv::vec4 data_extent,
		std::vector<std::string> const& color_maps
	);

	/// Select sensible default values for a dataset of the given size.
	void set_defaults (cgv::vec4 extent);

	/// Statically configure shaders through macros.
	void set_defines (cgv::render::shader_define_map&) const;
	/// Dynamically configure shaders through uniforms.
	void set_uniforms (cgv::render::context&, cgv::render::shader_program&) const;
};

// Reflect enum members so changes can be detected in `on_set`.
auto get_reflection_traits(enum otv::vis::trajectory_relation::shading const&)
	-> cgv::reflect::enum_reflection_traits<enum otv::vis::trajectory_relation::shading>;
auto get_reflection_traits(enum otv::vis::trajectory_relation::function const&)
	-> cgv::reflect::enum_reflection_traits<enum otv::vis::trajectory_relation::function>;

} // namespace otv::vis
