#pragma once

#include <cstdint>

#include <cgv/math/fvec.h>
#include <cgv/media/color_scale.h>
#include <cgv/reflect/reflect_enum.h>
#include <cgv/render/texture.h>

namespace cgv::gui {
	class provider;
}
namespace cgv::media {
	class transfer_function;
};
namespace cgv::render {
	class context;
	class shader_compile_options;
	class shader_program;
}

class color_map_manager;


/// Parameters for on-the-fly calculation and visualization of relations between trajectories.
struct relation_vis {
	/// Wrapper class presented to the CGV framework as an enum to get proper dropdowns.
	template <class T>
	struct PseudoEnum {T value;};

	/// The data variable to compute and visualize.
	enum class DataVar : uint32_t { // uint8_t causes problems with CGV GUI.
		none,               /// No visualization.
		relation,           /// Relation between trajectories calculated by `relation_def`.
		dbg_seg_t,          /// Segment-local curve parameter.
		dbg_velocity,       /// Length of the trajectory's spatial derivative w.r.t. time.
		dbg_index_xyz,      /// Spatial grid index.
		dbg_index_t,        /// Temporal grid index.
		dbg_signature,      /// Index hash signature.
		dbg_bucket_load,    /// Hash bucket load factor.
		dbg_local_interval, /// Curve parameter relative to local trajectory interval.
		dbg_num_cells,      /// Number of cells within the query volume.
		dbg_num_intervals,  /// Number of trajectory intervals within queried cells.
	};

	/// Describes for which pairs of trajectories the relation is evaluated.
	enum class Direction : uint32_t { // uint8_t causes problems with CGV GUI.
		ref_to_all, /// Only evaluate the relation for the reference trajectory.
		all_to_ref, /// Evaluate the relation with the reference trajectory.
		all_to_all, /// Evaluate the relation for every pair of trajectories.
	};

	/// Determines how the relation shader checks whether an AABB intersects the query volume.
	/// More precise tests are more expensive to perform, but may allow more grid cells to be
	/// skipped entirely. The tradeoff depends on many factors, such as the dataset, the grid,
	/// the relation, and the cutoff angle. No check should ever produce a false negative.
	enum class QueryIntersectionTest : uint32_t {
		none, // Iterate over the entire AABB. Only for validation.
		sphere, // Account only for radius, not angle. Only for validation.
		fast,
		exact, // Can take noticeably longer to compile.
	};

	struct {
		std::string name {};

		/// GLSL code defining the relation to evaluate (if `data_var == DataVar::relation`).
		std::string definition {};
	} relation;

	using ColorTransform = cgv::media::ContinuousMappingTransform;
	struct {
		using Scale = cgv::media::continuous_color_scale;
		/// Instance of the color scale used to visualize relations. Configured according to the
		/// parameters in this struct.
		std::shared_ptr<Scale> scale = std::make_shared<Scale>();
		cgv::media::continuous_color_scheme scheme {};

		/// Precalculated lookup for the configured color scale. Generated from `instance`.
		cgv::render::texture texture {
			"uint8[R,G,B]",
			cgv::render::TF_LINEAR, cgv::render::TF_LINEAR,
			cgv::render::TW_CLAMP_TO_EDGE, cgv::render::TW_CLAMP_TO_EDGE,
		};

		/// Index of the base color map in the color_map_manager. The base map is modified according
		/// to the other parameters.
		PseudoEnum<uint32_t> base {};
		/// Fixed color marking special trajectory parts.
		cgv::rgb highlight {1, 0, 1};
		/// Fixed color of trajectories for which no relation value is calculated.
		cgv::rgb background {1.0f/3};

		/// Relation values mapped onto the endpoints of the color scale.
		cgv::vec2 domain {0, 1};
		/// Percentage of the domain at which the scale diverges, if it does.
		float midpoint {50};

		/// Parameter of the exponential transform.
		float exponent {2};
		/// Parameter of the logarithmic transform.
		float log_base {10};
		ColorTransform transform {ColorTransform::Linear};

		bool diverging {false};
	} color_scale;

	struct {
		float space {}; // Query radius in 3D Euclidean space.
		float pre {}; // Query radius into the past.
		float post {}; // Query radius into the future.
	}
	/// For each trajectory point, calculate its relation to samples of other trjectories within the
	/// given distance.
	radius;
	/// Exponent of the cosine term optionally applied to the relation. Larger values reduce the
	/// influence of samples further away from the surface normal on the relation value at any given
	/// point.
	float cos_exp {5};
	/// If the direction from a fragment's surface point to a sample point on another trajectory
	/// deviates from the surface normal by more than this angle, the sample does not contribute to
	/// the relation. In degrees.
	float cutoff_angle {90};
	/// Samples for which the cosine term is no larger than this value may be ignored. Range [0, 1].
	/// Not set directly, but calculated from `cos_exp` and `cutoff_angle`. Stored as a member
	/// variable so it can be shown in a GUI view.
	float cos_cutoff {};
	/// The value to visualize.
	DataVar data_var {};
	/// Determines between which trajectories the relation is evaluated.
	Direction direction {Direction::all_to_all};
	QueryIntersectionTest query_isect_test {QueryIntersectionTest::fast};
	/// ID of the "reference trajectory" whose meaning depends on `direction`.
	uint32_t reference_trajectory {0};
	/// Trajectory evaluations per unit of time to calculate relation.
	float sample_rate {1};
	/// Determines whether the relation is averaged (true) or accumulated (false) over time.
	/// The exact meaning, howver, depends on the relation.
	bool normalize {true};

	[[nodiscard]] relation_vis();

	/// Generate GUI elements to control member variables.
	void build_gui (
		cgv::gui::provider&,
		uint32_t num_trajectories,
		cgv::vec4 data_extent,
		std::vector<std::string> const& color_maps
	);

	using UpdateFlags = uint32_t;
	/// Bitflags indicating which parts of the application need to be refreshed after a member
	/// variable was changed.
	struct UpdateFlag {enum : UpdateFlags {
		gui         = 1 << 0,
		shader_opts = 1 << 1,
	};};
	/// Must be called when a member variable has been changed. The return value indicates whether
	/// the GUI needs to be updated.
	auto on_set (
		void* member,
		cgv::render::context&,
		cgv::gui::provider&,
		color_map_manager const&
	) -> UpdateFlags;

	/// Regenerate the color scale object and texture according to the selected parameters.
	void update_color_scale (cgv::render::context& ctx, color_map_manager const& colors);

	/// Choose evaluation settings appropriate for a dataset of the given size.
	void set_to_default (cgv::vec4 extent);

	/// Statically configure shaders through text substitution.
	void set_shader_opts (cgv::render::shader_compile_options&) const;
	/// Dynamically configure shaders through uniforms.
	void set_uniforms (cgv::render::context&, cgv::render::shader_program&) const;

	/// Label for the value being visualized.
	auto data_var_name () const -> std::string_view;
};

// Reflect enum members so changes can be detected in `on_set`.
auto get_reflection_traits (enum relation_vis::DataVar const&)
	-> cgv::reflect::enum_reflection_traits<enum relation_vis::DataVar>;

/// Return the unqualified identifier for the given enum value.
[[nodiscard]] constexpr auto enum_id (enum relation_vis::Direction dir) noexcept
	-> std::string_view
{
	using std::operator""sv;
	return std::array{"ref_to_all"sv, "all_to_ref"sv, "all_to_all"sv}[static_cast<size_t>(dir)];
}
