#pragma once

// CGV framework
#include <cgv/math/fvec.h>

// Local includes
#include <OnTubeVis/OnTubeVis.h>
#include "../ambient_occlusion_style.h"
#include "../textured_spline_tube_renderer.h"
#include "../glyph_layer_manager.h"
#include "util.h"



/// data layout for per-node attributes within the attribute render SSBO
struct node_attribs {
	cgv::vec4 pos_rad;
	cgv::vec4 color;
	cgv::vec4 tangent;
	cgv::vec4 t; // only uses .x component to store t, yzw are reserved for future use

	inline static node_attribs from_api_node (
		const OTV_HermiteNode &node, const float radius, const cgv::vec4 &color
	){
		return {
			cgv::vec4(node.position.x, node.position.y, node.position.z, radius),
			color, cgv::vec4(node.tangent.x, node.tangent.y, node.tangent.z, 0),
			cgv::vec4(node.time, 0, 0, 0)
		};
	}

	inline OTV_HermiteNode into_api_node (void) const {
		return {
			t.x(), otv__Vec3(pos_rad.x(), pos_rad.y(), pos_rad.z()),
			otv__Vec3(tangent.x(), tangent.y(), tangent.z())
		};
	}
};

/// data layout for per-segment topology info within the corresponding SSBO
struct topology_info {
	unsigned traj; // id of the trajectory the segment belongs to
	unsigned next; // -1 means no next segment
};

// define GridMode outside of main on_tube_vis class to be able to use it with type reflection
enum GridMode {
	GM_NONE = 0,
	GM_COLOR = 1,
	GM_NORMAL = 2,
	GM_COLOR_AND_NORMAL = 3
};

/// tube shading settings
struct tube_shading_settings
{
	// grid-related
	struct grid_parameters {
		cgv::vec2 scaling;
		float thickness;
		float blend_factor;
	};
	cgv::rgba grid_color;
	GridMode grid_mode = GM_NONE;
	cgv::type::DummyEnum grid_normal_settings;
	bool grid_normal_inwards;
	bool grid_normal_variant;
	bool enable_fuzzy_grid;
	float normal_mapping_scale;
	std::vector<grid_parameters> grids;

	// ambient occlusion
	ambient_occlusion_style ao_style;

	// streaming-related
	bool alternative_ring_buffer = false;
	float playback_t = std::numeric_limits<float>::infinity();

	[[nodiscard]] cgv::render::shader_define_map build_tube_shading_defines (
		const glyph_layer_manager::configuration &glyph_layers_config, bool debug_highlight_segments=false
	) const;

	void set_uniforms (
		cgv::render::context &ctx, cgv::render::shader_program &prog,
		const cgv::render::textured_spline_tube_render_style &render_style,
		const glyph_layer_manager::configuration &glyph_layers_config
	#if RTX_SUPPORT
		, bool optix_enabled
	#endif
	) const;
};


/// An integer range consisting of a start value and a length.
template <class Index>
struct index_range {
	using index_type = Index;

	index_type i0 {}, n {};

	/// Calculate the index one past the last element in the range.
	[[nodiscard]] constexpr index_type end () const noexcept
	{
		return i0 + n;
	}
};

// helper struct for range entries with start index i0 and count n
// Moved from `glyph_compiler`.
using irange = index_range<int>;


/// Newtype wrapper storing a number of glyphs.
/// Used to avoid confusion between number of glyphs and number of glyph attributes.
struct glyph_count_type {
	using base_type = int;

	base_type value {0};

	[[nodiscard]] constexpr glyph_count_type() noexcept = default;
	[[nodiscard]] constexpr explicit glyph_count_type(base_type value) noexcept
		: value {value}
	{}

	[[nodiscard]] constexpr explicit operator bool () const noexcept {
		return value;
	}

// Implement operators by applying them to the raw value.
#define OTV_GLYPH_COUNT_BINARY_OP(op, result_type) \
	[[nodiscard]] friend constexpr result_type operator op ( \
		glyph_count_type lhs, \
		glyph_count_type rhs \
	) noexcept { \
		return result_type{lhs.value op rhs.value}; \
	}

OTV_GLYPH_COUNT_BINARY_OP(+, glyph_count_type);
OTV_GLYPH_COUNT_BINARY_OP(-, glyph_count_type);
OTV_GLYPH_COUNT_BINARY_OP(==, bool);
OTV_GLYPH_COUNT_BINARY_OP(!=, bool);
OTV_GLYPH_COUNT_BINARY_OP(<, bool);
OTV_GLYPH_COUNT_BINARY_OP(<=, bool);
OTV_GLYPH_COUNT_BINARY_OP(>, bool);
OTV_GLYPH_COUNT_BINARY_OP(>=, bool);

#undef OTV_GLYPH_COUNT_BIN_OP

#define OTV_GLYPH_COUNT_ASSIGN_OP(op) \
	glyph_count_type &operator op (glyph_count_type rhs) noexcept { \
		value op rhs.value; \
		return *this; \
	}

OTV_GLYPH_COUNT_ASSIGN_OP(+=);
OTV_GLYPH_COUNT_ASSIGN_OP(-=);

#undef OTV_GLYPH_COUNT_ASSIGN_OP

};


/// Type used to identify glyph layers.
using layer_index_type = uint8_t;

/// The maximum number of concurrent glyph layers supported by the implementation.
/// NOTE: This value is still hard-coded in several places, particulary in shaders.
static constexpr layer_index_type max_glyph_layers {4};

/// Stores generic data specific to each glyph layer.
template <class Elem>
using per_layer = std::array<Elem, max_glyph_layers>;


/// Type holding the number of 32-bit float attributes used to represent one glyph.
using glyph_size_type = uint8_t;

// helper struct for glyph attributes
// Moved from `glyph_compiler`.
struct glyph_attributes {
	glyph_size_type count = 0;
	std::vector<float> data;

	bool empty() const { return size() == 0; }

	size_t size() const { return data.size(); }

	glyph_count_type glyph_count() const {
		return static_cast<glyph_count_type>(size() / (2 + count));
	}

	void add(const float val) {
		data.emplace_back(val);
	}

	float& operator [](int idx) {
		return data[idx];
	}

	float operator [](int idx) const {
		return data[idx];
	}

	float last_glyph_s() const {
		if(size() > 0)
			return data[size() - 1 - 1 - count];
		else
			return 0.0f;
	}
};

namespace otv {

/// Extract the arclength position of the first glyph pointed to by the given @c ro_range @a glyph_data.
template <class Iter>
[[nodiscard]] inline float get_glyph_pos (const ro_range<Iter> &glyph_data) {
	return static_cast<float>(*glyph_data.begin);
}

}
