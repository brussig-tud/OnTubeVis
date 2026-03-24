#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/math/functions.h>
#include <cgv/utils/scan.h>


enum class GlyphType {
	kUndefined = -1,
	kColor,
	kCircle,
	kRectangle,
	kWedge,
	kArcFlat,
	kArcRounded,
	kTriangle,
	kDrop,
	kSignBlob,
	kStar,
	kLinePlot,
	kTemporalHeatMap,
};

static std::string to_string(GlyphType glyph_type) {
	static const std::array<std::string, 12> names = {
		"color",
		"circle",
		"rectangle",
		"wedge",
		"arc_flat",
		"arc_rounded",
		"triangle_isosceles",
		"drop",
		"sign_blob",
		"star",
		"line_plot",
		"temporal_heat_map"
	};
	return names[static_cast<int>(glyph_type)];
}

enum class GlyphAttributeType {
	kUnit = 0, // value in [0,1] determining a generic glyph attribute
	kSignedUnit = 1, // value in [-1,1] determining a generic glyph attribute
	kSize = 2, // value in [0,inf) determining radius, length or scale in general
	kAngle = 3, // value in [0,360] giving angle in degree
	kDoubleAngle = 4, // value in [0,360] giving angle in degree (divided by 2 for the actual mapping)
	kOrientation = 5, // value in [0,360] giving angle in degree used specifically to orient the glyph
	kColor = 6, // rgb color
	kOutline = 7,
};

enum class GlyphAttributeModifier {
	kNone = 0,
	kGlobal = 1, // global attributes are always constant (overrides non-const)
	kNonConst = 2, // cannot be set to constant value
	kForceMappable = 4,
};

static GlyphAttributeModifier operator&(GlyphAttributeModifier lhs, GlyphAttributeModifier rhs) {
	using T = std::underlying_type_t<GlyphAttributeModifier>;
	return static_cast<GlyphAttributeModifier>(static_cast<T>(lhs) & static_cast<T>(rhs));
}

static GlyphAttributeModifier operator|(GlyphAttributeModifier lhs, GlyphAttributeModifier rhs) {
	using T = std::underlying_type_t<GlyphAttributeModifier>;
	return static_cast<GlyphAttributeModifier>(static_cast<T>(lhs) | static_cast<T>(rhs));
}

enum class GuiLayoutHint {
	kNone = 0,
	kGlobalBlockStart = 1,
	kBlockStart = 2,
};

struct glyph_attribute {
	std::string name;
	GlyphAttributeType type = GlyphAttributeType::kUnit;
	GlyphAttributeModifier modifiers = GlyphAttributeModifier::kNone;
	GuiLayoutHint gui_hint = GuiLayoutHint::kNone;

	glyph_attribute(std::string name, GlyphAttributeType type) : name(name), type(type) {}
	glyph_attribute(std::string name, GlyphAttributeType type, GlyphAttributeModifier modifiers) : name(name), type(type), modifiers(modifiers) {}
	glyph_attribute(std::string name, GlyphAttributeType type, GuiLayoutHint gui_hint) : name(name), type(type), gui_hint(gui_hint) {}
	glyph_attribute(std::string name, GlyphAttributeType type, GlyphAttributeModifier modifiers, GuiLayoutHint gui_hint) : name(name), type(type), modifiers(modifiers), gui_hint(gui_hint) {}
};

class glyph_shape {
public:
	using mat2 = cgv::mat2;
	using vec2 = cgv::vec2;

	typedef std::vector<glyph_attribute> attribute_list;

	virtual ~glyph_shape() {}

	virtual std::unique_ptr<glyph_shape> clone() const = 0;

	static std::string display_name(GlyphType type) {
		return cgv::utils::snake_case_to_capitalized_case(to_string(type));
	}

	virtual GlyphType type() const = 0;

	std::string name() const {
		return to_string(type());
	}

	virtual const attribute_list& supported_attributes() const = 0;

	virtual size_t num_size_attribs() const {
		size_t n = 0;
		for(const glyph_attribute& attribute : supported_attributes())
			if(attribute.type != GlyphAttributeType::kColor) ++n;
		return n;
	}

	/// Calculate the extent of a glyph along the trajectory it is on.
	/// `param_values` must point to an array containing values for all visual variables that impact
	/// the glyph's size, but no others (i.e. color values).
	virtual float get_size(const float* param_values) const {
		return 0.0f;
	};

	virtual std::string splat_func() const {
		return "";
	}
};

class color_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<color_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kColor;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "interpolate", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// a negative size tells the glyph layout algorithm to never skip these glyphs
		// and that they are potentially infinite in size (the glyph will stretch as long
		// as a next one is placed)
		return -1.0f;
	}

	std::string splat_func() const override {
		return "splat_color";
	}
};


class circle_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<circle_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kCircle;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "radius", GlyphAttributeType::kSize },
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// size is two times the radius, a.k.a. the diameter of the circle
		return 2.0f * param_values[1];
	}
};

class rectangle_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<rectangle_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kRectangle;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "length", GlyphAttributeType::kSize },
			{ "height", GlyphAttributeType::kSize }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// size is just the length/width, height is irrelevant
		return param_values[1];
	}
};

class wedge_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<wedge_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kWedge;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "radius", GlyphAttributeType::kSize },
			{ "aperture", GlyphAttributeType::kDoubleAngle },
			{ "orientation", GlyphAttributeType::kOrientation }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// use just the radius as it gives a more uniform (or visually pleasing) spacing
		// for complete correctness, aperture and orientation would need to be considered as well
		return 2.0f * param_values[1];
	}
};

class flat_arc_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<flat_arc_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kArcFlat;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "radius", GlyphAttributeType::kSize },
			{ "thickness", GlyphAttributeType::kSize },
			{ "aperture", GlyphAttributeType::kDoubleAngle },
			{ "orientation", GlyphAttributeType::kOrientation }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// use just the radius and thickness as it gives a more uniform (or visually pleasing) spacing
		// for complete correctness, aperture and orientation would need to be considered as well
		return 2.0f * (param_values[1] + param_values[2]);
	}
};

class rounded_arc_glyph : public flat_arc_glyph {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<rounded_arc_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kArcRounded;
	}
};

class isoceles_triangle_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<isoceles_triangle_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kTriangle;
	}

	const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "base_width", GlyphAttributeType::kSize },
			{ "height", GlyphAttributeType::kSize },
			{ "orientation", GlyphAttributeType::kOrientation }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// TODO: bounding box does not work right now

		// build rotation matrix like in the shader
		const float quarter_turn = 1.57079632679f; // additional rotation of 90 degrees, so an angle of 0 degrees points the glyph into the direction of the segment
		float angle = cgv::math::deg2rad(param_values[2]) + quarter_turn;
		
		float as = sin(angle);
		float ac = cos(angle);
		mat2 R = {ac, as, -as, ac};
		
		float half_base_width = 0.5f * param_values[0];
		float half_height = 0.5f * param_values[1];

		vec2 box_min;
		vec2 box_max;

		vec2 p = vec2(-half_base_width, half_height);
		box_min = p;
		box_max = p;

		p = vec2(half_base_width, half_height);
		box_min = min(box_min, p);
		box_max = max(box_max, p);

		p = vec2(0.0f, -half_height);
		box_min = min(box_min, p);
		box_max = max(box_max, p);

		// the box can be off-center
		// first get the center and then the extent to each side
		float c = 0.5f * (box_min.x() + box_max.x());
		float el = c - box_min.x();
		float er = box_max.x() - c;
		// the largest of these extents defines the total width
		// TODO: use two sizes for gyphs?

		// only use height for now
		return param_values[2];// 2.0f * std::max(el, er);
	}
};

class drop_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<drop_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kDrop;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "base_radius", GlyphAttributeType::kSize },
			{ "tip_radius", GlyphAttributeType::kSize },
			{ "height", GlyphAttributeType::kSize },
			{ "orientation", GlyphAttributeType::kOrientation }
		};
		return attributes;
	}
};

class sign_blob_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<sign_blob_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kSignBlob;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "size", GlyphAttributeType::kSize, GlyphAttributeModifier::kGlobal },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "value", GlyphAttributeType::kSignedUnit },
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// size is the total width of the glyph but depends on the current shape, which depends on the mapped value
		float s = param_values[1];
		float v = param_values[2];
		float d_circle = 2.0f * 0.25f * s; // circle diameter
		// s directly corresponds to the width of the plus or minus shape
		if(v != 0.0f)
			int iii = 0;
		return v < 0.0 ?
			cgv::math::lerp(s, d_circle, v + 1.0f) :
			cgv::math::lerp(d_circle, s, v);
	}
};

class star_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<star_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kStar;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "radius", GlyphAttributeType::kSize, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "secondary_color", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal },
			{ "color_setting", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "blend_factor", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "inner_transparency", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "axes_setting", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "orientation", GlyphAttributeType::kOrientation, GlyphAttributeModifier::kGlobal },
			{ "color_0", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "axis_0", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_1", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "axis_1", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_2", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "axis_2", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_3", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "axis_3", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_4", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "axis_4", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// size is two times the radius, a.k.a. the diameter
		return 2.0f * param_values[0];
	}

	std::string splat_func() const override {
		return "splat_star";
	}
};

class line_plot_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<line_plot_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kLinePlot;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "interpolate", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "color_0", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "value_0", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_1", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "value_1", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_2", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "value_2", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "color_3", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kBlockStart },
			{ "value_3", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// a negative size tells the glyph layout algorithm to never skip these glyphs
		// and that they are potentially infinite in size (the glyph will stretch as long
		// as a next one is placed)
		return -1.0f;
	}

	std::string splat_func() const override {
		return "splat_line_plot";
	}
};

class temporal_heat_map_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::make_unique<temporal_heat_map_glyph>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kTemporalHeatMap;
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "interpolate", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal },
			{ "color", GlyphAttributeType::kColor, GlyphAttributeModifier::kGlobal | GlyphAttributeModifier::kForceMappable, GuiLayoutHint::kBlockStart },
			{ "value_0", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "value_1", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "value_2", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst },
			{ "value_3", GlyphAttributeType::kSize, GlyphAttributeModifier::kNonConst }
		};
		return attributes;
	}

	float get_size(const float* param_values) const override {
		// a negative size tells the glyph layout algorithm to never skip these glyphs
		// and that they are potentially infinite in size (the glyph will stretch as long
		// as a next one is placed)
		return -1.0f;
	}

	std::string splat_func() const override {
		return "splat_temporal_heat_map";
	}
};

struct glyph_type_registry {
	static std::vector<GlyphType> list_glyph_types() {
		static const std::vector<GlyphType> types = {
			GlyphType::kColor,
			GlyphType::kCircle,
			GlyphType::kRectangle,
			GlyphType::kWedge,
			GlyphType::kArcFlat,
			GlyphType::kArcRounded,
			GlyphType::kTriangle,
			GlyphType::kDrop,
			GlyphType::kSignBlob,
			GlyphType::kStar,
			GlyphType::kLinePlot,
			GlyphType::kTemporalHeatMap
		};
		return types;
	}

	static std::vector<std::string> list_names() {
		auto types = list_glyph_types();
		std::vector<std::string> names;
		std::transform(types.begin(), types.end(), std::back_inserter(names), [](GlyphType type) { return to_string(type); });
		return names;
	}

	static GlyphType get_type_by_name(const std::string& name) {
		auto types = list_glyph_types();
		auto it = std::find_if(types.begin(), types.end(), [&name](GlyphType type) { return to_string(type) == name; });
		if(it != types.end())
			return *it;
		return GlyphType::kUndefined;
	}
};

struct glyph_shape_factory {
	static std::unique_ptr<glyph_shape> create(const GlyphType type) {
		switch(type) {
		case GlyphType::kColor: return std::make_unique<color_glyph>();
		case GlyphType::kCircle: return std::make_unique<circle_glyph>();
		case GlyphType::kRectangle: return std::make_unique<rectangle_glyph>();
		case GlyphType::kWedge: return std::make_unique<wedge_glyph>();
		case GlyphType::kArcFlat: return std::make_unique<flat_arc_glyph>();
		case GlyphType::kArcRounded: return std::make_unique<rounded_arc_glyph>();
		case GlyphType::kTriangle: return std::make_unique<isoceles_triangle_glyph>();
		case GlyphType::kDrop: return std::make_unique<drop_glyph>();
		case GlyphType::kSignBlob: return std::make_unique<sign_blob_glyph>();
		case GlyphType::kStar: return std::make_unique<star_glyph>();
		case GlyphType::kLinePlot: return std::make_unique<line_plot_glyph>();
		case GlyphType::kTemporalHeatMap: return std::make_unique<temporal_heat_map_glyph>();
		default: return std::make_unique<color_glyph>();
		}
	}
};
