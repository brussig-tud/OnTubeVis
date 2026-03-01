#pragma once

#include <map>
#include <memory>

#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/math/functions.h>


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

	virtual GlyphType type() const = 0;
	virtual std::string name() const = 0;
	virtual const attribute_list& supported_attributes() const = 0;

	virtual size_t num_size_attribs() const {
		size_t n = 0;
		for(size_t i = 0; i < supported_attributes().size(); ++i)
			if(supported_attributes()[i].type != GlyphAttributeType::kColor) ++n;
		return n;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		return 0.0f;
	};

	virtual std::string splat_func() const {
		return "";
	}
};

class color_glyph : public glyph_shape {
public:
	std::unique_ptr<glyph_shape> clone() const override {
		return std::unique_ptr<glyph_shape>(new color_glyph(*this));
		//return std::make_unique<glyph_shape>(*this);
	}

	GlyphType type() const override {
		return GlyphType::kColor;
	}

	std::string name() const override {
		return "color";
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "interpolate", GlyphAttributeType::kUnit, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart }
		};
		return attributes;
	}

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "circle";
	}

	const attribute_list& supported_attributes() const override {
		static const attribute_list attributes = {
			{ "outline", GlyphAttributeType::kOutline, GlyphAttributeModifier::kGlobal, GuiLayoutHint::kGlobalBlockStart },
			{ "color", GlyphAttributeType::kColor, GuiLayoutHint::kBlockStart },
			{ "radius", GlyphAttributeType::kSize },
		};
		return attributes;
	}

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "rectangle";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "wedge";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "arc_flat";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "arc_rounded";
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

	std::string name() const override {
		return "triangle_isosceles";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "drop";
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

	std::string name() const override {
		return "sign_blob";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "star";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "line_plot";
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

	float get_size(const std::vector<float>& param_values) const override {
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

	std::string name() const override {
		return "temporal_heat_map";
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

	float get_size(const std::vector<float>& param_values) const override {
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
	static GlyphType type(const std::string& name) {
		const auto& n = names();
		static const std::map<std::string, GlyphType> mapping = {
			{ n[0], GlyphType::kColor },
			{ n[1], GlyphType::kCircle },
			{ n[2], GlyphType::kRectangle },
			{ n[3], GlyphType::kWedge },
			{ n[4], GlyphType::kArcFlat },
			{ n[5], GlyphType::kArcRounded },
			{ n[6], GlyphType::kTriangle },
			{ n[7], GlyphType::kDrop },
			{ n[8], GlyphType::kSignBlob },
			{ n[9], GlyphType::kStar },
			{ n[10], GlyphType::kLinePlot },
			{ n[11], GlyphType::kTemporalHeatMap }
		};

		auto it = mapping.find(name);
		if(it != mapping.end())
			return (*it).second;
		return GlyphType::kUndefined;
	}

	static std::vector<std::string> names() {
		static const std::vector<std::string> n = {
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

		return n;
	}

	static std::vector<std::string> display_names() {
		static const std::vector<std::string> names = {
			"Surface Color",
			"Circle",
			"Rectangle",
			"Wedge",
			"Flat Arc",
			"Rounded Arc",
			"Isosceles Triangle",
			"Drop",
			"Sign Blob",
			"Star",
			"Line Plot",
			"Temporal Heat Map"
		};

		return names;
	}

	static std::string display_name_enums() {
		const auto& names = display_names();
		std::string enums = "";

		for(size_t i = 0; i < names.size(); ++i) {
			enums += names[i];
			if(i < names.size() - 1)
				enums += ",";
		}

		return enums;
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
