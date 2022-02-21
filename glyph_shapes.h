#pragma once

#include <cgv/render/render_types.h>

enum GlyphType {
	GT_UNDEFINED = -1,
	GT_COLOR,
	GT_CIRCLE,
	GT_RECTANGLE,
	GT_WEDGE,
	GT_ARC_FLAT,
	GT_ARC_ROUNDED,
	GT_TRIANGLE,
	GT_DROP,
	GT_SIGN_BLOB,
	GT_STAR,
	GT_VIOLIN,
};

enum GlyphAttributeType {
	GAT_UNIT = 0, // value in [0,1] determining a generic glyph attribute
	GAT_SIGNED_UNIT = 1, // value in [-1,1] determining a generic glyph attribute
	GAT_SIZE = 2, // value in [0,inf) determining radius, length or scale in general
	GAT_ANGLE = 3, // value in [0,360] giving angle in degree
	GAT_DOUBLE_ANGLE = 4, // value in [0,360] giving angle in degree (divided by 2 for the actual mapping)
	GAT_ORIENTATION = 5, // value in [0,360] giving angle in degree used specifically to orient the glyph
	GAT_COLOR = 6, // rgb color
};

enum GlyphAttributeModifier {
	GAM_NONE = 0,
	GAM_GLOBAL = 1, // global attributes are always constant (overrides non-const)
	GAM_NON_CONST = 2, // cannot be set to constant value
};

struct glyph_attribute {
	std::string name;
	GlyphAttributeType type;
	GlyphAttributeModifier modifiers = GAM_NONE;

	glyph_attribute(std::string name, GlyphAttributeType type) : name(name), type(type) {}
	glyph_attribute(std::string name, GlyphAttributeType type, GlyphAttributeModifier modifiers) : name(name), type(type), modifiers(modifiers) {}
};

class glyph_shape : public cgv::render::render_types {
public:
	typedef std::vector<glyph_attribute> attribute_list;

	virtual glyph_shape* clone() const = 0;

	virtual GlyphType type() const = 0;
	virtual std::string name() const = 0;
	virtual const attribute_list& supported_attributes() const = 0;

	virtual size_t num_size_attribs() const {
		size_t n = 0;
		for(size_t i = 0; i < supported_attributes().size(); ++i)
			if(supported_attributes()[i].type != GAT_COLOR) ++n;
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
	virtual color_glyph* clone() const {
		return new color_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_COLOR;
	}

	virtual std::string name() const {
		return "color";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "interpolate", GAT_UNIT, GAM_GLOBAL },
			{ "classification", GAT_UNIT, GAM_GLOBAL },
			{ "color", GAT_COLOR }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// a negative size tells the glyph layout algorithm to never skip these glyphs
		// and that they are potentially infinite in size (the glyph will stretch as long
		// as a next one is placed)
		return -1.0f;
	}

	virtual std::string splat_func() const {
		return "splat_color";
	}
};


class circle_glyph : public glyph_shape {
public:
	virtual circle_glyph* clone() const {
		return new circle_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_CIRCLE;
	}

	virtual std::string name() const {
		return "circle";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "radius", GAT_SIZE },
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// size is two times the radius, a.k.a. the diameter of the circle
		return 2.0f * param_values[0];
	}
};

class rectangle_glyph : public glyph_shape {
public:
	virtual rectangle_glyph* clone() const {
		return new rectangle_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_RECTANGLE;
	}

	virtual std::string name() const {
		return "rectangle";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "length", GAT_SIZE },
			{ "height", GAT_SIZE }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// size is just the length/width
		return param_values[0];
	}
};

class wedge_glyph : public glyph_shape {
public:
	virtual wedge_glyph* clone() const {
		return new wedge_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_WEDGE;
	}

	virtual std::string name() const {
		return "wedge";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "radius", GAT_SIZE },
			{ "aperture", GAT_DOUBLE_ANGLE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// use just the radius as it gives a more uniform (or visually pleasing) spacing
		// for complete correctness, aperture and orientation would need to be considered as well
		return 2.0f * param_values[0];
	}
};

class flat_arc_glyph : public glyph_shape {
public:
	virtual flat_arc_glyph* clone() const {
		return new flat_arc_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_ARC_FLAT;
	}

	virtual std::string name() const {
		return "arc_flat";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "radius", GAT_SIZE },
			{ "thickness", GAT_SIZE },
			{ "aperture", GAT_DOUBLE_ANGLE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// use just the radius and thickness as it gives a more uniform (or visually pleasing) spacing
		// for complete correctness, aperture and orientation would need to be considered as well
		return 2.0f * (param_values[0] + param_values[1]);
	}
};

class rounded_arc_glyph : public flat_arc_glyph {
public:
	virtual rounded_arc_glyph* clone() const {
		return new rounded_arc_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_ARC_ROUNDED;
	}

	virtual std::string name() const {
		return "arc_rounded";
	}
};

class isoceles_triangle_glyph : public glyph_shape {
public:
	virtual isoceles_triangle_glyph* clone() const {
		return new isoceles_triangle_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_TRIANGLE;
	}

	virtual std::string name() const {
		return "triangle_isosceles";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "base_width", GAT_SIZE },
			{ "height", GAT_SIZE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// TODO: bounding box does not work right now

		// build rotation matrix like in the shader
		const float quarter_turn = 1.57079632679; // additional rotation of 90 degrees, so an angle of 0 degrees points the glyph into the direction of the segment
		float angle = cgv::math::deg2rad(param_values[2]) + quarter_turn;
		
		float as = sin(angle);
		float ac = cos(angle);
		mat2 R = (ac, as, -as, ac);
		
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
		return param_values[1];// 2.0f * std::max(el, er);
	}
};

class drop_glyph : public glyph_shape {
public:
	virtual drop_glyph* clone() const {
		return new drop_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_DROP;
	}

	virtual std::string name() const {
		return "drop";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "base_radius", GAT_SIZE },
			{ "tip_radius", GAT_SIZE },
			{ "height", GAT_SIZE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}
};

class sign_blob_glyph : public glyph_shape {
public:
	virtual sign_blob_glyph* clone() const {
		return new sign_blob_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_SIGN_BLOB;
	}

	virtual std::string name() const {
		return "sign_blob";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR },
			{ "radius", GAT_SIZE, GAM_GLOBAL },
			{ "value", GAT_SIGNED_UNIT },
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// size is two times the radius, a.k.a. the diameter
		return 2.0f * param_values[0];
	}
};

class star_glyph : public glyph_shape {
public:
	virtual star_glyph* clone() const {
		return new star_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_STAR;
	}

	virtual std::string name() const {
		return "star";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "radius", GAT_SIZE, GAM_GLOBAL },
			{ "secondary_color", GAT_COLOR, GAM_GLOBAL },
			{ "color_setting", GAT_UNIT, GAM_GLOBAL },
			{ "blend_factor", GAT_UNIT, GAM_GLOBAL },
			{ "inner_transparency", GAT_UNIT, GAM_GLOBAL },
			{ "axis_0", GAT_SIZE, GAM_NON_CONST },
			{ "color_0", GAT_COLOR, GAM_GLOBAL },
			{ "axis_1", GAT_SIZE, GAM_NON_CONST },
			{ "color_1", GAT_COLOR, GAM_GLOBAL },
			{ "axis_2", GAT_SIZE, GAM_NON_CONST },
			{ "color_2", GAT_COLOR, GAM_GLOBAL },
			{ "axis_3", GAT_SIZE, GAM_NON_CONST },
			{ "color_3", GAT_COLOR, GAM_GLOBAL },
			{ "axis_4", GAT_SIZE, GAM_NON_CONST },
			{ "color_4", GAT_COLOR, GAM_GLOBAL }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// size is two times the radius, a.k.a. the diameter
		return 2.0f * param_values[0];
	}

	virtual std::string splat_func() const {
		return "splat_star";
	}
};

class violin_glyph : public glyph_shape {
public:
	virtual violin_glyph* clone() const {
		return new violin_glyph(*this);
	}

	virtual GlyphType type() const {
		return GT_VIOLIN;
	}

	virtual std::string name() const {
		return "violin";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "value_0", GAT_SIZE, GAM_NON_CONST },
			{ "color_0", GAT_COLOR, GAM_GLOBAL },
			{ "value_1", GAT_SIZE, GAM_NON_CONST },
			{ "color_1", GAT_COLOR, GAM_GLOBAL },
			{ "value_2", GAT_SIZE, GAM_NON_CONST },
			{ "color_2", GAT_COLOR, GAM_GLOBAL },
			{ "value_3", GAT_SIZE, GAM_NON_CONST },
			{ "color_3", GAT_COLOR, GAM_GLOBAL }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
		// a negative size tells the glyph layout algorithm to never skip these glyphs
		// and that they are potentially infinite in size (the glyph will stretch as long
		// as a next one is placed)
		return -1.0f;
	}

	virtual std::string splat_func() const {
		return "splat_violin";
	}
};

struct glyph_type_registry {
	static GlyphType type(const std::string& name) {
		const auto& n = names();
		static const std::map<std::string, GlyphType> mapping = {
			{ n[0], GT_COLOR },
			{ n[1], GT_CIRCLE },
			{ n[2], GT_RECTANGLE },
			{ n[3], GT_WEDGE },
			{ n[4], GT_ARC_FLAT },
			{ n[5], GT_ARC_ROUNDED },
			{ n[6], GT_TRIANGLE },
			{ n[7], GT_DROP },
			{ n[8], GT_SIGN_BLOB },
			{ n[9], GT_STAR },
			{ n[10], GT_VIOLIN }
		};

		auto it = mapping.find(name);
		if(it != mapping.end())
			return (*it).second;
		return GT_UNDEFINED;
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
			"violin"
		};

		return n;
	}

	static std::vector<std::string> display_names() {
		static const std::vector<std::string> names = {
			"Color",
			"Circle",
			"Rectangle",
			"Wedge",
			"Flat Arc",
			"Rounded Arc",
			"Isosceles Triangle",
			"Drop",
			"Sign Blob",
			"Star",
			"Violin"
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
	static glyph_shape* create(const GlyphType type) {
		glyph_shape* shape_ptr = nullptr;

		switch(type) {
		case GT_COLOR: shape_ptr = new color_glyph(); break;
		case GT_CIRCLE: shape_ptr = new circle_glyph(); break;
		case GT_RECTANGLE: shape_ptr = new rectangle_glyph(); break;
		case GT_WEDGE: shape_ptr = new wedge_glyph(); break;
		case GT_ARC_FLAT: shape_ptr = new flat_arc_glyph(); break;
		case GT_ARC_ROUNDED: shape_ptr = new rounded_arc_glyph(); break;
		case GT_TRIANGLE: shape_ptr = new isoceles_triangle_glyph(); break;
		case GT_DROP: shape_ptr = new drop_glyph(); break;
		case GT_SIGN_BLOB: shape_ptr = new sign_blob_glyph(); break;
		case GT_STAR: shape_ptr = new star_glyph(); break;
		case GT_VIOLIN: shape_ptr = new violin_glyph(); break;
		default: shape_ptr = new circle_glyph(); break;
		}

		return shape_ptr;
	}
};
