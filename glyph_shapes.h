#pragma once

#include <cgv/render/render_types.h>
//#include <cgv/math/functions.h>

enum ActionType {
	AT_NONE = 0,
	AT_CONFIGURATION_CHANGE = 1,
	AT_VALUE_CHANGE = 2
};

enum GlyphType {
	GT_CIRCLE = 0,
	GT_RECTANGLE = 1,
	GT_WEDGE = 2,
	GT_ARC_FLAT = 3,
	GT_ARC_ROUNDED = 4,
	GT_TRIANGLE = 5,
	GT_DROP = 6,
	GT_STAR = 7
};

enum GlyphAttributeType {
	GAT_SIZE = 0, // value in [0,inf) determining radius, length or scale in general
	GAT_ANGLE = 1, // value in [0,360] giving angle in degree
	GAT_DOUBLE_ANGLE = 2, // value in [0,360] giving angle in degree (divided by 2 for the actual mapping)
	GAT_ORIENTATION = 3, // value in [0,360] giving angle in degree used specifically to orient the glyph
	GAT_COLOR = 4, // rgb color
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

protected:
	
public:
	virtual glyph_shape* clone() const = 0;
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

class circle_glyph : public glyph_shape {
public:
	virtual circle_glyph* clone() const {
		return new circle_glyph(*this);
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

	virtual std::string name() const {
		return "rectangle";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
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

	virtual std::string name() const {
		return "wedge";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
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

	virtual std::string name() const {
		return "arc_flat";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
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

	virtual std::string name() const {
		return "arc_rounded";
	}
};

class isoceles_triangle_glyph : public glyph_shape {
public:
	virtual isoceles_triangle_glyph* clone() const {
		return new isoceles_triangle_glyph(*this);
	}

	virtual std::string name() const {
		return "triangle_isosceles";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
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

	virtual std::string name() const {
		return "drop";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
			{ "base_radius", GAT_SIZE },
			{ "tip_radius", GAT_SIZE },
			{ "height", GAT_SIZE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}
};

class star_glyph : public glyph_shape {
public:
	virtual star_glyph* clone() const {
		return new star_glyph(*this);
	}

	virtual std::string name() const {
		return "star";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "color", GAT_COLOR, GAM_GLOBAL },
			{ "radius", GAT_SIZE, GAM_GLOBAL },
			{ "axis_0", GAT_SIZE, GAM_NON_CONST },
			{ "color_0", GAT_COLOR, GAM_GLOBAL },
			{ "axis_1", GAT_SIZE, GAM_NON_CONST },
			{ "color_1", GAT_COLOR, GAM_GLOBAL },
			{ "axis_2", GAT_SIZE, GAM_NON_CONST },
			{ "color_2", GAT_COLOR, GAM_GLOBAL }
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
