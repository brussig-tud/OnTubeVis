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
	GT_DROP = 6
};

enum GlyphAttributeType {
	GAT_SIZE = 0, // value in [0,inf) determining radius, length or scale in general
	GAT_ANGLE = 1, // value in [0,360] giving angle in degree
	GAT_DOUBLE_ANGLE = 2, // value in [0,360] giving angle in degree (divided by 2 for the actual mapping)
	GAT_ORIENTATION = 3 // value in [0,360] giving angle in degree used specifically to orient the glyph
};

struct glyph_attribute {
	std::string name;
	GlyphAttributeType type;

	glyph_attribute(std::string name, GlyphAttributeType type) : name(name), type(type) {}
};

class glyph_shape : public cgv::render::render_types {
public:
	typedef std::vector<glyph_attribute> attribute_list;

protected:
	
public:
	virtual glyph_shape* clone() const = 0;
	virtual std::string name() const = 0;
	virtual const attribute_list& supported_attributes() const = 0;

	virtual float get_size(const std::vector<float>& param_values) const {
		return 0.0f;
	};
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
			{ "radius", GAT_SIZE }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
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
			{ "length", GAT_SIZE },
			{ "height", GAT_SIZE }
		};
		return attributes;
	}

	virtual float get_size(const std::vector<float>& param_values) const {
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
			{ "radius", GAT_SIZE },
			{ "aperture", GAT_DOUBLE_ANGLE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
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
			{ "radius", GAT_SIZE },
			{ "thickness", GAT_SIZE },
			{ "aperture", GAT_DOUBLE_ANGLE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
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
			{ "base_width", GAT_SIZE },
			{ "height", GAT_SIZE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
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
			{ "base_radius", GAT_SIZE },
			{ "tip_radius", GAT_SIZE },
			{ "height", GAT_SIZE },
			{ "orientation", GAT_ORIENTATION }
		};
		return attributes;
	}
};
