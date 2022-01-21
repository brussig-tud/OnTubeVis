#pragma once

#include <cgv/render/render_types.h>
//#include <cgv/math/functions.h>

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
	GAT_ORIENTATION = 2 // value in [0,360] giving angle in degree used specifically to orient the glyph
};

struct glyph_attribute {
	std::string name;
	GlyphAttributeType type;

	glyph_attribute(std::string name, GlyphAttributeType type) : name(name), type(type) {}
};

class glyph_shape : public cgv::render::render_types {
public:
	typedef std::pair<bool, float> parameter_pair;
	typedef std::vector<parameter_pair> parameter_list;
	typedef std::vector<glyph_attribute> attribute_list;

protected:
	void add_parameter(std::string& code, size_t idx, const parameter_pair& parameter) const {
		if(parameter.first) {
			// mapped parameter
			code += "0.5"; // TODO: replace this with a call to the actual data
		} else {
			// constant parameter
			code += parameter.second;
		}
	}

	void fill_parameters(std::string& code, const parameter_list& parameters) const {
		for(size_t i = 0; i < parameters.size(); ++i) {
			add_parameter(code, i, parameters[i]);
			if(i < parameters.size() - 1)
				code += ", ";
		}
	}

public:

	virtual glyph_shape* clone() const = 0;
	virtual std::string name() const = 0;
	virtual const attribute_list& supported_attributes() const = 0;

	const std::string create_shader_code(const parameter_list& parameters) const {
		const attribute_list& attribs = supported_attributes();
		if(parameters.size() != attribs.size())
			return "";

		std::string code = "sd_";
		code += name();
		// TODO: find orientation attrib in list and use this to alter glyph coord (dont forget to skip the orientation param for the function call)
		code += "(glyphuv, ";
		fill_parameters(code, parameters);
		code += ");";
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
			{ "radius", GAT_SIZE }
		};
		return attributes;
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
};

class wedge_glyph : public glyph_shape {
public:
	virtual wedge_glyph * clone() const {
		return new wedge_glyph(*this);
	}

	virtual std::string name() const {
		return "wedge";
	}

	virtual const attribute_list& supported_attributes() const {
		static const attribute_list attributes = {
			{ "radius", GAT_SIZE },
			{ "aperture", GAT_ANGLE },
			{ "orientation", GAT_ORIENTATION },
		};
		return attributes;
	}
};
