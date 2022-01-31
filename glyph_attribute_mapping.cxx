#include "glyph_attribute_mapping.h"

glyph_attribute_mapping::glyph_attribute_mapping() {
	color = rgb(0.0f);
	create_glyph_shape();
}

glyph_attribute_mapping::glyph_attribute_mapping(const glyph_attribute_mapping& r) {
	type = r.type;
	if(r.shape_ptr)
		shape_ptr = r.shape_ptr->clone();
	color = r.color;
	attrib_source_indices = r.attrib_source_indices;
	attrib_mapping_values = r.attrib_mapping_values;
	attribute_names = r.attribute_names;
	attribute_ranges = r.attribute_ranges;
}

glyph_attribute_mapping& glyph_attribute_mapping::operator=(const glyph_attribute_mapping& r) {
	delete shape_ptr;
	shape_ptr = nullptr;
	if(r.shape_ptr)
		shape_ptr = r.shape_ptr->clone();
	return *this;
}

glyph_attribute_mapping::~glyph_attribute_mapping() {
	delete shape_ptr;
}

const std::vector<int> glyph_attribute_mapping::get_attrib_indices() const {
	std::vector<int> indices(attrib_source_indices.size(), -1);
	for(size_t i = 0; i < attrib_source_indices.size(); ++i)
		indices[i] = attribute_index_to_int(attrib_source_indices[i]);

	return indices;
}

ActionType glyph_attribute_mapping::action_type() {
	ActionType temp = last_action_type;
	last_action_type = AT_NONE;
	return temp;
}

void glyph_attribute_mapping::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	if(!shape_ptr)
		return;

	add_local_member_control(p, bp, "Shape", type, "dropdown", "enums='Circle,Rectangle,Wedge,Arc Flat,Arc Rounded,Isosceles Triangle,Drop,Star'");

	add_local_member_control(p, bp, "Color", color);

	for(size_t i = 0; i < shape_ptr->supported_attributes().size(); ++i)
		create_attribute_gui(bp, p, i);
}

void glyph_attribute_mapping::on_set(void* member_ptr, cgv::base::base* base_ptr) {
	last_action_type = AT_VALUE_CHANGE;

	if(member_ptr == &type) {
		last_action_type = AT_CONFIGURATION_CHANGE;
		create_glyph_shape();
	}

	for(size_t i = 0; i < attrib_source_indices.size(); ++i)
		if(member_ptr == &attrib_source_indices[i]) {
			last_action_type = AT_CONFIGURATION_CHANGE;
			int attrib_idx = attribute_index_to_int(attrib_source_indices[i]);
			if(attrib_idx > -1) {
				const vec2& range = attribute_ranges[attrib_idx];
				attrib_mapping_values[i].x() = range.x();
				attrib_mapping_values[i].y() = range.y();
			}
		}

	base_ptr->on_set(this);
}

void glyph_attribute_mapping::set_attribute_names(const std::vector<std::string>& names) {
	attribute_names = names;
}

void glyph_attribute_mapping::set_attribute_ranges(const std::vector<vec2>& ranges) {
	attribute_ranges = ranges;

	for(size_t i = 0; i < attrib_mapping_values.size(); ++i) {
		int attrib_idx = attribute_index_to_int(attrib_source_indices[i]);
		if(attrib_idx > -1) {
			const vec2& range = attribute_ranges[attrib_idx];
			attrib_mapping_values[i].x() = range.x();
			attrib_mapping_values[i].y() = range.y();
		}
	}
}

void glyph_attribute_mapping::create_glyph_shape() {
	if(shape_ptr)
		delete shape_ptr;

	switch(type) {
	case GT_CIRCLE: shape_ptr = new circle_glyph(); break;
	case GT_RECTANGLE: shape_ptr = new rectangle_glyph(); break;
	case GT_WEDGE: shape_ptr = new wedge_glyph(); break;
	case GT_ARC_FLAT: shape_ptr = new flat_arc_glyph(); break;
	case GT_ARC_ROUNDED: shape_ptr = new rounded_arc_glyph(); break;
	case GT_TRIANGLE: shape_ptr = new isoceles_triangle_glyph(); break;
	case GT_DROP: shape_ptr = new drop_glyph(); break;
	case GT_STAR: shape_ptr = new star_glyph(); break;
	default: shape_ptr = new circle_glyph(); break;
	}

	attrib_source_indices.clear();
	attrib_mapping_values.clear();
	size_t attrib_count = shape_ptr->supported_attributes().size();
	attrib_source_indices.resize(attrib_count, static_cast<cgv::type::DummyEnum>(0));

	for(size_t i = 0; i < attrib_count; ++i) {
		GlyphAttributeType type = shape_ptr->supported_attributes()[i].type;
		vec4 ranges(0.0f);

		switch(type) {
		case GAT_SIZE:
			ranges = vec4(0.0f, 1.0f, 0.0f, 1.0f);
			break;
		case GAT_ANGLE:
		case GAT_DOUBLE_ANGLE:
		case GAT_ORIENTATION:
			ranges = vec4(0.0f, 1.0f, 0.0f, 360.0f);
			break;
		default:
			ranges = vec4(0.0f, 1.0f, 0.0f, 1.0f);
			break;
		}

		attrib_mapping_values.push_back(ranges);
	}
}

int glyph_attribute_mapping::attribute_index_to_int(cgv::type::DummyEnum index) const {
	return static_cast<int>(index) - 1;
}

std::string glyph_attribute_mapping::to_display_str(const std::string& name) const {

	if(name.length() > 0) {
		std::string str = name;

		bool up_next = false;
		for(size_t i = 0; i < str.length(); ++i) {
			if(i == 0) {
				str[i] = toupper(name[i]);
			} else {
				if(name[i] == '_') {
					str[i] = ' ';
					up_next = true;
				} else {
					if(up_next) {
						str[i] = toupper(name[i]);
						up_next = false;
					}
				}
			}
		}
		return str;
	}
	return name;
}

void glyph_attribute_mapping::create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i) {
	const glyph_attribute& attrib = shape_ptr->supported_attributes()[i];

	std::string limit = "1";
	switch(attrib.type) {
	case GAT_SIZE: limit = "2"; break;
	case GAT_ANGLE:
	case GAT_DOUBLE_ANGLE:
	case GAT_ORIENTATION: limit = "360"; break;
	default: break;
	}

	std::string label = to_display_str(attrib.name);
	//if(label.length() > 0) {
	//	label[0] = toupper(label[0]);
	//}

	p.add_decorator(label, "heading", "level=4");


	std::string attrib_names_enums = "_,";
	for(size_t i = 0; i < attribute_names.size(); ++i) {
		attrib_names_enums += attribute_names[i];
		if(i < attribute_names.size() - 1)
			attrib_names_enums += ",";
	}

	//add_local_member_control(p, bp, "Attribute", attrib_source_indices[i], "value", "min=-1;max=10;step=1;ticks=true");
	add_local_member_control(p, bp, "Source Attribute", attrib_source_indices[i], "dropdown", "enums='" + attrib_names_enums + "'");

	int selected_attrib_src_idx = attribute_index_to_int(attrib_source_indices[i]);

	if(selected_attrib_src_idx < 0) {
		add_local_member_control(p, bp, "Value", attrib_mapping_values[i][3], "value_slider", "min=0;max=" + limit + ";step=0.001;ticks=true");
	} else {
		const vec2& range = attribute_ranges[selected_attrib_src_idx];
		std::string options_str = "min=" + std::to_string(range.x()) + ";max=" + std::to_string(range.y()) + ";step=0.001;ticks=true";
		add_local_member_control(p, bp, "In Min", attrib_mapping_values[i][0], "value_slider", options_str);
		add_local_member_control(p, bp, "In Max", attrib_mapping_values[i][1], "value_slider", options_str);
		add_local_member_control(p, bp, "Out Min", attrib_mapping_values[i][2], "value_slider", "min=0;max=" + limit + ";step=0.001;ticks=true");
		add_local_member_control(p, bp, "Out Max", attrib_mapping_values[i][3], "value_slider", "min=0;max=" + limit + ";step=0.001;ticks=true");
	}
}
