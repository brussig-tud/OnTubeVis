#include "glyph_attribute_mapping.h"

glyph_attribute_mapping::glyph_attribute_mapping() {
	create_glyph_shape();
}

glyph_attribute_mapping::glyph_attribute_mapping(const glyph_attribute_mapping& other) :
	name(other.name),
	active(other.active),
	sampling_strategy(other.sampling_strategy),
	sampling_step(other.sampling_step),
	type(other.type),
	attrib_source_indices(other.attrib_source_indices),
	color_source_indices(other.color_source_indices),
	attrib_mapping_values(other.attrib_mapping_values),
	reverse_colors(other.reverse_colors),
	attrib_colors(other.attrib_colors),
	visualization_variables(other.visualization_variables) {
	if(other.shape_ptr)
		shape_ptr = other.shape_ptr->copy();
}

glyph_attribute_mapping::glyph_attribute_mapping(glyph_attribute_mapping&& other) noexcept : glyph_attribute_mapping() {
	swap(*this, other);
}

glyph_attribute_mapping& glyph_attribute_mapping::operator=(glyph_attribute_mapping other) {
	swap(*this, other);
	return *this;
}

glyph_attribute_mapping::~glyph_attribute_mapping() {
	delete shape_ptr;
}

const std::vector<int> glyph_attribute_mapping::get_attrib_indices() const {
	std::vector<int> indices(attrib_source_indices.size(), -1);
	for(size_t i = 0; i < attrib_source_indices.size(); ++i)
		indices[i] = dummy_enum_to_int(attrib_source_indices[i]);

	return indices;
}

const std::vector<int> glyph_attribute_mapping::get_color_map_indices() const {
	std::vector<int> indices(color_source_indices.size(), -1);
	for(size_t i = 0; i < color_source_indices.size(); ++i)
		indices[i] = dummy_enum_to_int(color_source_indices[i]);

	return indices;
}

ActionType glyph_attribute_mapping::action_type() {
	ActionType temp = last_action_type;
	last_action_type = AT_NONE;
	return temp;
}

void glyph_attribute_mapping::set_glyph_type(GlyphType type) {
	this->type = type;
	create_glyph_shape();
}

void glyph_attribute_mapping::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	if(!shape_ptr)
		return;

	add_local_member_control(p, bp, "Name", name, "", "w=146", "%x+=2");
	connect_copy(p.add_button("@1edit", "w=20")->click, cgv::signal::rebind(this, &glyph_attribute_mapping::update_name, cgv::signal::_c<cgv::base::base*>(bp)));

	add_local_member_control(p, bp, "Sampling Strategy", sampling_strategy, "dropdown", "enums='Uniform Time,Equidistant,Original Samples'");
	add_local_member_control(p, bp, "Sampling Step", sampling_step, "value_slider", "min=0;max=10;step=0.001;ticks=true;log=true");
	p.add_decorator("", "separator");

	std::string enums = glyph_type_registry::display_name_enums();
	add_local_member_control(p, bp, "Shape", type, "dropdown", "enums='" + enums + "'");
	
	bool global_block = false;
	for(size_t i = 0; i < shape_ptr->supported_attributes().size(); ++i) {
		const glyph_attribute& attrib = shape_ptr->supported_attributes()[i];

		bool separator_requested = false;
		if(attrib.gui_hint == GH_BLOCK_START) {
			separator_requested = true;
			global_block = false;
		} else if(attrib.gui_hint == GH_GLOBAL_BLOCK_START) {
			separator_requested = true;
			global_block = true;
		}

		if(i != 0 && separator_requested)
			p.add_decorator("", "separator");

		create_attribute_gui(bp, p, i, attrib, global_block);
	}
}

void glyph_attribute_mapping::on_set(void* member_ptr, cgv::base::base* base_ptr) {
	last_action_type = AT_MAPPING_VALUE_CHANGE;

	if(member_ptr == &sampling_strategy) {
		last_action_type = AT_CONFIGURATION_CHANGE;
	}

	if(member_ptr == &sampling_step) {
		sampling_step = std::max(sampling_step, 0.0f);
		last_action_type = AT_CONFIGURATION_VALUE_CHANGE;
	}

	if(member_ptr == &type) {
		last_action_type = AT_CONFIGURATION_CHANGE;
		create_glyph_shape();
	}

	for(size_t i = 0; i < attrib_source_indices.size(); ++i) {
		if(member_ptr == &attrib_source_indices[i]) {
			last_action_type = AT_CONFIGURATION_CHANGE;
			int attrib_idx = dummy_enum_to_int(attrib_source_indices[i]);
			if(attrib_idx > -1) {
				const vec2& range = visualization_variables->ref_attribute_ranges()[attrib_idx];
				attrib_mapping_values[i].x() = range.x();
				attrib_mapping_values[i].y() = range.y();
			}
		}
	}

	for(size_t i = 0; i < color_source_indices.size(); ++i) {
		if(member_ptr == &color_source_indices[i])
			last_action_type = AT_CONFIGURATION_CHANGE;
	}

	for(size_t i = 0; i < reverse_colors.size(); ++i) {
		if(member_ptr == &reverse_colors[i].value) {
			if(reverse_colors[i].value) {
				attrib_mapping_values[i].z() = 1.0f;
				attrib_mapping_values[i].w() = 0.0f;
			} else {
				attrib_mapping_values[i].z() = 0.0f;
				attrib_mapping_values[i].w() = 1.0f;
			}
		}
	}
	
	base_ptr->on_set(this);
}

void glyph_attribute_mapping::update_name(cgv::base::base* base_ptr) {

	last_action_type = AT_CONFIGURATION_CHANGE;
	base_ptr->on_set(this);
}

void glyph_attribute_mapping::create_glyph_shape() {
	if(shape_ptr)
		delete shape_ptr;

	shape_ptr = glyph_shape_factory::create(type);

	attrib_source_indices.clear();
	color_source_indices.clear();
	attrib_mapping_values.clear();
	reverse_colors.clear();
	attrib_colors.clear();

	size_t attrib_count = shape_ptr->supported_attributes().size();
	attrib_source_indices.resize(attrib_count, static_cast<cgv::type::DummyEnum>(0));
	color_source_indices.resize(attrib_count, static_cast<cgv::type::DummyEnum>(0));

	for(size_t i = 0; i < attrib_count; ++i) {
		GlyphAttributeType type = shape_ptr->supported_attributes()[i].type;
		vec4 ranges(0.0f);

		switch(type) {
		case GAT_UNIT:
			ranges = vec4(0.0f, 1.0f, 0.0f, 1.0f);
			break;
		case GAT_SIGNED_UNIT:
			ranges = vec4(0.0f, 1.0f, -1.0f, 1.0f);
			break;
		case GAT_SIZE:
			ranges = vec4(0.0f, 1.0f, 0.0f, 1.0f);
			break;
		case GAT_ANGLE:
		case GAT_DOUBLE_ANGLE:
		case GAT_ORIENTATION:
			ranges = vec4(0.0f, 1.0f, 0.0f, 360.0f);
			break;
		case GAT_OUTLINE:
			ranges = vec4(0.0f, 1.0f, 0.0f, 0.0f);
			break;
		default:
			ranges = vec4(0.0f, 1.0f, 0.0f, 1.0f);
			break;
		}

		attrib_mapping_values.push_back(ranges);
	}

	reverse_colors.resize(attrib_count, Bool{ 0 });
	attrib_colors.resize(attrib_count, rgb(0.0f));
}

int glyph_attribute_mapping::dummy_enum_to_int(cgv::type::DummyEnum index) const {
	return static_cast<int>(index) - 1;
}

cgv::type::DummyEnum glyph_attribute_mapping::int_to_dummy_enum(int index) const {
	return static_cast<cgv::type::DummyEnum>(index + 1);
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

void glyph_attribute_mapping::create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i, const glyph_attribute& attrib, const bool global_block) {
	std::string lower_limit = "0";
	std::string upper_limit = "1";
	switch(attrib.type) {
	case GAT_SIGNED_UNIT: lower_limit = "-1"; break;
	case GAT_SIZE: upper_limit = "2"; break;
	case GAT_ANGLE:
	case GAT_DOUBLE_ANGLE:
	case GAT_ORIENTATION: upper_limit = "360"; break;
	case GAT_OUTLINE: upper_limit = "0.5"; break;
	default: break;
	}

	std::string label = to_display_str(attrib.name);
	
	bool is_global = attrib.modifiers & GAM_GLOBAL;
	bool is_non_const = attrib.modifiers & GAM_NON_CONST;

	int selected_attrib_src_idx = dummy_enum_to_int(attrib_source_indices[i]);
	int selected_color_src_idx = dummy_enum_to_int(color_source_indices[i]);

	std::string value_label = label;

	std::string attrib_name_enums = "-,";
	std::string color_map_name_enums = "-,";

	if(is_non_const) {
		attrib_name_enums = "(disabled),";
		color_map_name_enums = "(disabled),";
	}
	
	attrib_name_enums += visualization_variables->get_attribute_names_list();
	color_map_name_enums += visualization_variables->get_color_map_names_list();

	if(is_global) {
		if(!global_block) {
			value_label = "";
			p.add_decorator(label, "heading", "level=4");

			if(attrib.type == GAT_COLOR && attrib.modifiers & GAM_FORCE_MAPPABLE)
				add_local_member_control(p, bp, "Color Map", color_source_indices[i], "dropdown", "enums='" + color_map_name_enums + "'");
		}
	} else {
		value_label = "Value";
		p.add_decorator(label, "heading", "level=4");

		add_local_member_control(p, bp, "Source Attribute", attrib_source_indices[i], "dropdown", "enums='" + attrib_name_enums + "'");
		
		if(attrib.type == GAT_COLOR) {
			if(selected_attrib_src_idx > -1) {
				add_local_member_control(p, bp, "Color Map", color_source_indices[i], "dropdown", "enums='" + color_map_name_enums + "';w=126", " ");
				add_local_member_control(p, bp, "Reverse", reverse_colors[i].value, "check", "w=62");
			}
		}
	}

	std::string out_options_str = "min=" + lower_limit + ";max=" + upper_limit + ";step=0.001;ticks=true";

	if(selected_attrib_src_idx < 0 || is_global) {
		if(attrib.type == GAT_COLOR) {
			if(!(attrib.modifiers & GAM_FORCE_MAPPABLE))
				add_local_member_control(p, bp, value_label, attrib_colors[i]);
		} else {
			add_local_member_control(p, bp, value_label, attrib_mapping_values[i][3], "value_slider", out_options_str);
		}
	} else {
		const vec2& range = visualization_variables->ref_attribute_ranges()[selected_attrib_src_idx];
		std::string in_options_str = "min=" + std::to_string(range.x()) + ";max=" + std::to_string(range.y()) + ";step=0.001;ticks=true";
		
		add_local_member_control(p, bp, "In Min", attrib_mapping_values[i][0], "value_slider", in_options_str);
		add_local_member_control(p, bp, "In Max", attrib_mapping_values[i][1], "value_slider", in_options_str);
		
		if(attrib.type != GAT_COLOR && attrib.type != GAT_UNIT && attrib.type != GAT_SIGNED_UNIT) {
			add_local_member_control(p, bp, "Out Min", attrib_mapping_values[i][2], "value_slider", out_options_str);
			add_local_member_control(p, bp, "Out Max", attrib_mapping_values[i][3], "value_slider", out_options_str);
		}
	}
}
