#include "attribute_mapping.h"

#include <cgv/data/informed_ptr.h>
#include <cgv/utils/algorithm.h>

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
	shape = other.shape->clone();
}

glyph_attribute_mapping::glyph_attribute_mapping(glyph_attribute_mapping&& other) noexcept : glyph_attribute_mapping() {
	swap(*this, other);
}

glyph_attribute_mapping& glyph_attribute_mapping::operator=(glyph_attribute_mapping other) {
	swap(*this, other);
	return *this;
}

ActionType glyph_attribute_mapping::action_type() {
	ActionType temp = last_action_type;
	last_action_type = ActionType::kUndefined;
	return temp;
}

void glyph_attribute_mapping::set_glyph_type(GlyphType type) {
	this->type = type;
	create_glyph_shape();
}

void glyph_attribute_mapping::set_visualization_variables(std::shared_ptr<const visualization_variables_info> variables) {
	visualization_variables = variables;

	for(size_t i = 0; i < attrib_mapping_values.size(); ++i) {
		int attrib_idx = attrib_source_indices[i];
		if(attrib_idx != k_unmapped_index)
			attrib_mapping_values[i].input_range = visualization_variables->ref_attribute_ranges()[attrib_idx];
	}
}

void glyph_attribute_mapping::set_attrib_source_index(size_t attrib_idx, int source_idx) {
	if(attrib_idx < attrib_source_indices.size())
		attrib_source_indices[attrib_idx] = source_idx;
}

void glyph_attribute_mapping::set_color_source_index(size_t color_idx, int source_idx) {
	if(color_idx < color_source_indices.size())
		color_source_indices[color_idx] = source_idx;
}

void glyph_attribute_mapping::set_attrib_in_range(size_t idx, const cgv::vec2& range) {
	if(idx < attrib_mapping_values.size())
		attrib_mapping_values[idx].input_range = range;
}

void glyph_attribute_mapping::set_attrib_out_range(size_t idx, const cgv::vec2& range) {
	if(idx < attrib_mapping_values.size()) {
		attrib_mapping_values[idx].output_range = range;
		reverse_colors[idx] = range.x() > range.y();
	}
}

void glyph_attribute_mapping::set_attrib_color(size_t idx, const cgv::rgb& color) {
	if(idx < attrib_colors.size())
		attrib_colors[idx] = color;
}

void glyph_attribute_mapping::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	if(!shape)
		return;

	add_local_member_control(p, bp, "Name", name, "", "w=146", "%x+=2");
	connect_copy(p.add_button("@1edit", "w=20")->click, cgv::signal::rebind(this, &glyph_attribute_mapping::update_name, cgv::signal::_c<cgv::base::base*>(bp)));

	add_local_member_control(p, bp, "Sampling Strategy", sampling_strategy, "dropdown", "enums='Uniform Time,Equidistant,Original Samples'");
	add_local_member_control(p, bp, "Sampling Step", sampling_step, "value_slider", "min=0;max=10;step=0.001;ticks=true;log=true");
	p.add_decorator("", "separator");

	auto glyph_types = glyph_type_registry::list_glyph_types();
	std::string enums = cgv::utils::transform_join(glyph_types.begin(), glyph_types.end(), [](GlyphType type) { return glyph_shape::display_name(type); }, ",");
	add_local_member_control(p, bp, "Shape", type, "dropdown", "enums='" + enums + "'");
	
	bool global_block = false;
	for(size_t i = 0; i < shape->supported_attributes().size(); ++i) {
		const glyph_attribute& attrib = shape->supported_attributes()[i];

		bool separator_requested = false;
		if(attrib.gui_hint == GuiLayoutHint::kBlockStart) {
			separator_requested = true;
			global_block = false;
		} else if(attrib.gui_hint == GuiLayoutHint::kGlobalBlockStart) {
			separator_requested = true;
			global_block = true;
		}

		if(i != 0 && separator_requested)
			p.add_decorator("", "separator");

		create_attribute_gui(bp, p, i, attrib, global_block);
	}
}

void glyph_attribute_mapping::on_set(void* member_ptr, cgv::base::base* base_ptr) {
	cgv::data::informed_ptr ptr(member_ptr);
	last_action_type = ActionType::kMappingValueChange;

	if(ptr.points_to(sampling_strategy))
		last_action_type = ActionType::kConfigurationChange;

	if(ptr.points_to(sampling_step)) {
		sampling_step = std::max(sampling_step, 0.0f);
		last_action_type = ActionType::kConfigurationValueChange;
	}

	if(ptr.points_to(type)) {
		last_action_type = ActionType::kConfigurationChange;
		create_glyph_shape();
	}

	if(auto it = ptr.find_in_data_of(attrib_source_indices); it != attrib_source_indices.end()) {
		last_action_type = ActionType::kConfigurationChange;
		int attrib_idx = *it;
		if(attrib_idx != k_unmapped_index)
			attrib_mapping_values[std::distance(attrib_source_indices.begin(), it)].input_range = visualization_variables->ref_attribute_ranges()[attrib_idx];
	}

	if(ptr.points_to_data_of(color_source_indices))
		last_action_type = ActionType::kConfigurationChange;

	if(auto it = ptr.find_in_data_of(reverse_colors); it != reverse_colors.end()) {
		cgv::type::bool32_t is_reversed = *it;
		attrib_mapping_values[std::distance(reverse_colors.begin(), it)].output_range = is_reversed ? cgv::vec2(1.0f, 0.0f) : cgv::vec2(0.0f, 1.0f);
	}

	base_ptr->on_set(this);
}

void glyph_attribute_mapping::update_name(cgv::base::base* base_ptr) {

	last_action_type = ActionType::kConfigurationChange;
	base_ptr->on_set(this);
}

void glyph_attribute_mapping::create_glyph_shape() {
	shape = glyph_shape_factory::create(type);

	attrib_source_indices.clear();
	color_source_indices.clear();
	attrib_mapping_values.clear();
	reverse_colors.clear();
	attrib_colors.clear();

	size_t attrib_count = shape->supported_attributes().size();
	attrib_source_indices.resize(attrib_count, k_unmapped_index);
	color_source_indices.resize(attrib_count, k_unmapped_index);

	for(size_t i = 0; i < attrib_count; ++i) {
		GlyphAttributeType type = shape->supported_attributes()[i].type;

		// set unit ranges as default
		cgv::vec2 input_range = { 0.0f, 1.0f };
		cgv::vec2 output_range = { 0.0f, 1.0f };

		switch(type) {
		case GlyphAttributeType::kSignedUnit:
			output_range = { -1.0f, 1.0f };
			break;
		case GlyphAttributeType::kAngle:
		case GlyphAttributeType::kDoubleAngle:
		case GlyphAttributeType::kOrientation:
			output_range = { 0.0f, 360.0f };
			break;
		case GlyphAttributeType::kOutline:
			output_range = { 0.0f, 0.0f };
			break;
		default:
			break;
		}

		attrib_mapping_values.push_back({ input_range, output_range });
	}

	reverse_colors.resize(attrib_count, false);
	attrib_colors.resize(attrib_count, cgv::rgb(0.0f));
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
	case GlyphAttributeType::kSignedUnit: lower_limit = "-1"; break;
	case GlyphAttributeType::kSize: upper_limit = "2"; break;
	case GlyphAttributeType::kAngle:
	case GlyphAttributeType::kDoubleAngle:
	case GlyphAttributeType::kOrientation: upper_limit = "360"; break;
	case GlyphAttributeType::kOutline: upper_limit = "0.5"; break;
	default: break;
	}

	std::string label = to_display_str(attrib.name);
	
	bool is_global = (attrib.modifiers & GlyphAttributeModifier::kGlobal) != GlyphAttributeModifier::kNone;
	bool is_non_const = (attrib.modifiers & GlyphAttributeModifier::kNonConst) != GlyphAttributeModifier::kNone;

	int selected_attrib_src_idx = attrib_source_indices[i];
	int selected_color_src_idx = color_source_indices[i];

	std::string value_label = label;

	// Add a default choice representing disabled mappings a.k.a unmapped attributes.
	const std::string default_name = is_non_const ? "(disabled)" : "-";
	const std::string default_index = "=" + std::to_string(k_unmapped_index);
	std::string attrib_name_enums = default_name + "=" + default_index;
	std::string color_map_name_enums = default_name + "=" + default_index;
	
	attrib_name_enums += "," + visualization_variables->get_attribute_names_list();
	color_map_name_enums += "," + visualization_variables->get_color_map_names_list();

	if(is_global) {
		if(!global_block) {
			value_label = "";
			p.add_decorator(label, "heading", "level=4");

			if(attrib.type == GlyphAttributeType::kColor && (attrib.modifiers & GlyphAttributeModifier::kForceMappable) != GlyphAttributeModifier::kNone)
				add_local_member_control(p, bp, "Color Map", reinterpret_cast<cgv::type::DummyEnum&>(color_source_indices[i]), "dropdown", "enums='" + color_map_name_enums + "'");
		}
	} else {
		value_label = "Value";
		p.add_decorator(label, "heading", "level=4");

		add_local_member_control(p, bp, "Source Attribute", reinterpret_cast<cgv::type::DummyEnum&>(attrib_source_indices[i]), "dropdown", "enums='" + attrib_name_enums + "'");
		
		if(attrib.type == GlyphAttributeType::kColor) {
			if(selected_attrib_src_idx > -1) {
				add_local_member_control(p, bp, "Color Map", reinterpret_cast<cgv::type::DummyEnum&>(color_source_indices[i]), "dropdown", "enums='" + color_map_name_enums + "';w=126", " ");
				add_local_member_control(p, bp, "Reverse", reinterpret_cast<bool&>(reverse_colors[i]), "check", "w=62");
			}
		}
	}

	std::string out_options_str = "min=" + lower_limit + ";max=" + upper_limit + ";step=0.001;ticks=true";

	if(selected_attrib_src_idx < 0 || is_global) {
		if(attrib.type == GlyphAttributeType::kColor) {
			if((attrib.modifiers & GlyphAttributeModifier::kForceMappable) == GlyphAttributeModifier::kNone)
				add_local_member_control(p, bp, value_label, attrib_colors[i]);
		} else {
			add_local_member_control(p, bp, value_label, attrib_mapping_values[i].output_range.y(), "value_slider", out_options_str);
		}
	} else {
		const cgv::vec2& range = visualization_variables->ref_attribute_ranges()[selected_attrib_src_idx];
		std::string in_options_str = "min=" + std::to_string(range.x()) + ";max=" + std::to_string(range.y()) + ";step=0.001;ticks=true";
		
		add_local_member_control(p, bp, "In Min", attrib_mapping_values[i].input_range.x(), "value_slider", in_options_str);
		add_local_member_control(p, bp, "In Max", attrib_mapping_values[i].input_range.y(), "value_slider", in_options_str);
		
		if(attrib.type != GlyphAttributeType::kColor && attrib.type != GlyphAttributeType::kUnit && attrib.type != GlyphAttributeType::kSignedUnit) {
			add_local_member_control(p, bp, "Out Min", attrib_mapping_values[i].output_range.x(), "value_slider", out_options_str);
			add_local_member_control(p, bp, "Out Max", attrib_mapping_values[i].output_range.y(), "value_slider", out_options_str);
		}
	}
}
