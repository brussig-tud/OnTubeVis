#include "glyph_layer_manager.h"

void glyph_layer_manager::clear() {
	glyph_attribute_mappings.clear();
}

void glyph_layer_manager::set_attribute_names(const std::vector<std::string>& names) {
	attribute_names = names;

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i)
		glyph_attribute_mappings[i].set_attribute_names(names);
}

void glyph_layer_manager::set_attribute_ranges(const std::vector<vec2>& ranges) {
	attribute_ranges = ranges;

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mappings[i].set_attribute_ranges(ranges);

	}
}

void glyph_layer_manager::set_color_map_names(const std::vector<std::string>& names) {
	color_map_names = names;

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mappings[i].set_color_map_names(names);

	}
}

const std::string glyph_layer_manager::layer_configuration::constant_float_parameter_name_prefix = "glyph_cf_param";
const std::string glyph_layer_manager::layer_configuration::constant_color_parameter_name_prefix = "glyph_cc_param";
const std::string glyph_layer_manager::layer_configuration::mapped_parameter_name_prefix = "glyph_m_param";

const glyph_layer_manager::layer_configuration& glyph_layer_manager::get_configuration() {

	const auto join = [](const std::vector<std::string>& strings, const std::string& delimiter, bool trailing_delimiter = false) {
		std::string result = "";
		for(size_t i = 0; i < strings.size(); ++i) {
			result += strings[i];
			if(i < strings.size() - 1 || trailing_delimiter)
				result += delimiter;
		}
		return result;
	};

	// clear previous configuration
	layer_config.clear();

	std::string glyph_layers_definition = "";

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		const glyph_attribute_mapping& gam = glyph_attribute_mappings[i];

		const glyph_shape* shape_ptr = gam.get_shape_ptr();

		if(shape_ptr) {
			// TODO: this may be unsafe (make a copy? needs to be deleted afterwards)
			layer_config.shapes.push_back(shape_ptr);

			const glyph_shape::attribute_list& attribs = shape_ptr->supported_attributes();

			std::string func_name_str = "sd_" + shape_ptr->name();
			std::string glyph_coord_str = "glyphuv";
			
			/*
			std::vector<std::string> constant_float_parameters_strs;
			std::vector<std::string> mapped_float_parameters_strs;
			std::vector<std::string> constant_color_parameters_strs;
			std::vector<std::string> mapped_color_parameters_strs;
			*/

			// the parameters used in the signed distance and splat function calls
			std::vector<std::string> float_parameter_strs;
			std::vector<std::string> color_parameter_strs;

			const std::vector<int> attrib_indices = gam.get_attrib_indices();
			const std::vector<int> color_map_indices = gam.get_color_map_indices();
			const std::vector<vec4>& attrib_values = gam.ref_attrib_values();
			const std::vector<rgb>& attrib_colors = gam.ref_attrib_colors();

			for(size_t j = 0; j < attrib_indices.size(); ++j) {
				int idx = attrib_indices[j];
				int color_map_idx = color_map_indices[j];
				GlyphAttributeType type = attribs[j].type;
				GlyphAttributeModifier modifiers = attribs[j].modifiers;
				bool is_global = modifiers & GAM_GLOBAL;
				bool is_non_const = modifiers & GAM_NON_CONST;

				std::string parameter_str = "";

				// skipt this parameter if it is not mapped from an attribute and does not allow constant values
				if(idx < 0 && is_non_const && !is_global)
					continue;

				if(idx < 0 || is_global) {
					// constant non-mapped parameter
					std::string uniform_name;

					if(type == GAT_COLOR) {
						uniform_name = layer_config.constant_color_parameter_name_prefix + "[" + std::to_string(layer_config.constant_color_parameters.size()) + "]";

						layer_config.constant_color_parameters.push_back(std::make_pair(uniform_name, &attrib_colors[j]));
					} else {
						uniform_name = layer_config.constant_float_parameter_name_prefix + "[" + std::to_string(layer_config.constant_float_parameters.size()) + "]";
						
						layer_config.constant_float_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j][3]));
						layer_config.glyph_mapping_sources.push_back(0);
					}

					parameter_str = uniform_name;
				} else {
					// mapped parameter
					const std::string& attrib_variable_name = "v[" + std::to_string(layer_config.mapping_parameters.size()) + "]";
					std::string uniform_name = layer_config.mapped_parameter_name_prefix + "[" + std::to_string(layer_config.mapping_parameters.size()) + "]";

					std::string remap_func = type == GAT_COLOR ? "clamp_remap01" : "clamp_remap";
					parameter_str = remap_func + "(current_glyph." + attrib_variable_name + ", " + uniform_name + ")";

					if(type == GAT_COLOR) {
						parameter_str = "map_to_color(" + parameter_str + ", " + std::to_string(color_map_idx) + ")";
					} else {
						layer_config.glyph_mapping_sources.push_back(1);
					}

					layer_config.mapping_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j]));
					layer_config.mapped_attributes.push_back(idx);
				}

				switch(type) {
				case GAT_ORIENTATION:
				case GAT_ANGLE: parameter_str = "radians(" + parameter_str + ")"; break;
				case GAT_DOUBLE_ANGLE: parameter_str = "radians(0.5*" + parameter_str + ")"; break;
				default: break;
				}

				if(type == GAT_ORIENTATION) {
					glyph_coord_str = "rotate(glyphuv, " + parameter_str + ")";
				} else if(type == GAT_COLOR) {
					color_parameter_strs.push_back(parameter_str);
				} else {
					float_parameter_strs.push_back(parameter_str);
				}
			}

			// get glyph color
			//std::string color_str = constant_color_name_prefix + std::to_string(constant_glyph_colors.size());
			//constant_glyph_colors.push_back(std::make_pair(color_str, &gam.ref_color()));

			// generate the glyph splat function
			std::string splat_func = shape_ptr->splat_func();
			if(splat_func == "") {
				// This is a generic glyph.
				// It directly takes all the float parameters in the signed distance function call
				// and only ever uses one color, which is give to the splat function.
				std::string glyph_func = func_name_str + "(" + glyph_coord_str + ", ";
				//glyph_func += join(global_func_parameters_strs, ", ", true);
				glyph_func += join(float_parameter_strs, ", ");
				glyph_func += ")";

				std::string color_str = "vec3(0.0)";
				if(color_parameter_strs.size() > 0) {
					color_str = color_parameter_strs[0];
				}

				splat_func = "splat_generic_glyph(current_glyph, " + glyph_func + ", " + color_str + ")";
			} else {
				// TODO: check in glyph shape if global or constant/mapped params are used
				//std::string global_params_str = join(global_func_parameters_strs, ", ");// +(global_func_parameters_strs.size() > 0 ? ", " : "");

				//splat_func += "(current_glyph, " + glyph_coord_str + ", " + global_params_str + ", " + color_str + ")";
				splat_func += "(current_glyph, " + glyph_coord_str + ")";
			}

			//code = "splat_glyph(glyphuv, current_glyph, " + splat_func + ", " + color_str + ", color);";
			glyph_layers_definition = "finalize_glyph(glyphuv, current_glyph, " + splat_func + ", color);";
		}
	}

	layer_config.create_shader_config(glyph_layers_definition, true);

	return layer_config;
}

ActionType glyph_layer_manager::action_type() {
	ActionType temp = last_action_type;
	last_action_type = AT_NONE;
	return temp;
}

void glyph_layer_manager::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	base_ptr = bp;

	connect_copy(p.add_button("Add Layer")->click, cgv::signal::rebind(this, &glyph_layer_manager::create_glyph_attribute_mapping));
	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mapping& gam = glyph_attribute_mappings[i];
		bool node_is_open = p.begin_tree_node_void("Layer " + std::to_string(i + 1), &gam, -1, true, "level=2;options='w=180';align=''");
		connect_copy(p.add_button("X", "w=20")->click, cgv::signal::rebind(this, &glyph_layer_manager::remove_glyph_attribute_mapping, cgv::signal::_c<size_t>(i)));
		if(node_is_open) {
			p.align("\a");
			gam.create_gui(this, p);
			p.align("\b");
			p.end_tree_node(gam);
		}
	}
}

void glyph_layer_manager::on_set(void* member_ptr) {

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mapping& gam = glyph_attribute_mappings[i];
		if(member_ptr == &gam) {
			if(gam.action_type() == AT_CONFIGURATION_CHANGE)
				last_action_type = AT_CONFIGURATION_CHANGE;
		}
	}

	if(base_ptr)
		base_ptr->on_set(this);
}

void glyph_layer_manager::create_glyph_attribute_mapping() {
	glyph_attribute_mappings.push_back(glyph_attribute_mapping());
	auto& gam = glyph_attribute_mappings.back();
	gam.set_attribute_names(attribute_names);
	gam.set_attribute_ranges(attribute_ranges);
	gam.set_color_map_names(color_map_names);

	last_action_type = AT_CONFIGURATION_CHANGE;
	if(base_ptr)
		base_ptr->on_set(this);
}

void glyph_layer_manager::remove_glyph_attribute_mapping(const size_t index) {
	if(index < glyph_attribute_mappings.size()) {
		glyph_attribute_mappings.erase(glyph_attribute_mappings.begin() + index);
		
		last_action_type = AT_CONFIGURATION_CHANGE;
		if(base_ptr)
			base_ptr->on_set(this);
	}
}
