#include "glyph_layer_manager.h"

void glyph_layer_manager::clear() {
	visible.clear();
	glyph_attribute_mappings.clear();
	notify_configuration_change();
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

const std::string glyph_layer_manager::configuration::constant_float_parameter_name_prefix = "glyph_cf_param";
const std::string glyph_layer_manager::configuration::constant_color_parameter_name_prefix = "glyph_cc_param";
const std::string glyph_layer_manager::configuration::mapped_parameter_name_prefix = "glyph_m_param";

const glyph_layer_manager::configuration& glyph_layer_manager::get_configuration() {

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
	config.clear();

	size_t last_constant_float_parameters_size = 0;
	size_t last_constant_color_parameters_size = 0;
	size_t last_mapping_parameters_size = 0;

	// iterate over layers
	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		const glyph_attribute_mapping& gam = glyph_attribute_mappings[i];
		const glyph_shape* shape_ptr = gam.get_shape_ptr();

		config.layer_configs.push_back(configuration::layer_configuration());
		auto& layer_config = config.layer_configs.back();

		layer_config.visible = visible[i].v;
		layer_config.sampling_strategy = gam.get_sampling_strategy();
		layer_config.sampling_step = gam.get_sampling_step();

		if(shape_ptr) {
			// TODO: this may be unsafe (make a copy? needs to be deleted afterwards)
			layer_config.shape_ptr = shape_ptr;

			const glyph_shape::attribute_list& attribs = shape_ptr->supported_attributes();

			std::string func_name_str = "sd_" + shape_ptr->name();
			std::string glyph_coord_str = "glyphuv";
			std::string glyph_outline_str = "0.0";
			
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

				// skip this parameter if it is not mapped from an attribute and does not allow constant values
				if(idx < 0 && is_non_const && !is_global)
					continue;

				if(idx < 0 || is_global) {
					// constant non-mapped parameter
					std::string uniform_name;

					if(type == GAT_COLOR) {
						uniform_name = config.constant_color_parameter_name_prefix + "[" + std::to_string(config.constant_color_parameters.size()) + "]";

						config.constant_color_parameters.push_back(std::make_pair(uniform_name, &attrib_colors[j]));
					} else {
						uniform_name = config.constant_float_parameter_name_prefix + "[" + std::to_string(config.constant_float_parameters.size()) + "]";
						
						layer_config.glyph_mapping_parameters.push_back({ 0, config.constant_float_parameters.size() - last_constant_float_parameters_size, &attrib_values[j] });
						config.constant_float_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j][3]));
					}

					parameter_str = uniform_name;
				} else {
					// mapped parameter
					const std::string& attrib_variable_name = "v[" + std::to_string(config.mapping_parameters.size() - last_mapping_parameters_size) + "]";
					std::string uniform_name = config.mapped_parameter_name_prefix + "[" + std::to_string(config.mapping_parameters.size()) + "]";

					std::string remap_func = "clamp_remap";
					switch(type) {
					case GAT_SIGNED_UNIT: remap_func = "clamp_remap11"; break;
					case GAT_UNIT:
					case GAT_COLOR: remap_func = "clamp_remap01"; break;
					default: break;
					}
					parameter_str = remap_func + "(glyph." + attrib_variable_name + ", " + uniform_name + ")";

					if(type == GAT_COLOR) {
						if(color_map_idx < 0)
							parameter_str = "vec3(0.0)";
						else
							parameter_str = "map_to_color(" + parameter_str + ", " + std::to_string(color_map_idx) + ")";
					} else {
						layer_config.glyph_mapping_parameters.push_back({ 1, config.mapping_parameters.size() - last_mapping_parameters_size, &attrib_values[j] });
					}

					config.mapping_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j]));
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
				} else if(type == GAT_OUTLINE) {
					glyph_outline_str = parameter_str;
				} else {
					float_parameter_strs.push_back(parameter_str);
				}
			}

			const std::string layer_id = std::to_string(i);

			// generate the glyph splat function
			std::string splat_func = shape_ptr->splat_func();
			if(splat_func == "") {
				// This is a generic glyph.
				// It directly takes all the float parameters in the signed distance function call
				// and only ever uses one color, which is given to the splat function.
				std::string glyph_func = func_name_str + "(" + glyph_coord_str;
				if(float_parameter_strs.size() > 0)
					glyph_func += ", ";
				glyph_func += join(float_parameter_strs, ", ");
				glyph_func += ")";

				std::string color_str = "vec3(0.0)";
				if(color_parameter_strs.size() > 0)
					color_str = color_parameter_strs[0];

				splat_func = "splat_generic_glyph(glyph.debug_info, " + glyph_func + ", " + color_str + + ", " + glyph_outline_str + ")";
			} else {
				// This is a special glyph.
				// It uses its own splat function and value handling is fully manual.
				std::string index_params_string =
					std::to_string(last_constant_float_parameters_size) + ", " +
					std::to_string(last_constant_color_parameters_size) + ", " +
					std::to_string(last_mapping_parameters_size);

				switch(shape_ptr->type()) {
				case GT_COLOR:
				{
					splat_func += layer_id + "(closest.id, uv, " + std::to_string(color_map_indices[1]) + ", ";
					splat_func += index_params_string;
					splat_func += ")";
				} break;
				case GT_STAR:
				{
					splat_func += layer_id + "(glyph, " + glyph_coord_str + ", ";
					splat_func += index_params_string;
					splat_func += ")";
				} break;
				case GT_LINE_PLOT:
				{
					splat_func += layer_id + "(closest.id, uv, ";
					splat_func += index_params_string + ", ";
					splat_func += glyph_outline_str;
					splat_func += ")";
				} break;
				case GT_TEMPORAL_HEAT_MAP:
				{
					splat_func += layer_id + "(closest.id, uv, " + std::to_string(color_map_indices[2]) + ", ";
					splat_func += index_params_string + ", ";
					splat_func += glyph_outline_str;
					splat_func += ")";
				} break;
				default: break;
				}
			}

			//code = "splat_glyph(glyphuv, current_glyph, " + splat_func + ", " + color_str + ", color);";
			layer_config.glyph_definition = "finalize_glyph(glyph.debug_info, glyphuv, " + splat_func + ", color);";

			last_constant_float_parameters_size = config.constant_float_parameters.size();
			last_constant_color_parameters_size = config.constant_color_parameters.size();
			last_mapping_parameters_size = config.mapping_parameters.size();
		}
	}

	config.create_uniforms_definition();

#ifdef _DEBUG
	std::cout << std::endl << ">>> SHADER DEFINES <<<" << std::endl;
	std::cout << config.uniforms_definition << std::endl << std::endl;
	int i = 0;
	for(const auto& lc : config.layer_configs) {
		std::cout << "L" << std::to_string(i++) << std::endl;
		std::cout << "Mapped attrib count = " << lc.mapped_attributes.size() << std::endl;
		std::cout << lc.glyph_definition << std::endl << std::endl;
	}
	std::cout << ">>> ============== <<<" << std::endl << std::endl;
#endif

	return config;
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
		bool node_is_open = p.begin_tree_node_void("Layer " + std::to_string(i + 1), &gam, -1, false, "level=2;options='w=80';align=''");
		p.add_member_control(this, "Show", visible[i].v, "toggle", "w=40", " ");
			
		connect_copy(
			//p.add_button("", "image='res://up32.png';fit_image=true;w=20;h=20;label=''", " ")->click,
			p.add_button("@8>", "w=20;h=20", " ")->click,
			cgv::signal::rebind(this, &glyph_layer_manager::move_glyph_attribute_mapping, cgv::signal::_c<size_t>(i), cgv::signal::_c<size_t>(-1))
		);

		connect_copy(
			//p.add_button("", "image='res://down32.png';fit_image=true;w=20;h=20;label=''", " ")->click,
			p.add_button("@2>", "w=20;h=20", " ")->click,
			cgv::signal::rebind(this, &glyph_layer_manager::move_glyph_attribute_mapping, cgv::signal::_c<size_t>(i), cgv::signal::_c<size_t>(1))
		);

		connect_copy(p.add_button("@9+", "w=20")->click, cgv::signal::rebind(this, &glyph_layer_manager::remove_glyph_attribute_mapping, cgv::signal::_c<size_t>(i)));
		if(node_is_open) {
			p.align("\a");
			gam.create_gui(this, p);
			p.align("\b");
			p.end_tree_node(gam);
		}
	}
}

void glyph_layer_manager::on_set(void* member_ptr) {

	last_action_type = AT_MAPPING_VALUE_CHANGE;

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mapping& gam = glyph_attribute_mappings[i];

		if(member_ptr == &visible[i].v) {
			last_action_type = AT_CONFIGURATION_CHANGE;
		}

		if(member_ptr == &gam) {
			//if(gam.action_type() == AT_CONFIGURATION_CHANGE)
			//	last_action_type = AT_CONFIGURATION_CHANGE;
			last_action_type = gam.action_type();
		}
	}

	/*for(size_t i = 0; i < layers.size(); ++i) {
		glyph_attribute_mapping& gam = layers[i].gam;
		if(member_ptr == &gam) {
			if(gam.action_type() == AT_CONFIGURATION_CHANGE)
				last_action_type = AT_CONFIGURATION_CHANGE;
		}
	}*/

	if(base_ptr)
		base_ptr->on_set(this);
}

void glyph_layer_manager::create_glyph_attribute_mapping() {
	if(glyph_attribute_mappings.size() == 4) {
		std::cout << "Cannot use more than 4 layers" << std::endl;
		return;
	}

	glyph_attribute_mappings.push_back(glyph_attribute_mapping());
	auto& gam = glyph_attribute_mappings.back();
	gam.set_attribute_names(attribute_names);
	gam.set_attribute_ranges(attribute_ranges);
	gam.set_color_map_names(color_map_names);

	visible.push_back(true);

	notify_configuration_change();
}

void glyph_layer_manager::remove_glyph_attribute_mapping(const size_t index) {
	if(index < glyph_attribute_mappings.size()) {
		visible.erase(visible.begin() + index);
		glyph_attribute_mappings.erase(glyph_attribute_mappings.begin() + index);
		
		notify_configuration_change();
	}
}

void glyph_layer_manager::move_glyph_attribute_mapping(const size_t index, int offset) {
	if(index < glyph_attribute_mappings.size()) {
		int index1 = index;
		if(offset < 0 && index > 0) {
			// move up if not the first element
			index1--;
		} else if(offset > 0 && index < glyph_attribute_mappings.size() - 1) {
			// move down if not the last element
			index1++;
		}

		if(index != index1) {
			std::swap(visible[index], visible[index1]);
			std::swap(glyph_attribute_mappings[index], glyph_attribute_mappings[index1]);
			notify_configuration_change();
		}
	}
}
