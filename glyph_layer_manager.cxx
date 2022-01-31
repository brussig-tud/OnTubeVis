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

const glyph_layer_manager::layer_configuration& glyph_layer_manager::get_configuration() {

	// initialize local variables
	const std::string constant_parameter_name_prefix = "glyph_c_param";
	const std::string mapped_parameter_name_prefix = "glyph_m_param";
	const std::string constant_color_name_prefix = "cglyph_color";

	std::vector<std::pair<std::string, const float*>> constant_glyph_parameters;
	std::vector<std::pair<std::string, const vec4*>> mapped_glyph_parameters;
	std::vector<std::pair<std::string, const rgb*>> constant_glyph_colors;
	std::vector<std::string> mapped_attrib_parameter_names;

	std::string code = "";

	// clear previous configuration
	layer_config.shapes.clear();
	layer_config.mapped_attributes.clear();
	layer_config.glyph_mapping_sources.clear();

	/*auto rgb_to_glsl_str = [](const rgb& col) {
		std::string str = "vec3(";
		str += std::to_string(col.R()) + ", ";
		str += std::to_string(col.G()) + ", ";
		str += std::to_string(col.B()) + ")";
		return str;
	};*/

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		const glyph_attribute_mapping& gam = glyph_attribute_mappings[i];

		const glyph_shape* shape_ptr = gam.get_shape_ptr();

		if(shape_ptr) {
			// TODO: this may be unsafe (make a copy? needs to be deleted afterwards)
			layer_config.shapes.push_back(shape_ptr);

			const glyph_shape::attribute_list& attribs = shape_ptr->supported_attributes();

			std::string func_name_str = "sd_" + shape_ptr->name();
			std::string glyph_coord_str = "glyphuv";
			std::string func_parameters_str = "";

			const std::vector<int> attrib_indices = gam.get_attrib_indices();
			const std::vector<vec4>& attrib_values = gam.ref_attrib_values();

			std::vector<std::string> func_parameters_strs;

			for(size_t j = 0; j < attrib_indices.size(); ++j) {
				std::string parameter_str = "";

				if(attrib_indices[j] < 0) {
					// constant non-mapped parameter
					std::string uniform_name = constant_parameter_name_prefix + std::to_string(constant_glyph_parameters.size());

					switch(attribs[j].type) {
					case GAT_ORIENTATION:
					case GAT_SIZE: parameter_str = uniform_name; break;
					case GAT_ANGLE: parameter_str = "radians(" + uniform_name + ")"; break;
					case GAT_DOUBLE_ANGLE: parameter_str = "radians(0.5*" + uniform_name + ")"; break;
					default: break;
					}

					constant_glyph_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j][3]));
					layer_config.glyph_mapping_sources.push_back(0);
				} else {
					// mapped parameter
					//const std::string& attrib_name = attribs[j].name;
					const std::string& attrib_name = "v[" + std::to_string(mapped_attrib_parameter_names.size()) + "]";

					std::string uniform_name = mapped_parameter_name_prefix + std::to_string(mapped_glyph_parameters.size());
					parameter_str = "clamp_remap(current_glyph." + attrib_name + ", " + uniform_name + ")";

					switch(attribs[j].type) {
					case GAT_ORIENTATION:
					case GAT_SIZE: parameter_str = parameter_str; break;
					case GAT_ANGLE: parameter_str = "radians(" + parameter_str + ")"; break;
					case GAT_DOUBLE_ANGLE: parameter_str = "radians(0.5*" + parameter_str + ")"; break;
					default: break;
					}

					mapped_glyph_parameters.push_back(std::make_pair(uniform_name, &attrib_values[j]));

					mapped_attrib_parameter_names.push_back(attrib_name);
					layer_config.mapped_attributes.push_back(attrib_indices[j]);
					layer_config.glyph_mapping_sources.push_back(1);
				}

				if(attribs[j].type == GAT_ORIENTATION) {
					glyph_coord_str = "rotate(glyphuv, radians(" + parameter_str + "))";
				} else {
					func_parameters_strs.push_back(parameter_str);
				}
			}

			// get glyph color
			std::string color_str = constant_color_name_prefix + std::to_string(constant_glyph_colors.size());
			constant_glyph_colors.push_back(std::make_pair(color_str, &gam.ref_color()));

			// generate the glyph splat function
			std::string splat_func = shape_ptr->splat_func();
			if(splat_func == "") {
				std::string glyph_func = func_name_str + "(" + glyph_coord_str + ", ";

				for(size_t j = 0; j < func_parameters_strs.size(); ++j) {
					glyph_func += func_parameters_strs[j];
					if(j < func_parameters_strs.size() - 1)
						glyph_func += ", ";
				}

				glyph_func += ")";

				splat_func = "splat_generic_glyph(current_glyph, " + glyph_func + ", " + color_str + ")";
			} else {
				splat_func = "splat_star(current_glyph, " + glyph_coord_str + ", " + std::to_string(0.75f) + ", " + color_str + ")";
			}

			//code = "splat_glyph(glyphuv, current_glyph, " + splat_func + ", " + color_str + ", color);";
			code = "finalize_glyph(glyphuv, current_glyph, " + splat_func + ", color);";
		}
	}

	std::string uniforms_str = "";
	if(constant_glyph_parameters.size() > 0) {
		uniforms_str = "uniform float ";
		for(size_t i = 0; i < constant_glyph_parameters.size(); ++i) {
			uniforms_str += constant_glyph_parameters[i].first;
			if(i < constant_glyph_parameters.size() - 1)
				uniforms_str += ", ";
		}
		uniforms_str += ";";
	}

	if(mapped_glyph_parameters.size() > 0) {
		uniforms_str += "uniform vec4 ";
		for(size_t i = 0; i < mapped_glyph_parameters.size(); ++i) {
			uniforms_str += mapped_glyph_parameters[i].first;
			if(i < mapped_glyph_parameters.size() - 1)
				uniforms_str += ", ";
		}
		uniforms_str += ";";
	}

	if(constant_glyph_colors.size() > 0) {
		uniforms_str += "uniform vec3 ";
		for(size_t i = 0; i < constant_glyph_colors.size(); ++i) {
			uniforms_str += constant_glyph_colors[i].first;
			if(i < constant_glyph_colors.size() - 1)
				uniforms_str += ", ";
		}
		uniforms_str += ";";
	}

	/*std::string glyph_attrib_block = "";
	if(mapped_attrib_parameter_names.size() > 0) {
		glyph_attrib_block = "float ";
		for(size_t i = 0; i < mapped_attrib_parameter_names.size(); ++i) {
			glyph_attrib_block += mapped_attrib_parameter_names[i];
			if(i < mapped_attrib_parameter_names.size() - 1)
				glyph_attrib_block += ", ";
		}
		glyph_attrib_block += ";";
	}*/

	std::string glyph_attrib_block = "";
	if(mapped_attrib_parameter_names.size() > 0)
		glyph_attrib_block = "float v[" + std::to_string(mapped_attrib_parameter_names.size()) + "];";

	layer_config.constant_parameters = constant_glyph_parameters;
	layer_config.mapping_parameters = mapped_glyph_parameters;
	layer_config.constant_colors = constant_glyph_colors;

	// save shader configuration
	layer_config.shader_config.uniforms_definition = uniforms_str;
	layer_config.shader_config.glyph_layers_definition = code;
	layer_config.shader_config.attribute_buffer_definition = glyph_attrib_block;

	std::cout << std::endl << std::endl << uniforms_str << std::endl << std::endl;
	std::cout << glyph_attrib_block << std::endl << std::endl;
	std::cout << code << std::endl << std::endl;

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
