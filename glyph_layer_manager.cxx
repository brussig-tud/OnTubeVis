#include "glyph_layer_manager.h"

void glyph_layer_manager::clear() {
	glyph_attribute_mappings.clear();
}

void glyph_layer_manager::set_attribute_names(const std::vector<std::string>& names) {
	attribute_names = names;

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i)
		glyph_attribute_mappings[i].set_attribute_names(names);
}

const glyph_layer_manager::shader_configuration& glyph_layer_manager::generate_shader_configuration() {

	const std::string constant_parameter_name_prefix = "cglyph_param";
	const std::string constant_color_name_prefix = "cglyph_color";
	std::vector<std::pair<std::string, const float*>> constant_glyph_parameters;
	std::vector<std::pair<std::string, const rgb*>> constant_glyph_colors;
	std::vector<std::string> mapped_attrib_parameter_names;

	std::string code = "";

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
					parameter_str = constant_parameter_name_prefix + std::to_string(constant_glyph_parameters.size());
					std::string uniform_str = parameter_str;

					switch(attribs[j].type) {
					case GAT_SIZE: break;
					case GAT_ANGLE: parameter_str = "radians(" + parameter_str + ")"; break;
					case GAT_DOUBLE_ANGLE: parameter_str = "radians(0.5*" + parameter_str + ")"; break;
					default: break;
					}

					constant_glyph_parameters.push_back(std::make_pair(uniform_str, &attrib_values[j][3]));
				} else {
					// mapped parameter
					mapped_attrib_parameter_names.push_back(attribs[j].name);
					shader_config.mapped_attributes.push_back(attrib_indices[j]);
					parameter_str = "0.5"; // TODO: replace this with a call to the actual data
				}

				if(attribs[j].type == GAT_ORIENTATION) {
					glyph_coord_str = "rotate(glyphuv, radians(" + parameter_str + "))";
				} else {
					func_parameters_strs.push_back(parameter_str);
				}
			}

			code = func_name_str + "(" + glyph_coord_str + ", ";

			for(size_t j = 0; j < func_parameters_strs.size(); ++j) {
				code += func_parameters_strs[j];
				if(j < func_parameters_strs.size() - 1)
					code += ", ";
			}

			code += ")";

			std::string color_str = constant_color_name_prefix + std::to_string(constant_glyph_colors.size());
			constant_glyph_colors.push_back(std::make_pair(color_str, &gam.ref_color()));

			code = "splat_glyph(" + code + ", " + color_str + ", color);";
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

	if(constant_glyph_colors.size() > 0) {
		uniforms_str += "uniform vec3 ";
		for(size_t i = 0; i < constant_glyph_colors.size(); ++i) {
			uniforms_str += constant_glyph_colors[i].first;
			if(i < constant_glyph_colors.size() - 1)
				uniforms_str += ", ";
		}
		uniforms_str += ";";
	}

	std::string glyph_attrib_block = "";
	if(mapped_attrib_parameter_names.size() > 0) {
		glyph_attrib_block = "float ";
		for(size_t i = 0; i < mapped_attrib_parameter_names.size(); ++i) {
			glyph_attrib_block += mapped_attrib_parameter_names[i];
			if(i < mapped_attrib_parameter_names.size() - 1)
				glyph_attrib_block += ", ";
		}
		glyph_attrib_block += ";";
	}


	//uniform_block = uniforms_str;
	//glyph_block = code;
	//uniform_value_ptrs = std::move(constant_glyph_parameters);
	
	shader_config.uniforms_definition = uniforms_str;
	shader_config.glyph_layers_definition = code;
	shader_config.constant_parameters = constant_glyph_parameters;
	shader_config.constant_colors = constant_glyph_colors;

	std::cout << std::endl << std::endl << uniforms_str << std::endl << std::endl;
	std::cout << glyph_attrib_block << std::endl << std::endl;
	std::cout << code << std::endl << std::endl;

	return shader_config;
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
