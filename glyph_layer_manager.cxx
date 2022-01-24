#include "glyph_layer_manager.h"

void glyph_layer_manager::generate_shader_code(std::string& uniform_block, std::string& glyph_block, std::vector<std::pair<std::string, const float*>>& uniform_value_ptrs) const {

	const std::string constant_uniform_name_prefix = "cglyph_param";
	std::vector<std::pair<std::string, const float*>> constant_glyph_parameters;

	std::string code = "";

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		const glyph_attribute_mapping& gam = glyph_attribute_mappings[i];

		const glyph_shape* shape_ptr = gam.get_shape_ptr();

		if(shape_ptr) {
			const glyph_shape::attribute_list& attribs = shape_ptr->supported_attributes();

			std::string func_name_str = "sd_" + shape_ptr->name();
			std::string glyph_coord_str = "glyphuv";
			std::string func_parameters_str = "";

			const std::vector<int>& attrib_indices = gam.ref_attrib_indices();
			const std::vector<vec4>& attrib_values = gam.ref_attrib_values();

			std::vector<std::string> func_parameters_strs;

			for(size_t j = 0; j < attrib_indices.size(); ++j) {
				std::string parameter_str = "";

				if(attrib_indices[j] < 0) {
					// constant non-mapped parameter
					parameter_str = constant_uniform_name_prefix + std::to_string(constant_glyph_parameters.size());
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

			code += ");";
		}
	}

	std::string uniforms_str = "";

	for(size_t i = 0; i < constant_glyph_parameters.size(); ++i)
		uniforms_str += "uniform float " + constant_glyph_parameters[i].first + ";";

	uniform_block = uniforms_str;
	glyph_block = code;
	uniform_value_ptrs = std::move(constant_glyph_parameters);
	
	//std::cout << std::endl << std::endl << uniforms_str << std::endl;
	//std::cout << code << std::endl << std::endl;
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
