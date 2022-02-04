#pragma once

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <vector>

#include "glyph_attribute_mapping.h"



class glyph_layer_manager : public cgv::base::base, public cgv::render::render_types {
public:
	typedef std::pair<bool, float> parameter_pair;
	typedef std::vector<parameter_pair> parameter_list;

	struct layer_configuration {
		static const std::string constant_float_parameter_name_prefix;
		static const std::string constant_color_parameter_name_prefix;
		static const std::string mapped_parameter_name_prefix;

		std::vector<const glyph_shape*> shapes;
		std::vector<int> glyph_mapping_sources; // 0 for constant attrib, 1 for mapped attrib
		std::vector<std::pair<std::string, const float*>> constant_float_parameters;
		std::vector<std::pair<std::string, const rgb*>> constant_color_parameters;
		std::vector<std::pair<std::string, const vec4*>> mapping_parameters;
		std::vector<int> mapped_attributes;

		struct shader_configuration {
			unsigned mapped_attrib_count = 0u;
			std::string uniforms_definition = "";
			std::string attribute_buffer_definition = "";
			std::string glyph_layers_definition = "";

			void clear() {
				mapped_attrib_count = 0u;
				uniforms_definition = "";
				attribute_buffer_definition = "";
				glyph_layers_definition = "";
			}
		} shader_config;

		void clear() {
			shapes.clear();
			glyph_mapping_sources.clear();
			constant_float_parameters.clear();
			constant_color_parameters.clear();
			mapping_parameters.clear();
			mapped_attributes.clear();

			shader_config.clear();
		}

		void create_shader_config(const std::string& glyph_layers_definition, bool verbose = false) {
			std::string uniforms_str = "";
			std::string glyph_attrib_block = "";

			if(constant_float_parameters.size() > 0)
				uniforms_str += "uniform float " + constant_float_parameter_name_prefix + "[" + std::to_string(constant_float_parameters.size()) + "];";

			if(constant_color_parameters.size() > 0)
				uniforms_str += "uniform vec3 " + constant_color_parameter_name_prefix + "[" + std::to_string(constant_color_parameters.size()) + "];";

			if(mapping_parameters.size() > 0) {
				uniforms_str += "uniform vec4 " + mapped_parameter_name_prefix + "[" + std::to_string(mapping_parameters.size()) + "];";
				glyph_attrib_block = "float v[" + std::to_string(mapping_parameters.size()) + "];";
			}

			// save shader configuration
			shader_config.mapped_attrib_count = mapping_parameters.size();
			shader_config.uniforms_definition = uniforms_str;
			shader_config.glyph_layers_definition = glyph_layers_definition;
			shader_config.attribute_buffer_definition = glyph_attrib_block;

			std::cout << std::endl << ">>> SHADER DEFINES <<<" << std::endl;
			std::cout << uniforms_str << std::endl;
			std::cout << glyph_attrib_block << std::endl;
			std::cout << glyph_layers_definition << std::endl;
			std::cout << ">>> ============== <<<" << std::endl << std::endl;
		}
	};

protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	ActionType last_action_type = AT_NONE;

	// TODO: use a cgv::signal::managed_list for this?
	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	std::vector<std::string> attribute_names;
	std::vector<vec2> attribute_ranges;

	layer_configuration layer_config;

	void on_set(void* member_ptr);

	void create_glyph_attribute_mapping();

	void remove_glyph_attribute_mapping(const size_t index);

public:
	glyph_layer_manager() {
		base_ptr = nullptr;
	}

	~glyph_layer_manager() {}

	void clear();

	void set_attribute_names(const std::vector<std::string>& names);

	void set_attribute_ranges(const std::vector<vec2>& ranges);

	const layer_configuration& get_configuration();

	ActionType action_type();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
