#pragma once

#include <list>
#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>

#include "gui_util.h"
#include "glyph_attribute_mapping.h"



class glyph_layer_manager : public cgv::base::base, public cgv::render::render_types {
public:
	typedef std::pair<bool, float> parameter_pair;
	typedef std::vector<parameter_pair> parameter_list;

	struct configuration {
		static const std::string constant_float_parameter_name_prefix;
		static const std::string constant_color_parameter_name_prefix;
		static const std::string mapped_parameter_name_prefix;

		struct glyph_mapping_triple {
			int type; // 0 = constant, 1 = mapped
			size_t idx; // index into attribute values
			const vec4* v; // pointer to
		};

		struct layer_configuration {
			bool visible;
			AttributeSamplingStrategy sampling_strategy;
			float sampling_step;
			const glyph_shape* shape_ptr;
			std::vector<int> mapped_attributes;
			std::vector<glyph_mapping_triple> glyph_mapping_parameters;
			std::string glyph_definition = "";
		};

		struct shader_configuration {
			std::string uniforms_definition = "";
			std::vector<std::string> glyph_layer_definitions;

			void clear() {
				uniforms_definition = "";
				glyph_layer_definitions.clear();
			}
		};

		//
		std::vector<std::pair<std::string, const float*>> constant_float_parameters;
		std::vector<std::pair<std::string, const rgb*>> constant_color_parameters;
		std::vector<std::pair<std::string, const vec4*>> mapping_parameters;
		std::vector<layer_configuration> layer_configs;
		std::string uniforms_definition = "";

		void clear() {
			layer_configs.clear();
			constant_float_parameters.clear();
			constant_color_parameters.clear();
			mapping_parameters.clear();

			uniforms_definition = "";
		}

		void create_uniforms_definition(bool verbose = false) {
			std::string uniforms_str = "";

			if(constant_float_parameters.size() > 0)
				uniforms_str += "uniform float " + constant_float_parameter_name_prefix + "[" + std::to_string(constant_float_parameters.size()) + "];";

			if(constant_color_parameters.size() > 0)
				uniforms_str += "uniform vec3 " + constant_color_parameter_name_prefix + "[" + std::to_string(constant_color_parameters.size()) + "];";

			if(mapping_parameters.size() > 0)
				uniforms_str += "uniform vec4 " + mapped_parameter_name_prefix + "[" + std::to_string(mapping_parameters.size()) + "];";

			uniforms_definition = uniforms_str;
		}
	};

protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	ActionType last_action_type = AT_NONE;

	/*struct layer_settings {
		AttributeSamplingStrategy sampling_strategy;
		float sample_step = 1.0f;
	};

	struct layer{
		layer_settings settings;
		glyph_attribute_mapping gam;
	};
	*/
	// wrap bool to prevent vector specialization
	struct Bool {
		bool v;
		Bool(bool v) : v(v) {}
	};
	std::vector<Bool> visible;
	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	std::vector<std::string> attribute_names;
	std::vector<vec2> attribute_ranges;
	std::vector<std::string> color_map_names;

	configuration config;

	void on_set(void* member_ptr);

	void create_glyph_attribute_mapping();

	void remove_glyph_attribute_mapping(const size_t index);

	void move_glyph_attribute_mapping(const size_t index, int offset);

public:
	glyph_layer_manager() {
		base_ptr = nullptr;
	}

	~glyph_layer_manager() {}

	void clear();

	const std::vector<glyph_attribute_mapping>& ref_glyph_attribute_mappings() {
		return glyph_attribute_mappings;
	}

	//const std::vector<layer>& ref_layers() {
	//	return layers;
	//}

	void set_attribute_names(const std::vector<std::string>& names);

	void set_attribute_ranges(const std::vector<vec2>& ranges);

	void set_color_map_names(const std::vector<std::string>& names);
	
	const configuration& get_configuration();

	ActionType action_type();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);

	void notify_configuration_change() {
		last_action_type = AT_CONFIGURATION_CHANGE;
		if(base_ptr)
			base_ptr->on_set(this);
	}

	void add_glyph_attribute_mapping(const glyph_attribute_mapping& attribute_mapping) {
		if(glyph_attribute_mappings.size() == 4) {
			std::cout << "Cannot use more than 4 layers" << std::endl;
			return;
		}

		glyph_attribute_mappings.push_back(attribute_mapping);
		auto& gam = glyph_attribute_mappings.back();
		gam.set_attribute_names(attribute_names);
		gam.set_attribute_ranges(attribute_ranges);
		gam.set_color_map_names(color_map_names);
		
		visible.push_back(true);
	}

	/*void add_layer(const layer_settings& settings, const glyph_attribute_mapping& attribute_mapping) {
		if(layers.size() == 4) {
			std::cout << "Cannot use more than 4 layers" << std::endl;
			return;
		}

		layers.push_back({ settings, attribute_mapping });
		auto& gam = layers.back().gam;
		gam.set_attribute_names(attribute_names);
		gam.set_attribute_ranges(attribute_ranges);
		gam.set_color_map_names(color_map_names);
	}*/
};
