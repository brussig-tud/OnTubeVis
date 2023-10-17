#pragma once

// make sure this is the first thing the compiler sees, while preventing warnings if
// it happened to already be defined by something else including this header
#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES 1
#endif
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

	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	std::vector<std::string> attribute_names;
	std::vector<vec2> attribute_ranges;
	std::vector<std::string> color_map_names;

	configuration config;

	std::string new_attribute_mapping_name = "";

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

	const std::vector<glyph_attribute_mapping>& ref_glyph_attribute_mappings() const {
		return glyph_attribute_mappings;
	}

	void set_attribute_names(const std::vector<std::string>& names);

	void set_attribute_ranges(const std::vector<vec2>& ranges);

	void set_color_map_names(const std::vector<std::string>& names);
	
	const configuration& get_configuration();

	ActionType action_type();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);

	void notify_configuration_change();

	void add_glyph_attribute_mapping(const glyph_attribute_mapping& attribute_mapping);
};
