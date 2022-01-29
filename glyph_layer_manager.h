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
		std::vector<const glyph_shape*> shapes;
		// TODO: rename
		std::vector<int> glyph_mapping_sources; // 0 for constant attrib, 1 for mapped attrib
		std::vector<std::pair<std::string, const float*>> constant_parameters;
		std::vector<std::pair<std::string, const vec4*>> mapping_parameters;
		std::vector<std::pair<std::string, const rgb*>> constant_colors;
		std::vector<int> mapped_attributes;

		struct shader_configuration {
			std::string uniforms_definition = "";
			std::string attribute_buffer_definition = "";
			std::string glyph_layers_definition = "";
		} shader_config;
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
