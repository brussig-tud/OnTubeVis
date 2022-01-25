#pragma once

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <vector>

#include "glyph_attribute_mapping.h"



class glyph_layer_manager : public cgv::base::base, public cgv::render::render_types {
public:
	typedef std::pair<bool, float> parameter_pair;
	typedef std::vector<parameter_pair> parameter_list;

protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	ActionType last_action_type = AT_NONE;

	// TODO: use a cgv::signal::managed_list for this?
	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	std::vector<std::string> attribute_names;

	void on_set(void* member_ptr);

	void create_glyph_attribute_mapping();

	void remove_glyph_attribute_mapping(const size_t index);

	/*void add_parameter(std::string& code, size_t idx, const parameter_pair& parameter) const {
		if(parameter.first) {
			// mapped parameter
			code += "0.5"; // TODO: replace this with a call to the actual data
		} else {
			// constant parameter
			code += parameter.second;
		}
	}

	void fill_parameters(std::string& code, const parameter_list& parameters) const {
		for(size_t i = 0; i < parameters.size(); ++i) {
			add_parameter(code, i, parameters[i]);
			if(i < parameters.size() - 1)
				code += ", ";
		}
	}*/

	//const std::string create_shader_code(const parameter_list& parameters) const {
	//	
	//}

public:
	glyph_layer_manager() {
		base_ptr = nullptr;
		// TODO: remove later
		glyph_attribute_mappings.push_back(glyph_attribute_mapping());
	}

	~glyph_layer_manager() {}

	void clear();

	void set_attribute_names(const std::vector<std::string>& names);

	void generate_shader_code(std::string& uniform_block, std::string& glyph_block, std::vector<std::pair<std::string, const float*>>& uniform_value_ptrs, std::vector<int> attribute_source_indices) const;

	ActionType action_type();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
