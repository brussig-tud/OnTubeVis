#pragma once

#include <cgv/base/base.h>
#include <cgv/data/ref_ptr.h>
#include <cgv/gui/control.h>
#include <cgv/gui/provider.h>

#include "glyph_shapes.h"



class glyph_attribute_mapping : public cgv::render::render_types {
protected:
	ActionType last_action_type = AT_NONE;

	std::vector<std::string> attribute_names;
	std::vector<vec2> attribute_ranges;
	std::vector<std::string> color_map_names;

	GlyphType type = GT_CIRCLE;
	glyph_shape* shape_ptr = nullptr;

	std::vector<cgv::type::DummyEnum> attrib_source_indices;
	std::vector<cgv::type::DummyEnum> color_source_indices;
	std::vector<vec4> attrib_mapping_values;
	std::vector<rgb> attrib_colors;
	
	void on_set(void* member_ptr, cgv::base::base* base_ptr);

	void create_glyph_shape();

	template <typename T>
	cgv::data::ref_ptr<cgv::gui::control<T>> add_local_member_control(cgv::gui::provider& p, cgv::base::base* base_ptr, const std::string& label, T& value, const std::string& gui_type = "", const std::string& options = "", const std::string& align = "\n") {
		cgv::data::ref_ptr<cgv::gui::control<T>> cp = p.add_control(label, value, gui_type, options, align);
		if(cp)
			connect_copy(cp->value_change, cgv::signal::rebind(this, &glyph_attribute_mapping::on_set, &value, cgv::signal::_c<cgv::base::base*>(base_ptr)));
		return cp;
	}

	int dummy_enum_to_int(cgv::type::DummyEnum index) const;

	std::string to_display_str(const std::string& name) const;

	void create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i);

public:
	glyph_attribute_mapping();

	glyph_attribute_mapping(const glyph_attribute_mapping& r);

	glyph_attribute_mapping& operator=(const glyph_attribute_mapping& r);

	~glyph_attribute_mapping();

	ActionType action_type();

	const glyph_shape* get_shape_ptr() const { return shape_ptr; }

	const std::vector<int> get_attrib_indices() const;

	const std::vector<int> get_color_map_indices() const;
	
	const std::vector<vec4>& ref_attrib_values() const { return attrib_mapping_values; }

	const std::vector<rgb>& ref_attrib_colors() const { return attrib_colors; }

	const std::vector<std::string>& ref_attribute_names() const { return attribute_names; }
	
	const std::vector<std::string>& ref_color_map_names() const { return color_map_names; }

	void set_attribute_names(const std::vector<std::string>& names);

	void set_attribute_ranges(const std::vector<vec2>& ranges);

	void set_color_map_names(const std::vector<std::string>& names);

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
