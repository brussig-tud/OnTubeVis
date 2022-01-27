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

	GlyphType type = GT_CIRCLE;
	glyph_shape* shape_ptr = nullptr;

	rgb color;
	std::vector<cgv::type::DummyEnum> attrib_source_indices;
	std::vector<vec4> attrib_mapping_values;

	void on_set(void* member_ptr, cgv::base::base* base_ptr);

	void create_glyph_shape();

	template <typename T>
	cgv::data::ref_ptr<cgv::gui::control<T>> add_local_member_control(cgv::gui::provider& p, cgv::base::base* base_ptr, const std::string& label, T& value, const std::string& gui_type = "", const std::string& options = "", const std::string& align = "\n") {
		cgv::data::ref_ptr<cgv::gui::control<T>> cp = p.add_control(label, value, gui_type, options, align);
		if(cp)
			connect_copy(cp->value_change, cgv::signal::rebind(this, &glyph_attribute_mapping::on_set, &value, cgv::signal::_c<cgv::base::base*>(base_ptr)));
		return cp;
	}

	int attribute_index_to_int(cgv::type::DummyEnum index) const;

	void create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i);

public:
	glyph_attribute_mapping();

	glyph_attribute_mapping(const glyph_attribute_mapping& r);

	glyph_attribute_mapping& operator=(const glyph_attribute_mapping& r);

	~glyph_attribute_mapping();

	const glyph_shape* get_shape_ptr() const { return shape_ptr; }

	const rgb& ref_color() const{ return color; }

	const std::vector<int> get_attrib_indices() const;
	
	const std::vector<vec4>& ref_attrib_values() const { return attrib_mapping_values; }

	ActionType action_type();

	void set_attribute_names(const std::vector<std::string>& names);

	void set_attribute_ranges(const std::vector<vec2>& ranges);

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
