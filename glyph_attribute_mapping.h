#pragma once

#include <cgv/base/base.h>
#include <cgv/data/ref_ptr.h>
#include <cgv/gui/control.h>
#include <cgv/gui/provider.h>

#include "glyph_shapes.h"



class glyph_attribute_mapping : public cgv::render::render_types {
protected:
	GlyphType type = GT_CIRCLE;
	glyph_shape* shape_ptr = nullptr;

	std::vector<int> attrib_source_indices;
	std::vector<vec4> attrib_mapping_values;

	bool request_gui_redraw = false;

	void create_glyph_shape();

	void on_set(void* member_ptr, cgv::base::base* base_ptr);

	template <typename T>
	cgv::data::ref_ptr<cgv::gui::control<T>> add_local_member_control(cgv::gui::provider& p, cgv::base::base* base_ptr, const std::string& label, T& value, const std::string& gui_type = "", const std::string& options = "", const std::string& align = "\n") {
		cgv::data::ref_ptr<cgv::gui::control<T>> cp = p.add_control(label, value, gui_type, options, align);
		if(cp)
			connect_copy(cp->value_change, cgv::signal::rebind(this, &glyph_attribute_mapping::on_set, &value, cgv::signal::_c<cgv::base::base*>(base_ptr)));
		return cp;
	}

public:
	glyph_attribute_mapping();

	glyph_attribute_mapping(const glyph_attribute_mapping& r);

	glyph_attribute_mapping& operator=(const glyph_attribute_mapping& r);

	~glyph_attribute_mapping();

	bool gui_redraw_requested();

	void create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i);

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
