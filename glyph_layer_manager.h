#pragma once

#include <cgv/base/base.h>
//#include <cgv/data/ref_ptr.h>
//#include <cgv/gui/control.h>
#include <cgv/gui/provider.h>
#include <vector>

#include "glyph_attribute_mapping.h"



class glyph_layer_manager : public cgv::base::base {
protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	bool request_gui_redraw = false;

	// TODO: use a cgv::signal::managed_list for this?
	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	void on_set(void* member_ptr);

	void create_glyph_attribute_mapping() {
		glyph_attribute_mappings.push_back(glyph_attribute_mapping());
		request_gui_redraw = true;

		if(base_ptr)
			base_ptr->on_set(this);
	}

	void remove_glyph_attribute_mapping(const size_t index) {
		if(index < glyph_attribute_mappings.size()) {
			glyph_attribute_mappings.erase(glyph_attribute_mappings.begin() + index);
			request_gui_redraw = true;

			if(base_ptr)
				base_ptr->on_set(this);
		}
	}

public:
	glyph_layer_manager() {
		base_ptr = nullptr;
		glyph_attribute_mappings.push_back(glyph_attribute_mapping());
		glyph_attribute_mappings.push_back(glyph_attribute_mapping());
	}

	~glyph_layer_manager() {}

	bool gui_redraw_requested();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
