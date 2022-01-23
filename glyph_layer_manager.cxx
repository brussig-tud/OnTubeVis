#include "glyph_layer_manager.h"

bool glyph_layer_manager::gui_redraw_requested() {
	bool temp = request_gui_redraw;
	request_gui_redraw = false;
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
			gam.create_gui(this, p); // TODO: use this for base ptr
			p.align("\b");
			p.end_tree_node(gam);
		}
	}
}

void glyph_layer_manager::on_set(void* member_ptr) {

	for(size_t i = 0; i < glyph_attribute_mappings.size(); ++i) {
		glyph_attribute_mapping& gam = glyph_attribute_mappings[i];
		if(member_ptr == &gam) {
			if(gam.gui_redraw_requested())
				request_gui_redraw = true;
		}
	}

	if(base_ptr)
		base_ptr->on_set(this);
}
