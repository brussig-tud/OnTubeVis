#include "color_map_manager.h"

void color_map_manager::clear() {
	color_maps.clear();
}

ActionType color_map_manager::action_type() {
	ActionType temp = last_action_type;
	last_action_type = AT_NONE;
	return temp;
}

void color_map_manager::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	base_ptr = bp;

	p.add_member_control(bp, "Name", new_name);
	connect_copy(p.add_button("Add Color Map")->click, cgv::signal::rebind(this, &color_map_manager::create_color_map));
	for(size_t i = 0; i < color_maps.size(); ++i) {
		color_map_container& cmc = color_maps[i];
		p.add_member_control(bp, "", cmc.name, "string", "w=120", " ");
		connect_copy(p.add_button("Edit", "w=35", " ")->click, cgv::signal::rebind(this, &color_map_manager::edit_color_map, cgv::signal::_c<size_t>(i)));
		connect_copy(p.add_button("X", "w=20")->click, cgv::signal::rebind(this, &color_map_manager::remove_color_map, cgv::signal::_c<size_t>(i)));
	}
}

void color_map_manager::on_set(void* member_ptr) {
	if(base_ptr)
		base_ptr->on_set(this);
}

void color_map_manager::create_color_map() {
	if(new_name == "")
		return;
	bool found = false;
	for(size_t i = 0; i < color_maps.size(); ++i) {
		if(color_maps[i].name == new_name) {
			found = true;
			break;
		}
	}

	if(found)
		return;

	color_maps.push_back(color_map_container(new_name));
	auto& gam = color_maps.back();

	last_action_type = AT_CONFIGURATION_CHANGE;
	if(base_ptr)
		base_ptr->on_set(this);
}

void color_map_manager::remove_color_map(const size_t index) {
	if(index < color_maps.size()) {
		color_maps.erase(color_maps.begin() + index);
		
		last_action_type = AT_CONFIGURATION_CHANGE;
		if(base_ptr)
			base_ptr->on_set(this);
	}
}

void color_map_manager::edit_color_map(const size_t index) {
	if(base_ptr) {
		last_action_type = AT_EDIT_REQUEST;
		edit_idx = index;
		base_ptr->on_set(this);
	}
}
