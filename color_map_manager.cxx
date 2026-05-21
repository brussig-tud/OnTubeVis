#include "color_map_manager.h"

void color_map_manager::clear() {
	color_maps.clear();
}

bool color_map_manager::destruct(cgv::render::context& ctx) {
	if(tex.is_created())
		return tex.destruct(ctx);
	return true;
}

bool color_map_manager::init(cgv::render::context& ctx) {
	std::vector<cgv::rgb8> data(discretization_resolution, { 0 });
	return create_or_replace_texture(ctx, discretization_resolution, 1, data);
}

void color_map_manager::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	base_ptr = bp;

	for(size_t i = 0; i < color_maps.size(); ++i) {
		entry_type& color_map = color_maps[i];
		p.add_view("", color_map.name, "string", color_map.user_defined ? "w=136" : "", color_map.user_defined ? " " : "\n");
		if(color_map.user_defined) {
			const std::string button_options = "w=20";
			connect_copy(p.add_button("@1edit", button_options, " ")->click, cgv::signal::rebind(this, &color_map_manager::edit_color_map, cgv::signal::_c<size_t>(i)));
			connect_copy(p.add_button("@9+", button_options)->click, cgv::signal::rebind(this, &color_map_manager::remove_color_map, cgv::signal::_c<size_t>(i)));
		}
	}

	p.add_member_control(bp, "Name", new_name);
	connect_copy(p.add_button("Add Color Map")->click, cgv::signal::rebind(this, &color_map_manager::create_color_map));
}

ActionType color_map_manager::action_type() {
	ActionType temp = last_action_type;
	last_action_type = ActionType::kUndefined;
	return temp;
}

cgv::media::transfer_function* color_map_manager::get_edited_color_ramp() {
	if(edit_idx > -1 && static_cast<size_t>(edit_idx) < color_maps.size())
		return &color_maps[static_cast<size_t>(edit_idx)].ramp;
	return nullptr;
}

std::vector<std::string> color_map_manager::get_names() {
	std::vector<std::string> names;
	for(size_t i = 0; i < color_maps.size(); ++i)
		names.push_back(color_maps[i].name);
	return names;
}

void color_map_manager::add_color_map(const std::string& name, const cgv::media::transfer_function& color_ramp, bool user_defined) {
	entry_type color_map;
	color_map.name = name;
	color_map.ramp = color_ramp;
	color_map.user_defined = user_defined;
	color_maps.push_back(color_map);
}

void color_map_manager::remove_color_map_by_name(const std::string& name) {
	for(size_t i = 0; i < color_maps.size(); ++i) {
		if(color_maps[i].name == name) {
			remove_color_map(i);
			break;
		}
	}
}

bool color_map_manager::update_texture(cgv::render::context& ctx) {
	if(color_maps.size() == 0)
		return false;

	std::vector<cgv::rgb8> data;
	data.reserve(discretization_resolution * color_maps.size());

	for(const auto& color_map : color_maps) {
		size_t offset = data.size();
		data.resize(data.size() + discretization_resolution);
		std::vector<cgv::rgb> colors = color_map.ramp.quantize_color(discretization_resolution);
		std::transform(colors.begin(), colors.end(), data.begin() + offset, [](const cgv::rgba& color) {
			return cgv::rgb8(color);
		});
	}

	return create_or_replace_texture(ctx, discretization_resolution, color_maps.size(), data);
}

void color_map_manager::on_set(void* member_ptr) {
	if(base_ptr)
		base_ptr->on_set(this);
}

void color_map_manager::create_color_map() {
	if(new_name == "")
		return;

	for(size_t i = 0; i < color_maps.size(); ++i) {
		if(color_maps[i].name == new_name)
			return;
	}

	entry_type color_map;
	color_map.name = new_name;
	color_map.user_defined = true;
	color_map.ramp.add_color_point(0.0f, cgv::rgb(0.0f));
	color_map.ramp.add_color_point(1.0f, cgv::rgb(1.0f));
	color_maps.push_back(color_map);

	new_name.clear();

	last_action_type = ActionType::kConfigurationChange;
	if(base_ptr) {
		auto provider = dynamic_cast<cgv::gui::provider*>(&(*base_ptr));
		if(provider)
			provider->update_member(&new_name);
		base_ptr->on_set(this);
	}
}

void color_map_manager::remove_color_map(const size_t index) {
	if(index < color_maps.size()) {
		color_maps.erase(color_maps.begin() + index);

		last_action_type = ActionType::kConfigurationChange;
		if(base_ptr)
			base_ptr->on_set(this);
	}
}

void color_map_manager::edit_color_map(const size_t index) {
	if(base_ptr) {
		last_action_type = ActionType::kEditRequest;
		if(edit_idx == index)
			edit_idx = -1;
		else
			edit_idx = static_cast<int>(index);
		base_ptr->on_set(this);
	}
}

bool color_map_manager::create_or_replace_texture(cgv::render::context& ctx, size_t width, size_t height, std::vector<cgv::rgb8>& data) {
	cgv::data::data_format df(discretization_resolution, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGB);
	cgv::data::data_view dv(&df, data.data());
	return tex.create(ctx, dv, 0);
}
