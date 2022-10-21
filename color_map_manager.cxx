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
	std::vector<uint8_t> data(2 * 3 * res, 0u);
	return create_or_replace_texture(ctx, res, 2u, data);
}

void color_map_manager::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	base_ptr = bp;

	p.add_member_control(bp, "Name", new_name);
	connect_copy(p.add_button("Add Color Map")->click, cgv::signal::rebind(this, &color_map_manager::create_color_map));
	for(size_t i = 0; i < color_maps.size(); ++i) {
		color_map_container& cmc = color_maps[i];
		p.add_member_control(bp, "", cmc.name, "string", "w=120", " ");
		std::string active = cmc.custom ? "true" : "false";
		connect_copy(p.add_button("@1edit", "w=20;active=" + active, " ")->click, cgv::signal::rebind(this, &color_map_manager::edit_color_map, cgv::signal::_c<size_t>(i)));
		connect_copy(p.add_button("@9+", "w=20")->click, cgv::signal::rebind(this, &color_map_manager::remove_color_map, cgv::signal::_c<size_t>(i)));
	}
}

ActionType color_map_manager::action_type() {
	ActionType temp = last_action_type;
	last_action_type = AT_NONE;
	return temp;
}

std::vector<std::string> color_map_manager::get_names() {
	std::vector<std::string> names;
	for(size_t i = 0; i < color_maps.size(); ++i)
		names.push_back(color_maps[i].name);
	return names;
}

void color_map_manager::add_color_map(const std::string& name, const cgv::render::color_map& cm, bool custom) {
	color_map_container cmc(name);
	cmc.cm = cm;
	cmc.custom = custom;
	color_maps.push_back(cmc);
}

void color_map_manager::remove_color_map_by_name(const std::string& name) {
	int index = -1;
	for(size_t i = 0; i < color_maps.size(); ++i) {
		if(color_maps[i].name == name) {
			index = (int)i;
			break;
		}
	}

	if(index > -1)
		remove_color_map(static_cast<size_t>(index));
}

bool color_map_manager::update_texture(cgv::render::context& ctx) {
	if(color_maps.size() == 0)
		return false;

	std::vector<uint8_t> data(3 * color_maps.size() * res);

	/*const auto step = [](float edge, float x) {
		if(x < edge) return 0.0f;
		return 1.0f;
	};

	const auto rgb2hsv = [step](rgb c) {
		vec4 K = vec4(0.0f, -1.0f / 3.0f, 2.0f / 3.0f, -1.0f);
		vec4 p = cgv::math::lerp(vec4(c.B(), c.G(), K.w(), K.z()), vec4(c.G(), c.B(), K.x(), K.y()), step(c.B(), c.G()));
		vec4 q = cgv::math::lerp(vec4(p.x(), p.y(), p.w(), c.R()), vec4(c.R(), p.y(), p.z(), p.x()), step(p.x(), c.R()));

		float d = q.x() - std::min(q.w(), q.y());
		float e = 1.0e-10;
		return vec3(abs(q.z() + (q.w() - q.y()) / (6.0f * d + e)), d / (q.x() + e), q.x());
	};*/

	size_t base_idx = 0;
	for(size_t i = 0; i < color_maps.size(); ++i) {
		std::vector<rgb> cm_data = color_maps[i].cm.interpolate_color(static_cast<size_t>(res));

		for(size_t j = 0; j < res; ++j) {
			rgb color = cm_data[j];
			/*
			vec3 hsv = rgb2hsv(cm_data[j]);
			data[base_idx + 0] = static_cast<uint8_t>(255.0f * hsv.x());
			data[base_idx + 1] = static_cast<uint8_t>(255.0f * hsv.y());
			data[base_idx + 2] = static_cast<uint8_t>(255.0f * hsv.z());
			*/
			data[base_idx + 0] = static_cast<uint8_t>(255.0f * color.R());
			data[base_idx + 1] = static_cast<uint8_t>(255.0f * color.G());
			data[base_idx + 2] = static_cast<uint8_t>(255.0f * color.B());
			base_idx += 3;
		}
	}

	return create_or_replace_texture(ctx, res, static_cast<unsigned>(color_maps.size()), data);
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
	gam.custom = true;
	gam.cm.add_color_point(0.0f, rgb(0.0f));

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
		edit_idx = (int)index;
		base_ptr->on_set(this);
	}
}

bool color_map_manager::create_or_replace_texture(cgv::render::context& ctx, unsigned w, unsigned h, std::vector<uint8_t>& data) {
	cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res, h, TI_UINT8, cgv::data::CF_RGB), data.data());

	unsigned width = tex.get_width();
	unsigned height = tex.get_height();

	if(tex.is_created() && width == res && height == color_maps.size()) {
		return tex.replace(ctx, 0, 0, dv);
	} else {
		tex.destruct(ctx);
		tex = cgv::render::texture("uint8[R,G,B]", cgv::render::TF_LINEAR, cgv::render::TF_LINEAR, cgv::render::TW_CLAMP_TO_EDGE, cgv::render::TW_CLAMP_TO_EDGE);
		tex.set_border_color(0.0f, 0.0f, 0.0f, 1.0f);
		return tex.create(ctx, dv, 0);
	}
}
