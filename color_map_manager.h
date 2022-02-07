#pragma once

#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/render_types.h>
#include <cgv/render/texture.h>
#include <cgv_glutil/color_map.h>

#include "glyph_attribute_mapping.h"



class color_map_manager : public cgv::base::base, public cgv::render::render_types {
public:
	struct color_map_container {
		std::string name = "";
		cgv::glutil::color_map cm;

		color_map_container() {}
		color_map_container(const std::string& name) : name(name) {}
	};
protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	ActionType last_action_type = AT_NONE;
	int edit_idx = -1;

	std::string new_name = "";

	// TODO: use a cgv::signal::managed_list for this?
	std::vector<color_map_container> color_maps;
	cgv::render::texture tex;
	unsigned res = 256;

	void on_set(void* member_ptr);

	void create_color_map();

	void remove_color_map(const size_t index);

	void edit_color_map(const size_t index);

public:
	color_map_manager() {
		base_ptr = nullptr;
	}

	~color_map_manager() {}

	void clear();

	bool destruct(cgv::render::context& ctx) {
		if(tex.is_created())
			return tex.destruct(ctx);
		return true;
	}

	bool init(cgv::render::context& ctx) {
		std::vector<uint8_t> data(2 * 3 * res, 0u);

		tex.destruct(ctx);
		cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res, 2u, TI_UINT8, cgv::data::CF_RGB), data.data());
		tex = cgv::render::texture("uint8[R,G,B]", cgv::render::TF_LINEAR, cgv::render::TF_LINEAR, cgv::render::TW_CLAMP_TO_EDGE, cgv::render::TW_CLAMP_TO_EDGE);
		return tex.create(ctx, dv, 0);
	}

	ActionType action_type();

	int edit_index() { return edit_idx; }

	std::vector<color_map_container>& ref_color_maps() { return color_maps; }

	std::vector<std::string> get_names() {
		std::vector<std::string> names;
		for(size_t i = 0; i < color_maps.size(); ++i)
			names.push_back(color_maps[i].name);
		return names;
	}

	void update_texture(cgv::render::context& ctx) {
		if(color_maps.size() == 0)
			return;

		std::vector<uint8_t> data(3 * color_maps.size() * res);

		/*float step = 1.0f / static_cast<float>(res - 1);

		size_t base_idx = 0;
		for(size_t i = 0; i < color_maps.size(); ++i) {
			const auto& cm = color_maps[i].cm;
			for(size_t j = 0; j < res; ++j) {
				float t = j * step;
				rgb col = cm.interpolate_color(t);

				data[base_idx + 0] = static_cast<uint8_t>(255.0f * col.R());
				data[base_idx + 1] = static_cast<uint8_t>(255.0f * col.G());
				data[base_idx + 2] = static_cast<uint8_t>(255.0f * col.B());

				base_idx += 3;
			}
		}*/

		size_t base_idx = 0;
		for(size_t i = 0; i < color_maps.size(); ++i) {
			std::vector<rgb> cm_data = color_maps[i].cm.interpolate_color(static_cast<size_t>(res));

			for(size_t j = 0; j < res; ++j) {
				data[base_idx + 0] = static_cast<uint8_t>(255.0f * cm_data[j].R());
				data[base_idx + 1] = static_cast<uint8_t>(255.0f * cm_data[j].G());
				data[base_idx + 2] = static_cast<uint8_t>(255.0f * cm_data[j].B());
				base_idx += 3;
			}
		}

		cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res, color_maps.size(), TI_UINT8, cgv::data::CF_RGB), data.data());

		unsigned width = tex.get_width();
		unsigned height = tex.get_height();

		if(tex.is_created() && width == res && height == color_maps.size()) {
			tex.replace(ctx, 0, 0, dv);
		} else {
			tex.destruct(ctx);
			tex = cgv::render::texture("uint8[R,G,B]", cgv::render::TF_LINEAR, cgv::render::TF_LINEAR, cgv::render::TW_CLAMP_TO_EDGE, cgv::render::TW_CLAMP_TO_EDGE);
			tex.create(ctx, dv, 0);
		}
	}

	cgv::render::texture& ref_texture() { return tex; }

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
