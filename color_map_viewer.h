#pragma once

#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/texture.h>
#include <cgv_glutil/frame_buffer_container.h>
#include <cgv_glutil/overlay.h>
#include <cgv_glutil/color_map.h>
#include <cgv_glutil/2d/canvas.h>
#include <cgv_glutil/2d/shape2d_styles.h>

class color_map_viewer : public cgv::glutil::overlay {
protected:
	int last_theme_idx = -1;
	cgv::glutil::frame_buffer_container fbc;

	cgv::glutil::canvas canvas, overlay_canvas;
	cgv::glutil::shape2d_style container_style, border_style, color_map_style;
	
	struct layout_attributes {
		int padding;
		int band_height;
		int total_height;

		// dependent members
		cgv::glutil::rect color_map_rect;

		void update(const ivec2& parent_size) {
			color_map_rect.set_pos(ivec2(padding));
			color_map_rect.set_size(parent_size - 2 * padding);
		}
	} layout;
	
	texture* tex;

	void init_styles(context& ctx);

public:
	color_map_viewer();
	std::string get_type_name() const { return "color_map_viewer"; }

	void clear(cgv::render::context& ctx);

	bool self_reflect(cgv::reflect::reflection_handler& _rh);
	void stream_help(std::ostream& os) {}

	bool handle_event(cgv::gui::event& e);
	void on_set(void* member_ptr);

	bool init(cgv::render::context& ctx);
	void init_frame(cgv::render::context& ctx);
	void draw(cgv::render::context& ctx);
	
	void create_gui();
	void create_gui(cgv::gui::provider& p);

	void set_color_map_texture(texture* tex) {
		this->tex = tex;
		on_set(&layout.total_height);
	}
};
