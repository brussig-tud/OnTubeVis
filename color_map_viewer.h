#pragma once

#include <cgv/render/color_map.h>
#include <cgv/render/texture.h>
#include <cgv_app/canvas_overlay.h>
#include <cgv_g2d/msdf_gl_canvas_font_renderer.h>
#include <cgv_g2d/canvas.h>
#include <cgv_g2d/shape2d_styles.h>

class color_map_viewer : public cgv::app::canvas_overlay {
protected:
	struct layout_attributes {
		int padding;
		int band_height;
		int total_height;

		// dependent members
		cgv::g2d::rect color_map_rect;

		void update(const ivec2& parent_size) {
			color_map_rect.set_pos(ivec2(padding));
			color_map_rect.set_size(parent_size - 2 * padding);
		}
	} layout;

	bool texts_out_of_date = false;
	
	std::vector<std::string> names;

	cgv::g2d::shape2d_style container_style, border_style, color_map_style;
	cgv::render::texture* tex;

	// text appearance
	float font_size = 14.0f;

	cgv::g2d::shape2d_style text_style;
	cgv::g2d::msdf_text_geometry texts;
	
	void init_styles(cgv::render::context& ctx);
	void update_texts();

	void create_gui_impl();

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
	void draw_content(cgv::render::context& ctx);
	
	void set_color_map_names(const std::vector<std::string>& names);

	void set_color_map_texture(cgv::render::texture* tex);
};
