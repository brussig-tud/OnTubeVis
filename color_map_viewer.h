#pragma once

#include <cgv/render/texture.h>
#include <cgv_glutil/canvas_overlay.h>
#include <cgv_glutil/color_map.h>
#include <cgv_glutil/msdf_gl_canvas_font_renderer.h>
#include <cgv_glutil/2d/canvas.h>
#include <cgv_glutil/2d/shape2d_styles.h>

class color_map_viewer : public cgv::glutil::canvas_overlay {
protected:
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

	bool texts_out_of_date = false;
	
	std::vector<std::string> names;

	cgv::glutil::shape2d_style container_style, border_style, color_map_style;
	texture* tex;

	// text appearance
	float font_size = 14.0f;

	cgv::glutil::shape2d_style text_style;
	cgv::glutil::msdf_text_geometry texts;
	
	void init_styles(context& ctx);
	void update_texts();

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
	
	void create_gui();

	void set_color_map_names(const std::vector<std::string>& names);

	void set_color_map_texture(texture* tex);
};
