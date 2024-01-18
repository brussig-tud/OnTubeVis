#pragma once

#include <cgv_app/canvas_overlay.h>
#include <cgv_g2d/msdf_gl_canvas_font_renderer.h>
#include <cgv_g2d/canvas.h>
#include <cgv_g2d/shape2d_styles.h>

class mapping_legend : public cgv::app::canvas_overlay {
protected:
	struct layout_attributes {
		int padding;
		int total_height;

		// dependent members
		cgv::g2d::trect<int> color_map_rect;

		void update(const ivec2& parent_size) {
			color_map_rect.position = ivec2(padding);
			color_map_rect.size = parent_size - 2 * padding;
		}
	} layout;

	
	// geometry
	cgv::g2d::msdf_text_geometry text;
	bool texts_out_of_date = false;

	// appearance
	cgv::g2d::shape2d_style container_style;// , border_style, color_map_style;
	cgv::g2d::text2d_style text_style;
	
	void update_texts();

	void init_styles() override;
	void create_gui_impl() override;

public:
	mapping_legend();
	std::string get_type_name() const { return "mapping_legend"; }

	bool init(cgv::render::context& ctx) override;
	void clear(cgv::render::context& ctx) override;

	void handle_member_change(const cgv::utils::pointer_test& m) override;

	void init_frame(cgv::render::context& ctx) override;
	void draw_content(cgv::render::context& ctx) override;
	
	// TODO: set attrib mapping info
};
