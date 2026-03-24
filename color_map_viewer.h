#pragma once

#include <cgv/media/color_scale.h>
#include <cgv/render/texture.h>
#include <cgv_g2d/msdf_gl_font_renderer.h>
#include <cgv_g2d/canvas.h>
#include <cgv_g2d/shape2d_styles.h>
#include <cgv_overlay/themed_canvas_overlay.h>

class color_map_viewer : public cgv::overlay::themed_canvas_overlay {
protected:
	struct layout_attributes {
		int padding = 0;
		int band_height = 18;
		int preview_width = 200;
		int label_margin = 8;
		int label_width = 80;
		int total_height = 80;

		// dependent members
		cgv::g2d::trect<int> color_map_rect;

		void update(const cgv::ivec2& parent_size) {
			color_map_rect.position = cgv::ivec2(padding);
			color_map_rect.size = parent_size - 2 * padding;
			color_map_rect.w() -= label_width;
		}
	} layout;

	std::vector<std::string> names;
	bool texts_out_of_date = false;

	cgv::g2d::shape2d_style solid_style, color_map_style;
	cgv::render::texture* texture = nullptr;

	// text appearance
	cgv::g2d::text2d_style text_style;
	cgv::g2d::msdf_text_geometry texts;
	std::vector<float> text_positions;
	std::vector<cgv::render::TextAlignment> text_alignments;
	
	void init_styles();
	void update_texts(cgv::render::context& ctx);

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

	void set_color_map_texture(cgv::render::texture* texture);
};
