#pragma once

#include <cgv/render/color_map.h>
#include <cgv/render/texture.h>
#include <cgv_app/themed_canvas_overlay.h>
#include <cgv_g2d/msdf_gl_font_renderer.h>
#include <cgv_g2d/canvas.h>
#include <cgv_g2d/shape2d_styles.h>

class color_map_viewer : public cgv::app::themed_canvas_overlay {
public:
	using ivec2 = cgv::ivec2;
	using rgb = cgv::rgb;
	using rgba = cgv::rgba;

protected:
	struct layout_attributes {
		int padding;
		int band_height;
		int total_height;

		// dependent members
		cgv::g2d::trect<int> color_map_rect;

		void update(const ivec2& parent_size) {
			color_map_rect.position = ivec2(padding);
			color_map_rect.size = parent_size - 2 * padding;
		}
	} layout;

	bool texts_out_of_date = false;
	
	std::vector<std::string> names;

	cgv::g2d::shape2d_style border_style, color_map_style;
	cgv::render::texture* tex;

	// text appearance
	cgv::g2d::text2d_style text_style;
	cgv::g2d::msdf_text_geometry texts;
	std::vector<std::string> text_labels;
	std::vector<float> text_positions;
	std::vector<cgv::render::TextAlignment> text_alignments;
	
	void init_styles();
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
