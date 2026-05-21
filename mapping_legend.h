#pragma once

#include <cgv_g2d/msdf_gl_font_renderer.h>
#include <cgv_g2d/canvas.h>
#include <cgv_g2d/shape2d_styles.h>
#include <cgv_overlay/themed_canvas_overlay.h>

#include "glyph_layer_manager.h"
#include "traj_loader.h"


class mapping_legend : public cgv::overlay::themed_canvas_overlay {
protected:
	struct layer_info {
		struct line_info {
			std::string text;
			std::string range;
			bool has_color = false;
			cgv::rgb color = { 0.0f };

			bool empty() const { return text.empty() && !has_color; }
		};

		std::string title;
		std::vector<line_info> lines;

		size_t size() const { return lines.size(); }
	};

	std::vector<layer_info> layers;

	// geometry
	bool text_out_of_date = true;
	cgv::g2d::msdf_text_geometry text;
	std::vector<float> dividers;
	std::vector<std::pair<cgv::vec2, cgv::rgb>> color_boxes;

	// appearance
	cgv::g2d::shape2d_style border_style, color_box_style;
	cgv::g2d::text2d_style text_style;
	
	void create_geometry();

	void init_styles() override;

public:
	mapping_legend();
	std::string get_type_name() const override { return "mapping_legend"; }

	bool init(cgv::render::context& ctx) override;
	void clear(cgv::render::context& ctx) override;

	void init_frame(cgv::render::context& ctx) override;
	void draw_content(cgv::render::context& ctx) override;
	
	void update(const traj_dataset<float>& dataset, const glyph_layer_manager& glyph_manager);
};
