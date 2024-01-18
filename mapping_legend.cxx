#include "mapping_legend.h"

#include <cgv/gui/theme_info.h>
#include <cgv_gl/gl/gl.h>

mapping_legend::mapping_legend() {

	set_name("Mapping Legend");

	layout.padding = 13; // 10px plus 3px border
	layout.total_height = 80;

	set_overlay_alignment(AO_START, AO_START);
	set_overlay_stretch(SO_NONE);
	set_overlay_margin(ivec2(-3));
	set_overlay_size(ivec2(200u, layout.total_height));
}

bool mapping_legend::init(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx, 1);

	register_shader("rectangle", cgv::g2d::shaders::rectangle);

	bool success = canvas_overlay::init(ctx);
	success &= text.init(ctx);

	return success;
}

void mapping_legend::clear(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx, -1);
	canvas_overlay::clear(ctx);
	
	text.destruct(ctx);
}

void mapping_legend::handle_member_change(const cgv::utils::pointer_test& m) {

	/*
	if(m.one_of(layout.total_height, layout.band_height)) {
		ivec2 size = get_overlay_size();

		if(tex) {
			int h = static_cast<int>(tex->get_height());
			layout.total_height = 2 * layout.padding + h * layout.band_height;
		}

		size.y() = layout.total_height;
		set_overlay_size(size);
	}*/
}

void mapping_legend::init_frame(cgv::render::context& ctx)
{
	if(ensure_layout(ctx)) {
		ivec2 container_size = get_overlay_size();
		layout.update(container_size);

		update_texts();
	}

	if(texts_out_of_date)
		update_texts();
}

void mapping_legend::draw_content(cgv::render::context& ctx) {

	begin_content(ctx);
	enable_blending();
	
	ivec2 container_size = get_overlay_size();
	
	// draw container
	content_canvas.enable_shader(ctx, "rectangle");
	content_canvas.set_style(ctx, container_style);
	content_canvas.draw_shape(ctx, ivec2(0), container_size);
	
	// draw inner border
	/*border_style.apply(ctx, rect_prog);
	content_canvas.draw_shape(ctx, ivec2(layout.padding - 1), container_size - 2*layout.padding + 2);
	content_canvas.disable_current_shader(ctx);
	
	// draw color scale texture
	auto& color_maps_prog = content_canvas.enable_shader(ctx, "color_maps");
	color_map_style.apply(ctx, color_maps_prog);
	tex->enable(ctx, 0);
	content_canvas.draw_shape(ctx, layout.color_map_rect);
	tex->disable(ctx);
	content_canvas.disable_current_shader(ctx);
	*/
	
	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx).render(ctx, content_canvas, text, text_style);

	disable_blending();
	end_content(ctx);
}

void mapping_legend::update_texts() {

	text.clear();

	text.add_text("Test", vec2(20.0f, 20.0f), cgv::render::TA_BOTTOM_LEFT);

	/*if(names.size() == 0)
		return;

	int step = layout.color_map_rect.h() / (int)names.size();
	ivec2 base = layout.color_map_rect.position + ivec2(layout.color_map_rect.w() / 2, step / 2 - static_cast<int>(0.333f * text_style.font_size));
	int i = 0;
	for(const auto& name : names) {
		ivec2 p = base;
		p.y() += (static_cast<int>(names.size()) - i - 1) * step - 1;
		texts.add_text(name, p, cgv::render::TA_BOTTOM);
		++i;
	}*/

	texts_out_of_date = false;
}

void mapping_legend::init_styles() {
	// get theme colors
	auto& ti = cgv::gui::theme_info::instance();
	rgba background_color = rgba(ti.background(), 1.0f);
	rgba group_color = rgba(ti.group(), 1.0f);
	rgba border_color = rgba(ti.border(), 1.0f);

	// configure style for the container rectangle
	container_style.fill_color = ti.group();
	container_style.border_color = ti.background();
	container_style.border_width = 3.0f;
	container_style.feather_width = 0.0f;
	
	// configure style for the border rectangles
	//border_style = container_style;
	//border_style.fill_color = border_color;
	//border_style.border_width = 0.0f;
	
	// configure style for the color scale rectangle
	//color_map_style = border_style;
	//color_map_style.use_texture = true;

	// configure text style
	text_style = cgv::g2d::text2d_style::preset_default(ti.text());
	text_style.font_size = 12.0f;
}

void mapping_legend::create_gui_impl() {

	//add_member_control(this, "Band Height", layout.band_height, "value_slider", "min=5;max=50;step=5;ticks=true");
}
