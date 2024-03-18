#include "color_map_viewer.h"

#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/theme_info.h>
#include <cgv/math/ftransform.h>
#include <cgv_gl/gl/gl.h>


color_map_viewer::color_map_viewer() {

	set_name("Color Map Viewer");

	//blend_overlay = true;

	layout.padding = padding();
	layout.band_height = 18;
	layout.total_height = 80;

	set_size(ivec2(200u, layout.total_height));
	
	register_shader("rectangle", cgv::g2d::shaders::rectangle);
	register_shader("color_maps", "color_maps.glpr");
	
	tex = nullptr;
}

void color_map_viewer::clear(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx, -1);
	canvas_overlay::clear(ctx);
	
	texts.destruct(ctx);
}

bool color_map_viewer::self_reflect(cgv::reflect::reflection_handler& _rh) {

	return false;
}

bool color_map_viewer::handle_event(cgv::gui::event& e) {

	return false;
}

void color_map_viewer::on_set(void* member_ptr) {

	if(member_ptr == &layout.total_height || member_ptr == &layout.band_height) {
		ivec2 size = get_rectangle().size;

		if(tex) {
			int h = static_cast<int>(tex->get_height());
			layout.total_height = 2 * layout.padding + h * layout.band_height;
		}

		size.y() = layout.total_height;
		set_size(size);
	}

	update_member(member_ptr);
	post_damage();
}

bool color_map_viewer::init(cgv::render::context& ctx) {
	
	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx, 1);

	bool success = canvas_overlay::init(ctx);
	success &= texts.init(ctx);

	return success;
}

void color_map_viewer::init_frame(cgv::render::context& ctx) 
{
	if(ensure_layout(ctx)) {
		layout.update(get_rectangle().size);

		update_texts();
	}

	if(texts_out_of_date)
		update_texts();
}

void color_map_viewer::draw_content(cgv::render::context& ctx) {

	if(!tex)
		return;

	begin_content(ctx);
	//enable_blending();
	
	// draw inner border
	auto rectangle = get_content_rectangle();
	rectangle.scale(1);

	content_canvas.enable_shader(ctx, "rectangle");
	content_canvas.set_style(ctx, border_style);
	content_canvas.draw_shape(ctx, rectangle);
	content_canvas.disable_current_shader(ctx);
	
	// draw color scale texture
	auto& color_maps_prog = content_canvas.enable_shader(ctx, "color_maps");
	color_map_style.apply(ctx, color_maps_prog);
	tex->enable(ctx, 0);
	content_canvas.draw_shape(ctx, layout.color_map_rect);
	tex->disable(ctx);
	content_canvas.disable_current_shader(ctx);
	
	// draw color scale names
	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx).render(ctx, content_canvas, texts, text_style);

	//disable_blending();
	end_content(ctx);
}

void color_map_viewer::create_gui_impl() {

	add_member_control(this, "Band Height", layout.band_height, "value_slider", "min=5;max=50;step=5;ticks=true");
}

void color_map_viewer::set_color_map_names(const std::vector<std::string>& names) {

	this->names = names;
	texts_out_of_date = true;
	post_damage();
}

void color_map_viewer::set_color_map_texture(cgv::render::texture* tex) {

	this->tex = tex;
	on_set(&layout.total_height);
	post_damage();
}

void color_map_viewer::init_styles() {
	const auto& theme = cgv::gui::theme_info::instance();

	// configure style for the border rectangles
	border_style.fill_color = theme.border();
	border_style.border_width = 0.0f;
	border_style.feather_width = 0.0f;
	
	// configure style for the color scale rectangle
	color_map_style = border_style;
	color_map_style.use_texture = true;

	// configure text style
	text_style.use_blending = true;
	text_style.fill_color = rgb(1.0f);
	text_style.border_color = rgb(0.0f);
	text_style.border_radius = 0.5f;
	text_style.border_width = 0.75f;
	text_style.feather_origin = 0.35f;
	text_style.font_size = 16.0f;
}

void color_map_viewer::update_texts() {

	texts.clear();
	if(names.size() == 0)
		return;

	int step = layout.color_map_rect.h() / (int)names.size();
	ivec2 base = layout.color_map_rect.position + ivec2(layout.color_map_rect.w() / 2, step / 2 - static_cast<int>(0.333f*text_style.font_size));
	int i = 0;
	for(const auto& name : names) {
		ivec2 p = base;
		p.y() += (static_cast<int>(names.size()) - i - 1) * step - 1;
		texts.add_text(name, p, cgv::render::TA_BOTTOM);
		++i;
	}
	texts_out_of_date = false;
}
