#include "color_map_viewer.h"

#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/theme_info.h>
#include <cgv/math/ftransform.h>
#include <cgv_gl/gl/gl.h>


color_map_viewer::color_map_viewer() {

	set_name("Color Map Viewer");
	layout.padding = padding();
	set_size(cgv::ivec2(layout.preview_width + layout.label_width, layout.total_height));
	
	register_shader("rectangle", cgv::g2d::shaders::rectangle);
}

void color_map_viewer::clear(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx, -1);
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
		cgv::ivec2 size = get_rectangle().size;

		if(texture) {
			int h = static_cast<int>(texture->get_height());
			layout.total_height = 2 * layout.padding + h * layout.band_height + h - 1;
		}

		size.y() = layout.total_height;
		set_size(size);
	}

	update_member(member_ptr);
	post_damage();
}

bool color_map_viewer::init(cgv::render::context& ctx) {
	
	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx, 1);

	bool success = canvas_overlay::init(ctx);
	success &= texts.init(ctx);

	return success;
}

void color_map_viewer::init_frame(cgv::render::context& ctx) 
{
	if(ensure_layout(ctx)) {
		layout.update(get_rectangle().size);
		update_texts(ctx);
	}

	if(texts_out_of_date)
		update_texts(ctx);
}

void color_map_viewer::draw_content(cgv::render::context& ctx) {

	if(!texture)
		return;

	begin_content(ctx);
	
	// draw border around color map previews
	const auto& theme = cgv::gui::theme_info::instance();
	auto border_rectangle = layout.color_map_rect;
	border_rectangle.scale(1);

	content_canvas.enable_shader(ctx, "rectangle");
	content_canvas.set_style(ctx, solid_style);
	content_canvas.draw_shape(ctx, border_rectangle, theme.border());
	
	// draw color map previews
	auto color_map_rectangle = layout.color_map_rect;
	color_map_rectangle.y() = layout.color_map_rect.y1() - layout.band_height;
	color_map_rectangle.h() = layout.band_height;

	texture->enable(ctx, 0);
	
	const float u_step = 1.0f / static_cast<float>(names.size());
	float u_coord = 0.5f * u_step;

	for(const auto& name : names) {
		color_map_style.texcoord_offset = { 0.0f, u_coord };
		content_canvas.set_style(ctx, color_map_style);
		content_canvas.draw_shape(ctx, color_map_rectangle);

		color_map_rectangle.y() -= layout.band_height + 1;
		u_coord += u_step;
	}
	
	texture->disable(ctx);
	content_canvas.disable_current_shader(ctx);

	// draw color scale names
	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx).render(ctx, content_canvas, texts, text_style);

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

void color_map_viewer::set_color_map_texture(cgv::render::texture* texture) {

	this->texture = texture;
	on_set(&layout.total_height);
	post_damage();
}

void color_map_viewer::init_styles() {
	const auto& theme = cgv::gui::theme_info::instance();

	// configure style for solid_color rectangles
	solid_style.use_fill_color = false;
	solid_style.feather_width = 0.0f;
	
	// configure style for the color scale rectangle
	color_map_style = solid_style;
	color_map_style.use_texture = true;
	color_map_style.texcoord_scaling = { 1.0f, 0.0f };

	// configure text style
	text_style.use_blending = true;
	text_style.fill_color = theme.text();
	text_style.font_size = 14.0f;
}

void color_map_viewer::update_texts(cgv::render::context& ctx) {

	texts.clear();
	if(names.empty())
		return;

	texts.texts = names;

	cgv::vec3 position = cgv::vec3(static_cast<float>(layout.color_map_rect.x1() + layout.label_margin), static_cast<float>(layout.color_map_rect.y1() - layout.band_height), 0.0f);
	position.y() += 0.5f * static_cast<float>(layout.band_height);
	for(const auto& name : names) {
		texts.positions.push_back(position);
		position.y() -= static_cast<float>(layout.band_height) + 1.0f;
	}
	texts.alignment = cgv::render::TA_LEFT;
	texts.create(ctx);
	texts_out_of_date = false;
}
