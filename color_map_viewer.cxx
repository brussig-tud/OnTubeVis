#include "color_map_viewer.h"

#include <cgv/gui/key_event.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/theme_info.h>
#include <cgv/math/ftransform.h>
#include <cgv_gl/gl/gl.h>

color_map_viewer::color_map_viewer() {

	set_name("Color Map Viewer");

	layout.padding = 13; // 10px plus 3px border
	layout.band_height = 15;
	layout.total_height = 80;

	set_overlay_alignment(AO_END, AO_END);
	set_overlay_stretch(SO_NONE);
	set_overlay_margin(ivec2(-3));
	set_overlay_size(ivec2(200u, layout.total_height));
	
	fbc.add_attachment("color", "flt32[R,G,B,A]");
	fbc.set_size(get_overlay_size());

	canvas.register_shader("rectangle", cgv::glutil::canvas::shaders_2d::rectangle);
	canvas.register_shader("color_maps", "color_maps.glpr");
	
	overlay_canvas.register_shader("rectangle", cgv::glutil::canvas::shaders_2d::rectangle);

	tex = nullptr;
}

void color_map_viewer::clear(cgv::render::context& ctx) {

	msdf_font.destruct(ctx);
	font_renderer.destruct(ctx);

	canvas.destruct(ctx);
	overlay_canvas.destruct(ctx);
	fbc.clear(ctx);
}

bool color_map_viewer::self_reflect(cgv::reflect::reflection_handler& _rh) {

	return false;
}

bool color_map_viewer::handle_event(cgv::gui::event& e) {

	return false;
}

void color_map_viewer::on_set(void* member_ptr) {

	if(member_ptr == &layout.total_height || member_ptr == &layout.band_height) {
		ivec2 size = get_overlay_size();

		if(tex) {
			int h = static_cast<int>(tex->get_height());
			layout.total_height = 2 * layout.padding + h * layout.band_height;
		}

		size.y() = layout.total_height;
		set_overlay_size(size);
	}

	update_member(member_ptr);
	post_redraw();
}

bool color_map_viewer::init(cgv::render::context& ctx) {
	
	bool success = true;

	success &= fbc.ensure(ctx);
	success &= canvas.init(ctx);
	success &= overlay_canvas.init(ctx);

	success &= font_renderer.init(ctx);
	success &= msdf_font.init(ctx);
	texts.set_msdf_font(&msdf_font);
	texts.set_font_size(font_size);

	if(success)
		init_styles(ctx);

	return success;
}

void color_map_viewer::init_frame(cgv::render::context& ctx) {

	if(ensure_overlay_layout(ctx)) {
		ivec2 container_size = get_overlay_size();
		layout.update(container_size);

		update_texts();

		fbc.set_size(container_size);
		fbc.ensure(ctx);

		canvas.set_resolution(ctx, container_size);
		overlay_canvas.set_resolution(ctx, get_viewport_size());
	}

	int theme_idx = cgv::gui::theme_info::instance().get_theme_idx();
	if(last_theme_idx != theme_idx) {
		last_theme_idx = theme_idx;
		init_styles(ctx);
	}

	if(texts_out_of_date)
		update_texts();
}

void color_map_viewer::draw(cgv::render::context& ctx) {

	if(!show || !tex)
		return;

	fbc.enable(ctx);
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	
	ivec2 container_size = get_overlay_size();
	
	// draw container
	auto& rect_prog = canvas.enable_shader(ctx, "rectangle");
	container_style.apply(ctx, rect_prog);
	canvas.draw_shape(ctx, ivec2(0), container_size);
	
	// draw inner border
	border_style.apply(ctx, rect_prog);
	canvas.draw_shape(ctx, ivec2(layout.padding - 1), container_size - 2*layout.padding + 2);
	canvas.disable_current_shader(ctx);
	
	// draw color scale texture
	auto& color_maps_prog = canvas.enable_shader(ctx, "color_maps");
	color_map_style.apply(ctx, color_maps_prog);
	tex->enable(ctx, 0);
	canvas.draw_shape(ctx, layout.color_map_rect.pos(), layout.color_map_rect.size());
	tex->disable(ctx);
	canvas.disable_current_shader(ctx);
	
	// draw color scale names
	auto& font_prog = font_renderer.ref_prog();
	font_prog.enable(ctx);
	text_style.apply(ctx, font_prog);
	canvas.set_view(ctx, font_prog);
	font_prog.disable(ctx);
	font_renderer.render(ctx, get_overlay_size(), texts);

	glDisable(GL_BLEND);

	fbc.disable(ctx);

	// draw frame buffer texture to screen
	auto& final_prog = overlay_canvas.enable_shader(ctx, "rectangle");
	fbc.enable_attachment(ctx, "color", 0);
	overlay_canvas.draw_shape(ctx, get_overlay_position(), container_size);
	fbc.disable_attachment(ctx, "color");

	overlay_canvas.disable_current_shader(ctx);
	
	glEnable(GL_DEPTH_TEST);
}

void color_map_viewer::create_gui() {

	create_overlay_gui();
	add_member_control(this, "Band Height", layout.band_height, "value_slider", "min=5;max=50;step=5;ticks=true");
}

void color_map_viewer::create_gui(cgv::gui::provider& p) {

	p.add_member_control(this, "Show", show, "check");
}

void color_map_viewer::set_color_map_names(const std::vector<std::string>& names) {

	this->names = names;
	texts_out_of_date = true;
}

void color_map_viewer::set_color_map_texture(texture* tex) {
	this->tex = tex;
	on_set(&layout.total_height);
}

void color_map_viewer::init_styles(context& ctx) {
	// get theme colors
	auto& ti = cgv::gui::theme_info::instance();
	rgba background_color = rgba(ti.background(), 1.0f);
	rgba group_color = rgba(ti.group(), 1.0f);
	rgba border_color = rgba(ti.border(), 1.0f);

	// configure style for the container rectangle
	container_style.apply_gamma = false;
	//container_style.fill_color = rgba(0.9f, 0.9f, 0.9f, 1.0f);
	//container_style.border_color = rgba(0.2f, 0.2f, 0.2f, 1.0f);
	container_style.fill_color = group_color;
	container_style.border_color = background_color;
	container_style.border_width = 3.0f;
	container_style.feather_width = 0.0f;
	
	// configure style for the border rectangles
	border_style = container_style;
	//border_style.fill_color = rgba(0.2f, 0.2f, 0.2f, 1.0f);
	border_style.fill_color = border_color;
	border_style.border_width = 0.0f;
	
	// configure style for the color scale rectangle
	color_map_style = border_style;
	color_map_style.use_texture = true;

	// configure text style
	text_style.fill_color = rgba(rgb(1.0f), 0.666f);
	text_style.border_color = rgba(rgb(0.0f), 0.666f);
	text_style.border_radius = 0.25f;
	text_style.border_width = 0.75f;
	text_style.use_blending = true;

	// configure style for final blitting of overlay into main frame buffer
	cgv::glutil::shape2d_style final_style;
	final_style.fill_color = rgba(1.0f);
	final_style.use_texture = true;
	final_style.use_blending = false;
	final_style.feather_width = 0.0f;

	auto& final_prog = overlay_canvas.enable_shader(ctx, "rectangle");
	final_style.apply(ctx, final_prog);
	overlay_canvas.disable_current_shader(ctx);
}

void color_map_viewer::update_texts() {

	texts.clear();
	int step = layout.color_map_rect.size().y() / names.size();
	ivec2 base = layout.color_map_rect.pos() + ivec2(layout.color_map_rect.size().x() / 2, step / 2 - static_cast<int>(0.333f*font_size));
	int i = 0;
	for(const auto& name : names) {
		ivec2 p = base;
		p.y() += (names.size() - 1 - i)*step;
		texts.add_text(name, p, TA_BOTTOM);
		++i;
	}
	texts_out_of_date = false;
}
