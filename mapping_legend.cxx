#include "mapping_legend.h"

#include <cgv/gui/theme_info.h>
#include <cgv_gl/gl/gl.h>

mapping_legend::mapping_legend() {

	set_name("Mapping Legend");
	block_events = false;

	set_overlay_alignment(AO_START, AO_START);
	set_overlay_stretch(SO_NONE);
	set_overlay_margin(ivec2(-3));
	set_overlay_size(ivec2(316, 0));

	content_canvas.set_origin_setting(cgv::g2d::Origin::kTopLeft);
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

}

void mapping_legend::init_frame(cgv::render::context& ctx)
{
	if(ensure_layout(ctx))
		create_geometry();
}

void mapping_legend::draw_content(cgv::render::context& ctx) {

	begin_content(ctx);
	enable_blending();
	
	ivec2 container_size = get_overlay_size();
	
	// draw container
	content_canvas.enable_shader(ctx, "rectangle");
	content_canvas.set_style(ctx, container_style);
	content_canvas.draw_shape(ctx, ivec2(0), container_size);
	
	content_canvas.set_style(ctx, border_style);
	for(int position : dividers)
		content_canvas.draw_shape(ctx, vec2(12.0f, position), vec2(static_cast<float>(container_size.x() - 24), 1.0f));

	content_canvas.set_style(ctx, color_box_style);
	for(const auto& [position, color] : color_boxes)
		content_canvas.draw_shape(ctx, position, vec2(text_style.font_size), color);

	content_canvas.disable_current_shader(ctx);

	cgv::g2d::ref_msdf_gl_canvas_font_renderer(ctx).render(ctx, content_canvas, text, text_style);

	disable_blending();
	end_content(ctx);
}

void mapping_legend::update(const traj_dataset<float>& dataset, const glyph_layer_manager& glyph_manager) {

	layers.clear();

	const auto attribute_names = dataset.get_attribute_names();
	const auto& attribute_mappings = glyph_manager.ref_glyph_attribute_mappings();
	
	for(const auto& mapping : attribute_mappings) {
		const auto shape = mapping.get_shape_ptr();

		if(!shape || !mapping.get_active())
			continue;

		layer_info layer;
		
		const auto& shape_attributes = shape->supported_attributes();

		if(shape_attributes.size() != mapping.get_attrib_indices().size() ||
			shape_attributes.size() != mapping.get_color_map_indices().size()) {
			continue;
		}

		layer.title = mapping.get_name();
		if(!layer.title.empty())
			layer.title += " ";
		layer.title += "(" + glyph_type_registry::display_names()[shape->type()] + ")";

		for(size_t i = 0; i < mapping.get_attrib_indices().size(); ++i) {
			layer_info::line_info line;

			std::string visual_attribute_name = shape_attributes[i].name;
			int attribute_index = mapping.get_attrib_indices()[i];
			int color_index = mapping.get_color_map_indices()[i];
			vec2 range(vec3(mapping.ref_attrib_mapping_values()[i])); // narrow vec4 to vec2 by trimming the w and z components

			// The following two methods are in large parts borrowed from color_legend_manager and color_map_legend.
			// At some point it might be nice to have it in some library in one form or another.
			const auto float_to_string = [](float value, unsigned precision) {
				std::string str = cgv::utils::to_string(value, -1, precision, true);

				if(str.length() > 1) {
					cgv::utils::rtrim(str, "0");
					cgv::utils::rtrim(str, ".");
				}

				return str;
			};

			const auto range_to_string = [&float_to_string](vec2 range) {
				const float diff = range.y() - range.x();

				unsigned precision = 7;
				if(diff > 5)
					precision = 1;
				else if(diff > 1)
					precision = 2;
				else if(diff > .5f)
					precision = 3;
				else if(diff > .25f)
					precision = 4;
				else if(diff > .125f)
					precision = 5;
				else if(diff > .0625f)
					precision = 6;

				return "[" + float_to_string(range.x(), precision) + ", " + float_to_string(range.y(), precision) + "]";
			};

			// special handling for line and star plot
			if(shape->type() == GT_LINE_PLOT || shape->type() == GT_STAR) {
				if(attribute_index > -1 && shape_attributes[i].type == GAT_SIZE) {
					line.text = attribute_names[attribute_index];
					line.range = range_to_string(range);

					if(i > 0) {
						const auto& color = mapping.ref_attrib_colors()[i - 1];
						line.has_color = true;
						line.color = color;
					}
				}
			} else { // generic glyph
				if(attribute_index > -1) {
					// attribute is mapped
					line.text = visual_attribute_name + " from " + attribute_names[attribute_index];
					line.range = range_to_string(range);

					//if(color_index > -1)
					//	line.text += " map";
				} else {
					if(color_index < 0 && shape_attributes[i].type == GAT_COLOR) {
						// using constant color
						const auto& color = mapping.ref_attrib_colors()[i];

						line.text = "";// visual_attribute_name;
						line.has_color = true;
						line.color = color;
					}
				}
			}

			if(!line.empty())
				layer.lines.push_back(line);
		}

		layers.push_back(layer);
	}
	
	create_geometry();
}

void mapping_legend::create_geometry() {

	text.clear();
	dividers.clear();
	color_boxes.clear();

	const float padding = 12.0f;
	vec2 position(padding);
	ivec2 overlay_size = get_overlay_size();

	size_t layer_idx = 1;
	for(const auto& layer : layers) {
		text.add_text(std::to_string(layer_idx) + ": " + layer.title, position, cgv::render::TA_TOP_LEFT, 1.2f);
		position.y() += 1.75f * text_style.font_size;
		
		for(const auto& [str, range, has_color, color] : layer.lines) {
			vec2 offset(0.0f);
			
			if(has_color) {
				color_boxes.push_back({ cgv::math::round(position), color });
				offset.x() = text_style.font_size + 5.0f;
			}
			
			text.add_text(str, position + offset, cgv::render::TA_TOP_LEFT);

			offset.x() = static_cast<float>(overlay_size.x()) - 2.0f * padding;
			text.add_text(range, position + offset, cgv::render::TA_TOP_RIGHT);

			position.y() += 1.2f * text_style.font_size;
		}

		dividers.push_back(std::round(position.y() + 5.0f) + 0.5f);

		position.y() += padding;
		++layer_idx;
	}

	if(!dividers.empty()) {
		dividers.pop_back();
		position.y() -= padding;
	}

	float height = position.y() + padding;

	if(layers.empty())
		height = 0.0f;

	overlay_size.y() = static_cast<int>(height + 0.5f);
	set_overlay_size(overlay_size);

	post_damage();
}

void mapping_legend::init_styles() {

	auto& ti = cgv::gui::theme_info::instance();

	// configure style for the container rectangle
	container_style.fill_color = ti.group();
	container_style.border_color = ti.background();
	container_style.border_width = 3.0f;
	container_style.feather_width = 0.0f;
	
	// configure style for the border rectangles
	border_style = container_style;
	border_style.fill_color = ti.border();
	border_style.border_width = 0.0f;

	color_box_style = container_style;
	color_box_style.use_fill_color = false;
	color_box_style.border_width = 1.0f;
	color_box_style.border_color = ti.border();
	color_box_style.use_blending = true;
	color_box_style.feather_width = 1.0f;

	// configure text style
	text_style = cgv::g2d::text2d_style::preset_default(ti.text());
	text_style.font_size = 12.0f;
}