#include "mapping_legend.h"

#include <cgv/gui/theme_info.h>
#include <cgv/utils/number_format.h>
#include <cgv_gl/gl/gl.h>

mapping_legend::mapping_legend() {

	set_name("Mapping Legend");
	blocks_events(false);

	set_size(cgv::ivec2(316, 0));

	content_canvas.set_origin_setting(cgv::g2d::CoordinateOrigin::kUpperLeft);
}

bool mapping_legend::init(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx, 1);

	register_shader("rectangle", cgv::g2d::shaders::rectangle);

	bool success = canvas_overlay::init(ctx);
	success &= text.init(ctx);

	return success;
}

void mapping_legend::clear(cgv::render::context& ctx) {

	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx, -1);
	canvas_overlay::clear(ctx);
	
	text.destruct(ctx);
}

void mapping_legend::init_frame(cgv::render::context& ctx)
{
	if(ensure_layout(ctx))
		create_geometry();
}

void mapping_legend::draw_content(cgv::render::context& ctx) {

	begin_content(ctx);
	
	content_canvas.enable_shader(ctx, "rectangle");
	content_canvas.set_style(ctx, border_style);
	for(auto &position : dividers)
		content_canvas.draw_shape(ctx, cgv::vec2(12.0f, position), cgv::vec2(static_cast<float>(get_rectangle().w() - 24), 1.0f));

	content_canvas.set_style(ctx, color_box_style);
	for(const auto& [position, color] : color_boxes)
		content_canvas.draw_shape(ctx, position, cgv::vec2(text_style.font_size), color);

	content_canvas.disable_current_shader(ctx);

	if(text_out_of_date) {
		text.create(ctx);
		text_out_of_date = false;
	}
	cgv::g2d::ref_msdf_gl_font_renderer_2d(ctx).render(ctx, content_canvas, text, text_style);

	end_content(ctx);
}

void mapping_legend::update(const traj_dataset<float>& dataset, const glyph_layer_manager& glyph_manager) {

	layers.clear();

	const auto attribute_names = dataset.get_attribute_names();
	const auto& attribute_mappings = glyph_manager.ref_glyph_attribute_mappings();
	
	for(const auto& mapping : attribute_mappings) {
		const auto shape = mapping.get_shape();

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
		layer.title += "(" + glyph_type_registry::display_names()[static_cast<int>(shape->type())] + ")";

		cgv::utils::number_format format;
		format.fixed = true;
		format.trailing_zeros = false;

		for(size_t i = 0; i < mapping.get_attrib_indices().size(); ++i) {
			layer_info::line_info line;

			std::string visual_attribute_name = shape_attributes[i].name;
			int attribute_index = mapping.get_attrib_indices()[i];
			int color_index = mapping.get_color_map_indices()[i];
			cgv::vec2 input_range = mapping.ref_attrib_mapping_values()[i].input_range;

			const auto range_to_string = [&format](cgv::vec2 range) {
				format.precision_from_range(range.x(), range.y());
				format.precision += 1;
				return "[" + format.convert(range.x()) + ", " + format.convert(range.y()) + "]";
			};

			// special handling for line and star plot
			if(shape->type() == GlyphType::kLinePlot || shape->type() == GlyphType::kStar) {
				if(attribute_index > -1 && shape_attributes[i].type == GlyphAttributeType::kSize) {
					line.text = attribute_names[attribute_index];
					line.range = range_to_string(input_range);

					if(i > 0) {
						const auto& color = mapping.ref_attrib_colors()[i - 1];
						line.has_color = true;
						line.color = color;
					}
				}
			} else { // generic glyph
				if(attribute_index > -1) {
					// attribute is mapped
					line.text = visual_attribute_name + " <-- " + attribute_names[attribute_index];
					line.range = range_to_string(input_range);

					//if(color_index > -1)
					//	line.text += " map";
				} else {
					if(color_index < 0 && shape_attributes[i].type == GlyphAttributeType::kColor) {
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

void mapping_legend::create_geometry()
{
	text.clear();
	dividers.clear();
	color_boxes.clear();

	const float padding = 12.0f;
	cgv::vec2 position(padding);
	cgv::ivec2 overlay_size = get_rectangle().size;

	size_t layer_idx = 1;
	for(const auto& layer : layers) {
		text.positions.emplace_back(cgv::vec3(position, .0f));
		text.alignments.emplace_back(cgv::render::TA_TOP_LEFT);
		text.texts.emplace_back(std::to_string(layer_idx) + ": " + layer.title);
		position.y() += 1.75f * text_style.font_size;
		
		for(const auto& [str, range, has_color, color] : layer.lines) {
			cgv::vec2 offset(0.0f);
			
			if(has_color) {
				color_boxes.push_back({ cgv::math::round(position), color });
				offset.x() = text_style.font_size + 5.0f;
			}

			text.positions.emplace_back(cgv::vec3(position + offset, .0f));
			text.alignments.emplace_back(cgv::render::TA_TOP_LEFT);
			text.texts.emplace_back(str);

			offset.x() = static_cast<float>(overlay_size.x()) - 2.0f * padding;
			text.positions.emplace_back(cgv::vec3(position + offset, .0f));
			text.alignments.emplace_back(cgv::render::TA_TOP_RIGHT);
			text.texts.emplace_back(range);

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
	set_size(overlay_size);

	text_out_of_date = true;

	post_damage();
}

void mapping_legend::init_styles() {

	auto& theme = cgv::gui::theme_info::instance();

	// configure style for the border rectangles
	border_style.fill_color = theme.border();
	border_style.border_width = 0.0f;
	border_style.feather_width = 0.0f;

	// configure style for the color boxes
	color_box_style.use_fill_color = false;
	color_box_style.border_width = 1.0f;
	color_box_style.border_color = theme.border();
	color_box_style.use_blending = true;
	color_box_style.feather_width = 1.0f;

	// configure text style
	text_style.fill_color = theme.text();
	text_style.font_size = 12.0f;
}
