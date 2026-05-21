
// Implemented header
#include "common.h"

cgv::render::shader_compile_options tube_shading_settings::build_tube_shading_options (
	const glyph_layer_manager::configuration &glyph_layers_config, bool debug_highlight_segments
) const {
	cgv::render::shader_compile_options options;

	// streaming-related defines
	options.define_macro_if_not_default("ALTERNATIVE_RING_BUFFER", alternative_ring_buffer, false);

	// debug defines
	options.define_macro_if_not_default("DEBUG_SEGMENTS", debug_highlight_segments, false);

	// ambient occlusion defines
	options.define_macro_if_not_default("ENABLE_AMBIENT_OCCLUSION", ao_style.enable, true);

	// grid defines
	options.define_macro_if_not_default("GRID_MODE", grid_mode, GridMode::kColor);
	auto gs = static_cast<unsigned>(grid_normal_settings);
	if(grid_normal_inwards) gs += 4u;
	if(grid_normal_variant) gs += 8u;
	options.define_macro_if_not_default("GRID_NORMAL_SETTINGS", gs, 0u);
	options.define_macro_if_not_default("ENABLE_FUZZY_GRID", enable_fuzzy_grid, false);

	// glyph layer defines
	//if (render.visualizations.size() > 0)
	{
		options.define_macro_if_not_default("GLYPH_MAPPING_UNIFORMS", glyph_layers_config.uniforms_definition, std::string(""));

		options.define_macro_if_not_default("CONSTANT_FLOAT_UNIFORM_COUNT", glyph_layers_config.constant_float_parameters.size(), static_cast<size_t>(0));
		options.define_macro_if_not_default("CONSTANT_COLOR_UNIFORM_COUNT", glyph_layers_config.constant_color_parameters.size(), static_cast<size_t>(0));
		options.define_macro_if_not_default("MAPPING_PARAMETER_UNIFORM_COUNT", glyph_layers_config.mapping_parameters.size(), static_cast<size_t>(0));

		for(size_t i = 0; i < glyph_layers_config.layer_configs.size(); ++i) {
			const auto& lc = glyph_layers_config.layer_configs[i];
			options.define_macro_if_not_default("L" + std::to_string(i) + "_VISIBLE", lc.visible, true);
			options.define_macro_if_not_default("L" + std::to_string(i) + "_MAPPED_ATTRIB_COUNT", lc.mapped_attributes.size(), static_cast<size_t>(0));
			options.define_macro_if_not_default("L" + std::to_string(i) + "_GLYPH_DEFINITION", lc.glyph_definition, std::string(""));
		}
	}

	return options;
}

void tube_shading_settings::set_uniforms (
	cgv::render::context &ctx, cgv::render::shader_program &prog,
    const cgv::render::textured_spline_tube_render_style &render_style,
	const glyph_layer_manager::configuration &glyph_layers_config
	#if RTX_SUPPORT
		, bool optix_enabled
	#endif
) const {
	// set render parameters
	prog.set_uniform(ctx, "use_gamma", true);

	// set ambient occlusion parameters
	if(ao_style.enable) {
		//prog.set_uniform(ctx, "ambient_occlusion.enable", ao_style.enable);
		prog.set_uniform(ctx, "ambient_occlusion.sample_offset", ao_style.sample_offset);
		prog.set_uniform(ctx, "ambient_occlusion.sample_distance", ao_style.sample_distance);
		prog.set_uniform(ctx, "ambient_occlusion.strength_scale", ao_style.strength_scale);

		prog.set_uniform(ctx, "ambient_occlusion.tex_offset", ao_style.texture_offset);
		prog.set_uniform(ctx, "ambient_occlusion.tex_scaling", ao_style.texture_scaling);
		prog.set_uniform(ctx, "ambient_occlusion.texcoord_scaling", ao_style.texcoord_scaling);
		prog.set_uniform(ctx, "ambient_occlusion.texel_size", ao_style.texel_size);

		prog.set_uniform(ctx, "ambient_occlusion.cone_angle_factor", ao_style.angle_factor);
		prog.set_uniform_array(ctx, "ambient_occlusion.sample_directions", ao_style.sample_directions);
	}

	// set grid parameters
	prog.set_uniform(ctx, "grid_color", grid_color);
	prog.set_uniform(ctx, "normal_mapping_scale", normal_mapping_scale);
	for(size_t i=0; i<grids.size(); ++i) {
		std::string base_name = "grids[" + std::to_string(i) + "].";
		prog.set_uniform(ctx, base_name + "scaling", grids[i].scaling);
		prog.set_uniform(ctx, base_name + "thickness", grids[i].thickness);
		prog.set_uniform(ctx, base_name + "blend_factor", grids[i].blend_factor);
	}

	// set attribute mapping parameters
	for(const auto& p : glyph_layers_config.constant_float_parameters)
		prog.set_uniform(ctx, p.first, *p.second);
	for(const auto& p : glyph_layers_config.constant_color_parameters)
		prog.set_uniform(ctx, p.first, *p.second);
	for(const auto& p : glyph_layers_config.mapping_parameters)
		prog.set_uniform(ctx, p.first, *p.second);

	// set playback cursor position
	prog.set_uniform(ctx, "playback_t", playback_t);

	// map global settings
	prog.set_uniform(
		ctx, "use_curvature_correction", (
			#if RTX_SUPPORT
				optix_enabled ||
			#endif
			render_style.is_tube()
		) && render_style.use_curvature_correction
	);
	prog.set_uniform(ctx, "length_scale", render_style.length_scale);
	prog.set_uniform(ctx, "antialias_radius", render_style.antialias_radius);

	// CGV surface/lighting parameters
	ctx.set_material(render_style.material);
	prog.set_uniform(ctx, "map_color_to_material", static_cast<int>(render_style.map_color_to_material));
	prog.set_uniform(ctx, "culling_mode", static_cast<int>(render_style.culling_mode));
	prog.set_uniform(ctx, "illumination_mode", static_cast<int>(render_style.illumination_mode));
}
