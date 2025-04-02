
// Implemented header
#include "common.h"

cgv::render::shader_define_map tube_shading_settings::build_tube_shading_defines(
	const glyph_layer_manager::configuration &glyph_layers_config, bool debug_highlight_segments
) const {
	cgv::render::shader_define_map defines;

	// debug defines
	cgv::render::shader_code::set_define(defines, "DEBUG_SEGMENTS", debug_highlight_segments, false);

	// ambient occlusion defines
	cgv::render::shader_code::set_define(defines, "ENABLE_AMBIENT_OCCLUSION", ao_style.enable, true);

	// grid defines
	cgv::render::shader_code::set_define(defines, "GRID_MODE", grid_mode, GM_COLOR);
	auto gs = static_cast<unsigned>(grid_normal_settings);
	if(grid_normal_inwards) gs += 4u;
	if(grid_normal_variant) gs += 8u;
	cgv::render::shader_code::set_define(defines, "GRID_NORMAL_SETTINGS", gs, 0u);
	cgv::render::shader_code::set_define(defines, "ENABLE_FUZZY_GRID", enable_fuzzy_grid, false);

	// glyph layer defines
	//if (render.visualizations.size() > 0)
	{
		cgv::render::shader_code::set_define(defines, "GLYPH_MAPPING_UNIFORMS", glyph_layers_config.uniforms_definition, std::string(""));

		cgv::render::shader_code::set_define(defines, "CONSTANT_FLOAT_UNIFORM_COUNT", glyph_layers_config.constant_float_parameters.size(), static_cast<size_t>(0));
		cgv::render::shader_code::set_define(defines, "CONSTANT_COLOR_UNIFORM_COUNT", glyph_layers_config.constant_color_parameters.size(), static_cast<size_t>(0));
		cgv::render::shader_code::set_define(defines, "MAPPING_PARAMETER_UNIFORM_COUNT", glyph_layers_config.mapping_parameters.size(), static_cast<size_t>(0));

		for(size_t i = 0; i < glyph_layers_config.layer_configs.size(); ++i) {
			const auto& lc = glyph_layers_config.layer_configs[i];
			cgv::render::shader_code::set_define(defines, "L" + std::to_string(i) + "_VISIBLE", lc.visible, true);
			cgv::render::shader_code::set_define(defines, "L" + std::to_string(i) + "_MAPPED_ATTRIB_COUNT", lc.mapped_attributes.size(), static_cast<size_t>(0));
			cgv::render::shader_code::set_define(defines, "L" + std::to_string(i) + "_GLYPH_DEFINITION", lc.glyph_definition, std::string(""));
		}
	}

	return defines;
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
	const auto &srs = *static_cast<const cgv::render::surface_render_style*>(&render_style);
	prog.set_uniform(ctx, "map_color_to_material", int(srs.map_color_to_material));
	prog.set_uniform(ctx, "culling_mode", int(srs.culling_mode));
	prog.set_uniform(ctx, "illumination_mode", int(srs.illumination_mode));
}
