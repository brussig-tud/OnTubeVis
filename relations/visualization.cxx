#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
#endif

#include <format>

#include <cgv/gui/provider.h>
#include <cgv/render/shader_program.h>

#include "visualization.h"


template <class T>
struct cgv::type::info::type_name<relation_vis::pseudo_enum<T>> {
	static const char* get_name() {return "enum";}
};


void relation_vis::build_gui (
	cgv::gui::provider&             p,
	uint32_t                        num_trajectories,
	cgv::vec4                       data_extent,
	std::vector<std::string> const& color_maps
) {
	auto const b = dynamic_cast<cgv::base::base*>(&p);
	p.add_member_control(b, "Function", function, "dropdown", "enums='"
		"None,"
		"Proximity,"
		"Alignment,"
		"[debug] Curve Parameter,"
		"[debug] Velocity,"
		"[debug] Spatial Index,"
		"[debug] Temporal Index,"
		"[debug] Index Hash,"
		"[debug] Bucket Load,"
		"[debug] Trajectory Interval,"
		"[debug] Skipped Cells,"
		"[debug] Queried Cells,"
		"[debug] Intervals Found,"
		"[debug] Sampled Points,"
		"[debug] Contributing Samples'"
	);
	p.add_member_control(b, "Spatial Radius", radius[0], "value_slider",
		std::format("min=0;max={};ticks=true;log=true", max_value(cgv::vec3{data_extent}) * 0.1f)
	);
	p.add_member_control(b, "Temporal Radius", radius[1], "value_slider",
		std::format("min=0;max={};ticks=true;log=true", data_extent[3] * 0.1f)
	);
	p.add_member_control(b, "Sample Rate", sample_rate, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", 1e4 / data_extent[3])
	);
	p.add_member_control(b, "Direction", direction, "dropdown", "enums='"
		"Reference to All,"
		"All to Reference,"
		"All to All'"
	);
	p.add_member_control(b, "Reference Traj.", reference_trajectory, "value_slider",
		std::format("min=0;max={};step=1;ticks=true", num_trajectories - 1)
	);
	p.add_member_control(b, "Normalize", normalize, "check");
	p.add_member_control(b, "Color Scale", color_map, "dropdown", p.concat_enum_def(color_maps));
	p.add_member_control(b, "Scale Begin", color_range[0], "value_slider",
		"ticks=true;log=true;min=-1"
	);
	p.add_member_control(b, "Scale End", color_range[1], "value_slider",
		"ticks=true;log=true;min=-1"
	);
	p.add_member_control(b, "Scale Type", color_transform, "dropdown", "enums='"
		"Linear,"
		"Logarithmic,"
		"Diverging Logarithmic'"
	);
	p.add_member_control(b, "Highlight", highlight_color);
	p.add_member_control(b, "Background", background_color);
}

void relation_vis::set_defaults (cgv::vec4 extent)
{
	radius[0]   = max_value(cgv::vec3{extent}) * 0.01f;
	radius[1]   = extent[3] * 0.01f;
	sample_rate = 1e3f / extent[3];
}

void relation_vis::set_shader_opts (cgv::render::shader_compile_options& opts) const
{
	opts.define_macro("RELATION_FUNCTION", static_cast<uint32_t>(function));
}

void relation_vis::set_uniforms (
	cgv::render::context&        c,
	cgv::render::shader_program& p
) const {
	p.set_uniform(c, "relation_radius",                radius);
	p.set_uniform(c, "relation_sample_rate",           sample_rate);
	p.set_uniform(c, "relation_direction.value",       static_cast<uint32_t>(direction));
	p.set_uniform(c, "relation_ref_traj",              reference_trajectory);
	p.set_uniform(c, "relation_normalize",             normalize);
	p.set_uniform(c, "relation_color_map",             color_map.value);
	p.set_uniform(c, "relation_color_range",           color_range);
	p.set_uniform(c, "relation_color_transform.value", static_cast<uint32_t>(color_transform));
	p.set_uniform(c, "relation_highlight_color",       highlight_color);
	p.set_uniform(c, "relation_background_color",      background_color);
}

auto get_reflection_traits(enum relation_vis::function const&)
	-> cgv::reflect::enum_reflection_traits<enum relation_vis::function>
{
	return {
		"none,"
		"proximity,"
		"alignment,"
		"dbg_seg_t,"
		"dbg_velocity,"
		"dbg_index_xyz,"
		"dbg_index_t,"
		"dbg_signature,"
		"dbg_bucket_load,"
		"dbg_local_interval,"
		"dbg_skipped_cells,"
		"dbg_num_cells,"
		"dbg_num_intervals,"
		"dbg_num_samples,"
		"dbg_num_evals,"
	};
}
