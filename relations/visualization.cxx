#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
#endif

#include <format>

#include <cgv/data/informed_ptr.h>
#include <cgv/gui/provider.h>
#include <cgv/render/shader_program.h>

#include "color/color_map_manager.h"
#include "render_layout.h"
#include "visualization.h"


namespace {

/// GUI descriptor for the dropdown to select the relation function.
static auto const function_dropdown = []{
	std::string s = "enums='";
	for (auto variant : relation_vis::function_names) {s += variant; s += ",";}
	s.back() = '\'';
	return s;
}();

} // namespace


template <class T>
struct cgv::type::info::type_name<relation_vis::PseudoEnum<T>> {
	static const char* get_name() {return "enum";}
};


void relation_vis::build_gui (
	cgv::gui::provider&             p,
	uint32_t                        num_trajectories,
	cgv::vec4                       data_extent,
	std::vector<std::string> const& color_maps
) {
	auto const b = dynamic_cast<cgv::base::base*>(&p);
	p.add_member_control(b, "Function", function, "dropdown", function_dropdown);
	p.add_member_control(b, "Normalize", normalize, "check");

	// Evaluation.
	p.add_member_control(b, "Direction", direction, "dropdown", "enums='"
		"reference to all,"
		"all to reference,"
		"all to all'"
	);
	if (direction != Direction::all_to_all)
		p.add_member_control(b, "Reference traj.", reference_trajectory, "value_slider",
			std::format("min=0;max={};step=1;ticks=true", num_trajectories - 1)
		);
	p.add_member_control(b, "Spatial radius", radius[0], "value_slider",
		std::format("min=0;max={};ticks=true;log=true", max_value(cgv::vec3{data_extent}) * 0.1f)
	);
	p.add_member_control(b, "Temporal radius", radius[1], "value_slider",
		std::format("min=0;max={};ticks=true;log=true", data_extent[3] * 0.1f)
	);
	p.add_member_control(b, "Sample rate", sample_rate, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", 1e4 / data_extent[3])
	);
	p.add_member_control(b, "Directionality", cos_exp, "value_slider",
		"min=0;max=100;ticks=true;log=true"
	);
	if (scale_by_cos) p.add_member_control(b, "Cutoff weight", cos_cutoff, "value_slider",
		"min=0;max=1;ticks=true"
	);

	if (p.begin_tree_node("Color scale", color_scale)) {
		p.add_member_control(b, "Base", color_scale.base, "dropdown", p.concat_enum_def(color_maps));
		p.add_member_control(b, "Highlight", color_scale.highlight);
		p.add_member_control(b, "Background", color_scale.background);

		p.add_member_control(b, "Begin", color_scale.domain[0], "value_slider",
			"ticks=true;min=-10;max=10;log=true"
		);
		p.add_member_control(b, "End", color_scale.domain[1], "value_slider",
			"ticks=true;min=-10;max=10;log=true"
		);

		p.add_member_control(b, "Diverging", color_scale.diverging, "check");
		if (color_scale.diverging)
			p.add_member_control(b, "Midpoint%", color_scale.midpoint, "value_slider",
				"ticks=true;min=0;max=100;step=1"
			);

		p.add_member_control(b, "Transform", color_scale.transform, "dropdown", std::format(
			"enums='linear={},logarithmic={},exponential={}'",
			static_cast<int>(ColorTransform::Linear),
			static_cast<int>(ColorTransform::Log),
			static_cast<int>(ColorTransform::Pow)
		));
		switch (color_scale.transform) {
			case ColorTransform::Linear: break;
			case ColorTransform::Log:
				p.add_member_control(b, "Base", color_scale.log_base, "value_slider",
					"ticks=true;min=0.1;max=100;log=true"
				);
				break;
			case ColorTransform::Pow:
				p.add_member_control(b, "Exponent", color_scale.exponent, "value_slider",
					"ticks=true;min=0.25;max=10;log=true"
				);
				break;
		}
		p.end_tree_node(color_scale);
	}
}

auto relation_vis::on_set (void* member, cgv::render::context& ctx, color_map_manager const& colors)
	-> UpdateFlags
{
	auto const ptr = cgv::data::informed_ptr{member};
	if (ptr.points_to(direction)) return UpdateFlag::gui;
	if (ptr.points_to(cos_exp)) {
		if (scale_by_cos == (cos_exp > 0)) return 0;
		scale_by_cos = cos_exp > 0;
		return UpdateFlag::gui | UpdateFlag::shader_opts;
	}
	if (!ptr.points_to_member_of(color_scale)) return 0;

	update_color_scale(ctx, colors);
	return ptr.points_to_one_of(color_scale.diverging, color_scale.transform) ? UpdateFlag::gui : 0;
}

void relation_vis::update_color_scale (cgv::render::context& ctx, color_map_manager const& colors)
{
	// Configure color scale object.
	auto& scale = color_scale.scale;
	scale->set_scheme(cgv::media::continuous_color_scheme::linear(
		colors.ref_color_maps().at(color_scale.base.value).ramp.get_color_points()
	));

	auto min = color_scale.domain[0], max = color_scale.domain[1];
	scale->set_reversed(min > max);
	if (min > max) std::swap(min, max);
	scale->set_domain({min, max});

	scale->set_diverging(color_scale.diverging);
	scale->set_midpoint(lerp(color_scale.domain[0], color_scale.domain[1], color_scale.midpoint * 0.01f));
	scale->set_transform(color_scale.transform);
	scale->set_pow_exponent(color_scale.exponent);
	scale->set_log_base(color_scale.log_base);

	// Generate texture.
	std::array<cgv::rgb8, 256> samples;
	for (auto i = 0; i < samples.size(); ++i) samples[i] = scale->map_value(
		min + 1.f/(samples.size() - 1) * (max - min) * i
	);
	cgv::data::data_format fmt {samples.size(), 1, cgv::type::info::TI_UINT8, cgv::data::CF_RGB};
	color_scale.texture.create(ctx, {&fmt, samples.data()}, 0);
}

void relation_vis::set_defaults (cgv::vec4 extent)
{
	radius[0]   = max_value(cgv::vec3{extent}) * 0.01f;
	radius[1]   = extent[3] * 0.01f;
	sample_rate = 1e3f / extent[3];
}

void relation_vis::set_shader_opts (cgv::render::shader_compile_options& opts) const
{
	opts.define_macro("RELATION_FUNCTION",        static_cast<uint32_t>(function));
	opts.define_macro("RELATION_COLOR_SCALE_TEX", texture_idx::relation_color_map);
	opts.define_macro("RELATION_SCALE_BY_COS",    scale_by_cos                   );
}

void relation_vis::set_uniforms (
	cgv::render::context&        c,
	cgv::render::shader_program& p
) const {
	auto min = color_scale.domain[0], max = color_scale.domain[1];
	if (min > max) std::swap(min, max);
	p.set_uniform(c, "relation_radius",           radius                          );
	p.set_uniform(c, "relation_sample_rate",      sample_rate                     );
	p.set_uniform(c, "relation_direction.value",  static_cast<uint32_t>(direction));
	p.set_uniform(c, "relation_ref_traj",         reference_trajectory            );
	p.set_uniform(c, "relation_normalize",        normalize                       );
	p.set_uniform(c, "relation_color_domain",     cgv::vec2{min, max}             );
	p.set_uniform(c, "relation_highlight_color",  color_scale.highlight           );
	p.set_uniform(c, "relation_background_color", color_scale.background          );
	p.set_uniform(c, "relation_cos_exp",          cos_exp                         );
	p.set_uniform(c, "relation_min_cos", scale_by_cos ? 2*pow(cos_cutoff, 1/cos_exp) - 1 : 0);
}

auto get_reflection_traits(enum relation_vis::Function const&)
	-> cgv::reflect::enum_reflection_traits<enum relation_vis::Function>
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
