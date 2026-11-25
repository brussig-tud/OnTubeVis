#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
#include "cgv/math/functions.h"
#endif

#include <filesystem>
#include <format>

#include <cgv/data/informed_ptr.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/gui/provider.h>
#include <cgv/render/shader_program.h>
#include <cgv/utils/file.h>

#include "color/color_map_manager.h"
#include "render_layout.h"
#include "visualization.h"


namespace {

/// Human-readable names of the data variables that can be visualized.
static constexpr auto data_var_names = std::to_array<std::string_view>({
	"Nothing",
	"Relation",
	"[debug] Curve parameter",
	"[debug] Curve velocity",
	"[debug] Spatial index",
	"[debug] Temporal index",
	"[debug] Index hash",
	"[debug] Bucket load",
	"[debug] Trajectory interval",
	"[debug] Queried cells",
	"[debug] Intervals found",
});

/// GUI descriptor for the dropdown to select the visualized data variable.
static auto const data_var_dropdown = []{
	std::string s = "enums='";
	for (auto variant : data_var_names) {s += variant; s += ',';}
	s.back() = '\'';
	return s;
}();

/// Minimum cosine term for a sample to contribute to the relation.
[[nodiscard]] constexpr auto min_cos (float cutoff_angle) -> float
{
	return cos(cgv::math::deg2rad(cutoff_angle));
}

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
	connect_copy(
		p.add_button("Load")->click,
		[&](auto&& _) {
			auto path = cgv::gui::file_open_dialog(
				"Load relation definition", "*", "relations/def");
			size_t size;
			auto const data = cgv::utils::file::read(path, false, &size);
			if (!data) return;

			data_var = DataVar::relation;
			p.update_member(&data_var);

			relation = {
				.name = std::filesystem::path{std::move(path)}.stem(),
				.definition = {data, size},
			};
			dynamic_cast<cgv::base::base*>(&p)->on_set(&relation.definition);
		}
	);
	p.add_member_control(b, "Show", data_var, "dropdown", data_var_dropdown);
	p.add_member_control(b, "", filter, "dropdown", "enums='"
		"reference to all,all to reference,all to all'"
	);
	if (filter != TrajFilter::all_to_all)
		p.add_member_control(b, "Reference traj.", reference_trajectory, "value_slider",
			std::format("min=0;max={};step=1;ticks=true", num_trajectories - 1)
		);
	p.add_member_control(b, "Normalize", normalize, "check");

	p.add_member_control(b, "Query dir.", query_dir, "dropdown",
		"enums='surface normal,movement direction'"
	);
	p.add_member_control(b, "Reverse", reverse_qdir, "check");
	p.add_decorator("Query radius", "text");
	p.add_member_control(b, "Spatial", radius.space, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", max_value(cgv::vec3{data_extent}) * 0.1f)
	);
	p.add_member_control(b, "Past", radius.pre, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", data_extent[3] * 0.1f)
	);
	p.add_member_control(b, "Future", radius.post, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", data_extent[3] * 0.1f)
	);
	p.add_member_control(b, "Angle in °", cutoff_angle, "value_slider",
		"min=0;max=180;ticks=true"
	);
	p.add_member_control(b, "Isect. test", query_isect_test, "dropdown",
		"enums='none,sphere,fast,exact'"
	);

	p.add_decorator("Sampling", "text");
	p.add_member_control(b, "Strategy", sampling, "dropdown",
		"enums='global,global aligned,local,local aligned'"
	);
	p.add_member_control(b, "Frequency", sample_rate, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", 1e4 / data_extent[3])
	);

	p.add_decorator("Distance weight exponent\n(uncheck for linear weight)", "text");
	p.add_member_control(b, "Space", weight_exp.space, "value_slider",
		"min=0;max=10;ticks=true;log=true;w=180", ""
	);
	p.add_member_control(b, "", weight_exp.use_space, "check", "w=20");

	p.add_member_control(b, "Time", weight_exp.time, "value_slider",
		"min=0;max=10;ticks=true;log=true;w=180", ""
	);
	p.add_member_control(b, "", weight_exp.use_time, "check", "w=20");

	p.add_member_control(b, "Angle", weight_exp.angle, "value_slider",
		"min=0;max=10;ticks=true;log=true;w=180", ""
	);
	p.add_member_control(b, "", weight_exp.use_angle, "check", "w=20");

	if (p.begin_tree_node("Color scale", color_scale)) {
		p.add_member_control(b, "Base", color_scale.base, "dropdown", p.concat_enum_def(color_maps));
		p.add_member_control(b, "Highlight", color_scale.highlight);
		p.add_member_control(b, "Background", color_scale.background);

		p.add_member_control(b, "Layout", color_scale.layout, "dropdown", std::format(
			"enums='monotonic={},diverging={},symmetric={}'",
			static_cast<uint32_t>(ColorScale::Layout::monotonic),
			static_cast<uint32_t>(ColorScale::Layout::diverging),
			static_cast<uint32_t>(ColorScale::Layout::symmetric)
		));

		switch (color_scale.layout) {
		case ColorScale::Layout::monotonic:
		case ColorScale::Layout::diverging:
			p.add_member_control(b, "Begin", color_scale.domain[0], "value_slider",
				"ticks=true;min=-10;max=10;log=true"
			);
			p.add_member_control(b, "End", color_scale.domain[1], "value_slider",
				"ticks=true;min=-10;max=10;log=true"
			);
			if (color_scale.layout != ColorScale::Layout::diverging) break;
			p.add_member_control(b, "Midpoint", color_scale.midpoint, "value_slider",
				"ticks=true;min=-10;max=10;step=1"
			);
			break;
		case ColorScale::Layout::symmetric:
			p.add_member_control(b, "Midpoint", color_scale.midpoint, "value_slider",
				"ticks=true;min=-10;max=10;log=true"
			);
			p.add_member_control(b, "Range", color_scale.radius, "value_slider",
				"ticks=true;min=0;max=10;log=true"
			);
			break;
		}

		p.add_member_control(b, "Transform", color_scale.transform, "dropdown", std::format(
			"enums='linear={},logarithmic={},exponential={}'",
			static_cast<int>(ColorScale::Transform::Linear),
			static_cast<int>(ColorScale::Transform::Log),
			static_cast<int>(ColorScale::Transform::Pow)
		));
		switch (color_scale.transform) {
			case ColorScale::Transform::Linear: break;
			case ColorScale::Transform::Log:
				p.add_member_control(b, "Base", color_scale.log_base, "value_slider",
					"ticks=true;min=0.1;max=100;log=true"
				);
				break;
			case ColorScale::Transform::Pow:
				p.add_member_control(b, "Exponent", color_scale.exponent, "value_slider",
					"ticks=true;min=0.25;max=10;log=true"
				);
				break;
		}
		p.end_tree_node(color_scale);
	}
}

auto relation_vis::on_set (
	void* member,
	cgv::render::context& ctx,
	cgv::gui::provider& gui,
	color_map_manager const& colors
) -> UpdateFlags {
	auto const ptr = cgv::data::informed_ptr{member};
	if (ptr.points_to_one_of(data_var, relation.definition, query_isect_test, sampling))
		return UpdateFlag::shader_opts;
	if (ptr.points_to(filter)) return UpdateFlag::gui;
	if (!ptr.points_to_member_of(color_scale)) return 0;

	if (ptr.points_to(color_scale.base)) {
		color_scale.scheme = colors.get_color_scheme(color_scale.base.value);
		color_scale.scale->set_scheme(color_scale.scheme);
	}
	update_color_scale(ctx, colors);
	return ptr.points_to_one_of(color_scale.layout, color_scale.transform) ? UpdateFlag::gui : 0;
}

void relation_vis::update_color_scale (cgv::render::context& ctx, color_map_manager const& colors)
{
	// Configure color scale object.
	auto& scale = color_scale.scale;

	switch (color_scale.layout) {
	case ColorScale::Layout::monotonic:
		scale->set_diverging(false);
		break;
	case ColorScale::Layout::symmetric:
		color_scale.domain = {
			color_scale.midpoint - color_scale.radius,
			color_scale.midpoint + color_scale.radius};
	case ColorScale::Layout::diverging:
		scale->set_midpoint(color_scale.midpoint);
		scale->set_diverging(true);
		break;
	}

	auto min = color_scale.domain[0], max = color_scale.domain[1];
	scale->set_reversed(min > max);
	if (min > max) std::swap(min, max);
	scale->set_domain({min, max});

	scale->set_transform(color_scale.transform);
	scale->set_pow_exponent(color_scale.exponent);
	scale->set_log_base(color_scale.log_base);

	// Generate texture.
	std::array<cgv::rgb8, 256> samples;
	for (auto i = 0u; i < samples.size(); ++i) samples[i] = scale->map_value(
		min + 1.f/(samples.size() - 1) * (max - min) * i
	);
	cgv::data::data_format fmt {samples.size(), 1, cgv::type::info::TI_UINT8, cgv::data::CF_RGB};
	color_scale.texture.create(ctx, {&fmt, samples.data()}, 0);
}

void relation_vis::set_to_default (cgv::vec4 extent)
{
	radius.space = max_value(cgv::vec3{extent}) * 0.01f;
	radius.pre = radius.post = extent[3] * 0.01f;
	sample_rate = 1e3f / extent[3];
}

void relation_vis::set_shader_opts (cgv::render::shader_compile_options& opts) const
{
	opts.define_macro("RELATION_DATA_VAR", static_cast<uint32_t>(data_var));
	opts.define_macro("RELATION_COLOR_MAP_TEX", texture_idx::relation_color_map);
	opts.define_macro("RELATION_QUERY_ISECT_TEST", query_isect_test);
	opts.define_macro("RELATION_SAMPLING", sampling);
	if (data_var == DataVar::relation && !relation.definition.empty())
		opts.define_snippet("relation_def", relation.definition);
}

void relation_vis::set_uniforms (
	cgv::render::context&        c,
	cgv::render::shader_program& p
) const {
	auto qdir = static_cast<uint32_t>(query_dir);
	if (reverse_qdir) qdir |= 1<<31;
	p.set_uniform(c, "relation_query_dir", qdir);
	p.set_uniform(c, "relation_normalize", normalize);
	p.set_uniform(c, "relation_filter.value", static_cast<uint32_t>(filter));
	p.set_uniform(c, "relation_ref_traj", reference_trajectory);

	auto const r = radius;
	auto const min_cos = ::min_cos(cutoff_angle);
	p.set_uniform(c, "relation_radius", cgv::vec4{r.space, r.pre, r.post, min_cos});

	p.set_uniform(c, "relation_sample_rate", sample_rate);

	auto const wx = weight_exp;
	auto const sqr = [](auto x) {return x*x;};
	auto const r_time = fmax(r.pre, r.post);
	p.set_uniform(c, "relation_weight_norm", cgv::vec3{
		1 / (wx.use_space ? sqr(r.space) : r.space),
		1 / (wx.use_time ? sqr(r_time) : r_time),
		1 / (wx.use_angle ? 1 - min_cos : cgv::math::deg2rad(cutoff_angle))
	});
	auto constexpr lin_weight = std::numeric_limits<float>::quiet_NaN();
	p.set_uniform(c, "relation_weight_exp", cgv::vec3{
		wx.use_space ? wx.space : lin_weight,
		wx.use_time ? wx.time : lin_weight,
		wx.use_angle ? wx.angle : lin_weight
	});

	auto domain = color_scale.domain;
	if (domain[0] > domain[1]) std::swap(domain[0], domain[1]);
	p.set_uniform(c, "relation_color_domain", domain);
	p.set_uniform(c, "relation_highlight_color", color_scale.highlight);
	p.set_uniform(c, "relation_background_color", color_scale.background);
}

auto relation_vis::data_var_name () const -> std::string_view
{
	if (data_var == DataVar::relation) return relation.name;
	return data_var_names[static_cast<size_t>(data_var)];
}

auto get_reflection_traits (enum relation_vis::DataVar const&)
	-> cgv::reflect::enum_reflection_traits<enum relation_vis::DataVar>
{
	return {
		"none,"
		"relation,"
		"dbg_seg_t,"
		"dbg_velocity,"
		"dbg_index_xyz,"
		"dbg_index_t,"
		"dbg_signature,"
		"dbg_bucket_load,"
		"dbg_local_interval,"
		"dbg_num_cells,"
		"dbg_num_intervals,"
	};
}
