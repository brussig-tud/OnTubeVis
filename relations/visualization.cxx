#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES
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

[[nodiscard]] constexpr auto cos_cutoff (float cutoff_angle, float cos_exp) -> float
{
	return pow(.5 + .5*min_cos(cutoff_angle), cos_exp);
}

} // namespace


template <class T>
struct cgv::type::info::type_name<relation_vis::PseudoEnum<T>> {
	static const char* get_name() {return "enum";}
};


relation_vis::relation_vis()
{
	cos_cutoff = ::cos_cutoff(cutoff_angle, cos_exp);
}

void relation_vis::build_gui (
	cgv::gui::provider&             p,
	uint32_t                        num_trajectories,
	cgv::vec4                       data_extent,
	std::vector<std::string> const& color_maps
) {
	auto const b = dynamic_cast<cgv::base::base*>(&p);
	p.add_member_control(b, "Show", data_var, "dropdown", data_var_dropdown);
	connect_copy(
		p.add_button("Load relation")->click,
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
	p.add_member_control(b, "Sample rate", sample_rate, "value_slider",
		std::format("min=0;max={};ticks=true;log=true", 1e4 / data_extent[3])
	);
	p.add_member_control(b, "Directionality", cos_exp, "value_slider",
		"min=0;max=100;ticks=true;log=true"
	);
	p.add_member_control(b, "Cutoff angle in °", cutoff_angle, "value_slider",
		"min=0;max=180;ticks=true"
	);
	p.add_view("Cutoff weight", cos_cutoff);
	p.add_member_control(b, "Query intersection", query_isect_test, "dropdown",
		"enums='none,sphere,fast,exact'"
	);

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
	if (ptr.points_to_one_of(data_var, relation.definition, query_isect_test))
		return UpdateFlag::shader_opts;
	if (ptr.points_to(direction)) return UpdateFlag::gui;
	if (ptr.points_to_one_of(cos_exp, cutoff_angle)) {
		cos_cutoff = ::cos_cutoff(cutoff_angle, cos_exp);
		gui.update_member(&cos_cutoff);
	}
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
	case ColorScale::Layout::monotonic: break;
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
	for (auto i = 0; i < samples.size(); ++i) samples[i] = scale->map_value(
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
	if (data_var == DataVar::relation && !relation.definition.empty())
		opts.define_snippet("relation_def", relation.definition);
}

void relation_vis::set_uniforms (
	cgv::render::context&        c,
	cgv::render::shader_program& p
) const {
	auto min = color_scale.domain[0], max = color_scale.domain[1];
	if (min > max) std::swap(min, max);
	auto const r = radius;
	p.set_uniform(c, "relation_radius",           cgv::vec3{r.space, r.pre, r.post});
	p.set_uniform(c, "relation_sample_rate",      sample_rate                     );
	p.set_uniform(c, "relation_direction.value",  static_cast<uint32_t>(direction));
	p.set_uniform(c, "relation_ref_traj",         reference_trajectory            );
	p.set_uniform(c, "relation_normalize",        normalize                       );
	p.set_uniform(c, "relation_color_domain",     cgv::vec2{min, max}             );
	p.set_uniform(c, "relation_highlight_color",  color_scale.highlight           );
	p.set_uniform(c, "relation_background_color", color_scale.background          );
	p.set_uniform(c, "relation_cos_exp",          cos_exp                         );
	p.set_uniform(c, "relation_min_cos",          min_cos(cutoff_angle)           );
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
		"proximity,"
		"alignment,"
		"dbg_seg_t,"
		"dbg_velocity,"
		"dbg_index_xyz,"
		"dbg_index_t,"
		"dbg_signature,"
		"dbg_bucket_load,"
		"dbg_local_interval,"
		"dbg_num_cells,"
		"dbg_num_intervals,"
		"dbg_num_samples,"
	};
}
