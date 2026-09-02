#include "color_map_manager.h"

#include <utility>


namespace {
	/// Utility for constructing an invocable from multiple overloads, which can be used to branch
	/// on the type of an expression. This is an established pattern for variant visitors, see e.g.
	/// https://en.cppreference.com/cpp/utility/variant/visit2. Note that the arguments may be
	/// implicitely converted, add an `auto` overload to prevent this.
	template <class... T> struct TypeSwitch : T... { using T::operator()...; };
}


void color_map_manager::clear() {
	color_maps.clear();
}

bool color_map_manager::destruct(cgv::render::context& ctx) {
	if(tex.is_created())
		return tex.destruct(ctx);
	return true;
}

bool color_map_manager::init(cgv::render::context& ctx) {
	std::vector<cgv::rgb8> data(discretization_resolution, { 0 });
	return create_or_replace_texture(ctx, discretization_resolution, 1, data);
}

void color_map_manager::create_gui(cgv::base::base* bp, cgv::gui::provider& p) {
	base_ptr = bp;

	for(size_t i = 0; i < color_maps.size(); ++i) {
		auto const user_defined = std::holds_alternative<TransferFunctionPtr>(color_maps[i]);
		p.add_view("", color_map_names[i], "string", user_defined ? "w=136" : "", user_defined ? " " : "\n");
		if(!user_defined) continue;

		const std::string button_options = "w=20";
		connect_copy(p.add_button("@1edit", button_options, " ")->click, cgv::signal::rebind(this, &color_map_manager::edit_color_map, cgv::signal::_c<size_t>(i)));
		connect_copy(p.add_button("@9+", button_options)->click, cgv::signal::rebind(this, &color_map_manager::remove_color_map, cgv::signal::_c<size_t>(i)));
	}

	p.add_member_control(bp, "Name", new_name);
	connect_copy(p.add_button("Add Color Map")->click, cgv::signal::rebind(this, &color_map_manager::create_color_map));
}

ActionType color_map_manager::action_type() {
	return std::exchange(last_action_type, ActionType::kUndefined);
}

auto color_map_manager::get_edited_transfer_function() -> TransferFunctionPtr* {
	if(edit_idx < 0 || static_cast<size_t>(edit_idx) >= color_maps.size()) return nullptr;
	return &std::get<TransferFunctionPtr>(color_maps[static_cast<size_t>(edit_idx)]);
}

auto color_map_manager::get_color_scheme(size_t idx) const -> cgv::media::continuous_color_scheme {
	return std::visit(TypeSwitch{
		[](cgv::media::continuous_color_scheme const& cs) {return cs;},
		[](color_map_manager::TransferFunctionPtr const& tf) {
			// Because interpolation mode is ignored the scheme may look different.
			return cgv::media::continuous_color_scheme::linear(tf->get_color_points());
		},
	}, color_maps[idx]);
}

void color_map_manager::add_color_map(std::string name, ColorMap&& color_map) {
	color_maps.emplace_back(std::move(color_map));
	color_map_names.emplace_back(std::move(name));
}

auto color_map_manager::find_name(std::string const& name) -> size_t {
	auto const hit = std::find(color_map_names.begin(), color_map_names.end(), name);
	return hit == color_map_names.end() ? -1 : hit - color_map_names.begin();
}

void color_map_manager::remove_color_map_by_name(const std::string& name) {
	if (auto const idx = find_name(name); idx != -1) remove_color_map(idx);
}

bool color_map_manager::update_texture(cgv::render::context& ctx) {
	if(color_maps.size() == 0)
		return false;

	std::vector<cgv::rgb8> data;
	data.reserve(discretization_resolution * color_maps.size());

	#define QUANTIZE(SAMPLE) \
		for (auto i = 0u; i < discretization_resolution; ++i) \
			data.emplace_back((SAMPLE)(1.f/(discretization_resolution - 1) * i));

	for(const auto& color_map : color_maps)
		std::visit(TypeSwitch{
			[&](TransferFunctionPtr const& tf) {QUANTIZE(tf->get_mapped_color);},
			[&](cgv::media::continuous_color_scheme const& cs) {QUANTIZE(cs.interpolate);},
		}, color_map);

	#undef QUANTIZE
	return create_or_replace_texture(ctx, discretization_resolution, color_maps.size(), data);
}

void color_map_manager::on_set(void* member_ptr) {
	if(base_ptr)
		base_ptr->on_set(this);
}

void color_map_manager::create_color_map() {
	if(new_name.empty() || find_name(new_name) != -1) return;

	add_color_map(
		std::exchange(new_name, {}),
		TransferFunctionPtr{new cgv::media::transfer_function({
			{0.f, cgv::rgb{0.f}}, {1.f, cgv::rgb{1.f}}
		})}
	);

	last_action_type = ActionType::kConfigurationChange;
	if(base_ptr) {
		auto provider = dynamic_cast<cgv::gui::provider*>(&(*base_ptr));
		if(provider)
			provider->update_member(&new_name);
		base_ptr->on_set(this);
	}
}

void color_map_manager::remove_color_map(const size_t index) {
	if(index < color_maps.size()) {
		color_maps.erase(color_maps.begin() + index);
		color_map_names.erase(color_map_names.begin() + index);

		last_action_type = ActionType::kConfigurationChange;
		if(base_ptr)
			base_ptr->on_set(this);
	}
}

void color_map_manager::edit_color_map(const size_t index) {
	if(base_ptr) {
		last_action_type = ActionType::kEditRequest;
		if(edit_idx == index)
			edit_idx = -1;
		else
			edit_idx = static_cast<int>(index);
		base_ptr->on_set(this);
	}
}

bool color_map_manager::create_or_replace_texture(cgv::render::context& ctx, size_t width, size_t height, std::vector<cgv::rgb8>& data) {
	cgv::data::data_format df(discretization_resolution, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGB);
	cgv::data::data_view dv(&df, data.data());
	return tex.create(ctx, dv, 0);
}
