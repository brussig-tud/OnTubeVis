#pragma once

#include <variant>
#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/texture.h>
#include <cgv/media/transfer_function.h>

#include "util/gui.h"


class color_map_manager : public cgv::base::base {
public:
	using TransferFunctionPtr = std::shared_ptr<cgv::media::transfer_function>;

	/// Tagged union of supported color map types.
	using ColorMap = std::variant<
		cgv::media::continuous_color_scheme, // predefined
		TransferFunctionPtr // user defined
	>;

	/// clear all color maps (this will not update the texture)
	void clear();
	/// destruct the texture
	bool destruct(cgv::render::context& ctx);
	/// init the texture with all black values
	bool init(cgv::render::context& ctx);
	/// create the gui for this manager
	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
	/// return and clear the last action
	ActionType action_type();
	/// Get the currently edited color ramp or nullptr. Color schemes cannot be edited.
	TransferFunctionPtr* get_edited_transfer_function();
	/// Return the index of the color map selected for GUI editing. May return invalid indices.
	auto get_edit_index() const -> int { return edit_idx; };
	/// Return a view of all registered color maps.
	std::span<ColorMap> ref_color_maps() { return color_maps; }
	/// Return a readonly view of all registered color maps.
	std::span<const ColorMap> ref_color_maps() const { return color_maps; }
	/// The list of all color map names. Entries correspond to those in `ref_color_maps` (SoA).
	std::vector<std::string> const& ref_names() const { return color_map_names; }
	/// reference to the texture
	cgv::render::texture& ref_texture() { return tex; }
	/// Return the color map with the given index as a color scheme, converting it if necessary.
	auto get_color_scheme (size_t idx) const -> cgv::media::continuous_color_scheme;
	/// add a color map from outside of this manager
	void add_color_map(std::string name, ColorMap&&);
	/// Return the index of the color map with the given name, or -1 if the name is not registered.
	auto find_name(std::string const&) -> size_t;
	/// remove a color map by its name
	void remove_color_map_by_name(const std::string& name);
	/// update the color map texture to the contents of the color maps
	bool update_texture(cgv::render::context& ctx);

private:
	/// resolution (width) of the created texture
	static constexpr size_t discretization_resolution = 256;

	/// pointer to the base that uses this manager
	cgv::base::base_ptr base_ptr = nullptr;
	/// type of the last gui action
	ActionType last_action_type = ActionType::kUndefined;
	/// index of currently edited color map
	int edit_idx = -1;
	/// name of the new color map
	std::string new_name = "";
	/// list of all currently managed color maps
	std::vector<ColorMap> color_maps;
	/// The names of each registered color map. Entries correspond to `color_maps` (SoA).
	std::vector<std::string> color_map_names;
	/// 2d texture to store the sampled color map data
	cgv::render::texture tex = { "uint8[R,G,B]", cgv::render::TF_LINEAR, cgv::render::TF_LINEAR, cgv::render::TW_CLAMP_TO_EDGE, cgv::render::TW_CLAMP_TO_EDGE };
	/// handle gui changes
	void on_set(void* member_ptr);
	/// create a new color map with name from new_name and add it to the list, if name is not already in use
	void create_color_map();
	/// remove a color map by index
	void remove_color_map(const size_t index);
	/// set a color map to be edited
	void edit_color_map(const size_t index);
	/// create or update the actual texture object
	bool create_or_replace_texture(cgv::render::context& ctx, size_t width, size_t height, std::vector<cgv::rgb8>& data);
};
