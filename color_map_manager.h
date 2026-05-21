#pragma once

#include <memory>
#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/texture.h>
#include <cgv/media/transfer_function.h>

#include "gui_util.h"
#include "glyph_attribute_mapping.h"



class color_map_manager : public cgv::base::base {
public:
	/// container to store color map with name and additional info
	struct entry_type {
		/// the name of this color map
		std::string name;
		/// a transfer function acts as a color ramp allowing for later editing
		cgv::media::transfer_function ramp;
		/// true for user-defined color maps that can be deleted and edited.
		bool user_defined = false;
	};

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
	/// get the currently edited color ramp or nullptr
	cgv::media::transfer_function* get_edited_color_ramp();
	/// const reference to the list of color maps
	const std::vector<entry_type>& ref_color_maps() const { return color_maps; }
	/// reference to the texture
	cgv::render::texture& ref_texture() { return tex; }
	/// return a list of all color map names
	std::vector<std::string> get_names();
	/// add a color map from outside of this manager
	void add_color_map(const std::string& name, const cgv::media::transfer_function& cm, bool user_defined);
	/// remove a color map by its name
	void remove_color_map_by_name(const std::string& name);
	/// update the color map texture to the contents of the color maps
	bool update_texture(cgv::render::context& ctx);

private:
	/// resolution (width) of the created texture
	static const size_t discretization_resolution = 256;

	/// pointer to the base that uses this manager
	cgv::base::base_ptr base_ptr = nullptr;
	/// type of the last gui action
	ActionType last_action_type = ActionType::kUndefined;
	/// index of currently edited color map
	int edit_idx = -1;
	/// name of the new color map
	std::string new_name = "";
	/// list of all currently managed color maps
	std::vector<entry_type> color_maps;
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
