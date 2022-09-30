#pragma once

#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/render_types.h>
#include <cgv/render/texture.h>
#include <cgv/render/color_map.h>

#include "gui_util.h"
#include "glyph_attribute_mapping.h"



class color_map_manager : public cgv::base::base, public cgv::render::render_types {
public:
	/// container to store color map with name and additional info
	struct color_map_container {
		/// the name of this color map
		std::string name = "";
		/// the actual color map object
		cgv::render::color_map cm;
		/// whether this color map is custom or a default choice
		bool custom = false;

		color_map_container() {}
		color_map_container(const std::string& name) : name(name) {}
	};
protected:
	/// pointer to the base that uses this manager
	cgv::base::base_ptr base_ptr;
	/// type of the last gui action
	ActionType last_action_type = AT_NONE;
	/// index of currently edited color map
	int edit_idx = -1;
	/// name for the new color map
	std::string new_name = "";
	/// list of all currently managed color maps
	std::vector<color_map_container> color_maps;
	/// resolution (width) of the created texture
	unsigned res = 256;
	/// 2d texture to store the sampled color map data
	cgv::render::texture tex;
	/// handle gui changes
	void on_set(void* member_ptr);
	/// create a new color map with name from new_name and add it to the list, if name is not already in use
	void create_color_map();
	/// remove a colro map by index
	void remove_color_map(const size_t index);
	/// set a color map to be edited
	void edit_color_map(const size_t index);
	/// create or update the actual texture object
	bool create_or_replace_texture(cgv::render::context& ctx, unsigned w, unsigned h, std::vector<uint8_t>& data);

public:
	/// construct a new color map manager
	color_map_manager() : base_ptr(nullptr) {}
	~color_map_manager() {}
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
	/// get the index of the currently edited color map
	int edit_index() { return edit_idx; }
	/// reference to the list of color maps
	std::vector<color_map_container>& ref_color_maps() { return color_maps; }
	/// reference to the texture
	cgv::render::texture& ref_texture() { return tex; }
	/// return a list of all color map names
	std::vector<std::string> get_names();
	/// add a color map from outside of this manager
	void add_color_map(const std::string& name, const cgv::render::color_map& cm, bool custom);
	/// remove a color map by its name
	void remove_color_map_by_name(const std::string& name);
	/// update the color map texture to the contents of the color maps
	bool update_texture(cgv::render::context& ctx);
};
