#pragma once

#include <vector>

#include <cgv/base/base.h>
#include <cgv/gui/provider.h>
#include <cgv/render/context.h>
#include <cgv/render/texture.h>
#include <cgv/render/color_map.h>

#include "gui_util.h"
#include "glyph_attribute_mapping.h"



class color_legend_manager /*: public cgv::base::base, public cgv::render::render_types*/
{
public:

	/// 2D vector type
	typedef cgv::math::fvec<float, 2> vec2;

	/// 3D vector type
	typedef cgv::math::fvec<float, 3> vec3;

	/// 4D vector type
	typedef cgv::math::fvec<float, 4> vec4;

	/// rgb color type
	typedef cgv::media::color<float, cgv::media::RGB> Color;


protected:
	/// pointer to the base that uses this manager
	//cgv::base::base_ptr base_ptr;
	/// list of all currently managed color maps
	std::vector<cgv::render::color_map*> legends;

public:
	/// construct a new color map manager
	color_legend_manager() /*: base_ptr(nullptr)*/ {}
	~color_legend_manager() {}
	/// clear all color legends from the manager
	void clear();
	/// create the gui for this manager
	//void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
	/// reference to the list of color maps
	const std::vector<cgv::render::color_map*>& ref_legends (void) const { return legends; }
};
