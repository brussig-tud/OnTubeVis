#pragma once

#include <vector>

#include <cgv/base/base.h>
#include <cgv/base/group.h>
#include <cgv/gui/provider.h>
#include <cgv/media/color_scale.h>
#include <cgv/render/context.h>
#include <cgv/render/texture.h>
#include <cgv_overlay/color_scale_legend.h>

#include "traj_loader.h"
#include "glyph_attribute_mapping.h"
#include "color_map_manager.h"


/// a manager automatically creating and maintaining color legend overlays for each mapped color scale in the given
/// on-tube visualization layers.
/// @todo special handling for temporal heatmaps since they can have up to for different sub-ranges each requiring
///       their own legend.
class color_legend_manager
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

	/// reference to the application plugin using the manager
	cgv::base::group &owner;

	/// the (fixed) pool of color scale legend overlays. currently set to 4 since we can have max. 4 on-tube glyph layers.
	std::array<cgv::overlay::color_scale_legend_ptr, 4> legends;

	/// the number of active legends
	unsigned num_active;


public:

	/// construct a new color map manager
	color_legend_manager(cgv::base::group &owner);

	/// construct a new color map manager, taking the owning applicatino plugin as a pointer
	inline color_legend_manager(cgv::base::group *owner) : color_legend_manager(*owner) {}

	// the destructor
	~color_legend_manager() {}

	/// disable / un-use all managed color map legends in the pool
	void clear (void);

	/// compose the legend for the given glyph layer configuration.
	void compose (
		const traj_dataset<float> &dataset, const color_map_manager &color_map_mgr,
		const std::vector<glyph_attribute_mapping> &layers
	);

	/// create the gui for this manager
	//void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
