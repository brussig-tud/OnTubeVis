#include <sstream>
#include "color_legend_manager.h"

color_legend_manager::color_legend_manager(cgv::app::application_plugin_base &owner) : owner(owner) {
	std::stringstream str;
	for (unsigned i=0; i<legends.size(); i++) {
		str.flush();
		str << "Legend_layer" << i;
		legends[i] = owner.register_overlay<cgv::app::color_map_legend>(str.str());
		legends[i]->set_visibility(false);
		std::cout.flush();
	}
	std::cout.flush();
}

void color_legend_manager::clear() {
	for (auto &l : legends)
		l->set_visibility(false);
	num_active = 0;
}

void color_legend_manager::compose (
	cgv::render::context &ctx, const traj_dataset<float> &dataset,
	const color_map_manager &color_map_mgr, const std::vector<glyph_attribute_mapping> &layers
){
	clear();
	if (layers.empty())
		return;

	int voffset = -3;
	const auto attrib_names = dataset.get_attribute_names();
	for (const auto &layer : layers)
	{
		// identify whether the layer uses a color map
		const auto &[id, cmi, ai] = [&layer]() {
			const auto cmi_list = layer.get_color_map_indices();
			const auto ai_list = layer.get_attrib_indices();
			for (unsigned i=0; i< cmi_list.size(); i++) {
				if (cmi_list[i] > -1 && ai_list[i] > -1)
					return std::make_tuple((int)i, cmi_list[i], ai_list[i]);
			}
			return std::make_tuple(-1, -1, -1);
		}();
		if (cmi > -1)
		{
			// set up a legend for the found color mapping
			const auto &colormaps = color_map_mgr.ref_color_maps();
			const auto &cm = colormaps[cmi];
			auto &new_legend = legends[num_active];
			new_legend->set_color_map(ctx, cm.cm);
			new_legend->set_title(attrib_names[ai]);
			{ const auto &ranges = layer.ref_attrib_mapping_values()[id];
			  new_legend->set_range({ranges.x(), ranges.y()}); }
			new_legend->set_overlay_margin({-3, voffset});
			new_legend->set_visibility(true);
			voffset += new_legend->get_overlay_size().y()-3;
			num_active++;
		}
	}
}
