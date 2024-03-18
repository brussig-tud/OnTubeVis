#include <sstream>
#include "glyph_shapes.h"
#include "color_legend_manager.h"


color_legend_manager::color_legend_manager(cgv::app::application_plugin_base &owner) : owner(owner) {
	std::stringstream str;
	for (unsigned i=0; i<legends.size(); i++) {
		str.flush();
		str << "Legend_layer" << i;
		legends[i] = owner.register_overlay<cgv::app::color_map_legend>(str.str());
		legends[i]->set_size(legends[i]->get_rectangle().size + cgv::ivec2(16, 0));
		legends[i]->set_num_ticks(5);
		legends[i]->set_visibility(false);
		legends[i]->set_alignment(cgv::app::overlay::AO_START, cgv::app::overlay::AO_END);
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
	// throw out old configuration
	clear();

	// collect in-use color maps
	if (layers.empty())
		return;
	int voffset = -1;
	const auto attrib_names = dataset.get_attribute_names(),
	           ltype_names = glyph_type_registry::display_names();
	for (const auto &layer : layers)
	{
		// identify whether the layer uses a color map
		const auto &[id, cmi, ai] = [&layer]() -> std::tuple<int, int, int> {
			const auto cmi_list = layer.get_color_map_indices();
			const auto ai_list = layer.get_attrib_indices();
			for (unsigned i=0; i<cmi_list.size(); i++)
				if (cmi_list[i] > -1 && ai_list[i] > -1)
					return {i, cmi_list[i], ai_list[i]};
			return {-1, -1, -1};
		}();
		if (cmi > -1)
		{
			// find out the layer name and glyph/plot type to include in the legend title
			std::stringstream stitle;
			stitle << ltype_names[layer.get_shape_ptr()->type()];
			stitle << " -- "<<attrib_names[ai];

			// set up a legend for the found color mapping
			const auto &colormaps = color_map_mgr.ref_color_maps();
			const auto &cmc = colormaps[cmi];
			auto &new_legend = legends[num_active];
			new_legend->set_title(stitle.str());

			// set the color map
			const auto& ranges = layer.ref_attrib_mapping_values()[id];
			// for color-mapped attributes an "invalid" output range of 1 to 0 indicates reversed color mapping
			if(ranges.z() > ranges.w()) {
				// copy the color map to take over settings
				cgv::render::color_map cm = cmc.cm;
				cm.clear();
				// now re-add the mirrored control points
				for(const auto& p : cmc.cm.ref_color_points())
					cm.add_color_point(1.0f - p.first, p.second);
				for(const auto& p : cmc.cm.ref_opacity_points())
					cm.add_opacity_point(1.0f - p.first, p.second);

				new_legend->set_color_map(ctx, cm);
			} else {
				new_legend->set_color_map(ctx, cmc.cm);
			}
			new_legend->set_range({ranges.x(), ranges.y()});
			new_legend->set_margin({-1, voffset});
			new_legend->set_visibility(true);
			voffset += new_legend->get_rectangle().h()-1;
			num_active++;
		}
	}
}
