#include <sstream>
#include "glyph/shapes.h"
#include "color_legend_manager.h"

#include "load/traj_loader.h"


color_legend_manager::color_legend_manager(cgv::base::group &owner) : owner(owner) {
	std::stringstream str;
	for (unsigned i=0; i<legends.size(); i++) {
		str.flush();
		str << "Legend_layer" << i;
		legends[i] = owner.create_and_append_child<cgv::overlay::color_scale_legend>(str.str());
		legends[i]->set_size(legends[i]->get_rectangle().size + cgv::ivec2(16, 0));
		legends[i]->set_num_ticks(5);
		legends[i]->set_visibility(false);
		legends[i]->set_alignment(cgv::overlay::Alignment::kStart, cgv::overlay::Alignment::kEnd);
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
	const traj_dataset<float> &dataset, const color_map_manager &color_map_mgr,
	const std::vector<glyph_attribute_mapping> &layers
){
	// throw out old configuration
	clear();

	// collect in-use color maps
	if (layers.empty())
		return;
	int voffset = -1;
	const auto attrib_names = dataset.get_attribute_names();
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
			stitle << glyph_shape::display_name(layer.get_shape()->type());
			stitle << " -- " <<attrib_names[ai];

			// set up a legend for the found color mapping
			const auto &color_map = color_map_mgr.ref_color_maps()[cmi];
			auto &new_legend = legends[num_active];
			new_legend->set_title(stitle.str());

			// set the color map
			const auto& mapping = layer.ref_attrib_mapping_values()[id];
			auto color_scale = std::make_shared<cgv::media::continuous_color_scale>();
			// convert the color ramp of type transfer function to a scheme and use this in a continuous color scale as
			// this allows us to set the domain without altering the original color ramp
			color_scale->set_scheme(cgv::media::continuous_color_scheme::linear(color_map.ramp.quantize_color(256)));
			color_scale->set_domain(mapping.input_range);
			// for color-mapped attributes an "invalid" output range of 1 to 0 indicates reversed color mapping
			color_scale->set_reversed(mapping.output_range.x() > mapping.output_range.y());
			new_legend->set_color_scale(color_scale);
			
			new_legend->set_margin({-1, voffset});
			new_legend->set_visibility(true);
			voffset += new_legend->get_rectangle().h()-1;
			num_active++;
		}
	}
}
