#include "otv_client.h"

#include "gpumem/ring_buffer.inl"
#include "render/state.h"

// Public streaming API
#include <thread>
#include <cgv/gui/gui_driver.h>
#include <OnTubeVis/OnTubeVis.h>


namespace otv {

OTV_ColorMap colormap_name_to_api_enum (const std::string &name)
{
	const std::string name_lower = cgv::utils::to_lower(name);

	// Sequential color maps
	if (name_lower == "acton")
		return OTV_ColorMap::Acton;
	if (name_lower == "bamako")
		return OTV_ColorMap::Bamako;
	if (name_lower == "batlow")
		return OTV_ColorMap::Batlow;
	if (name_lower == "batlowk")
		return OTV_ColorMap::BatlowK;
	if (name_lower == "batloww")
		return OTV_ColorMap::BatlowW;
	if (name_lower == "bilbao")
		return OTV_ColorMap::Bilbao;
	if (name_lower == "buda")
		return OTV_ColorMap::Buda;
	if (name_lower == "davos")
		return OTV_ColorMap::Davos;
	if (name_lower == "devon")
		return OTV_ColorMap::Devon;
	if (name_lower == "glasgow")
		return OTV_ColorMap::Glasgow;
	if (name_lower == "grayc")
		return OTV_ColorMap::GrayC;
	if (name_lower == "hawaii")
		return OTV_ColorMap::Hawaii;
	if (name_lower == "imola")
		return OTV_ColorMap::Imola;
	if (name_lower == "lajolla")
		return OTV_ColorMap::Lajolla;
	if (name_lower == "lapaz")
		return OTV_ColorMap::Lapaz;
	if (name_lower == "lipari")
		return OTV_ColorMap::Lipari;
	if (name_lower == "navia")
		return OTV_ColorMap::Navia;
	if (name_lower == "nuuk")
		return OTV_ColorMap::Nuuk;
	if (name_lower == "oslo")
		return OTV_ColorMap::Oslo;
	if (name_lower == "rainbow")
		return OTV_ColorMap::Rainbow;
	if (name_lower == "tokyo")
		return OTV_ColorMap::Tokyo;
	if (name_lower == "turbo")
		return OTV_ColorMap::Turbo;
	if (name_lower == "turku")
		return OTV_ColorMap::Turku;

	// Diverging color maps
	if (name_lower == "bam")
		return OTV_ColorMap::Bam;
	if (name_lower == "berlin")
		return OTV_ColorMap::Berlin;
	if (name_lower == "broc")
		return OTV_ColorMap::Broc;
	if (name_lower == "cork")
		return OTV_ColorMap::Cork;
	if (name_lower == "lisbon")
		return OTV_ColorMap::Lisbon;
	if (name_lower == "managua")
		return OTV_ColorMap::Managua;
	if (name_lower == "roma")
		return OTV_ColorMap::Roma;
	if (name_lower == "tofino")
		return OTV_ColorMap::Tofino;
	if (name_lower == "vanimo")
		return OTV_ColorMap::Vanimo;
	if (name_lower == "vik")
		return OTV_ColorMap::Vik;

	// Unknown/unsupported
	cgv::gui::get_gui_driver()->message(
		"Config refers to unknown or unsupported color map: "+name+"\nOnTubeVis will now most likely crash."
	);
	assert(false && "INTERNAL LOGIC ERROR: Unknown color map name!");
	return OTV_ColorMap::UndefinedColormap;
}

OTV_InterpolationMode interpolation_attribval_to_api_enum (const float value)
{
	if (value < 1/3.f)
		return OTV_InterpolationMode::Nearest;
	if (value < 2/3.f)
		return OTV_InterpolationMode::Linear;
	return OTV_InterpolationMode::Cubic;
}

void otv_client::new_session (void) {
	session.reset();
}

bool otv_client::new_session_if_not_in_setup (void) {
	if (!session.is_setup_pending()) {
		session.reset();
		return true;
	}
	return false;
}

void otv_client::begin_setup (const std::string &name) {
	session.start_setup(otv__create_VisSetup(name.c_str()));
}

void otv_client::commit_session (void)
{
	// Transition out of setup state
	OTV_VisSetupHandle setup = session.finish_setup();

	// Add trajectories to setup
	for (unsigned i=0; i<trajectories.size(); i++) {
		auto &traj = trajectories[i];
		traj.id = otv__add_trajectory(setup, data->datasets[0].trajs[i].med_radius);
	}

	// Add layers to setup
	const auto &colormaps = render.visualizations.front().variables->ref_color_map_names();
	const auto &vis = render.visualizations.front();
	for (unsigned i=0; i<vis.config.layer_configs.size(); i++)
	{
		const auto &lcfg = vis.config.layer_configs[i];
		const auto &lmappings = vis.manager.ref_glyph_attribute_mappings()[i];
		switch (lcfg.shape_ptr->type())
		{
			case GT_COLOR: {
				constexpr unsigned vattrib_idx__color=1, metaattrib_idx__interpolate=0;
				auto colormap = colormap_name_to_api_enum(
					colormaps[lmappings.get_color_map_indices()[vattrib_idx__color]]
				);
				auto interpolate= interpolation_attribval_to_api_enum(
					lmappings.ref_attrib_mapping_values()[metaattrib_idx__interpolate].w()
				);
				OTV_LayerConfig cfg {
					OTV_GlyphType::SurfaceColor,
					-1, // unused
					otv__construct_SurfaceColorInfo(colormap, interpolate)
				};
				otv__add_layer(setup, &cfg);
				break;
			}
			case GT_LINE_PLOT: {
				break;
			}
			case GT_RECTANGLE: {
				break;
			}
			case GT_SIGN_BLOB: {
				break;
			}

			case GT_TRIANGLE:
			case GT_CIRCLE:
			case GT_WEDGE:
			case GT_ARC_FLAT:
			case GT_ARC_ROUNDED:
			case GT_DROP:
			case GT_STAR:
			case GT_TEMPORAL_HEAT_MAP:
				cgv::gui::get_gui_driver()->message(
					"Config contains unimplemented glyph type: "+lcfg.shape_ptr->name()+"\n"
					"OnTubeVis will now most likely crash."
				);
				assert(false && "INTERNAL LOGIC ERROR: Unimplemented glyph type in layer configuration!");

			default:
				assert(false && "INTERNAL LOGIC ERROR: Unknown glyph type in layer configuration!");
		}
	}

	// Request starting the session
	std::thread init_runner([this, setup] {
		assert(otv__start_vis_session(setup) && "FATAL ERROR - otv_client: Failed to initialize streaming session!");
		this->session.signal_init_done();
		otv__free_VisSetup(setup);
	});
	init_runner.detach(); // let session init continue in the background
}

void otv_client::update ()
{
	// wait for the session to be ready if necessary (adds a very small constant overhead every update once the session
	// actually is ready)
	session.wait_init_ready();

	// extend all trajectories based on the animation time, emulating streaming
	for (auto &traj : trajectories) {
		// add new nodes, and thereby new segments
		for (
			auto &node_idx = traj.node_idcs.begin;
			node_idx < traj.node_idcs.end;
			++node_idx
		) {
			// only add data up to the current playback time
			if (data->timestamps[node_idx] > render.style.max_t) {
				break;
			}

			auto &render_traj {*render.try_get_trajectory(traj.id)};

			// the first node of each trajectory does not create a segment, so there is no arc
			// length parametrization
			const cgv::mat4 *t_to_s {nullptr};

			if (! render_traj.is_empty()) {
				t_to_s = &arclen_data.t_to_s.at(traj.segment_idx);
			}

			// append a node, potentially creating a new segment
			auto col = data->colors[node_idx];
			render.enqueue_node(
				traj.id,
				{
					{data->positions[node_idx], data->radii[node_idx]},
					{col.R(), col.G(), col.B(), 1},
					data->tangents[node_idx],
					{data->timestamps[node_idx], 0, 0, 0}
				},
				t_to_s
			);

			if (t_to_s == nullptr) {
				continue;
			}

			// if the node created a segment, enqueue its glyphs and advance the index
			render.for_each_active_glyph_layer([&](const auto layer_idx, const auto &layer) {
				const auto data  {glyphs[layer_idx].attribs.data.begin()};
				const auto begin {glyphs[layer_idx].ranges.at(traj.segment_idx).i0};
				const auto end   {glyphs[layer_idx].ranges.at(traj.segment_idx).end()};

				render_traj.enqueue_glyphs(layer_idx, ro_range{
					data + render_traj.glyph_to_attrib_count(layer_idx, begin),
					data + render_traj.glyph_to_attrib_count(layer_idx, end)
				});
			});

			++traj.segment_idx;
		}
	}
}


} // namespace otv
