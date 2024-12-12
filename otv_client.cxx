#include "otv_client.h"

#include "gpumem/ring_buffer.inl"
#include "render/state.h"

// Public streaming API
#include <thread>
#include <cgv/gui/gui_driver.h>
#include <OnTubeVis/OnTubeVis.h>


namespace otv {

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
	const auto &vis = render.visualizations.front();
	for (unsigned i=0; i<vis.config.layer_configs.size(); i++)
	{
		const auto &layer = vis.config.layer_configs[i];
		switch (layer.shape_ptr->type())
		{
			case GT_COLOR: {
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
					"Config containes unimplemented glyph type: "+layer.shape_ptr->name()+"\nOnTubeVis will now crash."
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
