
#ifndef __STREAM_DATASET_H__
#define __STREAM_DATASET_H__


//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <cstring>
#include <array>
#include <memory>
#include <vector>
#include <map>
#include <optional>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include "./nodes.h"
#include "./glyphs.h"



//////
//
// Module namespace(s)
//

namespace stream {



//////
//
// Classes
//

struct Dataset
{
	struct Trajectory {
		Nodes nodes;
		std::vector<Glyphs> layers;

		explicit Trajectory (const unsigned num_layers)
			: layers(num_layers)
		{}
	};

	std::vector<Trajectory> trajs;
	unsigned num_layers;

	template <class Config>
	static Dataset construct (const Config &config)
	{
		Dataset events;
		events.num_layers = Config::num_layers;
		events.trajs.reserve(config.num_trajs);
		for (unsigned i=0; i<config.num_trajs; ++i)
			events.trajs.emplace_back(Config::num_layers);
		return events;
	}

	void set_node_stream (const unsigned traj_id, const Nodes &nodes) {
		trajs[traj_id].nodes = nodes;
	}
	void set_node_stream (const unsigned traj_id, Nodes &&nodes) {
		trajs[traj_id].nodes = std::move(nodes);
	}

	void set_glyph_stream (const unsigned traj_id, const unsigned layer, const Glyphs &glyphs) {
		trajs[traj_id].layers[layer] = glyphs;
	}
	void set_glyph_stream (const unsigned traj_id, const unsigned layer, Glyphs &&glyphs) {
		trajs[traj_id].layers[layer] = std::move(glyphs);
	}
};



//////
//
// Module namespace(s) close
//

// ::stream
}


#endif // ifndef __STREAM_DATASET_H__
