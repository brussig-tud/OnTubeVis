
//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <memory>
#include <iostream>
#include <limits>

// Public interface
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include <helper/glyphs.h>
#include "on_tube_vis.h"
#include "streaming.h"



//////
//
// Class implementations
//

////
// on_tube_vis

/*std::mutex on_tube_vis_api::init_mtx;
bool on_tube_vis_api::init_pending = true;
std::condition_variable on_tube_vis_api::init_cv;*/
bool on_tube_vis_api::running = false;
std::string on_tube_vis_api::ds_name;
std::vector<OTV_LayerConfig> on_tube_vis_api::layers;
std::vector<trajectory> on_tube_vis_api::trajectories;
std::unordered_map<uint32_t, unsigned> on_tube_vis_api::traj_id_map;


////
// Streaming commands used by this unit

// start a new session
struct new_session_command : public command
{
	new_session_command(const VisSetup &setup) : setup(setup) {}

	VisSetup setup;

	virtual const std::string& describe (void) final {
		const static std::string desc = "new_session";
		return desc;
	}

	virtual bool handle (void) final
	{
		// prepare new "dataset"
		decltype(on_tube_vis_api::trajectories) trajectories;
		decltype(on_tube_vis_api::traj_id_map) traj_id_map;
		for (const auto &traj_setup : setup.trajs) {
			if (!traj_id_map.emplace(traj_setup.id, (unsigned)trajectories.size()).second)
				return notify_result(false);
			auto &traj = trajectories.emplace_back();
			traj.layers.resize(setup.layers.size());
			traj.radius = traj_setup.radius;
		}

		// commit
		/*on_tube_vis_api::ds_name = std::move(setup.name);
		on_tube_vis_api::layers = std::move(setup.layers);
		on_tube_vis_api::trajectories = std::move(trajectories);
		on_tube_vis_api::traj_id_map = std::move(traj_id_map);*/

		otv_instance->start_new_streaming_session(setup);
		on_tube_vis_api::ds_name = std::move(setup.name);
		on_tube_vis_api::layers = std::move(setup.layers);
		on_tube_vis_api::trajectories = std::move(trajectories);
		on_tube_vis_api::traj_id_map = std::move(traj_id_map);;
		return notify_result(true);
	}
};

// stream a spline sample
struct stream_spline_node_command : public command
{
	const uint32_t traj_id;
	const OTV_HermiteNode node;
	const OTV_SegmentArclen arclen;

	stream_spline_node_command(const uint32_t traj_id, const OTV_HermiteNode &node)
		: traj_id(traj_id), node(node), arclen()
	{}

	stream_spline_node_command(const uint32_t traj_id, const OTV_HermiteNode &node, const OTV_SegmentArclen &arclen)
		: traj_id(traj_id), node(node), arclen(arclen)
	{}

	virtual const std::string& describe (void) final {
		const static std::string desc = "stream_spline_node";
		return desc;
	}

	virtual bool handle (void) final
	{
		static const irange_api empty_irange{};
		const auto idx_it = on_tube_vis_api::traj_id_map.find(traj_id);
		if (idx_it != on_tube_vis_api::traj_id_map.end())
		{
			auto &traj = on_tube_vis_api::trajectories[idx_it->second];
			if (traj.nodes.size() > 0)
			{
				// Check monotonicity
				const float prevt = traj.nodes.back().time;
				if (prevt >= node.time)
				{
					std::clog << describe()<<':'<<std::endl<<" - traj #"<<traj_id<<" <- pos("
					          	<<node.position.x<<", "<<node.position.y<<", "<<node.position.z<<") tan("
					          	<<node.tangent.x<<", "<<node.tangent.y<<", "<<node.tangent.z<<")" << std::endl
					          << " ! WARNING: node timing is out-of-order: "<<node.time<<"(new) ≤ "<<prevt<<"(old)"
					          << "   !!!DISCARDING!!!" << std::endl;
					return notify_result(false);
				}

				// Enter node and the new segment it spawns
				traj.nodes.emplace_back(node);
				const unsigned new_segid = (unsigned)traj.segment_arclens.size();
				traj.segment_arclens.emplace_back(arclen);
				std::clog << describe()<<':'<<std::endl<<" - traj #"<<traj_id<<" <- pos("
				          	<<node.position.x<<", "<<node.position.y<<", "<<node.position.z<<") tan("
				          	<<node.tangent.x<<", "<<node.tangent.y<<", "<<node.tangent.z<<")" << std::endl
				          << " - arc length: ["<<arclen.coeffs[0].x<<".."<<arclen.coeffs[1].x<<".."
				          	<<arclen.coeffs[2].x<<".."<<arclen.coeffs[3].x<<".."<<arclen.coeffs[3].w<<"]" << std::endl;

				// Update glyph references
				const float smin = traj.segment_arclens.back().coeffs[0].x,
				            smax = traj.segment_arclens.back().coeffs[3].w;
				for (unsigned l=0; l<(unsigned)traj.layers.size(); l++)
				{
					// Preamble
					auto &layer = traj.layers[l];
					auto &range = layer.ranges.emplace_back(irange_api{std::numeric_limits<unsigned>::max(), 0});

					// Check if previously linked glyph still overlaps with this new segment
					if (layer.latest_refed.exists() && layer.latest_refed.s >= smin)
					{
						assert(layer.ranges.size() > 1 &&
						       "INTERNAL LOGIC ERROR: Including previously linked glyph even though no previous segment"
						       " exists!");
						auto &prev_range =
							layer.latest_refed.exists() ? layer.ranges[layer.latest_refed.idx] : empty_irange;
						assert(layer.ranges.size()-2 == (size_t)layer.latest_refed.idx &&
						       "INTERNAL LOGIC ERROR: Index of glyph linked to previous segment and stored index of"
						       " of latest linked glyph do not match!");
						const unsigned prev_idx = prev_range.i0+prev_range.n - 1;
						std::clog << " - LINKING: layer["<<l<<"] <- overlapping glyph (idx "<<prev_idx
						          << "), s ≤ .."<<layer.latest_refed.s<<" from previous segment" << std::endl;
						layer.latest_refed.idx = new_segid;
						range.i0 = prev_idx;
						range.n++;
					}

					// Try to link currently unreferenced glyphs
					if (layer.earliest_unrefed.exists())
					{
						for (unsigned i=layer.earliest_unrefed.idx; i<(unsigned)layer.glyphs.size(); i++)
						{
							const auto &glyph = layer.glyphs[i];
							const auto gext = OTV_Vec2{glyph.s, glyph.s} + otv__instantiate_Glyph(
								idx_it->second, l, &glyph
							);
							assert(gext.x == layer.earliest_unrefed.s &&
							      "INTERNAL LOGIC ERROR: arc length stored in earliest_unrefed does not match!");
							if (gext.x <= smax)
							{
								std::clog << " - LINKING: layer["<<l<<"] <- previously orphaned glyph (idx "
								          	<<layer.earliest_unrefed.idx<<"), s=["<<gext.x<<".."<<gext.y<<']'
								          << std::endl;
								range.i0 = std::min(range.i0, i);
								range.n++;
								layer.latest_refed.idx = new_segid;
								layer.latest_refed.s = gext.y;
								if (i<(unsigned)layer.glyphs.size()-1) {
									layer.earliest_unrefed.idx = i+1;
									const auto &next_glyph = layer.glyphs[layer.earliest_unrefed.idx];
									layer.earliest_unrefed.s =
										next_glyph.s + otv__instantiate_Glyph(idx_it->second, l, &next_glyph).x;
								}
								else
									layer.earliest_unrefed.invalidate();
							}
							else
								break;
						}
					}
					assert(layer.ranges.size() == traj.segment_arclens.size() &&
						  "INTERNAL LOGIC ERROR: Length of segment arclength and glyph index buffers diverged!");
				}
			}
			else
			{
				assert(traj.segment_arclens.empty() &&
				      "INTERNAL LOGIC ERROR: Arc length buffer not empty as first node is added!");
				traj.nodes.emplace_back(node);
				std::clog << describe()<<':'<<std::endl<<" - traj #"<<traj_id<<" <- pos("
				          	<<node.position.x<<", "<<node.position.y<<", "<<node.position.z<<") tan("
				          	<<node.tangent.x<<", "<<node.tangent.y<<", "<<node.tangent.z<<")" << std::endl
				          << " - <first sample, no arc length>" << std::endl;
			}
			return notify_result(true);
		}
		std::clog << describe()<<": trajectory id #"<<traj_id<<" is invalid!" << std::endl;
		return notify_result(false);
	}
};

// stream a spline sample
struct stream_glyph_command : public command
{
	const uint32_t traj_id;
	const uint32_t layer;
	const OTV_GlyphData glyph;

	stream_glyph_command(const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData &glyph)
		: traj_id(traj_id), layer(layer), glyph(glyph)
	{}

	virtual const std::string& describe (void) final {
		const static std::string desc = "stream_glyph";
		return desc;
	}

	virtual bool handle (void) final
	{
		if (layer >= on_tube_vis_api::layers.size())
		{
			std::clog << describe()<<": layer "<<layer<<" exceeds no. of configured layers!" << std::endl
					  << " - " << (on_tube_vis_api::layers.empty() ?
					  	  '\''+on_tube_vis_api::ds_name+"' has no layers configured."
					  	: "must be in the range of [0.."+std::to_string(on_tube_vis_api::layers.size()-1)+"].")
					  << std::endl;
			return notify_result(false);
		}
		const auto idx_it = on_tube_vis_api::traj_id_map.find(traj_id);
		if (idx_it != on_tube_vis_api::traj_id_map.end())
		{
			// Preamble
			const auto &layer_conf = on_tube_vis_api::layers[layer];
			auto &traj = on_tube_vis_api::trajectories[idx_it->second];
			auto &glayer = traj.layers[layer];

			// Sanity check
			if (glyph.s < 0) {
				std::clog << describe()<<':' << std::endl
				          << " ! ERROR: invalid arc length, should be s ≥ 0   !!!DISCARDING!!!" << std::endl;
				return notify_result(false);
			}

			// Check monotonicity
			if (glayer.not_empty())
			{
				const float prevs = glayer.glyphs.back().s;
				if (prevs >= glyph.s) {
					std::clog << describe()<<':' << std::endl
					          << " - traj #"<<traj_id<<".layer["<<layer<<"] <- "
					          	<<fmt_glyph_instance(layer_conf, glyph) << std::endl
					          << " ! ERROR: glyph arc length is out-of-order: "<<prevs<<"(old) ≥ "<<glyph.s<<"(new)"
					          << "   !!!DISCARDING!!!" << std::endl;
					return notify_result(false);
				}
			}

			// Enter new glyph
			const unsigned new_glyph_idx = (unsigned)glayer.glyphs.size();
			std::clog << describe()<<':' << std::endl;
			if (!check_glyph_instance_type(layer_conf.type, glyph))
				std::clog << " ! WARNING: provided glyph instance is potentially not of type '"
							  <<otv__string_from_GlyphType(layer_conf.type)<<'\'' << std::endl;
			// - check overlap
			const auto gext =
				OTV_Vec2{glyph.s, glyph.s} + otv__instantiate_Glyph(idx_it->second, layer, &glyph);
			if (!glayer.glyphs.empty()) {
				const float
					s_min = glayer.glyphs.back().s + otv__instantiate_Glyph(traj_id, layer, &glayer.glyphs.back()).y,
					dist = gext.x - s_min;
				if (dist < 0)
					std::clog << " ! WARNING: provided glyph instance will overlap with preceding glyph:" << std::endl
					          << "            "<<fmt_glyph_instance(layer_conf, glayer.glyphs.back())
					          << ", dist="<<dist << std::endl;
			}
			// - commit
			glayer.glyphs.emplace_back(glyph);
			std::clog << " - traj #"<<traj_id<<", layer["<<layer<<"] <- "
			          	<<fmt_glyph_instance(layer_conf, glyph) << std::endl;

			// Update glyph references
			// - output helper function
			const auto output_orphan_info = [&gext, &traj]() {
				std::clog << " - ORPHANING: trajectory:s_max="<<(
				          	traj.segment_arclens.empty() ? 0 : traj.segment_arclens.back().coeffs[3].w
				          )<<" < ["<<gext.x<<".."<<gext.y<<"]=s:glyph" << std::endl;
			};
			// (we check many redundant conditions, more than would be needed for each branch, to assert that we don't
			//  have any logic errors anywhere)
			if (   !glayer.earliest_unrefed.exists()
			    && (traj.segment_arclens.empty() || gext.x > traj.segment_arclens.back().coeffs[3].w))
			{
				// This is the first orphaned glyph on that trajectory and layer for which no segment exists (yet)
				output_orphan_info();
				glayer.earliest_unrefed.idx = new_glyph_idx;
				glayer.earliest_unrefed.s = gext.x;
			}
			else if (    glayer.earliest_unrefed.exists()
			         && (traj.segment_arclens.empty() || gext.x > traj.segment_arclens.back().coeffs[3].w))
				// This is another orphaned glyph on that trajectory and layer for which no segment exists (yet)
				output_orphan_info();
			else if (   !glayer.earliest_unrefed.exists()
			         && !traj.segment_arclens.empty() && gext.x <= traj.segment_arclens.back().coeffs[3].w)
			{
				// This glyph lies somewhere on the trajectory - link it to its segment
				const unsigned search_start = glayer.latest_refed.exists() ? glayer.latest_refed.idx : 0;
				bool found = false;
				for (unsigned i=search_start; i<(unsigned)traj.segment_arclens.size(); i++) {
					const auto &alen = traj.segment_arclens[i];
					if (gext.y >= alen.coeffs[0].x && gext.x <= alen.coeffs[3].w) {
						std::clog << " - LINKING: to segment "<<i<<", alen=["
						          	<<alen.coeffs[0].x<<".."<<alen.coeffs[3].w<<']'
						          << std::endl;
						auto &range = glayer.ranges[i];
						range.i0 = std::min(range.i0, new_glyph_idx);
						range.n++;
						glayer.latest_refed.idx = i;
						glayer.latest_refed.s = gext.y;
						found = true; // for assertion (see below)
						continue; // <-- no breaking - the glyph could overlap more segments
					}
					if (gext.y < alen.coeffs[0].x)
						break; // <-- now it's safe to break
				}
				assert(found && "INTERNAL LOGIC ERROR: In-range glyph was not assigned to a segment!");
			}
			else
				assert(false &&
				      "INTERNAL LOGIC ERROR: Uncovered condition for deciding whether to link or orphan a new glyph!");
			return notify_result(true);
		}
		std::clog << describe()<<':' << std::endl << " ! ERROR: trajectory id #"<<traj_id<<" is invalid!" << std::endl;
		return notify_result(false);
	}
};



//////
//
// Functions
//

OTV_API bool otv__start_vis_session (OTV_VisSetupHandle vis_setup)
{
	// Retrieve actual vis setup object
	auto &vs = *(VisSetup*)vis_setup;

	// Validity checks
	if (vs.trajs.size() < 1) {
		std::cerr << "otv__start_vis_session: unable to start new visualization session!" << std::endl
		          << " - '"+vs.name+"' does not contain any trajectories." << std::endl;
		return false;
	}

	// Compile and submit command
	auto ns_cmd = std::make_shared<new_session_command>(vs);
	std::clog << "otv__start_vis_session: requesting to start visualization of '"<<vs.name<<"'." << std::endl;
	command_stream::push(ns_cmd);

	// Check result
	const bool result = ns_cmd->fetch_result();
	std::clog << "otv__start_vis_session: implementation returned '"<<result<<"' for request to start visualizing '"
	          << vs.name<<"'." << std::endl;
	return result;
}

OTV_API OTV_Vec2 otv__instantiate_Glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *data)
{
	const auto &traj = on_tube_vis_api::trajectories[on_tube_vis_api::traj_id_map.at(traj_id)];
	const auto &lcfg = on_tube_vis_api::layers[layer];
	switch (lcfg.type)
	{
		// Plot-likes (they all take infinitely small space, i.e. they're point-like)
		case OTV_GlyphType::SurfaceColor:
		case OTV_GlyphType::LinePlot:
			return {0, 0};

		// Discrete glyphs
		case OTV_GlyphType::Rect:
			return otv__instantiate_Rectangle(traj.radius, (OTV_RectangleInfo*)&lcfg.static_params,
			                                  (OTV_RectangleData*)data);
		case OTV_GlyphType::SignBlob:
			return otv__instantiate_SignBlob(traj.radius, (OTV_SignBlobInfo*)&lcfg.static_params,
			                                 (OTV_SignBlobData*)data);
		default:
			/* do_nothing() */;
	}
	// This should never happen
	assert(false && "INTERNAL LOGIC ERROR: glyph layer stores an invalid type!");
	return {}; // Prevent MSVC complaining about there being a code path that doesn't return anything
}

OTV_API void otv__stream_spline_node (
	const uint32_t traj_id, const OTV_HermiteNode *node, const OTV_SegmentArclen *arclen
){
	// Compile and submit command
	const auto nn_cmd = arclen ?
		  std::make_shared<stream_spline_node_command>(traj_id, *node, *arclen)
		: std::make_shared<stream_spline_node_command>(traj_id, *node);
	command_stream::push(nn_cmd);

	/* we don't wait for the result... */
}

OTV_API void otv__stream_glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyph_data)
{
	// Compile and submit command
	const auto nn_cmd = std::make_shared<stream_glyph_command>(traj_id, layer, *glyph_data);
	command_stream::push(nn_cmd);

	/* we don't wait for the result... */
}
