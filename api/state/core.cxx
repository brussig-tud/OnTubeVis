
//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <memory>
#include <iostream>
#include <limits>

// 3rd party libs
#include <WGS84toCartesian.hpp>

// Public interface
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include <helper/glyphs.h>
#include "core.h"
#include "streaming.h"



//////
//
// Class implementations
//

////
// streaming_dataset

std::string streaming_dataset::name;
std::vector<OTV_LayerConfig> streaming_dataset::layers;
std::vector<VAttribSources> streaming_dataset::layer_sources;
std::vector<trajectory> streaming_dataset::trajectories;
std::unordered_map<uint32_t, unsigned> streaming_dataset::traj_id_map;
std::optional<latlon> streaming_dataset::georef;


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
		decltype(streaming_dataset::trajectories) trajectories;
		decltype(streaming_dataset::traj_id_map) traj_id_map;
		for (const auto &traj_setup : setup.trajs) {
			if (!traj_id_map.emplace(traj_setup.id, (unsigned)trajectories.size()).second)
				return notify_result(false);
			auto &traj = trajectories.emplace_back();
			traj.layers.resize(setup.layers.size());
			traj.radius = traj_setup.radius;
		}

		// commit
		otv_instance->start_new_streaming_session(setup);
		streaming_dataset::name = std::move(setup.name);
		streaming_dataset::layers = std::move(setup.layers);
		streaming_dataset::layer_sources = std::move(setup.layer_sources);
		streaming_dataset::trajectories = std::move(trajectories);
		streaming_dataset::traj_id_map = std::move(traj_id_map);
		streaming_dataset::georef = map_optional(setup.georef, [](const cgv::dvec2 &wgs84pos) {
			return latlon{wgs84pos.x(), wgs84pos.y()};
		});
		return notify_result(true);
	}
};

// stream a spline sample
struct stream_spline_node_command : public command
{
	const uint32_t traj_id;
	const OTV_HermiteNode node;
	const OTV_SegmentArclen arclen;
	const std::vector<OTV_Extrapolation> extrapol;

	// Constructors without extrapolation
	stream_spline_node_command(const uint32_t traj_id, const OTV_HermiteNode &node)
		: traj_id(traj_id), node(node), arclen()
	{}
	stream_spline_node_command(const uint32_t traj_id, const OTV_HermiteNode &node, const OTV_SegmentArclen &arclen)
		: traj_id(traj_id), node(node), arclen(arclen)
	{}

	// Constructors with extrapolation
	stream_spline_node_command(
		const uint32_t traj_id, const OTV_HermiteNode &node, std::vector<OTV_Extrapolation> &&extrapol
	)
		: traj_id(traj_id), node(node), arclen(), extrapol(std::move(extrapol))
	{}
	stream_spline_node_command(
		const uint32_t traj_id, const OTV_HermiteNode &node, const OTV_SegmentArclen &arclen,
		std::vector<OTV_Extrapolation> &&extrapol
	)
		: traj_id(traj_id), node(node), arclen(arclen), extrapol(std::move(extrapol))
	{}

	virtual const std::string& describe (void) final {
		const static std::string desc = "stream_spline_node";
		return desc;
	}

	virtual bool handle (void) final
	{
		otv_instance->client.service_push_spline_node(
			traj_id, otv::otv_client::convert_api_node_to_internal(node), (cgv::mat4*)&arclen,
			otv::otv_client::convert_api_extrapol_to_internal(extrapol)
		);

		static const irange_api empty_irange{};
		const auto idx_it = streaming_dataset::traj_id_map.find(traj_id);
		if (idx_it != streaming_dataset::traj_id_map.end())
		{
			auto &traj = streaming_dataset::trajectories[idx_it->second];
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
struct stream_glyphs_command : public command
{
	const uint32_t traj_id;
	const uint32_t layer;
	std::vector<OTV_GlyphData> glyphs;

	stream_glyphs_command(
		const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyphs, const uint32_t num_glyphs
	)
		: traj_id(traj_id), layer(layer), glyphs(glyphs, glyphs+num_glyphs)
	{}

	virtual const std::string& describe (void) final {
		const static std::string desc = "stream_glyphs";
		return desc;
	}

	virtual bool handle (void) final
	{
		if (layer >= streaming_dataset::layers.size())
		{
			std::clog << describe()<<": layer "<<layer<<" exceeds no. of configured layers!" << std::endl
					  << " - " << (streaming_dataset::layers.empty() ?
					  	  '\''+streaming_dataset::name+"' has no layers configured."
					  	: "must be in the range of [0.."+std::to_string(streaming_dataset::layers.size()-1)+"].")
					  << std::endl;
			return notify_result(false);
		}
		otv_instance->client.service_push_glyphs(
			traj_id, layer, otv::otv_client::convert_api_glyphs_to_internal(traj_id, layer, glyphs)
		);

		const auto idx_it = streaming_dataset::traj_id_map.find(traj_id);
		if (idx_it != streaming_dataset::traj_id_map.end())
		{
			// Preamble
			const auto &layer_conf = streaming_dataset::layers[layer];
			auto &traj = streaming_dataset::trajectories[idx_it->second];
			auto &glayer = traj.layers[layer];

			// Treat each glyph individually
			bool success = true;
			const auto num_glyphs = (unsigned)glyphs.size();
			std::clog << describe()<<": received "<<num_glyphs<<" glyph "<<(num_glyphs==1?"instance":"instances")
			          << std::endl;
			for (unsigned g=0; g<num_glyphs; ++g)
			{
				// Sanity check
				if (glyphs[g].s < 0) {
					std::clog << describe()<<':' << std::endl
					          << " ! ERROR: invalid arc length, should be s ≥ 0   !!!DISCARDING!!!" << std::endl;
					success = false;
					continue;
				}

				// Check monotonicity
				if (glayer.not_empty())
				{
					const float prevs = glayer.glyphs.back().s;
					if (prevs >= glyphs[g].s) {
						std::clog << describe()<<':' << std::endl
						          << " - traj #"<<traj_id<<".layer["<<layer<<"] <- "
					          		<<fmt_glyph_instance(layer_conf, glyphs[g]) << std::endl
						          << " ! ERROR: glyph arc length is out-of-order: "<<prevs<<"(old) ≥ "<<glyphs[g].s<<"(new)"
						          << "   !!!DISCARDING!!!" << std::endl;
						success = false;
						continue;
					}
				}

				// Enter new glyph
				const unsigned new_glyph_idx = (unsigned)glayer.glyphs.size();
				std::clog << describe()<<':' << std::endl;
				if (!check_glyph_instance_type(layer_conf.type, glyphs[g]))
					std::clog << " ! WARNING: provided glyph instance is potentially not of type '"
				          		<<otv__string_from_GlyphType(layer_conf.type)<<'\'' << std::endl;
				// - check overlap
				const auto gext =
					OTV_Vec2{glyphs[g].s, glyphs[g].s} + otv__instantiate_Glyph(idx_it->second, layer, &glyphs[g]);
				if (!glayer.glyphs.empty())
				{
					const float
						s_min = glayer.glyphs.back().s + otv__instantiate_Glyph(traj_id, layer, &glayer.glyphs.back()).y,
						dist = gext.x - s_min;
					if (dist < 0)
						std::clog << " ! WARNING: provided glyph instance will overlap with preceding glyph:" << std::endl
						          << "            "<<fmt_glyph_instance(layer_conf, glayer.glyphs.back())
						          << ", dist="<<dist << std::endl;
				}
				// - commit
				glayer.glyphs.emplace_back(glyphs[g]);
				std::clog << " - traj #"<<traj_id<<", layer["<<layer<<"] <- "
			          		<<fmt_glyph_instance(layer_conf, glyphs[g]) << std::endl;

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
			}

			// Done!
			return notify_result(success);
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
	const auto &traj = streaming_dataset::trajectories[streaming_dataset::traj_id_map.at(traj_id)];
	const auto &lcfg = streaming_dataset::layers[layer];
	switch (lcfg.type)
	{
		// Plot-likes (they all take infinitely small space, i.e. they're point-like)
		case OTV_GlyphType::SurfaceColor:
		case OTV_GlyphType::LinePlot:
			return {0, 0};

		// Discrete glyphs
		case OTV_GlyphType::Circle:
		{
			// Apply mapping windows
			const auto &attr_sources = streaming_dataset::layer_sources[layer];
			const auto radius_src = get_vattrib_source(
				attr_sources, VAttrib::Radius, {0, 1}, {0, 1}
			);
			OTV_CircleData mapped_data = *(OTV_CircleData*)data;
			mapped_data.radius = map_attrib_value(radius_src.in, radius_src.out, mapped_data.radius);

			// Instantiate
			return otv__instantiate_Circle(traj.radius, (OTV_CircleInfo*)&lcfg.static_params, &mapped_data);
		}
		case OTV_GlyphType::Rect:
		{
			// Apply mapping windows
			const auto &attr_sources = streaming_dataset::layer_sources[layer];
			const auto width_src = get_vattrib_source(
				attr_sources, VAttrib::Width, {0, 2}, {0, 2}
			);
			const auto height_src = get_vattrib_source(
				attr_sources, VAttrib::Height, {0, 2}, {0, 2}
			);
			OTV_RectangleData mapped_data = *(OTV_RectangleData*)data;
			mapped_data.width = map_attrib_value(width_src.in, width_src.out, mapped_data.width);
			mapped_data.height = map_attrib_value(height_src.in, height_src.out, mapped_data.height);

			// Instantiate
			return otv__instantiate_Rectangle(traj.radius, (OTV_RectangleInfo*)&lcfg.static_params, &mapped_data);
		}
		case OTV_GlyphType::IsoscelesTriangle:
		{
			// Apply mapping windows
			const auto &attr_sources = streaming_dataset::layer_sources[layer];
			const auto width_src = get_vattrib_source(
				attr_sources, VAttrib::Width, {0, 2}, {0, 2}
			);
			const auto height_src = get_vattrib_source(
				attr_sources, VAttrib::Height, {0, 2}, {0, 2}
			);
			const auto orientation_src = get_vattrib_source(
				attr_sources, VAttrib::Orientation, {-180, 180}, {-180, 180}
			);
			OTV_IsoscelesTriangleData mapped_data = *(OTV_IsoscelesTriangleData*)data;
			mapped_data.width = map_attrib_value(width_src.in, width_src.out, mapped_data.width);
			mapped_data.height = map_attrib_value(height_src.in, height_src.out, mapped_data.height);
			mapped_data.orientation = map_attrib_value(orientation_src.in, orientation_src.out, mapped_data.orientation);

			// Instantiate
			return otv__instantiate_IsoscelesTriangle(
				traj.radius, (OTV_IsoscelesTriangleInfo*)&lcfg.static_params, &mapped_data
			);
		}
		case OTV_GlyphType::SignBlob:
			return otv__instantiate_SignBlob(
				traj.radius, (OTV_SignBlobInfo*)&lcfg.static_params, (OTV_SignBlobData*)data
			);

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

OTV_API void otv__stream_spline_node_and_extrapol (
	const uint32_t traj_id, const OTV_HermiteNode *node, const OTV_SegmentArclen *arclen,
	const OTV_Extrapolation *extrapol
){
	// Compile and submit command
	auto extrapolations = std::vector(
		extrapol, extrapol+otv_instance->client.num_extrapol_segments
	);
	const auto nn_cmd = arclen ?
		  std::make_shared<stream_spline_node_command>(traj_id, *node, *arclen, std::move(extrapolations))
		: std::make_shared<stream_spline_node_command>(traj_id, *node, std::move(extrapolations));
	command_stream::push(nn_cmd);

	/* we don't wait for the result... */
}

OTV_API OTV_SegmentArclen otv__compute_arclen (
	const OTV_HermiteNode *node0, const OTV_HermiteNode *node1, const float sigma
){
	const cgv::mat4 alen = otv::arclen::compute_single_t_to_s<float>(
		*(cgv::vec3*)(float*)&node0->position, *(cgv::vec3*)(float*)&node0->tangent,
		*(cgv::vec3*)(float*)&node1->position,*(cgv::vec3*)(float*)&node1->tangent, sigma
	);
	return *(const OTV_SegmentArclen*)&alen;
}

OTV_API float otv__eval_arclen (const OTV_SegmentArclen *s, const float t) {
	return otv::arclen::eval(*(cgv::mat4*)s->coeffs, t);
}

OTV_API OTV_Vec3 otv__latlon_height_to_cartesian (const double latitude, const double longitude, const double height)
{
	const auto &georef = streaming_dataset::georef.value();
	const auto mercator = wgs84::toCartesian(
		georef, latlon{latitude, longitude}
	);
	return {(float)mercator[0], float(height), -(float)mercator[1]};
}

OTV_API void otv__compute_extrapol (
	OTV_Extrapolation *out, const uint32_t num, const OTV_HermiteNode *ref_node0, const OTV_HermiteNode *ref_node1,
	const OTV_SegmentArclen *ref_arclen
){
	// Delegate computation to internal facilities
	const auto extrapol = otv::extrapol::compute_path(
		num, node_attribs::from_api_node(*ref_node0, 0, cgv::vec4(0,0,0,0)),
		node_attribs::from_api_node(*ref_node1, 0, cgv::vec4(0,0,0,0)),
		*(cgv::mat4*)ref_arclen
	);

	// Convert to API data structures
	for (unsigned i=0; i<num; i++) {
		out[i] = extrapol[i].into_api_extrapol();
	}
}

OTV_API void otv__stream_glyph (const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyph_data)
{
	// Compile and submit command
	const auto ng_cmd = std::make_shared<stream_glyphs_command>(traj_id, layer, glyph_data, 1);
	command_stream::push(ng_cmd);

	/* we don't wait for the result... */
}

OTV_API void otv__stream_glyphs (
	const uint32_t traj_id, const uint32_t layer, const OTV_GlyphData *glyphs_data, const uint32_t num_glyphs
){
	// Compile and submit command
	const auto ngs_cmd = std::make_shared<stream_glyphs_command>(traj_id, layer, glyphs_data, num_glyphs);
	command_stream::push(ngs_cmd);

	/* we don't wait for the result... */
}
