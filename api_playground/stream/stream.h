
#ifndef __LAYER_STREAM_H__
#define __LAYER_STREAM_H__


//////
//
// Includes
//

// C++ STL
#include <tuple>
#include <array>
#include <memory>
#include <vector>
#include <map>
#include <cassert>
#include <optional>

// CGV Framework
#include <cgv/math/fvec.h>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// OnTubeVis internals
#include "../util.h"

// Local includes
#include "../layer/properties.h"



//////
//
// Module namespace(s)
//

namespace stream {



//////
//
// Classes
//

struct NodeEvent
{
	OTV_HermiteNode node;
	std::optional<OTV_SegmentArclen> t_to_s;
	std::vector<OTV_Extrapolation> extrapolation;

	void stream (const unsigned traj_id) const {
		otv__stream_spline_node(traj_id, &node, t_to_s.has_value() ? &t_to_s.value() : nullptr);
	}
};

struct Nodes
{
	struct Event {
		float t;
		cgv::vec3 pos;
		cgv::vec3 tan;
	};

	struct Segment {
		const std::pair<unsigned, unsigned> nidx;
		const OTV_HermiteNode &n0;
		const OTV_HermiteNode &n1;
		const OTV_SegmentArclen &t_to_s;
		const std::vector<OTV_Extrapolation> &extrapol;

		Segment(
			const std::pair<unsigned, unsigned> &nidx, const OTV_HermiteNode &n0, const OTV_HermiteNode &n1,
			const OTV_SegmentArclen &t_to_s, const std::vector<OTV_Extrapolation> &extrapol
		)
			: nidx(nidx), n0(n0), n1(n1), t_to_s(t_to_s), extrapol(extrapol)
		{}

		[[nodiscard]] inline float t (const float time) const {
			return (time-n0.time) / (n1.time-n0.time);
		}
		[[nodiscard]] inline float s (const float t) const {
			return otv__eval_arclen(&t_to_s, t);
		}
		[[nodiscard]] inline float s_from_time (const float time) const {
			return s(t(time));
		}
	};
	std::vector<NodeEvent> nodes;
	std::map<float, unsigned> time_sequence;

	static Nodes compile (const std::vector<Event>& events)
	{
		// Static constant helpers
		static const OTV_SegmentArclen zero_alen{{
			otv__Vec4(0,0,0,0), otv__Vec4(0,0,0,0),
			otv__Vec4(0,0,0,0), otv__Vec4(0,0,0,0)
		}};

		// Sanity check
		const unsigned num_nodes = events.size();
		assert(num_nodes > 1 && "stream::Nodes::compile() needs at least two nodes to form a trajectory!");

		// prepare the fictitious -1st node for computing the initial extrapolation when no complete segment exists yet
		const auto fictitious_prev_node = [&] {
			const auto &tan = events.front().tan;
			const auto p = events.front().pos - tan;
			return OTV_HermiteNode{
				.time = events.front().t - 1,
				.position = otv__Vec3(p.x(), p.y(), p.z()), .tangent = otv__Vec3(tan.x(), tan.y(), tan.z())
			};
		}();
		const auto fictitious_prev_segment_sigma = -events.front().tan.length();
		constexpr float fictitious_alen_tolerance = 4*std::numeric_limits<float>::epsilon();

		// Prepare storage for the computed extrapolations
		constexpr unsigned num_extrapol_segs = 3;
		std::vector<OTV_Extrapolation> extrapol(num_extrapol_segs);

		// Compile the trajectory
		Nodes stream;
		stream.nodes.reserve(num_nodes);
		for (unsigned i=0; i<num_nodes; ++i)
		{
			const auto &event = events[i];
			const auto new_node = OTV_HermiteNode{
				.time     = event.t,
				.position = otv__Vec3(event.pos.x(), event.pos.y(), event.pos.z()),
				.tangent  = otv__Vec3(event.tan.x(), event.tan.y(), event.tan.z())
			};
			const auto &prev_node = i>0 ? stream.nodes.back().node : fictitious_prev_node;
			const auto cur_alen = otv__compute_arclen(
				&prev_node, &new_node,
				i>0 ? stream.nodes.back().t_to_s.value_or(zero_alen).coeffs[3].w
					: fictitious_prev_segment_sigma
			);
			assert(i>0 || std::abs(cur_alen.coeffs[3].w) < fictitious_alen_tolerance);
			otv__compute_extrapol(extrapol.data(), num_extrapol_segs, &prev_node, &new_node, &cur_alen);
			stream.nodes.emplace_back(
				new_node,
				i>0 ? std::make_optional(cur_alen) : std::nullopt,
				extrapol
			);
			stream.time_sequence.emplace_hint(stream.time_sequence.end(), events[i].t, i);
		}
		return stream;
	}

	[[nodiscard]] Segment segment (const unsigned idx) const {
		const unsigned nidx = idx+1;
		return Segment{
			std::make_pair(idx, nidx), nodes[idx].node, nodes[nidx].node,
			nodes[nidx].t_to_s.value(), nodes[nidx].extrapolation
		};
	}

	[[nodiscard]] inline unsigned segment_idx_containing_time (const float time) const
	{
		auto it = time_sequence.lower_bound(time);
		if (it == time_sequence.end() || it->second==nodes.size()-1)
			// Return the very last segment
			return (unsigned)nodes.size()-2;
		if (it == time_sequence.begin())
			// Return the very first segment
			return 0;
		// Return the containing segment
		return it->first!=time ? it->second-1 : it->second;
	}

	[[nodiscard]] inline Segment segment_containing_time (const float time) const {
		return segment(segment_idx_containing_time(time));
	}
};

/// A generic stream event defining the interface all concrete glyph stream events need to implement.
struct GlyphEvent {
	float s;

	virtual ~GlyphEvent() = default;
	virtual void stream (const unsigned traj_id, const unsigned layer) const = 0;
};

struct Glyphs {
	std::vector<std::unique_ptr<GlyphEvent>> glyphs;
	std::map<float, unsigned> time_sequence;
	std::map<float, unsigned> dist_sequence;
};



//////
//
// Module namespace(s) close
//

// ::stream
}


#endif // ifndef __LAYER_STREAM_H__
