
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
		const OTV_HermiteNode &n0;
		const OTV_HermiteNode &n1;
		const OTV_SegmentArclen &t_to_s;

		Segment(const OTV_HermiteNode &n0, const OTV_HermiteNode &n1, const OTV_SegmentArclen &t_to_s)
			: n0(n0), n1(n1), t_to_s(t_to_s)
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

		const unsigned num_nodes = events.size();
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
			stream.nodes.emplace_back(
				new_node,
				i>0 ? std::make_optional(otv__compute_arclen(
					&stream.nodes.back().node, &new_node,
					stream.nodes.back().t_to_s.value_or(zero_alen).coeffs[3].w
				)) : std::nullopt
			);
			stream.time_sequence.emplace_hint(stream.time_sequence.end(), events[i].t, i);
		}
		return stream;
	}

	[[nodiscard]] Segment segment (const unsigned idx) const {
		const unsigned nidx = idx+1;
		return Segment{nodes[idx].node, nodes[nidx].node, nodes[nidx].t_to_s.value()};
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
