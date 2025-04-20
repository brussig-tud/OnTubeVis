
#ifndef __STREAM_NODES_H__
#define __STREAM_NODES_H__


//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <vector>
#include <map>
#include <optional>

// CGV Framework
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/math/inv.h>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// OnTubeVis internals
#include "../util.h"

// Local includes
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

struct NodeInstance
{
	OTV_HermiteNode node;
	std::optional<OTV_SegmentArclen> t_to_s;
	std::vector<OTV_Extrapolation> extrapolation;

	[[nodiscard]] const OTV_SegmentArclen& alen (void) const {
		return t_to_s.value();
	}

	[[nodiscard]] NodeInstance transformed (const cgv::mat4 &trans, const cgv::mat4 &tangents_trans) const
	{
		// Transform the actual/real position and tangent
		NodeInstance result;
		result.node = transform_hermite_node(node, trans, tangents_trans);

		// Transform the extrapolation
		result.extrapolation.reserve(extrapolation.size());
		for (const auto &extraol : extrapolation) {
			result.extrapolation.push_back({
				.node = transform_hermite_node(extraol.node, trans, tangents_trans),
				.arclen = extraol.arclen,
			});
		}

		// Copy over t_to_s
		result.t_to_s = t_to_s;

		// Done!
		return result;
	}

	void stream (const unsigned traj_id) const {
		otv__stream_spline_node_and_extrapol(
			traj_id, &node, t_to_s.has_value() ? &t_to_s.value() : nullptr, extrapolation.data()
		);
	}
};

struct Nodes
{
	typedef std::map<float, unsigned> Sequence;

	struct EventData {
		float t;
		cgv::vec3 pos;
		cgv::vec3 tan;
	};
	struct Event : public EventData {
		std::optional<EventData> extrapol_ref;
	};

	struct Segment
	{
		const std::pair<unsigned, unsigned> nidx;
		const OTV_HermiteNode &n0;
		const OTV_HermiteNode &n1;
		const OTV_SegmentArclen &t_to_s;
		const std::vector<OTV_Extrapolation> &extrapols;

		Segment(
			const std::pair<unsigned, unsigned> &nidx, const OTV_HermiteNode &n0, const OTV_HermiteNode &n1,
			const OTV_SegmentArclen &t_to_s, const std::vector<OTV_Extrapolation> &extrapol
		)
			: nidx(nidx), n0(n0), n1(n1), t_to_s(t_to_s), extrapols(extrapol)
		{}

		[[nodiscard]] inline float t (const float time) const {
			return (time-n0.time) / (n1.time-n0.time);
		}

		[[nodiscard]] inline float time_from_t (const float t) const {
			return n0.time + t*(n1.time-n0.time);
		}

		[[nodiscard]] inline float s (const float t) const {
			return otv__eval_arclen(&t_to_s, t);
		}

		[[nodiscard]] inline float s_from_time (const float time) const {
			return s(t(time));
		}

		[[nodiscard]] Segment extrapol (const unsigned idx) const {
			return Segment{
				std::make_pair(idx-1, idx), idx==0?n1:extrapols[idx-1].node, extrapols[idx].node,
				extrapols[idx].arclen, {}
			};
		}

		[[nodiscard]] inline unsigned extrapol_idx_containing_time (const float time) const
		{
			unsigned e=0;
			for (; e<(unsigned)extrapols.size(); ++e) {
				const auto &extrapol = extrapols[e];
				if (time <= extrapol.node.time)
					return e;
			}
			return e-1;
		}

		[[nodiscard]] inline Segment extrapol_containing_time (const float time) const {
			return extrapol(extrapol_idx_containing_time(time));
		}
	};
	std::vector<NodeInstance> nodes;
	Sequence time_sequence;

	template <class Config>
	static Nodes compile (const Config &config, const std::vector<Event>& events)
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
		const auto fictitious_prev_node = events.front().extrapol_ref.has_value() ?
			  [&] {
			  	const auto &ref = events.front().extrapol_ref.value();
			  	return OTV_HermiteNode{
			  		.time = ref.t, .position = otv__Vec3(ref.pos.x(), ref.pos.y(), ref.pos.z()),
			  		.tangent = otv__Vec3(ref.tan.x(), ref.tan.y(), ref.tan.z())
			  	};
			  }()
			: [&] {
			  	const auto &tan = events.front().tan;
			  	const auto p = events.front().pos - tan;
			  	return OTV_HermiteNode{
			  		.time = events.front().t - 1,
			  		.position = otv__Vec3(p.x(), p.y(), p.z()), .tangent = otv__Vec3(tan.x(), tan.y(), tan.z())
			  	};
			  }();
		const auto fictitious_prev_segment_sigma = -(
			events.front().pos - reinterpret_cast<const cgv::vec3&>(fictitious_prev_node.position)
		).length();
		constexpr float fictitious_alen_tolerance = 4*std::numeric_limits<float>::epsilon();

		// Prepare storage for the computed extrapolations
		std::vector<OTV_Extrapolation> extrapol(config.extrapol_length);

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
			otv__compute_extrapol(
				extrapol.data(), config.extrapol_length, &prev_node, &new_node, &cur_alen
			);
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

	[[nodiscard]] Nodes transformed (
		const cgv::mat4 &trans, const cgv::mat4 &tangents_trans, const float time_shift=0
	) const
	{
		// Transform all nodes
		Nodes result;
		result.nodes.reserve(nodes.size());
		for (const auto &node : nodes) {
			auto &transformed_node = result.nodes.emplace_back(node.transformed(trans, tangents_trans));
			transformed_node.node.time += time_shift;
		}

		// Apply time shift to sequence view, if any
		if (time_shift==0)
			result.time_sequence = time_sequence;
		else
			for (const auto &elem : time_sequence)
				result.time_sequence.emplace_hint(
					result.time_sequence.end(), elem.first+time_shift, elem.second
				);

		// Done!
		return result;
	}

	[[nodiscard]] inline Nodes transformed (const cgv::mat4 &trans, const float time_shift=0) const {
		const auto tangents_trans = cgv::math::inv(cgv::math::transpose(trans));
		return transformed(trans, tangents_trans, time_shift);
	}

	[[nodiscard]] Glyphs adapt_glyphs (const Glyphs &glyphs, const float time_shift=0) const
	{
		Glyphs result;
		result.glyphs.reserve(glyphs.glyphs.size());
		for (const auto &event : glyphs.time_sequence)
		{
			// Prelude
			const auto cur_glyph_idx = (unsigned)result.glyphs.size();
			const float glyph_time = event.first + time_shift;

			// Create (to-be-adapted) glyph and keep reference
			auto &glyph = result.glyphs.emplace_back(glyphs.glyphs[cur_glyph_idx]);
			result.time_sequence.emplace_hint(result.time_sequence.end(), glyph_time, cur_glyph_idx);

			// Find segment for the (potentially shifted) glyph
			const auto seg = segment(segment_idx_containing_time(glyph_time));

			// Adapt arclength position
			if (seg.n0.time <= glyph_time) {
				glyph.glyph.s = seg.s_from_time(glyph_time);
				result.dist_sequence.emplace_hint(result.dist_sequence.end(), glyph.glyph.s, cur_glyph_idx);

			}
			else {
				glyph.glyph.s = glyph_time - seg.n0.time;
				result.dist_sequence.emplace_hint(result.dist_sequence.end(), glyph.glyph.s, cur_glyph_idx);
			}
		}

		// Done!
		return result;
	}
};

struct NodesStreamer
{
	const unsigned traj_id;
	const Nodes &nodes;
	float time_cursor = -std::numeric_limits<float>::infinity();

	NodesStreamer (const Nodes &nodes, const unsigned traj_id)
		: traj_id(traj_id), nodes(nodes)
	{}

	void stream_up_to_time (float time)
	{
		// Sanity check
		assert(time > time_cursor);

		// Iterate until target time
		auto it_cursor = nodes.time_sequence.upper_bound(time_cursor);
		while (it_cursor != nodes.time_sequence.end() && it_cursor->first <= time) {
			nodes.nodes[it_cursor->second].stream(traj_id);
			++it_cursor;
		}
		time_cursor = time;
	}
};



//////
//
// Module namespace(s) close
//

// ::stream
}


#endif // ifndef __STREAM_NODES_H__
