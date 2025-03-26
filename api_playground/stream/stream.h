
#ifndef __LAYER_STREAM_H__
#define __LAYER_STREAM_H__


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

// CGV Framework
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/math/inv.h>

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
// Functions
//

OTV_HermiteNode transform_hermite_node (
	const OTV_HermiteNode &node, const cgv::mat4 &trans, const cgv::mat4 &tangents_trans
){
	const auto pos = trans * cgv::vec4(cgv::vec3(3, (float*)&node.position), 1);
	const auto tan = tangents_trans * cgv::vec4(cgv::vec3(3, (float*)&node.tangent), 0);
	return {
		.time = node.time,
		.position = otv__Vec3(pos.x()/pos.w(), pos.y()/pos.w(), pos.z()/pos.w()),
		.tangent = *(OTV_Vec3*)&tan
	};
}



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
	std::map<float, unsigned> time_sequence;

	/*inline Nodes (const Nodes &other) = default;
	inline Nodes (Nodes &&other) = default;
	Nodes& operator= (const Nodes &other) = default;
	Nodes& operator= (Nodes &&other) = default;*/

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
};

struct GlyphInstance
{
	OTV_GlyphData glyph;

	GlyphInstance(const GlyphInstance &other) = default;
	GlyphInstance& operator= (const GlyphInstance &other) {
		memcpy(&glyph, &other.glyph, sizeof(OTV_GlyphData)); // <- WTF C++??? Why do I have to resort to this???
		return *this;
	};

	explicit inline GlyphInstance (const OTV_GlyphData &glyph) : glyph(glyph) {}
};

struct Glyphs
{
	std::vector<GlyphInstance> glyphs;
	std::map<float, unsigned> time_sequence;
	std::map<float, unsigned> dist_sequence;

	void add (const float t, const OTV_GlyphData &glyph)
	{
		// Sanity checks
		assert(time_sequence.upper_bound(t) == time_sequence.end());
		assert(dist_sequence.upper_bound(glyph.s) == dist_sequence.end());

		// Insert into stream
		const unsigned idx = (unsigned)glyphs.size();
		glyphs.emplace_back(glyph);
		time_sequence.emplace_hint(time_sequence.end(), t, idx);
		dist_sequence.emplace_hint(dist_sequence.end(), glyph.s, idx);
	}
};

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
	static Dataset construct (const Config &config) {
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

struct EventSequence
{
	struct Event {
		virtual ~Event () = default;
		virtual void stream (void) const = 0;
	};

	struct NodeEvent : public Event
	{
		unsigned traj_id;
		NodeInstance node;

		NodeEvent(const unsigned traj_id, const NodeInstance &node)
			: traj_id(traj_id), node(node)
		{}

		void stream (void) const override {
			otv__stream_spline_node_and_extrapol(
				traj_id, &node.node, node.t_to_s.has_value() ? &node.t_to_s.value() : nullptr,
				node.extrapolation.data()
			);
		}
	};

	struct GlyphEvent : public Event
	{
		unsigned traj_id;
		unsigned layer;
		GlyphInstance glyph;

		GlyphEvent(const unsigned traj_id, const unsigned layer, const GlyphInstance &glyph)
			: traj_id(traj_id), layer(layer), glyph(glyph)
		{}

		void stream (void) const override {
			otv__stream_glyph(traj_id, layer, &glyph.glyph);
		}
	};

	typedef std::vector<std::unique_ptr<Event>> EventList;
	std::map<float, EventList> events;

	template <class EventType>
	bool insert (const float time, EventType &&event)
	{
		// Move input event to heap
		auto new_event = std::make_unique<EventType>(std::move(event));

		// Insert into sequence according to provided time
		auto it = events.find(time);
		if (it == events.end()) {
			EventList new_event_list; new_event_list.emplace_back(std::move(new_event));
			events.emplace(time, std::move(new_event_list));
			return false; // report no collision
		}
		it->second.emplace_back(std::move(new_event));
		return true; // report collision
	}

	static EventSequence compile (const Dataset &dataset)
	{
		EventSequence events;

		// Insert trajectory by trajectory
		for (unsigned traj_id=0; traj_id<dataset.trajs.size(); ++traj_id)
		{
			// Convenience shorthand
			const auto &traj = dataset.trajs[traj_id];

			// Insert node events
			for (const auto &[time, node_id] : traj.nodes.time_sequence) {
				const auto &node = traj.nodes.nodes[node_id];
				assert(time == node.node.time);
				events.insert(time, NodeEvent(traj_id, node));
			}

			// Insert glyph events
			for (unsigned l=0; l<dataset.num_layers; ++l) {
				const auto &layer = traj.layers[l];
				for (const auto &[time, glyph_id] : layer.time_sequence) {
					const auto &glyph = layer.glyphs[glyph_id];
					events.insert(time, GlyphEvent(traj_id, l, glyph));
				}
			}
		}
		return events;
	}

	void play_back (void) const
	{
		// Log reference time point of playback start
		const auto time_start = std::chrono::high_resolution_clock::now();

		// Loop until events run out
		auto it = events.begin();
		if (it != events.end()) do
		{
			// Figure out when next event will be and sleep until then
			const auto event_time = time_start + std::chrono::microseconds(static_cast<long>(1000000.*double(it->first)));
			std::this_thread::sleep_until<std::chrono::high_resolution_clock>(event_time);

			// Stream event(s)
			for (const auto &event : it->second)
				event->stream();

			// Advance
			++it;
		}
		while (it != events.end());

		// Done!
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
};


//////
//
// Module namespace(s) close
//

// ::stream
}


#endif // ifndef __LAYER_STREAM_H__
