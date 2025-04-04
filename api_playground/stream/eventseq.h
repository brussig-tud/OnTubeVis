
#ifndef __STREAM_EVENTSEQ_H__
#define __STREAM_EVENTSEQ_H__


//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <memory>
#include <vector>
#include <map>
#include <chrono>
#include <thread>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include "./dataset.h"



//////
//
// Module namespace(s)
//

namespace stream {



//////
//
// Functions
//

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
			node.stream(traj_id);
		}
	};

	struct GlyphEvent : public Event
	{
		unsigned traj_id, layer;
		GlyphInstance glyph;

		GlyphEvent(const unsigned traj_id, const unsigned layer, const GlyphInstance &glyph)
			: traj_id(traj_id), layer(layer), glyph(glyph)
		{}

		void stream (void) const override {
			glyph.stream(traj_id, layer);
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

		// Done!
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


#endif // ifndef __STREAM_EVENTSEQ_H__
