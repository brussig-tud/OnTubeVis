
#ifndef __STREAM_GLYPHS_H__
#define __STREAM_GLYPHS_H__


//////
//
// Includes
//

// C++ STL
#include <cassert>
#include <cstring>
#include <vector>
#include <map>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>



//////
//
// Module namespace(s)
//

namespace stream {



//////
//
// Classes
//

struct GlyphInstance
{
	OTV_GlyphData glyph;

	GlyphInstance(const GlyphInstance &other) = default;
	GlyphInstance& operator= (const GlyphInstance &other) {
		memcpy(&glyph, &other.glyph, sizeof(OTV_GlyphData)); // <- WTF C++??? Why do I have to resort to this???
		return *this;
	};

	explicit inline GlyphInstance (const OTV_GlyphData &glyph) : glyph(glyph) {}

	void stream (const unsigned traj_id, const unsigned layer) const {
		otv__stream_glyph(traj_id, layer, &glyph);
	}
};

struct Glyphs
{
	typedef std::map<float, unsigned> Sequence;

	std::vector<GlyphInstance> glyphs;
	Sequence time_sequence;
	Sequence dist_sequence;

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

struct GlyphsStreamer
{
	const unsigned traj_id, layer;
	const Glyphs &glyphs;
	float time_cursor = -std::numeric_limits<float>::infinity();

	GlyphsStreamer (const Glyphs &glyphs, const unsigned traj_id, const unsigned layer)
		: glyphs(glyphs), traj_id(traj_id), layer(layer)
	{}

	void stream_up_to_time (float time)
	{
		// Sanity check
		assert(time > time_cursor);

		// Iterate until target time
		auto it_cursor = glyphs.time_sequence.upper_bound(time_cursor);
		while (it_cursor != glyphs.time_sequence.end() && it_cursor->first <= time) {
			glyphs.glyphs[it_cursor->second].stream(traj_id, layer);
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


#endif // ifndef __STREAM_GLYPHS_H__
