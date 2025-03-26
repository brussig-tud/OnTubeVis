
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



//////
//
// Module namespace(s) close
//

// ::stream
}


#endif // ifndef __STREAM_GLYPHS_H__
