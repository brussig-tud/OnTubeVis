#pragma once

// CGV framework
#include <cgv/math/fvec.h>


/// data layout for per-node attributes within the attribute render SSBO
struct node_attribs {
	cgv::vec4 pos_rad;
	cgv::vec4 color;
	cgv::vec4 tangent;
	cgv::vec4 t; // only uses .x component to store t, yzw are reserved for future use
};

// helper struct for range entries with start index i0 and count n
// Moved from `glyph_compiler`.
struct glyph_range {
	using index_type = int;

	index_type i0, n;
};
