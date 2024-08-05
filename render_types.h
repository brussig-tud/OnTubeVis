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
struct irange {
	using index_type = int;

	index_type i0, n;

	[[nodiscard]] constexpr index_type end () const noexcept
	{
		return i0 + n;
	}
};

/// Uniquely identifies trajectory instances.
using trajectory_id = uint;

/// A range of glyphs within a trajectory's sub-buffer.
struct glyph_range : irange {
	trajectory_id trajectory;
};


/// Type holding the number of 32-bit float attributes used to represent one glyph.
using glyph_size_type = uint8_t;

// helper struct for glyph attributes
// Moved from `glyph_compiler`.
struct glyph_attributes {
	glyph_size_type count = 0;
	std::vector<float> data;

	bool empty() const { return size() == 0; }

	size_t size() const { return data.size(); }

	size_t glyph_count() const {
		return size() / (2 + count);
	}

	void add(const float& x) {
		data.push_back(x);
	}

	float& operator [](int idx) {
		return data[idx];
	}

	float operator [](int idx) const {
		return data[idx];
	}

	float last_glyph_s() const {
		if(size() > 0)
			return data[size() - 1 - 1 - count];
		else
			return 0.0f;
	}
};
