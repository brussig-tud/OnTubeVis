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


/// An integer range consisting of a start value and a length.
template <class Index>
struct index_range {
	using index_type = Index;

	index_type i0, n;

	/// Calculate the index one past the last element in the range.
	[[nodiscard]] constexpr index_type end () const noexcept
	{
		return i0 + n;
	}
};

// helper struct for range entries with start index i0 and count n
// Moved from `glyph_compiler`.
using irange = index_range<int>;


/// Newtype wrapper storing a number of glyphs.
/// Used to avoid confusion between number of glyphs and number of glyph attributes.
struct glyph_count_type {
	using base_type = int;

	base_type value;

	[[nodiscard]] constexpr explicit glyph_count_type(base_type value) noexcept
		: value {value}
	{}

// Implement operators by applying them to the raw value.
#define OTV_GLYPH_COUNT_BINARY_OP(op, result_type) \
	[[nodiscard]] friend constexpr result_type operator op ( \
		glyph_count_type lhs, \
		glyph_count_type rhs \
	) noexcept { \
		return result_type{lhs.value op rhs.value}; \
	}

OTV_GLYPH_COUNT_BINARY_OP(+, glyph_count_type);
OTV_GLYPH_COUNT_BINARY_OP(-, glyph_count_type);
OTV_GLYPH_COUNT_BINARY_OP(==, bool);
OTV_GLYPH_COUNT_BINARY_OP(!=, bool);
OTV_GLYPH_COUNT_BINARY_OP(<, bool);
OTV_GLYPH_COUNT_BINARY_OP(<=, bool);
OTV_GLYPH_COUNT_BINARY_OP(>, bool);
OTV_GLYPH_COUNT_BINARY_OP(>=, bool);

#undef OTV_GLYPH_COUNT_BIN_OP

#define OTV_GLYPH_COUNT_ASSIGN_OP(op) \
	glyph_count_type &operator op (glyph_count_type rhs) noexcept { \
		value op rhs.value; \
		return *this; \
	}

OTV_GLYPH_COUNT_ASSIGN_OP(+=);
OTV_GLYPH_COUNT_ASSIGN_OP(-=);

#undef OTV_GLYPH_COUNT_ASSIGN_OP

};


/// Type used to identify glyph layers.
using layer_index_type = uint8_t;

/// The maximum number of concurrent glyph layers supported by the implementation.
/// NOTE: This value is still hard-coded in several places, particulary in shaders.
static constexpr layer_index_type max_glyph_layers {4};

/// Stores generic data specific to each glyph layer.
template <class Elem>
using per_layer = std::array<Elem, max_glyph_layers>;


/// Type holding the number of 32-bit float attributes used to represent one glyph.
using glyph_size_type = uint8_t;

// helper struct for glyph attributes
// Moved from `glyph_compiler`.
struct glyph_attributes {
	glyph_size_type count = 0;
	std::vector<float> data;

	bool empty() const { return size() == 0; }

	size_t size() const { return data.size(); }

	glyph_count_type glyph_count() const {
		return static_cast<glyph_count_type>(size() / (2 + count));
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
