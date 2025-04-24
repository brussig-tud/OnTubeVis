
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __CORE_H__
#define __CORE_H__


//////
//
// Includes
//

// C++ STL
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <atomic>
#include <optional>

// Public interface
#include <cgv/math/fvec.h>
#include <OnTubeVis/OnTubeVis.h>



//////
//
// Injected operators
//

// 2-Vector addition
inline OTV_Vec2 operator + (const OTV_Vec2 &v0, const OTV_Vec2 &v1) {
	return {v0.x+v1.x, v0.y+v1.y};
}

// 2-Vector in-place addition
inline OTV_Vec2& operator += (OTV_Vec2 &self, const OTV_Vec2 &other) {
	self.x += other.x;
	self.y += other.y;
	return self;
}

// 2-Vector subtraction
inline OTV_Vec2 operator - (const OTV_Vec2 &v0, const OTV_Vec2 &v1) {
	return {v0.x-v1.x, v0.y-v1.y};
}

// 2-Vector in-place subtraction
inline OTV_Vec2& operator -= (OTV_Vec2 &self, const OTV_Vec2 &other) {
	self.x -= other.x;
	self.y -= other.y;
	return self;
}

// 4-Vector scalar addition
inline OTV_Vec4 operator + (const OTV_Vec4 &v, const float scalar) {
	return {v.x+scalar, v.y+scalar, v.z+scalar, v.w+scalar};
}



//////
//
// Typedefs & structs
//

struct irange_api {
	unsigned i0, n;
};

struct alen_ref {
	unsigned idx=-1;
	float s;
	inline bool exists(void) const { return int(idx) > -1; }
	inline void invalidate(void) { idx=-1; }
};

struct glyph_layer {
	std::vector<irange_api> ranges;
	std::vector<OTV_GlyphData> glyphs;
	alen_ref latest_refed, earliest_unrefed;
	inline bool not_empty(void) const { return !glyphs.empty(); }
};

struct trajectory {
	std::vector<OTV_HermiteNode> nodes;
	std::vector<OTV_SegmentArclen> segment_arclens;
	std::vector<glyph_layer> layers;
	float radius;
};

struct trajectory_setup {
	uint32_t id;
	float radius;
};

enum class VisualAttribute : unsigned {
	Color, Height, Orientation, Radius, Value0, Value1, Value2, Value3, Width
};
typedef VisualAttribute VAttrib;

struct VisualAttributeSource {
	cgv::vec2 in;
	cgv::vec2 out;
	std::string desc;
};
typedef std::map<VisualAttribute, VisualAttributeSource> VAttribSources;

inline VisualAttributeSource get_vattrib_source (
	const VAttribSources &sources, const VisualAttribute vattrib, const cgv::vec2 &default_in_range,
	const cgv::vec2 &default_out_range
){
	const auto it = sources.find(vattrib);
	return it==sources.end() ?
		  VisualAttributeSource{default_in_range, default_out_range, ""}
		: it->second;
}

inline float map_attrib_value (const cgv::vec2 &in_range, const cgv::vec2 &out_range, const float value) {
	const float t = std::clamp((value - in_range.x())/(in_range.y() - in_range.x()), 0.0f, 1.0f);
	return cgv::math::lerp(out_range.x(), out_range.y(), t);
}

struct VisSetup
{
	VisSetup(const std::string &name) : name(name) {}

	VisSetup(const VisSetup &other)
		: name(other.name), counter(other.counter.load()), layers(other.layers),
		  layer_sources(other.layer_sources), trajs(other.trajs), georef(other.georef),
		  num_extrapol_segments(other.num_extrapol_segments), use_natural_progression(other.use_natural_progression)
	{}

	VisSetup(VisSetup &&other) noexcept
		: name(std::move(other.name)), counter(other.counter.load()), layers(std::move(other.layers)),
		  layer_sources(std::move(other.layer_sources)), trajs(std::move(other.trajs)), georef(std::move(other.georef)),
		  num_extrapol_segments(other.num_extrapol_segments), use_natural_progression(other.use_natural_progression)
	{}

	std::string name;
	std::atomic<uint32_t> counter{0};
	std::vector<OTV_LayerConfig> layers;
	std::vector<VAttribSources> layer_sources;
	std::vector<trajectory_setup> trajs;

	std::optional<cgv::dvec2> georef;
	unsigned num_extrapol_segments = 0;
	bool use_natural_progression = false;

	VisSetup& operator = (const VisSetup &other)
	{
		name = other.name;
		layers.clear();
		for (const auto &l : other.layers)
			layers.emplace_back(l);
		for (const auto &ls : other.layer_sources)
			layer_sources.emplace_back(ls);
		trajs = other.trajs;
		georef = other.georef;
		return *this;
	}

	VisSetup& operator = (VisSetup &&other) noexcept
	{
		name = std::move(other.name);
		counter.store(other.counter.load());
		layers = std::move(other.layers);
		layer_sources = std::move(other.layer_sources);
		trajs = std::move(other.trajs);
		georef = std::move(other.georef);
		return *this;
	}
};

// ToDo: REMOVE ME once node/glyph streaming is properly implemented
struct streaming_dataset {
	static std::string name;
	static std::vector<OTV_LayerConfig> layers;
	static std::vector<VAttribSources> layer_sources;
	static std::vector<trajectory> trajectories;
	static std::unordered_map<uint32_t, unsigned> traj_id_map;
};


#endif // ifdef __CORE_H__
