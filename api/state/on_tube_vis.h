
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __STATE_H__
#define __STATE_H__


//////
//
// Includes
//

// C++ STL
#include <vector>
#include <string>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <condition_variable>

// Public interface
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

struct VisSetup
{
	VisSetup(const std::string &name) : name(name) {}

	VisSetup(const VisSetup &other)
		: name(other.name), counter(other.counter.load()), layers(other.layers), trajs(other.trajs)
	{}

	VisSetup(VisSetup &&other) noexcept
		: name(std::move(other.name)), counter(other.counter.load()), layers(std::move(other.layers)),
		  trajs(std::move(other.trajs))
	{}

	std::string name;
	std::atomic<uint32_t> counter{0};
	std::vector<OTV_LayerConfig> layers;
	std::vector<trajectory_setup> trajs;

	VisSetup& operator = (const VisSetup &other)
	{
		name = other.name;
		layers.clear();
		for (const auto &l : other.layers)
			layers.emplace_back(l);
		trajs = other.trajs;
		return *this;
	}

	VisSetup& operator = (VisSetup &&other) noexcept
	{
		name = std::move(other.name);
		counter.store(other.counter.load());
		layers = std::move(other.layers);
		trajs = std::move(other.trajs);
		return *this;
	}
};

struct on_tube_vis_api
{
	// Control flow
	/*static std::mutex init_mtx;
	static bool init_pending;
	static std::condition_variable init_cv;*/
	static bool running;

	// Dataset
	static std::string ds_name;
	static std::vector<OTV_LayerConfig> layers;
	static std::vector<trajectory> trajectories;
	static std::unordered_map<uint32_t, unsigned> traj_id_map;
};


#endif // ifdef __STATE_H__
