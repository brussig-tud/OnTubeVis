#pragma once

#include <cstdint>

#include <cgv/math/fvec.h>


/// data layout for per-node attributes within the attribute render SSBO
struct node_attribs {
	cgv::vec4  pos_rad;
	cgv::vec4  color;
	cgv::vec4  tangent;
	float      t;
	uint32_t   traj_id; // unique across all datasets
	cgv::uvec2 pad {};
};

