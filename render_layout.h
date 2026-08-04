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

/// Maximum supported number of simultaneous glyph layers.
constexpr uint8_t max_glyph_layers = 4;

/// Bind indices used for SSBOs.
struct ssbo_idx {enum : uint32_t {
	nodes,
	t_to_s,
	node_idcs,
	seg_to_traj,
	glyphs_base,
	glyph_memory = glyphs_base + max_glyph_layers*2,
	grid_memory,
	count
};};

/// Bind indices used for textures.
struct texture_idx {enum : uint32_t {
	albedo,
	position,
	normal,
	tangent,
	depth,
	density,
	color_maps,
	relation_color_map,
};};

