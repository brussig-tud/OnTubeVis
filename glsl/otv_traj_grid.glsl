#version 440 core

// Static configuration ############################################################################
// Emulate SPIR-V specialization constants with macros.
// Default values are provided only for linting, all macros must be replaced at runtime using
// otv::traj_grid::set_shader_defines.
#define TRAJ_GRID_BUFFER_BINDING   0
#define TRAJ_GRID_ADDRESS_UNIT     0
#define TRAJ_GRID_NUM_HASH_FNS     0
#define TRAJ_GRID_SLOTS_PER_BUCKET 0
#define TRAJ_GRID_CELL_HEADER_SIZE 0
#define TRAJ_GRID_DIMENSIONS       0
#define TRAJ_GRID_COLOR_FN         0

const uint buffer_binding   = TRAJ_GRID_BUFFER_BINDING;
const uint address_unit     = TRAJ_GRID_ADDRESS_UNIT;
const uint num_hash_fns     = TRAJ_GRID_NUM_HASH_FNS;
const uint slots_per_bucket = TRAJ_GRID_SLOTS_PER_BUCKET;
const uint cell_header_size = TRAJ_GRID_CELL_HEADER_SIZE;
const uint dimensions       = TRAJ_GRID_DIMENSIONS;
const uint color_fn         = TRAJ_GRID_COLOR_FN;

// Types ###########################################################################################
struct node_data_type {
	vec4 pos_rad;
	vec4 color;
	vec4 tangent;
	vec4 t; // only uses .x component to store t, yzw are reserved for future use
};

// An offset into the grid buffer in units of `address_unit` bytes.
struct ptr_t {
	uint address;
};
const ptr_t null = {~0u};

// The space that the grid indexes.
#if TRAJ_GRID_DIMENSIONS == 3
	#define coord_t vec3
	#define index_t ivec3
#else
	#define coord_t vec4
	#define index_t ivec4
#endif

// An entry of the hash table.
// May be empty (cell == null).
struct slot_t {
	uint  signature;
	ptr_t cell;
};
const uint sizeof_slot = 8; // bytes

// Header of a cell allocation, corresponding to otv::traj_grid::cell_t::data_t.
// Trajectory intervals start at an offset of cell_header_size bytes from the header's base address.
struct cell_t {
	index_t index;
	uint    size;
};

// Pointer to a contiguous array with runtime length.
struct span_t {
	ptr_t start;
	uint  len;
};

// Part of a trajectory segment contained in a single grid cell.
struct interval_t {
	uvec2 nodes;
	vec2  range;
};
const uint sizeof_interval = 16; // bytes

// SSBOs ###########################################################################################
layout(std430, binding = 0) readonly buffer data_buffer {
	node_data_type nodes[];
};
layout(std430, binding = 2) readonly buffer nid_buffer {
	uvec2 node_ids[];
};
// The grid buffer is aliased as multiple types to allow aligned loads of different sizes.
layout(binding = buffer_binding) readonly buffer traj_grid_buffer1 {
	uint traj_grid_data1[];
};
layout(binding = buffer_binding) readonly buffer traj_grid_buffer2 {
	uvec2 traj_grid_data2[];
};
layout(binding = buffer_binding) readonly buffer traj_grid_buffer4 {
	uvec4 traj_grid_data4[];
};

// Uniforms ########################################################################################
uniform coord_t traj_grid_scale;
uniform ptr_t   traj_grid_buckets;
uniform uint    traj_grid_buckets_mask;

// External functions ##############################################################################
// Defined in otv_shading.glsl.
vec3 map_to_color (float v, int color_map_idx);

// Local functions #################################################################################
// Memory ==========================================================================================
// Offset a pointer into the grid buffer by a number of bytes.
ptr_t offset_bytes (ptr_t ptr, uint offset)
{
	return ptr_t(ptr.address + offset/address_unit);
}

// Perform an aligned read of 1/2/4 32-bit words from the grid buffer.
uint load1 (ptr_t ptr)
{
	return traj_grid_data1[ptr.address / (4/address_unit)];
}
uvec2 load2 (ptr_t ptr)
{
	return traj_grid_data2[ptr.address / (8/address_unit)];
}
uvec4 load4 (ptr_t ptr)
{
	return traj_grid_data4[ptr.address / (16/address_unit)];
}

// Hash grid =======================================================================================
// Calculate the index of the cell containing a given point.
index_t cell_index (coord_t coords)
{
	return index_t(round(coords * traj_grid_scale));
}

// Hash a cell index into a 32-bit signature using xxhash32 as implemented by Jarzynski and Olano
// 2020 with coordinates rotated.
uint signature (index_t index)
{
	// Prime constants.
	const uint[] p = {2246822519, 3266489917, 668265263, 374761393};

	uint sig = uint(index[0]) + p[3];

	for (uint d = 1; d < TRAJ_GRID_DIMENSIONS; ++d) {
		sig += uint(index[d]) * p[1];
		sig  = p[2] * ((sig << 17) | (sig >> 15));
	}
	sig = p[0] * (sig ^ (sig >> 15));
	sig = p[1] * (sig ^ (sig >> 13));
	return sig ^ (sig >> 16);
}

// Calculate the base address of the bucket that `hash_fn` maps `signature` onto.
ptr_t bucket (uint signature, uint hash_fn)
{
	uint hash = signature;

	if (hash_fn != 0) {
		// PCG hash function by O'Neill 2014, as implemented by Jarzynski and Olano 2020.

		// Choose the seed.
		uvec3 s = uvec3[](
			uvec3(747796405, 2891336453, 277803737)
		)[hash_fn - 1];

		hash = signature * s[0] + s[1];
		hash = ((hash >> ((hash >> 28) + 4)) ^ hash) * s[2];
		hash = (hash >> 22) ^ hash;
	}

	return offset_bytes(
		traj_grid_buckets,
		(hash & traj_grid_buckets_mask) * slots_per_bucket * sizeof_slot
	);
}

// Load an entry of the hash table from the grid buffer.
slot_t load_slot (ptr_t bucket, uint index)
{
	uvec2 data = load2(offset_bytes(bucket, index * sizeof_slot));
	return slot_t(data[0], ptr_t(data[1]));
}

// Count the occupied slots in a hash bucket.
uint bucket_fill (ptr_t bucket)
{
	uint fill = slots_per_bucket;
	while (fill > 0 && load_slot(bucket, fill - 1).cell == null) --fill;
	return fill;
}

// Load a grid cell's header from the grid buffer.
cell_t load_cell (ptr_t ptr)
{
	return cell_t(index_t(load4(ptr)), load1(offset_bytes(ptr, 16)));
}

// Search the hash table for a given cell and return the trajectory intervals it contains.
// If the cell is not stored in the table, return an empty span.
span_t query (index_t index)
{
	uint signature = signature(index);

	// Try all hash functions.
	for (uint fn = 0; fn < num_hash_fns; ++fn) {
		ptr_t bucket = bucket(signature, fn);

		for (uint i = 0; i < slots_per_bucket; ++i) {
			slot_t slot = load_slot(bucket, i);
			// Buckets are filled front to back, so if one slot is empty, the remaining ones are as
			// well.
			if (slot.cell == null) break;
			// Look for a matching signature.
			if (slot.signature != signature) continue;

			// If the signature matches, check the index.
			cell_t cell = load_cell(slot.cell);
			if (cell.index != index) continue;

			// The cell has been found.
			return span_t(offset_bytes(slot.cell, cell_header_size), cell.size);
		}
	}

	// The cell could not be found.
	return span_t(null, 0u);
}

// Load one of the trajectory intervals in a cell from the grid buffer.
interval_t load_interval (span_t intervals, uint index)
{
	uvec4 data = load4(offset_bytes(intervals.start, index * sizeof_interval));
	return interval_t(data.xy, uintBitsToFloat(data.zw));
}

// Trajectories ====================================================================================
// Calculate the trajectory coordinates at parameter t in [0, 1] of a given segment.
coord_t trajectory_point (node_data_type start, node_data_type end, float t)
{
	// Spatial coordinates: Evaluate cubic Bézier curve.
	mat4x3 bezier = {
		start.pos_rad.xyz,
		start.pos_rad.xyz + start.tangent.xyz,
		  end.pos_rad.xyz -   end.tangent.xyz,
		  end.pos_rad.xyz,
	};

	float t2 = t*t;
	float s  = 1.0f - t;
	float s2 = s*s;

	return coord_t(
		bezier * vec4(s2*s, 3*s2*t, 3*s*t2, t*t2)
		#if TRAJ_GRID_DIMENSIONS != 3
			// Time: linear interpolation.
			, s*start.t[0] + t*end.t[0]
		#endif
	);
}

// Shading =========================================================================================
// Determine the color to use for a trajectory fragment using the trajectory grid.
vec3 otv_traj_grid_color (int seg_id, float seg_t)
{
	// Color by local time.
	if (color_fn == 2) return map_to_color(seg_t, 12);

	// Load node data.
	node_data_type start = nodes[node_ids[seg_id][0]];
	node_data_type end   = nodes[node_ids[seg_id][1]];

	// Calculate data-space coordinates and grid cell at the given trajectory point.
	coord_t local_coords = trajectory_point(start, end, seg_t);
	index_t local_index  = cell_index(local_coords);

	// Color by grid cell (spatial).
	if (color_fn == 3) return local_coords.xyz * traj_grid_scale.xyz - local_index.xyz + 0.5;
	#if TRAJ_GRID_DIMENSIONS != 3
		// Color by grid cell (temporal).
		if (color_fn == 4) return map_to_color(local_coords.w * traj_grid_scale.w - local_index.w + 0.5, 12);
	#endif
	// Color by cell hash.
	if (color_fn == 5) return map_to_color(signature(local_index) / float(~0u), 19);
	// Color by hash bucket load.
	if (color_fn == 6) {
		uint fill = bucket_fill(bucket(signature(local_index), 1));
		return map_to_color(float(fill) / slots_per_bucket, 12);
	}
	// Color by local trajectory interval.
	if (color_fn == 7) {
		// Find the cell containing this fragment.
		span_t local_intervals = query(local_index);
		// Within that cell, find the interval containing the fragment.
		for (uint i = 0; i < local_intervals.len; ++i) {
			interval_t interval = load_interval(local_intervals, i);
			float tmin          = interval.range[0];
			float tmax          = interval.range[1];

			if (interval.nodes == node_ids[seg_id] && seg_t >= tmin && seg_t <= tmax)
				// Color by interval-local curve parameter.
				return map_to_color((seg_t - tmin)/(tmax - tmin), 12);
		}
		// Highlight points not stored within their local cell.
		return vec3(1, 0, 1);
	}
	return vec3(1);
}
