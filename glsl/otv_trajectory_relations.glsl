#version 430 core

// Static configuration ############################################################################
// Default values are provided only for linting and must be replaced at runtime.
#define HASH_GRID_BUFFER_BINDING   0
#define HASH_GRID_ADDRESS_UNIT     0
#define HASH_GRID_NUM_HASH_FNS     0
#define HASH_GRID_SLOTS_PER_BUCKET 0
#define HASH_GRID_CELL_HEADER_SIZE 0
#define HASH_GRID_DIMENSIONS       0
#define TRAJ_REL_FUNCTION          0

const uint buffer_binding   = HASH_GRID_BUFFER_BINDING;
const uint address_unit     = HASH_GRID_ADDRESS_UNIT;
const uint num_hash_fns     = HASH_GRID_NUM_HASH_FNS;
const uint slots_per_bucket = HASH_GRID_SLOTS_PER_BUCKET;
const uint cell_header_size = HASH_GRID_CELL_HEADER_SIZE;
const uint dimensions       = HASH_GRID_DIMENSIONS;
const uint relation         = TRAJ_REL_FUNCTION;

// Types ###########################################################################################
struct node_data_type {
	vec4 pos_rad;
	vec4 color;
	vec4 tangent;
	vec4 t; // time, trajectory id, undefined, undefined
};

// An offset into the grid buffer in units of `address_unit` bytes.
struct ptr_t {
	uint address;
};
const ptr_t null = {~0u};

// The space that the hash grid indexes.
#if HASH_GRID_DIMENSIONS == 3
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

// Header of a cell allocation, corresponding to otv::hash_grid::cell_t::data_t.
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
	vec2  time;
};
const uint sizeof_interval = 16; // in bytes

const uint rel_dir_ref_to_all = 0;
const uint rel_dir_all_to_ref = 1;
const uint rel_dir_all_to_all = 2;

// Constants #######################################################################################
// Relations with this index or higher show debug information or stats.
const uint debug = 2;

// SSBOs ###########################################################################################
layout(std430, binding = 0) readonly buffer data_buffer {
	node_data_type nodes[];
};
layout(std430, binding = 2) readonly buffer nid_buffer {
	uvec2 node_ids[];
};
// The grid buffer is aliased as multiple types to allow aligned loads of different sizes.
layout(binding = HASH_GRID_BUFFER_BINDING) readonly buffer hash_grid_buffer1 {
	uint hash_grid_data1[];
};
layout(binding = HASH_GRID_BUFFER_BINDING) readonly buffer hash_grid_buffer2 {
	uvec2 hash_grid_data2[];
};
layout(binding = HASH_GRID_BUFFER_BINDING) readonly buffer hash_grid_buffer4 {
	uvec4 hash_grid_data4[];
};

// Uniforms ########################################################################################
uniform vec4    hash_grid_cell_size;
uniform coord_t hash_grid_scale; // == 1 / hash_grid_cell_size
uniform ptr_t   hash_grid_buckets;
uniform uint    hash_grid_buckets_mask;
uniform vec2    traj_rel_radius;
uniform float   traj_rel_sample_rate;
uniform uint    traj_rel_direction;
uniform uint    traj_rel_ref_traj;
uniform bool    traj_rel_normalize;
uniform int     traj_rel_color_map;
uniform vec2    traj_rel_color_range;
uniform bool    traj_rel_log_scale;
uniform vec3    traj_rel_highlight_color;
uniform vec3    traj_rel_background_color;

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
	return hash_grid_data1[ptr.address / (4/address_unit)];
}
uvec2 load2 (ptr_t ptr)
{
	return hash_grid_data2[ptr.address / (8/address_unit)];
}
uvec4 load4 (ptr_t ptr)
{
	return hash_grid_data4[ptr.address / (16/address_unit)];
}

// Hash grid =======================================================================================
// Calculate the index of the cell containing a given point.
index_t cell_index (coord_t coords)
{
	return index_t(round(coords * hash_grid_scale));
}

// Hash a cell index into a 32-bit signature using xxhash32 as implemented by Jarzynski and Olano
// 2020 with coordinates rotated.
uint signature (index_t index)
{
	// Prime constants.
	const uint[] p = {2246822519, 3266489917, 668265263, 374761393};

	uint sig = uint(index[0]) + p[3];

	for (uint d = 1; d < dimensions; ++d) {
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
		// PCG hash relation by O'Neill 2014, as implemented by Jarzynski and Olano 2020.

		// Choose the seed.
		uvec3 s = uvec3[](
			uvec3(747796405, 2891336453, 277803737)
		)[hash_fn - 1];

		hash = signature * s[0] + s[1];
		hash = ((hash >> ((hash >> 28) + 4)) ^ hash) * s[2];
		hash = (hash >> 22) ^ hash;
	}

	return offset_bytes(
		hash_grid_buckets,
		(hash & hash_grid_buckets_mask) * slots_per_bucket * sizeof_slot
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
vec4 trajectory_point (node_data_type start, node_data_type end, float t)
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

	return vec4(
		// Space: cubic Bézier curve.
		bezier * vec4(s2*s, 3*s2*t, 3*s*t2, t*t2),
		// Time: linear interpolation.
		s*start.t[0] + t*end.t[0]
	);
}

// Shading =========================================================================================
float eval_relation (vec4 ref_point, interval_t interval)
{
	// Load node data.
	node_data_type n0 = nodes[interval.nodes[0]];
	node_data_type n1 = nodes[interval.nodes[1]];

	// Intersect trajectory interval and evaluated time frame.
	float start    = max(interval.time[0], ref_point[3] - traj_rel_radius[1]);
	float end      = min(interval.time[1], ref_point[3] + traj_rel_radius[1]);
	float timespan = end - start;
	if (timespan <= 0) return 0;

	// Determine how often the interval should be sampled.
	float num_samples = ceil(timespan * traj_rel_sample_rate);
	if (relation == debug + 9) return num_samples;

	// Divide the evaluated range into equal steps.
	float time_scale  = 1.0 / (n1.t[0] - n0.t[0]);
	float sampling    = 1.0 / num_samples;
	float sample_step = timespan * time_scale * sampling;

	// Map time to curve parameter.
	float tmin = (start - n0.t[0]) * time_scale;
	float tmax = (end   - n0.t[0]) * time_scale;

	// Evaluate the relation at one or more sample points along the interval.
	float result = 0.0;
	for (float t = tmin + 0.5*sample_step; t < tmax; t += sample_step) {
		// Evaluate the trajectory for the current curve parameter.
		vec3 sample_point = trajectory_point(n0, n1, t).xyz;

		// Ignore points outside the evaluation radius.
		vec3 offset = sample_point - ref_point.xyz;
		if (dot(offset, offset) > traj_rel_radius[0]*traj_rel_radius[0]) continue;

		// Evaluate the relation.
		switch (relation) {
		case 1: // Distance.
			result += traj_rel_radius[0] - distance(ref_point.xyz, sample_point);
		}
	}

	// Average samples and weight by time.
	return result * sampling * timespan;
}

// Determine the color to use for a trajectory fragment by calculating its relation to other
// trajectories using the hash grid.
vec3 otv_shade_relation (int seg_id, float seg_t)
{
	// Load node data.
	node_data_type start = nodes[node_ids[seg_id][0]];
	node_data_type end   = nodes[node_ids[seg_id][1]];

	// If only the reference trajectory is to be shaded, mark all others as background.
	if (traj_rel_direction == rel_dir_ref_to_all && start.t[1] != traj_rel_ref_traj)
		return traj_rel_background_color;
	// Mark the reference trajectory.
	if (traj_rel_direction == rel_dir_all_to_ref && start.t[1] == traj_rel_ref_traj)
		return traj_rel_highlight_color;

	// Color by local time.
	if (relation == debug) return map_to_color(seg_t, traj_rel_color_map);

	// Calculate data-space coordinates and grid cell at the given trajectory point.
	vec4 local_point    = trajectory_point(start, end, seg_t);
	index_t local_index = cell_index(coord_t(local_point));

	// Color by grid cell (spatial).
	if (relation == debug + 1)
		return local_point.xyz * hash_grid_scale.xyz - local_index.xyz + 0.5;
	#if HASH_GRID_DIMENSIONS != 3
		// Color by grid cell (temporal).
		if (relation == debug + 2)
			return map_to_color(
				local_point[3] * hash_grid_scale[3] - local_index[3] + 0.5,
				traj_rel_color_map
			);
	#endif
	// Color by cell hash.
	if (relation == debug + 3) return map_to_color(signature(local_index) / float(~0u), 19);
	// Color by hash bucket load.
	if (relation == debug + 4) {
		uint fill = bucket_fill(bucket(signature(local_index), 1));
		return map_to_color(float(fill) / slots_per_bucket, traj_rel_color_map);
	}
	// Color by local trajectory interval.
	if (relation == debug + 5) {
		// Find the cell containing this fragment.
		span_t local_intervals = query(local_index);
		// Within that cell, find the interval containing the fragment.
		for (uint i = 0; i < local_intervals.len; ++i) {
			interval_t interval = load_interval(local_intervals, i);
			float t0            = interval.time[0];
			float t1            = interval.time[1];

			if (interval.nodes == node_ids[seg_id] && local_point[3] >= t0 && local_point[3] <= t1)
				// Color by interval-local curve parameter.
				return map_to_color((local_point[3] - t0)/(t1 - t0), traj_rel_color_map);
		}
		// Mark points not stored within their local cell.
		return traj_rel_highlight_color;
	}

	// AABB of cells to query.
	coord_t max_offset = coord_t(vec4(vec3(traj_rel_radius[0]), traj_rel_radius[1]));
	index_t min_cell   = cell_index(coord_t(local_point) - max_offset);
	index_t max_cell   = cell_index(coord_t(local_point) + max_offset);

	// Iff the distance squared between a cell's center and the local point is larger than this
	// value, all points within that cell are outside the evaluation radius.
	float max_cell_dist2 = traj_rel_radius[0] + length(hash_grid_cell_size.xyz)*0.5;
	max_cell_dist2      *= max_cell_dist2;

	// Value of the relation at the local point.
	float result = 0;

#if HASH_GRID_DIMENSIONS != 3
	for (int time = min_cell[3]; time <= max_cell[3]; ++time)
#endif
	for (int z = min_cell.z; z <= max_cell.z; ++z)
	for (int y = min_cell.y; y <= max_cell.y; ++y)
	for (int x = min_cell.x; x <= max_cell.x; ++x) {
		index_t index = {x, y, z
			#if HASH_GRID_DIMENSIONS != 3
				, time
			#endif
		};

		// Skip cells within the AABB, but outside the evaluation radius.
		vec3 offset = local_point.xyz - index.xyz*hash_grid_cell_size.xyz;
		if (dot(offset, offset) > max_cell_dist2) {
			if (relation == debug + 6) ++result;
			continue;
		}

		// Look up trajectory contents in the hash map.
		span_t intervals = query(index);

		switch (relation) {
		case debug + 7:
			result += float(intervals.len != 0);
			continue;
		case debug + 8:
			result += intervals.len;
			continue;
		}

		// Process all intervals within the cell.
		for (uint i = 0; i < intervals.len; ++i) {
			interval_t interval = load_interval(intervals, i);

			if (
				// Only evaluate intervals of the reference trajectory.
				   traj_rel_direction == rel_dir_all_to_ref
				&& nodes[interval.nodes[0]].t[1] == traj_rel_ref_traj
				// Evaluate all intervals on different trajectories.
				|| traj_rel_direction != rel_dir_all_to_ref
				&& nodes[interval.nodes[0]].t[1] != start.t[1]
			) result += eval_relation(local_point, interval);
		}
	}

	// Normalize the relation value.
	float norm_time = traj_rel_normalize ? traj_rel_radius[0] : 1;

	switch (relation) {
	case 1: // Distance.
		result /= (traj_rel_radius[0] * norm_time);
		break;
	case debug + 6: // Skipped cells.
		result /= float(dot(max_cell - min_cell + 1, coord_t(1)));
		break;
	}

	// Map the relation value to a color.
	vec2 range = traj_rel_color_range;

	if (traj_rel_log_scale)
		result = log(9 * (result - range[0])/(range[1] - range[0]) + 1);
	else
		result = (result - range[0])/(range[1] - range[0]);

	return map_to_color(result, traj_rel_color_map);
}
