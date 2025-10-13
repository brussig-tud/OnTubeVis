#version 430 core

// Constants #######################################################################################
// Determines for which pairs of trajectories the relation is visualized.
struct direction_t {uint value;};
const direction_t dir_ref_to_all = {0};
const direction_t dir_all_to_ref = {1};
const direction_t dir_all_to_all = {2};

// The value to visualize.
struct function_t {uint value;};
const function_t fn_none               = {0};
const function_t fn_distance           = {1 + fn_none.value};
const function_t fn_dbg_seg_t          = {1 + fn_distance.value};
const function_t fn_dbg_index_xyz      = {1 + fn_dbg_seg_t.value};
const function_t fn_dbg_index_t        = {1 + fn_dbg_index_xyz.value};
const function_t fn_dbg_signature      = {1 + fn_dbg_index_t.value};
const function_t fn_dbg_bucket_load    = {1 + fn_dbg_signature.value};
const function_t fn_dbg_local_interval = {1 + fn_dbg_bucket_load.value};
const function_t fn_dbg_skipped_cells  = {1 + fn_dbg_local_interval.value};
const function_t fn_dbg_num_cells      = {1 + fn_dbg_skipped_cells.value};
const function_t fn_dbg_num_intervals  = {1 + fn_dbg_num_cells.value};
const function_t fn_dbg_num_samples    = {1 + fn_dbg_num_intervals.value};
const function_t fn_dbg_num_evals      = {1 + fn_dbg_num_samples.value};

// Describes which dimensions the hash grid indexes and how it is organized in memory.
#define LAYOUT_XYZ   0
#define LAYOUT_T_XYZ 1
#define LAYOUT_XYZT  2

// Special values returned by `otv_trajectory_relation`, encoded as quiet NaNs.
const uint nan              = 0xffc00000u;
const uint background_value = nan | 1;
const uint highlight_value  = nan | 2;

uint traj_rel_background_value()
{
	return background_value;
}

// Static configuration ############################################################################
// Default values are provided only for linting and must be replaced at runtime.
#define HASH_GRID_BUFFER_BINDING   0
#define HASH_GRID_ADDRESS_UNIT     0
#define HASH_GRID_NUM_HASH_FNS     0
#define HASH_GRID_SLOTS_PER_BUCKET 0
#define HASH_GRID_CELL_HEADER_SIZE 0
#define HASH_GRID_LAYOUT           0
#define TRAJ_REL_FUNCTION          0

// Index at which the SSBO containing the hash grid is bound.
const uint buffer_binding = HASH_GRID_BUFFER_BINDING;
// Minimal addressable unit of the grid buffer in bytes.
const uint address_unit = HASH_GRID_ADDRESS_UNIT;
// Number of candidate hash buckets a cell can be stored in.
const uint num_hash_fns = HASH_GRID_NUM_HASH_FNS;
// Number of cells with equal hash that can be stored in each bucket of a hash table.
const uint slots_per_bucket = HASH_GRID_SLOTS_PER_BUCKET;
// Offset of a cell's intervals array from its base address.
const uint cell_header_size = HASH_GRID_CELL_HEADER_SIZE;
// The relation or debug value to calculte and visualize.
const function_t function = {TRAJ_REL_FUNCTION};

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

// Pointer to a contiguous array with runtime length.
struct span_t {
	ptr_t start;
	uint  len;
};
const span_t null_span = {null, 0};

// Key used to identify grid cells within each hash table.
// The grid as a whole always uses 4D indices, though some layouts may ignore the time component.
#if HASH_GRID_LAYOUT == LAYOUT_XYZT
	#define index_t ivec4
#else
	#define index_t ivec3
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

// Part of a trajectory segment contained in a single grid cell.
struct interval_t {
	uvec2 nodes;
	vec2  time;
};
const uint sizeof_interval = 16; // in bytes

// SSBOs ###########################################################################################
layout(std430, binding = 0) readonly buffer data_buffer {
	node_data_type nodes[];
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
uniform vec4   hash_grid_cell_size;
uniform vec4   hash_grid_scale; // == 1 / hash_grid_cell_size
uniform span_t hash_grid_data;
#if HASH_GRID_LAYOUT == LAYOUT_T_XYZ
uniform ptr_t hash_grid_timesteps;
#endif
uniform vec2        traj_rel_radius;
uniform float       traj_rel_sample_rate;
uniform direction_t traj_rel_direction;
uniform uint        traj_rel_ref_traj;
uniform bool        traj_rel_normalize;
uniform int         traj_rel_color_map;
uniform vec2        traj_rel_color_range;
uniform bool        traj_rel_log_scale;
uniform vec3        traj_rel_highlight_color;
uniform vec3        traj_rel_background_color;

// External functions ##############################################################################
// Defined in otv_color_map.glsl.
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
ivec4 cell_index (vec4 point)
{
	return ivec4(round(point * hash_grid_scale));
}

// Hash a cell index into a 32-bit signature using xxhash32 as implemented by Jarzynski and Olano
// 2020 with coordinates rotated.
uint signature (index_t index)
{
	// Prime constants.
	const uint[] p = {2246822519u, 3266489917u, 668265263u, 374761393u};

	uint sig = uint(index[0]) + p[3];

	for (uint d = 1; d < (HASH_GRID_LAYOUT == LAYOUT_XYZT ? 4 : 3); ++d) {
		sig += uint(index[d]) * p[1];
		sig  = p[2] * ((sig << 17) | (sig >> 15));
	}
	sig = p[0] * (sig ^ (sig >> 15));
	sig = p[1] * (sig ^ (sig >> 13));
	return sig ^ (sig >> 16);
}

// Calculate the base address of the bucket that `hash_fn` maps `signature` onto.
ptr_t bucket (span_t table, uint signature, uint hash_fn)
{
	uint hash = signature;

	if (hash_fn != 0) {
		// PCG hash relation by O'Neill 2014, as implemented by Jarzynski and Olano 2020.

		// Choose the seed.
		const uvec3 s = uvec3[](
			uvec3(747796405u, 2891336453u, 277803737u)
		)[hash_fn - 1];

		hash = signature * s[0] + s[1];
		hash = ((hash >> ((hash >> 28) + 4)) ^ hash) * s[2];
		hash = (hash >> 22) ^ hash;
	}

	return offset_bytes(
		table.start,
		(hash & (table.len - 1)) * slots_per_bucket * sizeof_slot
	);
}

// Load an entry of the hash table from the grid buffer.
slot_t load_slot (ptr_t bucket, uint index)
{
	const uvec2 data = load2(offset_bytes(bucket, index * sizeof_slot));
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
span_t query (span_t table, index_t index)
{
	const uint signature = signature(index);

	// Try all hash functions.
	for (uint fn = 0; fn < num_hash_fns; ++fn) {
		const ptr_t bucket = bucket(table, signature, fn);

		for (uint i = 0; i < slots_per_bucket; ++i) {
			const slot_t slot = load_slot(bucket, i);
			// Buckets are filled front to back, so if one slot is empty, the remaining ones are as
			// well.
			if (slot.cell == null) break;
			// Look for a matching signature.
			if (slot.signature != signature) continue;

			// If the signature matches, check the index.
			const cell_t cell = load_cell(slot.cell);
			if (cell.index != index) continue;

			// The cell has been found.
			return span_t(offset_bytes(slot.cell, cell_header_size), cell.size);
		}
	}

	// The cell could not be found.
	return null_span;
}

#if HASH_GRID_LAYOUT != LAYOUT_T_XYZ
// Return a pointer to the buckets for a given temporal index.
span_t find_table (int timestep)
{
	// All layouts other than t_xyz have only one table.
	return hash_grid_data;
}

#else

// Load a bucket range from the table vector.
span_t load_table (uint table_idx)
{
	const uvec2 data = load2(offset_bytes(hash_grid_data.start, table_idx * 8));
	return span_t(ptr_t(data[0]), data[1]);
}

// Load the timestep corresponding to the given index in the table vector.
int load_timestep (uint table_idx)
{
	return int(load1(offset_bytes(hash_grid_timesteps, table_idx * 4)));
}

// Find the subspan of the table vector that lies between two temporal indices.
// Both the query and the result are inclusive ranges.
uvec2 table_range (int min_timestep, int max_timestep)
{
	// Binary search for the earliest timestep no smaller than the query's lower limit.
	int lo = 0;
	int hi = int(hash_grid_data.len) - 1;
	while (lo < hi) {
		const int mid = (lo + hi) / 2;
		if (min_timestep <= load_timestep(mid)) hi = mid;
		else lo = mid + 1;
	}
	// Save the result.
	const uint start = lo;

	// Binary search for the latest timestep no larger than the query's upper limit.
	hi = int(hash_grid_data.len) - 1;
	while (lo < hi) {
		const int mid = (lo + hi) / 2;
		if (max_timestep > load_timestep(mid)) lo = mid + 1;
		else hi = mid;
	}
	// Return the resulting range.
	return uvec2(start, hi);
}

// Retrieve the bucket range corresponding to a given temporal index from the table vector.
// If the vector contains no entry for the queried timestep, an empty null span is returned.
span_t find_table (int timestep)
{
	const uint table_idx = table_range(timestep, timestep)[0];
	if (table_idx >= hash_grid_data.len || load_timestep(table_idx) != timestep) return null_span;
	return load_table(table_idx);
}
#endif

// Load one of the trajectory intervals in a cell from the grid buffer.
interval_t load_interval (span_t intervals, uint index)
{
	const uvec4 data = load4(offset_bytes(intervals.start, index * sizeof_interval));
	return interval_t(data.xy, uintBitsToFloat(data.zw));
}

// Trajectories ====================================================================================
// Formulas for evaluating cubic Hermite splines are taken from
// https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Representations

// Calculate the coefficients of a trajectory's 3D position represented as a cubic Hermite spline in
// monomial basis ordered by increasing degree.
// Since these coefficients are constant over the entire segment, they can be used to efficiently
// calculate several positions using the function `eval_position`.
mat4x3 position_coeffs (node_data_type start, node_data_type end)
{
	const vec3 p0 = start.pos_rad.xyz;
	const vec3 m0 = start.tangent.xyz;
	const vec3 p1 = end.pos_rad.xyz;
	const vec3 m1 = end.tangent.xyz;

	return mat4x3(
		p0,
		m0,
		-3*p0 - 2*m0 + 3*p1 - m1,
		2*p0 + m0 - 2*p1 + m1
	);
}

// Obtain the 3D position at parameter `t` in [0, 1] along a trajectory segment using coefficients
// precalculated by `position_coeffs`.
vec3 eval_position (mat4x3 coeffs, float t)
{
	const float t2 = t*t;
	const float t3 = t*t2;
	return coeffs * vec4(1, t, t2, t3);
}

// Calculate the 4D trajectory point at parameter t in [0, 1] of a given segment.
// If only a single point is required, this function is faster than
// `eval_position(position_coeffs)`.
vec4 trajectory_point (node_data_type start, node_data_type end, float t)
{
	const float t2 = t*t;
	const float t3 = t*t2;

	return vec4(
		(2*t3 - 3*t2 + 1)*start.pos_rad.xyz
		+ (t3 - 2*t2 + t)*start.tangent.xyz
		+ (-2*t3 + 3*t2)*end.pos_rad.xyz
		+ (t3 - t2)*end.tangent.xyz,
		mix(start.t[0], end.t[0], t)
	);
}

// Shading =========================================================================================
// Calculate the trajectory relation selected by `TRAJ_REL_FUNCTION` from a fixed starting point to
// one or more sampled points on a given interval, provided the samples lie within the query radius.
float eval_relation (vec4 ref_point, interval_t interval)
{
	// Load node data.
	const node_data_type n0 = nodes[interval.nodes[0]];
	const node_data_type n1 = nodes[interval.nodes[1]];

	// Intersect trajectory interval and evaluated time frame.
	const float start    = max(interval.time[0], ref_point[3] - traj_rel_radius[1]);
	const float end      = min(interval.time[1], ref_point[3] + traj_rel_radius[1]);
	const float timespan = end - start;
	if (timespan <= 0) return 0;

	// Determine how often the interval should be sampled.
	const float num_samples = ceil(timespan * traj_rel_sample_rate);
	if (function == fn_dbg_num_samples) return num_samples;

	// Divide the evaluated range into equal steps.
	const float time_scale  = 1.0 / (n1.t[0] - n0.t[0]);
	const float sampling    = 1.0 / num_samples;
	const float sample_step = timespan * time_scale * sampling;

	// Map time to curve parameter.
	const float tmin = (start - n0.t[0]) * time_scale;
	const float tmax = (end   - n0.t[0]) * time_scale;

	// Calculate spline coefficients.
	const mat4x3 coeffs = position_coeffs(n0, n1);

	// Evaluate the relation at one or more sample points along the interval.
	float result = 0.0;
	for (float t = tmin + 0.5*sample_step; t < tmax; t += sample_step) {
		// Evaluate the trajectory for the current curve parameter.
		const vec3 sample_point = eval_position(coeffs, t);

		// Ignore points outside the evaluation radius.
		const vec3 offset = sample_point - ref_point.xyz;
		if (dot(offset, offset) > traj_rel_radius[0]*traj_rel_radius[0]) continue;

		// Evaluate the relation.
		switch (function.value) {
		case fn_distance.value:
			result += traj_rel_radius[0] - distance(ref_point.xyz, sample_point);
		case fn_dbg_num_evals.value:
			result += 1;
		}
	}

	if (function == fn_dbg_num_evals) return result;

	// Average samples and weight by time.
	return result * sampling * timespan;
}

// Evaluate the relation selected by `TRAJ_REL_FUNCTION` between one point of a segment and the
// surrounding trajectories using the hash grid.
// The return value can be mapped to a color using `otv_shade_relation`.
float otv_trajectory_relation (uvec2 node_ids, float seg_t)
{
	// Load node data.
	const node_data_type start = nodes[node_ids[0]];
	const node_data_type end   = nodes[node_ids[1]];

	// If only the reference trajectory is to be shaded, mark all others as background.
	if (traj_rel_direction == dir_ref_to_all && start.t[1] != traj_rel_ref_traj)
		return uintBitsToFloat(background_value);
	// Mark the reference trajectory.
	if (traj_rel_direction == dir_all_to_ref && start.t[1] == traj_rel_ref_traj)
		return uintBitsToFloat(highlight_value);

	// Color by local time.
	if (function == fn_dbg_seg_t) return seg_t;

	// Calculate data-space coordinates and grid cell at the given trajectory point.
	const vec4 local_point  = trajectory_point(start, end, seg_t);
	const ivec4 local_index = cell_index(local_point);

	// Color by grid cell (spatial).
	if (function == fn_dbg_index_xyz)
		return uintBitsToFloat(packUnorm4x8(
			vec4(vec3(local_point * hash_grid_scale - local_index + 0.5), 0)
		));
	// Color by grid cell (temporal).
	if (function == fn_dbg_index_t)
		return local_point[3] * hash_grid_scale[3] - local_index[3] + 0.5;
	// Color by cell hash.
	if (function == fn_dbg_signature)
		return signature(index_t(local_index)) / float(~0u);
	// Color by hash bucket load.
	if (function == fn_dbg_bucket_load) {
		// Load the bucket range for the local timestep.
		const span_t table = find_table(local_index[3]);
		if (table.start == null) return uintBitsToFloat(highlight_value);
		// Find the bucket containing the local point and count how many of tis slots are in use.
		const uint fill = bucket_fill(bucket(table, signature(index_t(local_index)), 1));
		return float(fill) / slots_per_bucket;
	}
	// Color by local trajectory interval.
	if (function == fn_dbg_local_interval) {
		// Load the bucket range for the local timestep.
		const span_t table = find_table(local_index[3]);
		if (table.start == null) return uintBitsToFloat(highlight_value);
		// Find the cell containing this fragment.
		const span_t local_intervals = query(table, index_t(local_index));
		// Within that cell, find the interval containing the fragment.
		for (uint i = 0; i < local_intervals.len; ++i) {
			const interval_t interval = load_interval(local_intervals, i);
			const float t0            = interval.time[0];
			const float t1            = interval.time[1];

			if (interval.nodes == node_ids && local_point[3] >= t0 && local_point[3] <= t1)
				// Color by interval-local curve parameter.
				return (local_point[3] - t0)/(t1 - t0);
		}
		// Mark points not stored within their local cell.
		return uintBitsToFloat(highlight_value);;
	}

	// AABB of cells to query.
	const vec4 max_offset = vec4(vec3(traj_rel_radius[0]), traj_rel_radius[1]);
	const ivec4 min_cell  = cell_index(local_point - max_offset);
	const ivec4 max_cell  = cell_index(local_point + max_offset);

	// Iff the distance squared between a cell's center and the local point is larger than this
	// value, all points within that cell are outside the evaluation radius.
	const float max_cell_dist  = traj_rel_radius[0] + length(hash_grid_cell_size.xyz)*0.5;
	const float max_cell_dist2 = max_cell_dist*max_cell_dist;

	// Value of the relation at the local point.
	float result = 0;

#if HASH_GRID_LAYOUT == LAYOUT_T_XYZ
	// Iterate over the hash tables of all timesteps within the evaluated radius.
	const uvec2 table_range = table_range(min_cell[3], max_cell[3]);
	for (uint table_idx = table_range[0]; table_idx <= table_range[1]; ++table_idx) {
		const span_t table = load_table(table_idx);
#else
	// All layouts other than t_xyz have only one hash table.
	const span_t table = hash_grid_data;

#if HASH_GRID_LAYOUT == LAYOUT_XYZT
	// Iterate over the AABB's temporal extent.
	for (int time = min_cell[3]; time <= max_cell[3]; ++time)
#endif
	{
#endif

	// Iterate over the AABB's spatial extent
	for (int z = min_cell.z; z <= max_cell.z; ++z)
	for (int y = min_cell.y; y <= max_cell.y; ++y)
	for (int x = min_cell.x; x <= max_cell.x; ++x) {
		const index_t index = {x, y, z
			#if HASH_GRID_LAYOUT == LAYOUT_XYZT
				, time
			#endif
		};

		// Skip cells within the AABB, but outside the evaluation radius.
		const vec3 offset = local_point.xyz - index.xyz*hash_grid_cell_size.xyz;
		if (dot(offset, offset) > max_cell_dist2) {
			if (function == fn_dbg_skipped_cells) ++result;
			continue;
		}

		// Search the hash table for the current cell and return the intervals it contains.
		const span_t intervals = query(table, index);

		switch (function.value) {
		case fn_dbg_num_cells.value:
			result += float(intervals.len != 0);
			continue;
		case fn_dbg_num_intervals.value:
			result += intervals.len;
			continue;
		}

		// Process all intervals within the cell.
		for (uint i = 0; i < intervals.len; ++i) {
			const interval_t interval = load_interval(intervals, i);

			if (
				// Only evaluate intervals of the reference trajectory.
				   traj_rel_direction == dir_all_to_ref
				&& nodes[interval.nodes[0]].t[1] == traj_rel_ref_traj
				// Evaluate all intervals on different trajectories.
				|| traj_rel_direction != dir_all_to_ref
				&& nodes[interval.nodes[0]].t[1] != start.t[1]
			) result += eval_relation(local_point, interval);
		}
	}
	}

	// Normalize the relation value.
	const float norm_time = traj_rel_normalize ? 2*traj_rel_radius[1] : 1;

	switch (function.value) {
	case fn_distance.value: // Distance.
		result /= (traj_rel_radius[0] * norm_time);
		break;
	case fn_dbg_skipped_cells.value: // Skipped cells.
		result /= float(dot(max_cell - min_cell + 1, vec4(1)));
		break;
	}
	return result;
}

// Visualize a trajectory relation value as a color.
vec3 otv_shade_relation (float value)
{
	const uint bits = floatBitsToUint(value);
	if (function == fn_dbg_index_xyz) return unpackUnorm4x8(bits).xyz;
	if (bits == background_value)     return traj_rel_background_color;
	if (bits == highlight_value)      return traj_rel_highlight_color;

	// Apply transform function.
	const vec2 range = traj_rel_color_range;
	if (traj_rel_log_scale) // log base 10.
		value = log2(9 * (value - range[0])/(range[1] - range[0]) + 1)
			/ 3.32192809489 /*log2(10)*/;
	else value = (value - range[0])/(range[1] - range[0]);

	return map_to_color(value, function == fn_dbg_signature ? 19 : traj_rel_color_map);
}
