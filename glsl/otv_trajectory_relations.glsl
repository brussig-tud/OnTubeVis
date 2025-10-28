#version 430 core

// Constants #######################################################################################
// Determines for which pairs of trajectories the relation is visualized.
struct direction_t {uint value;};
const direction_t dir_ref_to_all = {0};
const direction_t dir_all_to_ref = {1};
const direction_t dir_all_to_all = {2};

// The value to visualize, see vis::trajectory_relation::function.
#define FN_NONE               0
#define FN_PROXIMITY          1 + FN_NONE
#define FN_ALIGNMENT          1 + FN_PROXIMITY
#define FN_DBG_SEG_T          1 + FN_ALIGNMENT
#define FN_DBG_VELOCITY       1 + FN_DBG_SEG_T
#define FN_DBG_INDEX_XYZ      1 + FN_DBG_VELOCITY
#define FN_DBG_INDEX_T        1 + FN_DBG_INDEX_XYZ
#define FN_DBG_SIGNATURE      1 + FN_DBG_INDEX_T
#define FN_DBG_BUCKET_LOAD    1 + FN_DBG_SIGNATURE
#define FN_DBG_LOCAL_INTERVAL 1 + FN_DBG_BUCKET_LOAD
#define FN_DBG_SKIPPED_CELLS  1 + FN_DBG_LOCAL_INTERVAL
#define FN_DBG_NUM_CELLS      1 + FN_DBG_SKIPPED_CELLS
#define FN_DBG_NUM_INTERVALS  1 + FN_DBG_NUM_CELLS
#define FN_DBG_NUM_SAMPLES    1 + FN_DBG_NUM_INTERVALS
#define FN_DBG_NUM_EVALS      1 + FN_DBG_NUM_SAMPLES

// Describes which dimensions the hash grid indexes and how it is organized in memory.
#define LAYOUT_XYZ   0
#define LAYOUT_T_XYZ 1
#define LAYOUT_XYZT  2

// Determines how the cell indices' signatures are calculated.
#define SIG_FN_MULT_XOR 0
#define SIG_FN_XXHASH32 1
#define SIG_FN_Z_ORDER  2

// Functions used to map relation values to color.
struct transform_t {uint value;};
const transform_t transform_linear = {0};
const transform_t transform_log    = {1};
const transform_t transform_symlog = {2};

// Special values returned by `otv_trajectory_relation`, encoded as quiet NaNs.
const uint nan              = 0xffc00000u;
const uint background_value = nan | 1u;
const uint highlight_value  = nan | 2u;

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
#define HASH_GRID_SIGNATURE_FN     0
#define TRAJ_REL_SHADING           0
#define TRAJ_REL_FUNCTION          0

// Boolean expression determining whether or not the evaluated function depends on trajectories'
// derivatives.
#define FN_USES_DERIVATIVE (TRAJ_REL_FUNCTION == FN_ALIGNMENT)

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
	#define index_t  ivec4
	#define uindex_t uvec4
	const uint index_dims = 4;
#else
	#define index_t  ivec3
	#define uindex_t uvec3
	const uint index_dims = 3;
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
uniform transform_t traj_rel_color_transform;
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

// Calculate the index of the cell containing a given point.
ivec4 cell_index (vec4 point)
{
	return ivec4(round(point * hash_grid_scale));
}

// Lookup tables to calculate Z-order by interleaving bits, generated using bit_spread_lut in
// hash_grid.cxx.
#if HASH_GRID_SIGNATURE_FN == SIG_FN_Z_ORDER
#if HASH_GRID_LAYOUT != LAYOUT_XYZT
const uint[256] bit_spread_lut = {
	0x000000,0x000001,0x000008,0x000009,0x000040,0x000041,0x000048,0x000049,0x000200,0x000201,0x000208,0x000209,0x000240,0x000241,0x000248,0x000249,
	0x001000,0x001001,0x001008,0x001009,0x001040,0x001041,0x001048,0x001049,0x001200,0x001201,0x001208,0x001209,0x001240,0x001241,0x001248,0x001249,
	0x008000,0x008001,0x008008,0x008009,0x008040,0x008041,0x008048,0x008049,0x008200,0x008201,0x008208,0x008209,0x008240,0x008241,0x008248,0x008249,
	0x009000,0x009001,0x009008,0x009009,0x009040,0x009041,0x009048,0x009049,0x009200,0x009201,0x009208,0x009209,0x009240,0x009241,0x009248,0x009249,
	0x040000,0x040001,0x040008,0x040009,0x040040,0x040041,0x040048,0x040049,0x040200,0x040201,0x040208,0x040209,0x040240,0x040241,0x040248,0x040249,
	0x041000,0x041001,0x041008,0x041009,0x041040,0x041041,0x041048,0x041049,0x041200,0x041201,0x041208,0x041209,0x041240,0x041241,0x041248,0x041249,
	0x048000,0x048001,0x048008,0x048009,0x048040,0x048041,0x048048,0x048049,0x048200,0x048201,0x048208,0x048209,0x048240,0x048241,0x048248,0x048249,
	0x049000,0x049001,0x049008,0x049009,0x049040,0x049041,0x049048,0x049049,0x049200,0x049201,0x049208,0x049209,0x049240,0x049241,0x049248,0x049249,
	0x200000,0x200001,0x200008,0x200009,0x200040,0x200041,0x200048,0x200049,0x200200,0x200201,0x200208,0x200209,0x200240,0x200241,0x200248,0x200249,
	0x201000,0x201001,0x201008,0x201009,0x201040,0x201041,0x201048,0x201049,0x201200,0x201201,0x201208,0x201209,0x201240,0x201241,0x201248,0x201249,
	0x208000,0x208001,0x208008,0x208009,0x208040,0x208041,0x208048,0x208049,0x208200,0x208201,0x208208,0x208209,0x208240,0x208241,0x208248,0x208249,
	0x209000,0x209001,0x209008,0x209009,0x209040,0x209041,0x209048,0x209049,0x209200,0x209201,0x209208,0x209209,0x209240,0x209241,0x209248,0x209249,
	0x240000,0x240001,0x240008,0x240009,0x240040,0x240041,0x240048,0x240049,0x240200,0x240201,0x240208,0x240209,0x240240,0x240241,0x240248,0x240249,
	0x241000,0x241001,0x241008,0x241009,0x241040,0x241041,0x241048,0x241049,0x241200,0x241201,0x241208,0x241209,0x241240,0x241241,0x241248,0x241249,
	0x248000,0x248001,0x248008,0x248009,0x248040,0x248041,0x248048,0x248049,0x248200,0x248201,0x248208,0x248209,0x248240,0x248241,0x248248,0x248249,
	0x249000,0x249001,0x249008,0x249009,0x249040,0x249041,0x249048,0x249049,0x249200,0x249201,0x249208,0x249209,0x249240,0x249241,0x249248,0x249249,
};
#else
const uint[256] bit_spread_lut = {
	0x00000000,0x00000001,0x00000010,0x00000011,0x00000100,0x00000101,0x00000110,0x00000111,0x00001000,0x00001001,0x00001010,0x00001011,0x00001100,0x00001101,0x00001110,0x00001111,
	0x00010000,0x00010001,0x00010010,0x00010011,0x00010100,0x00010101,0x00010110,0x00010111,0x00011000,0x00011001,0x00011010,0x00011011,0x00011100,0x00011101,0x00011110,0x00011111,
	0x00100000,0x00100001,0x00100010,0x00100011,0x00100100,0x00100101,0x00100110,0x00100111,0x00101000,0x00101001,0x00101010,0x00101011,0x00101100,0x00101101,0x00101110,0x00101111,
	0x00110000,0x00110001,0x00110010,0x00110011,0x00110100,0x00110101,0x00110110,0x00110111,0x00111000,0x00111001,0x00111010,0x00111011,0x00111100,0x00111101,0x00111110,0x00111111,
	0x01000000,0x01000001,0x01000010,0x01000011,0x01000100,0x01000101,0x01000110,0x01000111,0x01001000,0x01001001,0x01001010,0x01001011,0x01001100,0x01001101,0x01001110,0x01001111,
	0x01010000,0x01010001,0x01010010,0x01010011,0x01010100,0x01010101,0x01010110,0x01010111,0x01011000,0x01011001,0x01011010,0x01011011,0x01011100,0x01011101,0x01011110,0x01011111,
	0x01100000,0x01100001,0x01100010,0x01100011,0x01100100,0x01100101,0x01100110,0x01100111,0x01101000,0x01101001,0x01101010,0x01101011,0x01101100,0x01101101,0x01101110,0x01101111,
	0x01110000,0x01110001,0x01110010,0x01110011,0x01110100,0x01110101,0x01110110,0x01110111,0x01111000,0x01111001,0x01111010,0x01111011,0x01111100,0x01111101,0x01111110,0x01111111,
	0x10000000,0x10000001,0x10000010,0x10000011,0x10000100,0x10000101,0x10000110,0x10000111,0x10001000,0x10001001,0x10001010,0x10001011,0x10001100,0x10001101,0x10001110,0x10001111,
	0x10010000,0x10010001,0x10010010,0x10010011,0x10010100,0x10010101,0x10010110,0x10010111,0x10011000,0x10011001,0x10011010,0x10011011,0x10011100,0x10011101,0x10011110,0x10011111,
	0x10100000,0x10100001,0x10100010,0x10100011,0x10100100,0x10100101,0x10100110,0x10100111,0x10101000,0x10101001,0x10101010,0x10101011,0x10101100,0x10101101,0x10101110,0x10101111,
	0x10110000,0x10110001,0x10110010,0x10110011,0x10110100,0x10110101,0x10110110,0x10110111,0x10111000,0x10111001,0x10111010,0x10111011,0x10111100,0x10111101,0x10111110,0x10111111,
	0x11000000,0x11000001,0x11000010,0x11000011,0x11000100,0x11000101,0x11000110,0x11000111,0x11001000,0x11001001,0x11001010,0x11001011,0x11001100,0x11001101,0x11001110,0x11001111,
	0x11010000,0x11010001,0x11010010,0x11010011,0x11010100,0x11010101,0x11010110,0x11010111,0x11011000,0x11011001,0x11011010,0x11011011,0x11011100,0x11011101,0x11011110,0x11011111,
	0x11100000,0x11100001,0x11100010,0x11100011,0x11100100,0x11100101,0x11100110,0x11100111,0x11101000,0x11101001,0x11101010,0x11101011,0x11101100,0x11101101,0x11101110,0x11101111,
	0x11110000,0x11110001,0x11110010,0x11110011,0x11110100,0x11110101,0x11110110,0x11110111,0x11111000,0x11111001,0x11111010,0x11111011,0x11111100,0x11111101,0x11111110,0x11111111,
};
#endif
#endif

// Hash a cell index into a 32-bit signature.
uint signature (index_t index)
{
	const uindex_t uidx = index;

#if HASH_GRID_SIGNATURE_FN == SIG_FN_MULT_XOR
	const uint[] c = {1u, 73856093u, 19349663u, 83492791u};

	uint sig = 0;
	for (uint d = 0; d < index_dims; ++d) sig ^= c[d]*uidx[d];
	return sig;
#elif HASH_GRID_SIGNATURE_FN == SIG_FN_XXHASH32
	// Prime constants.
	const uint[] c = {2246822519u, 3266489917u, 668265263u, 374761393u};

	uint sig = uidx[0] + c[3];
	for (uint d = 1; d < index_dims; ++d) {
		sig += c[1] * uidx[d];
		sig  = c[2] * ((sig << 17) | (sig >> 15));
	}
	sig = c[0] * (sig ^ (sig >> 15));
	sig = c[1] * (sig ^ (sig >> 13));
	return sig ^ (sig >> 16);
#elif HASH_GRID_SIGNATURE_FN == SIG_FN_Z_ORDER
	uint sig = 0;
	for (uint d = 0; d < index_dims; ++d)
		for (uint b = 0; b <= (HASH_GRID_LAYOUT == LAYOUT_XYZT ? 0 : 8); b += 8)
			sig |= bit_spread_lut[uidx[d]>>b & 0xff] << (b*index_dims + d);
	return sig;
#endif
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

// Load one of the trajectory intervals in a cell from the grid buffer.
interval_t load_interval (span_t intervals, uint index)
{
	const uvec4 data = load4(offset_bytes(intervals.start, index * sizeof_interval));
	return interval_t(data.xy, uintBitsToFloat(data.zw));
}

// Trajectories ====================================================================================
// Formulas for evaluating cubic Hermite splines are taken from
// https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Representations

// Calculate the coefficients of a trajectory segments's 3D position represented as a cubic Hermite
// spline in monomial basis ordered by increasing degree.
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
		  ( 2*t3 - 3*t2 + 1) * start.pos_rad.xyz
		+ (   t3 - 2*t2 + t) * start.tangent.xyz
		+ (-2*t3 + 3*t2    ) * end.pos_rad.xyz
		+ (   t3 -   t2    ) * end.tangent.xyz,
		mix(start.t[0], end.t[0], t)
	);
}

/// Calculate the derivative of a trajectory segment's position as represented by `position_coeffs`.
mat3 derive_coeffs (mat4x3 pos_coeffs)
{
	return mat3(pos_coeffs[1], 2*pos_coeffs[2], 3*pos_coeffs[3]);
}
/// Calculate the derivative of a trajectory's position w.r.t. curve parameter using coefficients
/// precomputed by `derive_coeffs` for fast evaluation of mutliple samples.
vec3 trajectory_derivative (mat3 coeffs, float t)
{
	return coeffs * vec3(1, t, t*t);
}
/// Calculate the derivative of a trajectory's position w.r.t. curve parameter.
vec3 trajectory_derivative (node_data_type start, node_data_type end, float t)
{
	const float dt2 = 2*t;
	const float dt3 = 3*t*t;
	return
		  ( 2*dt3 - 3*dt2    ) * start.pos_rad.xyz
		+ (   dt3 - 2*dt2 + 1) * start.tangent.xyz
		+ (-2*dt3 + 3*dt2    ) * end.pos_rad.xyz
		+ (   dt3 -   dt2    ) * end.tangent.xyz;
}

// Shading =========================================================================================
// Calculate the trajectory relation selected by `TRAJ_REL_FUNCTION` from a fixed starting point to
// one or more sampled points on a given interval, provided the samples lie within the query radius.
float eval_relation (
	vec4 base_point,
#if FN_USES_DERIVATIVE
	vec3 base_derivative,
#endif
	interval_t interval
) {
	// Load node data.
	const node_data_type n0 = nodes[interval.nodes[0]];
	const node_data_type n1 = nodes[interval.nodes[1]];

	// Intersect trajectory interval and evaluated time frame.
	const float start    = max(interval.time[0], base_point[3] - traj_rel_radius[1]);
	const float end      = min(interval.time[1], base_point[3] + traj_rel_radius[1]);
	const float timespan = end - start;
	if (timespan <= 0) return 0;

	// Determine how often the interval should be sampled.
	const float num_samples = ceil(timespan * traj_rel_sample_rate);
	if (TRAJ_REL_FUNCTION == FN_DBG_NUM_SAMPLES)
		// TODO: For whatever reason, if the return value of this function is a simple conditional
		// (0 if timespan <= 0, potentially different value otherwise), it can freeze the system.
		// This was observed under amdgpu, but only in the deferred shading pass and only if the
		// result is trivial; if actual samples are evaluated there is no issue.
		// For now, the problematic conditional is avoided by always returning 0.
		return TRAJ_REL_SHADING == 3 ? 0 : num_samples;

	// Divide the evaluated range into equal steps.
	const float time_scale  = 1.0 / (n1.t[0] - n0.t[0]);
	const float sampling    = 1.0 / num_samples;
	const float sample_step = timespan * time_scale * sampling;

	// Map time to curve parameter.
	const float tmin = (start - n0.t[0]) * time_scale;
	const float tmax = (end   - n0.t[0]) * time_scale;

	// Calculate spline coefficients.
	const mat4x3 coeffs = position_coeffs(n0, n1);
#if FN_USES_DERIVATIVE
	const mat3 coeffs_dt = derive_coeffs(coeffs);
#endif

	const float radius2 = traj_rel_radius[0] * traj_rel_radius[0];

	// Evaluate the relation at one or more sample points along the interval.
	float result = 0.0;
	for (float t = tmin + 0.5*sample_step; t < tmax; t += sample_step) {
		// Evaluate the trajectory for the current curve parameter.
		const vec3 sample_pos = eval_position(coeffs, t);

		// Ignore points outside the evaluation radius.
		const vec3 offset = sample_pos - base_point.xyz;
		const float dist2 = dot(offset, offset);
		if (dist2 > radius2) continue;

		// Evaluate the relation.
		#if TRAJ_REL_FUNCTION == FN_PROXIMITY
			result += traj_rel_radius[0] - sqrt(dist2);
		#elif TRAJ_REL_FUNCTION == FN_ALIGNMENT
			result += dot(base_derivative, normalize(trajectory_derivative(coeffs_dt, t)))
				* exp(dist2 * (-6/radius2)); // Gaussian weight function.
		#elif TRAJ_REL_FUNCTION == FN_DBG_NUM_EVALS
			result += 1;
		#endif
	}
	if (TRAJ_REL_FUNCTION == FN_DBG_NUM_EVALS) return result;

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
	#if TRAJ_REL_FUNCTION == FN_DBG_SEG_T
		return seg_t;
	#elif TRAJ_REL_FUNCTION == FN_DBG_VELOCITY
		// Derivative w.r.t. curve parameter, divide by duration to get physical velocity.
		return length(trajectory_derivative(start, end, seg_t)) / (end.t[0] - start.t[0]);
	#endif

	// Calculate data-space coordinates and grid cell at the given trajectory point.
	const vec4 local_point  = trajectory_point(start, end, seg_t);
	const ivec4 local_index = cell_index(local_point);

	// Color by grid cell (spatial).
	#if TRAJ_REL_FUNCTION == FN_DBG_INDEX_XYZ
		return uintBitsToFloat(packUnorm4x8(
			vec4(vec3(local_point * hash_grid_scale - local_index + 0.5), 0)
		));
	// Color by grid cell (temporal).
	#elif TRAJ_REL_FUNCTION == FN_DBG_INDEX_T
		return local_point[3] * hash_grid_scale[3] - local_index[3] + 0.5;
	// Color by cell hash.
	#elif TRAJ_REL_FUNCTION == FN_DBG_SIGNATURE
		return signature(index_t(local_index)) / float(~0u);
	// Color by hash bucket load.
	#elif TRAJ_REL_FUNCTION == FN_DBG_BUCKET_LOAD
	{
		// Find the bucket containing the local point.
		const span_t table = find_table(local_index[3]);
		if (table.start == null) return uintBitsToFloat(highlight_value);
		const ptr_t bucket = bucket(table, signature(index_t(local_index)), 1);
		if (bucket == null) return uintBitsToFloat(highlight_value);
		// Count how many of its slots are in use.
		uint fill = slots_per_bucket;
		while (fill > 0 && load_slot(bucket, fill - 1).cell == null) --fill;
		return float(fill) / slots_per_bucket;
	}
	// Color by local trajectory interval.
	#elif TRAJ_REL_FUNCTION == FN_DBG_LOCAL_INTERVAL
	{
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
		return uintBitsToFloat(highlight_value);
	}
	#endif

	// AABB of cells to query.
	const vec4 max_offset = vec4(vec3(traj_rel_radius[0]), traj_rel_radius[1]);
	const ivec4 min_cell  = cell_index(local_point - max_offset);
	const ivec4 max_cell  = cell_index(local_point + max_offset);

	// Iff the distance squared between a cell's center and the local point is larger than this
	// value, all points within that cell are outside the evaluation radius.
	const float max_cell_dist  = traj_rel_radius[0] + length(hash_grid_cell_size.xyz)*0.5;
	const float max_cell_dist2 = max_cell_dist*max_cell_dist;

#if TRAJ_REL_FUNCTION == FN_ALIGNMENT
	// Normalized trajectory direction for calculating angle to other trajectories.
	const vec3 local_derivative = normalize(trajectory_derivative(start, end, seg_t));
#endif

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
			if (TRAJ_REL_FUNCTION == FN_DBG_SKIPPED_CELLS) ++result;
			continue;
		}

		// Search the hash table for the current cell and return the intervals it contains.
		const span_t intervals = query(table, index);

		#if TRAJ_REL_FUNCTION == FN_DBG_NUM_CELLS
			result += float(intervals.len != 0);
			continue;
		#elif TRAJ_REL_FUNCTION == FN_DBG_NUM_INTERVALS
			result += intervals.len;
			continue;
		#endif

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
			) result += eval_relation(
				local_point,
			#if FN_USES_DERIVATIVE
				local_derivative,
			#endif
				interval
			);
		}
	}
	}

	// Normalize the relation value.
	const float norm_time = traj_rel_normalize ? 2*traj_rel_radius[1] : 1;

	#if TRAJ_REL_FUNCTION == FN_PROXIMITY
		result /= (traj_rel_radius[0] * norm_time);
	#elif TRAJ_REL_FUNCTION == FN_ALIGNMENT
		result /= norm_time;
	#elif TRAJ_REL_FUNCTION == FN_DBG_SKIPPED_CELLS
		result /= float(dot(max_cell - min_cell + 1, vec4(1)));
	#endif
	return result;
}

// Visualize a trajectory relation value as a color.
vec3 otv_shade_relation (float value)
{
	const uint bits = floatBitsToUint(value);
	// Special values.
	if (bits == background_value) return traj_rel_background_color;
	if (bits == highlight_value)  return traj_rel_highlight_color;
	// Multivariate mapping.
	if (TRAJ_REL_FUNCTION == FN_DBG_INDEX_XYZ) return unpackUnorm4x8(bits).xyz;

	// Mapping parameters.
	vec2 range     = traj_rel_color_range;
	transform_t tf = traj_rel_color_transform;
	// The diverging logarithmic transform is constructed by applying a logarithmic transform to the
	// upper half of the range, then rotating it into the lower half around the midpoint.
	if (tf == transform_symlog) range[0] = 0.5*(range[0] + range[1]);

	// Linearly map `range` to [0, 1].
	value = (value - range[0])/(range[1] - range[0]);

	if (tf != transform_linear) {
		// Symlog: Reflect the upper half of the original range into the lower half.
		const bool rotate = value < 0;
		if (tf == transform_symlog) value = abs(value);

		// Apply base 10 logarithm, preserving points (range[0], 0) and (range[1], 1).
		value = log2(9*value + 1) / 3.32192809489;

		if (tf == transform_symlog) {
			// Complete the rotation by reflecting values in the lower half range.
			if (rotate) value = -value;
			value = 0.5*value + 0.5; // Map values from [-1, 1] to [0, 1].
		}
	}
	// Get color from texture.
	return map_to_color(value, TRAJ_REL_FUNCTION == FN_DBG_SIGNATURE ? 21 : traj_rel_color_map);
}
