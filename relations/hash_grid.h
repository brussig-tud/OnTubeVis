#pragma once

// Debug builds can have a configurable amount of additional validation through duplicate state.
#if !defined(OTV_HASH_GRID_VALIDATION)
	#if defined(NDEBUG)
		#define OTV_HASH_GRID_VALIDATION 0
	#else
		#define OTV_HASH_GRID_VALIDATION 2
	#endif
#elif defined(NDEBUG) && OTV_HASH_GRID_VALIDATION != 0
	#warning OTV_HASH_GRID_VALIDATION has no effect if NDEBUG is defined.
#endif

#include <array>
#include <cstdint>
#include <memory>
#include <random>
#include <span>

#if OTV_HASH_GRID_VALIDATION
#include <bitset>
#include <unordered_map>
#endif

#include <cgv/math/fmat.h>
#include <cgv/math/fvec.h>

#include "util/gl.h"

// forward declarations
namespace cgv::render {
	class context;
	class shader_program;
	class shader_compile_options;
}


/// An acceleration data structure storing which trajectory intervals lie in each cell of an
/// infinite regular grid, stored as a hash map that can be uploaded to the GPU.
class hash_grid {
	using vec2 = cgv::vec2;
	using vec3 = cgv::vec3;
	using vec4 = cgv::vec4;

public:
	/// Describes which dimensions a grid indexes and how it is organized in memory.
	/// Regardless of layout, queries are always spatiotemporal 4D vectors (x, y, z, t).
	enum class layout : uint32_t { // uint8_t causes problems with CGV GUI.
		xyz,   // Single 3D spatial grid, time is ignored (infinite cell size).
		t_xyz, // Separate 3D spatial grids for each time index.
		xyzt,  // Single 4D spatiotemporal grid.
	};
	/// Hash function used to reduce cell indices to a 32 bit signature.
	enum class signature_fn : uint32_t {
		mult_xor, // Prime multiplication, then XOR reduction following Teschner et al.
		xxhash32, // xxHash32 as implemented by Jarzynski and Olano.
		z_order,  // Z-order curve obtained by interleaving bits, see e.g. García et al.
	};

	/// Fundamental aspects of a grid's structure that cannot be changed after construction.
	struct params {
		/// The extent of each grid cell in the dimensions (x, y, z, t).
		/// For 3D grids (`layout == xyz`) the time component is implicitely infinite.
		vec4 cell_size {1};
		/// Minimum distance in (arclength, time) between trajectory samples used to determine in
		/// which grid cells a trajectory segment lies.
		/// Smaller steps yield more accurate results at the expense of slower insertions and slower
		/// evaluation of relations due to more, smaller intervals.
		vec2 sample_step {0.1};
		/// Describes which dimensions the grid indexes and how it is organized in memory.
		layout layout {layout::xyzt};
		/// Determines how cell signatures are calculated.
		signature_fn signature_fn {signature_fn::z_order};
	};

	[[nodiscard]] hash_grid() = default;
	[[nodiscard]] hash_grid(params const&, uint32_t initial_buckets);

	// Forbid copying.
	hash_grid(hash_grid const&) = delete;
	auto operator= (hash_grid const&) = delete;

	// Allow moving.
	[[nodiscard]] hash_grid(hash_grid&&) noexcept = default;
	auto operator= (hash_grid&&) noexcept -> hash_grid& = default;

	struct node_attribs {
		vec3  pos;
		float time;
		vec3  tangent;
	};

	/// Update the grid with a new trajectory segment.
	/// `start` and `end` must be stored at `node_idcs` in the render buffer.
	void add_segment (
		node_attribs const& start,
		node_attribs const& end,
		cgv::uvec2          node_idcs,
		cgv::mat4 const&    t_to_s
	);

	/// Adapt the grid to its content without changing parameters. Currently, this means sorting
	/// cells along a Z-order curve and rebuilding tables to reduce empty slots. To be called after
	/// all segments have been inserted.
	void reorganize ();

	/// Return some metrics for evaluation in human-readable form.
	[[nodiscard]] auto stats () const -> std::string;

	/// Store a linear representation of the grid in a newly created GL buffer.
	[[nodiscard]] auto upload () -> gl_buffer;

	/// Statically configure shaders through text substitution.
	void set_shader_opts (cgv::render::shader_compile_options&) const;
	/// Dynamically configure shaders through uniforms.
	void set_uniforms (cgv::render::context&, cgv::render::shader_program&) const;

	[[nodiscard]] auto num_buckets () const -> size_t;
	[[nodiscard]] auto buffer_size () const -> size_t;

private:
	/// The interval of a trajectory segment that lies within a given grid cell.
	struct Interval {
		/// Index into the render buffer of the segment's start node.
		uint32_t start_node;
		/// Two packed 16 bit unsigned normalized integers in range [0, 1] indicating which part of
		/// the segment lies in the cell, relative to the segment's duration. The lower bound is
		/// stored in the less significant, the upper bound in the more significant bytes.
		uint32_t range;
	};

	/// Spatiotemporal vector (x, y, z, t) used to identify a cell in the grid.
	/// Depending on the grid's layout, the time component may be ignored in certain contexts.
	using Index = cgv::ivec4;

	/// Hash of a cell index. Like in Mega-KV (Zhang et al. 2015), only the signature is stored
	/// directly in the buckets of the hash tables to reduce memory transfers.
	struct Signature {uint32_t value = -1;};

	struct Cell {
		Index index {};
		std::vector<Interval> intervals {};
	};

	/// A slot in one of the hash table's buckets, storing a cell pointer and its index signature.
	/// Iff the cell is `no_cell` the slot is considered empty and the signature ignored.
	struct Slot {
		Signature signature {};
		uint32_t  cell      {no_cell};
	};
	static constexpr uint32_t no_cell = -1;
	/// A bucket of the hash table, sized to hopefully match GPU memory transfers.
	struct Bucket {
		static constexpr auto size_bytes = size_t{128};
		static constexpr auto num_slots  = size_bytes / sizeof(Slot);
		std::array<Slot, num_slots> slots {};
	};

	/// Bucket vector for a hash table.
	struct Table {
		std::unique_ptr<Bucket[]> data {};
		uint32_t length {}; // number of buckets
		uint32_t num_entries {}; // number of filled slots
		/// Only used by the strided 3D layout, which has one table per temporal index value.
		int32_t timestep {};

		[[nodiscard]] constexpr Table() = default;
		[[nodiscard]] Table(uint32_t num_buckets, int32_t timestep);
		[[nodiscard]] constexpr auto buckets () noexcept -> std::span<Bucket>;
		[[nodiscard]] constexpr auto buckets () const noexcept -> std::span<const Bucket>;
	};
	/// Hash buckets. Length one unless the strided layout is used.
	std::vector<Table> _tables {};
	/// Grid cells containing at least one interval.
	std::vector<Cell> _cells {};

#if OTV_HASH_GRID_VALIDATION
	/// Test the implementation by mirroring operations with an STL container.
	struct {
		/// Function object hashing grid indices for use in STL containers.
		/// Uses a different hash function than the main table.
		struct index_hash_t {
			[[nodiscard]] auto operator()(Index const& idx) const noexcept -> size_t
			{
				using index_bits = std::bitset<sizeof(Index) * 8>;
				return std::hash<index_bits>{}(*reinterpret_cast<index_bits const*>(&idx));
			}
		};

		/// Stores the number of trajectory intervals in each grid cell.
		std::unordered_map<Index, uint32_t, index_hash_t> cell_fill {};
	} _validation {};
#endif

	/// Random number generator used to determine which entry to displace from a full bucket when
	/// inserting a new cell.
	std::minstd_rand _rng {std::random_device{}()};

	/// Extent of each grid cell.
	vec4 _cell_size {};
	/// Points are mapped to cell indices by `index = round(point * _scale)`.
	/// Reciprocal of `_cell_size`.
	vec4 _scale {};
	/// Minimum distance (arclength, time) between sample points when adding a segment to the grid.
	vec2 _sample_step {};
	/// Number of buckets in a newly allocated table.
	uint32_t _initial_buckets {};
	/// Total number of segment intervals stored in all grid cells.
	uint32_t _num_intervals {};
	/// Describes which dimensions the grid indexes and how it is organized in memory.
	layout _layout {};
	/// Determines the function used to calculate cell signatures.
	signature_fn _signature_fn {};

	/// Find or create a cell by index, then insert a trajectory interval into that cell.
	void add_interval (Index cell, uint32_t start_node, vec2 range);
	/// Access a grid cell by its spatiotemporal index. If the cell does not yet exist, it is
	/// created empty and added to the grid. If the cell exists in `_cells` but is not stored in the
	/// correct hash table (e.g. when recreating tables as part of `reorganize`), its handle may be
	/// given as `new_cell`, which will then be inserted into the appropriate table instead.
	/// During insertion, hash tables may be resized if necessary, with capacity multiplied by
	/// `growth_factor` each time. Therefore, `growth_factor` must be > 1.
	[[nodiscard]] auto find_or_insert (Index, float growth_factor, uint32_t new_cell = no_cell)
		-> Cell&;
	/// Access the hash buckets containing all cells for a given temporal index.
	auto table (int32_t timestep) -> Table&;
	/// Find a cell in a hash table by its index. If there is no matching entry, insert `new_cell`,
	/// which must have the same index. If `new_cell` is `no_cell`, it is allocated on demand.
	/// Returns a pointer to the queried entry in the table, or nullptr if insertion failed.
	auto try_find_or_insert (Table&, Index, uint32_t new_cell = no_cell) -> Cell*;

	/// Hash a cell index into a shorter signature.
	[[nodiscard]] auto signature (Index) const noexcept -> Signature;
	/// Access the bucket in which a cell with the given signature is stored by `hash_fn`.
	[[nodiscard]] auto bucket (std::span<Bucket>, Signature, uint8_t hash_fn) noexcept -> Bucket&;
};

/// Return the unqualified identifier for the given enum value.
[[nodiscard]] constexpr auto enum_id (hash_grid::layout layout) noexcept -> std::string_view
{
	using std::operator""sv;
	return std::array{"xyz", "t_xyz", "xyzt"}[static_cast<size_t>(layout)];
}

/// Return the unqualified identifier for the given enum value.
[[nodiscard]] constexpr auto enum_id (hash_grid::signature_fn fn) noexcept -> std::string_view
{
	using std::operator""sv;
	return std::array{"mult_xor", "xxhash32", "z_order"}[static_cast<size_t>(fn)];
}

