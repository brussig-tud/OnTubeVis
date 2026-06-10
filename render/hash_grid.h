#pragma once

// Debug builds can have a configurable amount of additional validation through duplicate state.
#if !defined(OTV_HASH_GRID_VALIDATION) && !defined(NDEBUG)
#define OTV_HASH_GRID_VALIDATION 2
#elif !defined(OTV_HASH_GRID_VALIDATION) && defined(NDEBUG)
#define OTV_HASH_GRID_VALIDATION 0
#elif defined(OTV_HASH_GRID_VALIDATION) && defined(NDEBUG)
#warning OTV_HASH_GRID_VALIDATION has no effect in release builds.
#endif

// C++ STL
#include <array>
#include <cstdint>
#include <ostream>
#include <random>
#include <span>

#if OTV_HASH_GRID_VALIDATION
#include <bitset>
#include <unordered_map>
#endif

// CGV framework
#include <cgv/math/fvec.h>
#include <cgv/render/shader_code.h>

// local includes
#include "pmr/base.h"
#include "render/common.h"

// forward declarations
namespace cgv::render {
	class context;
	class shader_program;
}


namespace otv {

/// An acceleration data structure storing which trajectory intervals lie in each cell of an
/// infinite regular grid, stored as a hash map that can be made accessible to the GPU.
class hash_grid {
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
		cgv::vec4 cell_size {1};
		/// Minimum distance in (arclength, time) between trajectory samples used to determine in
		/// which grid cells a trajectory segment lies.
		/// Smaller steps yield more accurate results at the expense of slower insertions and slower
		/// evaluation of relations due to more, smaller intervals.
		cgv::vec2 sample_step {0.1};
		/// Describes which dimensions the grid indexes and how it is organized in memory.
		layout layout {layout::xyzt};
		/// Determines how cell signatures are calculated.
		signature_fn signature_fn {signature_fn::xxhash32};
	};

	/// Create a grid with no backing memory, so it cannot store anything.
	[[nodiscard]] hash_grid() = default;
	/// Create an empty grid that can allocate memory from the given region.
	/// The memory region must not be null and must outlive the grid.
	/// New tables are created with at least `initial_buckets` buckets.
	[[nodiscard]] hash_grid(pmr::memory_region*, params const&, uint32_t initial_buckets);

	// Forbid copying.
	hash_grid(hash_grid const&) = delete;
	auto operator= (hash_grid const&) = delete;

	// Allow moving.
	[[nodiscard]] hash_grid(hash_grid&&) noexcept;
	auto operator= (hash_grid&&) noexcept -> hash_grid&;

	/// Free all allocated memory when an instance is destroyed.
	~hash_grid() noexcept;

	/// Update the grid with a new trajectory segment.
	/// `start` and `end` must be stored at `node_idcs` in the render buffer.
	void add_segment (
		node_attribs const& start,
		node_attribs const& end,
		cgv::uvec2          node_idcs,
		cgv::mat4 const&    t_to_s
	);

	/// Write some metrics useful for evaluation to the given stream. If the macro
	/// OTV_HASH_GRID_NO_STATS is defined, output is reduced.
	auto write_stats (std::ostream&) const -> std::ostream&;

	/// Statically configure shaders through text substitution.
	/// `buffer_binding` must be the index at which the GPU buffer used by this grid will be bound.
	void set_shader_opts (cgv::render::shader_compile_options&, GLuint buffer_binding) const;
	/// Dynamically configure shaders through uniforms.
	void set_uniforms (cgv::render::context&, cgv::render::shader_program&) const;

	/// Free all allocated memory, emptying the grid.
	void free () noexcept;
	/// Forget all entries without freeing associated memory.
	void leak () noexcept;

private:
	/// The interval of a trajectory segment that lies within a given grid cell.
	struct alignas(16) interval_t {
		/// Indices into the render buffer of the nodes defining the segment.
		/// Both nodes may lie outside the grid cell this interval belongs to.
		cgv::uvec2 nodes;
		/// Timestamps at which the segment enters and exits the grid cell this interval belongs to.
		cgv::vec2 time;
	};

	/// Spatiotemporal vector (x, y, z, t) used to identify a cell in the grid.
	/// Depending on the grid's layout, the time component may be ignored in certain contexts.
	using index_t = cgv::ivec4;

	/// Hash of a cell index.
	/// Like in Mega-KV (Zhang et al. 2015), only the signature is stored directly in the buckets
	/// of the hash tables to reduce memory transfers.
	struct signature_t {uint32_t value = ~0u;};

	/// The range in which `_memory` allocates grid data.
	using memory_region = std::span<std::byte>;

	/// A pointer represented as a 32 bit offset into `_memory` for use in shaders.
	template <class T>
	struct ptr_t {
		/// Offset into `_memory` as a multiple of `address_unit` bytes.
		/// Since 0 is a valid allocation, null pointers are represented as ~0.
		uint32_t address {~0u};

		/// Create a null pointer.
		[[nodiscard]] constexpr ptr_t() noexcept = default;
		/// Convert a native pointer to an offset.
		/// The pointer must lie within the memory region, and all future operations on this
		/// instance must use the same region.
		[[nodiscard]] constexpr ptr_t(memory_region, T*) noexcept;

		/// `false` iff this instance is null.
		[[nodiscard]] constexpr operator bool () const noexcept;
		/// Check if two pointers refer to the same data, assuming they use the same memory region.
		[[nodiscard]] constexpr auto operator== (ptr_t) const noexcept -> bool;

		/// Convert from offset to native pointer.
		/// Must be called with the same memory region that the instance was created with.
		[[nodiscard]] constexpr auto get (memory_region) const noexcept -> T*;
	};

	/// Stores a grid cell's index and the trajectory intervals it contains as a dynamic array with
	/// pointer semantics, meaning copies are shallow and allocations are not automatically freed.
	struct cell_t {
		/// Format of the actual allocations that instances point to.
		struct alignas(interval_t) data_t {
			/// The index identifying the cell.
			alignas(16) index_t index;
			/// Number of trajectory intervals in the cell.
			uint32_t size;
			/// Maximum number of intervals that fit in the current allocation.
			uint32_t capacity;
			/// Trajectory intervals within the cell are stored like in a flexible array member
			/// `interval_t[]`, starting at offset `sizeof(data_t)`.
			/// Use this function to index into the array.
			/// As with a C array, there is no bounds check, and memory may be uninitialized.
			[[nodiscard]] auto interval(size_t idx) noexcept -> interval_t*;
		};

		/// Pointer to the allocation within `_memory` that stores the data for this cell.
		ptr_t<data_t> data;

		/// Create a null handle.
		[[nodiscard]] constexpr cell_t() noexcept {}
		/// Allocate a new cell.
		/// All future operations on the cell must use the same memory region.
		[[nodiscard]] cell_t(pmr::memory_region&, index_t);

		/// Store a trajectory interval in the cell.
		void add_interval (pmr::memory_region&, interval_t);
		/// Deallocate cell data.
		void free (pmr::memory_region&) noexcept;

	private:
		/// Allocate data with enough memory to store the requested number of intervals.
		[[nodiscard]] static auto allocate (pmr::memory_region&, uint32_t capacity) -> data_t*;
	};

	/// A slot in one of the hash table's buckets, storing a cell pointer and its index signature.
	/// Iff the cell is null the slot is considered empty and the signature indeterminate.
	struct alignas(8) slot_t {
		signature_t signature;
		cell_t      cell;
	};
	/// A bucket of the hash table, dimensioned and aligned to match GPU memory transfers.
	struct bucket_t {
		static constexpr auto size_bytes = size_t{128};
		static constexpr auto num_slots  = size_bytes / sizeof(slot_t);
		alignas(size_bytes) std::array<slot_t, num_slots> slots;
	};

	/// Nullable pointer to a contiguous array with dynamic extent, stored as 32 bit integers for
	/// use in shaders.
	template <class T>
	struct alignas(8) span_t {
		/// Offset of the first entry within the memory region.
		ptr_t<T> start {};
		/// Number of elements in the array.
		uint32_t len {0};

		/// Create a null instance.
		[[nodiscard]] constexpr span_t() noexcept = default;
		/// Convert a native `std::span` to 32 bit offsets.
		/// The span must lie entirely within the memory region, and all future operations on this
		/// instance must use the same region.
		[[nodiscard]] constexpr span_t(memory_region, std::span<T>) noexcept;

		/// Convert from offsets to native span.
		/// The memory region must the same one that the instance was created with.
		[[nodiscard]] constexpr auto get (memory_region) const noexcept -> std::span<T>;
	};


	/// Random number generator used to determine which entry to displace from a full bucket when
	/// inserting a new cell.
	std::minstd_rand _rng {std::random_device{}()};
	/// The memory resource in which hash buckets and grid cells are dynamically allocated.
	pmr::memory_region* _memory {};
#ifndef OTV_HASH_GRID_NO_STATS
	struct {
		/// The number of occupied grid cells.
		uint32_t num_cells {0};
		/// The total number of segment intervals stored in the grid.
		uint32_t num_intervals {0};
	}
	/// Tracks additional metrics for evaluation.
	_stats {};
#endif
	union {
		/// Hash table buckets each containing multiple slots for cells.
		/// Length is a power of two.
		/// Used by all layouts except `t_xyz`.
		span_t<bucket_t> buckets {};
		struct {
			/// Hash buckets per timestep, in ascending order.
			/// The length of every table is a power of two.
			span_t<span_t<bucket_t>> buckets {};
			/// Array storing the temporal index values that the entries of `buckets` correspond to.
			/// Has the same length as `buckets`.
			ptr_t<int32_t> timesteps {};
			/// Allocation size in elements of both `buckets` and `timesteps`.
			/// Only the first `buckets.len` elements are valid.
			uint32_t capacity {0};
		}
		/// Separate hash tables for each timestep stored as a struct of arrays.
		/// Used by layout `t_xyz`.
		tables;

#if __cpp_lib_is_layout_compatible
		// Ensure that all variants have a common `span_t` subsequence.
		static_assert(std::is_layout_compatible_v<decltype(buckets), decltype(tables.buckets)>);
#endif
	}
	/// Hash buckets storing grid cells.
	/// The active member is determined by `_layout`.
	_data {};
	// Required for variants to have a common subsequence.
	static_assert(std::is_standard_layout_v<decltype(_data)>);

#if OTV_HASH_GRID_VALIDATION
	/// Test the implementation by mirroring operations with an STL container.
	struct {
		/// Function object hashing grid indices for use in STL containers.
		/// Uses a different hash function than the main table.
		struct index_hash_t {
			[[nodiscard]] auto operator()(index_t const& idx) const noexcept -> size_t
			{
				using index_bits = std::bitset<sizeof(index_t) * 8>;
				return std::hash<index_bits>{}(*reinterpret_cast<index_bits const*>(&idx));
			}
		};

		/// Stores the number of trajectory intervals in each grid cell.
		std::unordered_map<index_t, uint32_t, index_hash_t> cell_fill {};
	} _validation {};
#endif
	/// Extent of each grid cell.
	cgv::vec4 _cell_size {};
	/// Points are mapped to cell indices by `index = round(point * _scale)`.
	/// Reciprocal of `_cell_size`.
	cgv::vec4 _scale {};
	/// Minimum distance (arclength, time) between sample points when adding a segment to the grid.
	cgv::vec2 _sample_step;
	/// Describes which dimensions the grid indexes and how it is organized in memory.
	layout _layout {};
	/// Determines the function used to calculate cell signatures.
	signature_fn _signature_fn {};
	/// New hash tables are allocated with 2 ^ `_initial_buckets` buckets.
	uint8_t _initial_buckets {};

	/// Allocate a contiguous array from `_memory` without initializing its elements.
	template <class T>
	[[nodiscard]] auto allocate (uint32_t count) -> std::span<T>;
	/// Deallocate an array obtained from `_memory` without destroying its elements.
	template <class T>
	void free (std::span<T>) noexcept;
	/// Find or create a cell by index, then insert a trajectory interval into that cell.
	void add_interval (index_t, interval_t);
	/// Access a cell by its index.
	/// If the cell is not yet stored in the grid, it is created.
	auto cell (index_t) -> cell_t&;
	/// Access the hash buckets containing all cells for a given temporal index.
	auto table (int32_t timestep) -> span_t<bucket_t>&;
	/// Find a cell in a hash table by its index.
	/// If there is no matching entry, insert `new_cell`, which must have the same index.
	/// If `new_cell` is null, it is allocated on demand.
	/// Returns a pointer to the queried entry in the table or nullptr if insertion failed.
	auto find_or_insert (std::span<bucket_t>, index_t, cell_t new_cell = {}) -> cell_t*;

	/// Hash a cell index into a shorter signature.
	[[nodiscard]] auto signature (index_t) const noexcept -> signature_t;
	/// Access the bucket in which a cell with the given signature is stored by `hash_fn`.
	[[nodiscard]] auto bucket (std::span<bucket_t>, signature_t, uint8_t hash_fn) noexcept
		-> bucket_t&;
};

/// Return the unqualified identifier for the given enum value.
[[nodiscard]] constexpr auto enum_id (hash_grid::layout layout) noexcept -> std::string_view
{
	using std::operator""sv;
	return std::array{"xyz", "t_xyz", "xyzt"}[static_cast<size_t>(layout)];
}

/// Return the unqualified identifier for the given enum value.
[[nodiscard]] constexpr auto enum_id (hash_grid::signature_fn sigfn) noexcept -> std::string_view
{
	using std::operator""sv;
	return std::array{"mult_xor", "xxhash32", "z_order"}[static_cast<size_t>(sigfn)];
}

} // namespace otv
