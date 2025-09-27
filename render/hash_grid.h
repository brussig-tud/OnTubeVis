#pragma once

// Additional validation can be turned on or off independently from the standard debug flag.
#ifndef OTV_HASH_GRID_VALIDATION
#ifndef NDEBUG
#define OTV_HASH_GRID_VALIDATION 1
#else
#define OTV_HASH_GRID_VALIDATION 0
#endif
#endif

// C++ STL
#include <array>
#include <cstdint>
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
	/// The coordinate system that defines the grid.
	constexpr static enum class dimensions : uint8_t {
		xyz  = 3, // 3D spatial grid.
		xyzt = 4, // 4D spatio-temporal grid.
	} dimensions = dimensions::xyzt;
	/// A vector in the space that the grid indexes.
	using coord_t = std::conditional_t<dimensions == dimensions::xyz, cgv::vec3, cgv::vec4>;

	/// Create an empty grid.
	[[nodiscard]] hash_grid() = default;
	/// Create a grid with 2 ^ `order` hash buckets.
	/// The memory region must not be null and must outlive the grid.
	[[nodiscard]] hash_grid(pmr::memory_region*, cgv::vec4 cell_size, uint8_t order);

	// Forbid copying.
	hash_grid(hash_grid const&) = delete;
	auto operator= (hash_grid const&) = delete;

	// Allow moving.
	[[nodiscard]] hash_grid(hash_grid&&) noexcept;
	auto operator= (hash_grid&&) noexcept -> hash_grid&;

	/// Free all allocated memory when an instance is destroyed.
	~hash_grid() noexcept
	{
		free();
	}

	/// Update the grid with a new trajectory segment.
	/// `start` and `end` must be stored at `node_idcs` in the render buffer.
	void add_segment (
		node_attribs const& start,
		node_attribs const& end,
		cgv::uvec2          node_idcs,
		cgv::mat4 const&    t_to_s
	);

	/// Statically configure shaders through macros.
	/// `buffer_binding` must be the index at which the GPU buffer used by this grid will be bound.
	void set_defines (cgv::render::shader_define_map&, GLuint buffer_binding) const;
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

	/// Key used to identify a cell in the grid.
	using index_t = std::conditional_t<dimensions == dimensions::xyz, cgv::ivec3, cgv::ivec4>;

	/// Hash of a cell index.
	/// Like in Mega-KV (Zhang et al. 2015), only the signature is stored directly in the buckets
	/// of the hash tables to reduce memory transfers.
	struct signature_t {uint32_t value = ~0u;};

	/// Stores a grid cell's index and the trajectory intervals it contains as a dynamic array with
	/// pointer semantics, meaning copies are shallow and allocations are not automatically freed.
	struct cell_t {
		/// Format of the actual allocations instances point to.
		struct data_t {
			/// The index identifying the cell.
			alignas(16) index_t index;
			/// Number of trajectory intervals in the cell.
			alignas(16) uint32_t size;
			/// Maximum number of intervals that fit in the current allocation.
			uint32_t capacity;
			/// Trajectory intervals within the cell.
			interval_t intervals[];
		};

		/// Create a null handle.
		[[nodiscard]] constexpr cell_t() noexcept {};
		/// Allocate a new cell.
		/// All future operations on the cell must use the same memory region.
		[[nodiscard]] cell_t(pmr::memory_region&, index_t);

		/// Check if a handle is not null.
		[[nodiscard]] constexpr operator bool () const noexcept;
		/// Check if two handles point to the same data, assuming they reside in the same buffer.
		[[nodiscard]] constexpr auto operator== (cell_t const&) const noexcept -> bool;

		/// Dereference the handle to access the cell data.
		[[nodiscard]] auto get (std::span<std::byte>) noexcept -> data_t&;
		/// Store a trajectory interval in the cell.
		void add_interval (pmr::memory_region&, interval_t);
		/// Deallocate cell data.
		void free (pmr::memory_region&) noexcept;

	private:
		/// Allocate data with enough memory to store the requested number of intervals.
		[[nodiscard]] static auto allocate (pmr::memory_region&, uint32_t capacity) -> data_t&;

		/// Offset of the data allocation within the grid buffer.
		uint32_t _address = ~0u;
	};

	/// A slot in one of the hash table's buckets, storing a cell pointer and its index signature.
	/// Iff the cell is null the slot is considered empty and the signature indeterminate.
	struct alignas(8) slot_t {
		signature_t signature;
		cell_t      cell;
	};
	/// A bucket of the hash table, dimensioned and aligned to match GPU memory transfers.
	struct bucket_t {
		static constexpr auto size_bytes = 128uz;
		static constexpr auto num_slots  = size_bytes / sizeof(slot_t);
		alignas(size_bytes) std::array<slot_t, num_slots> slots;
	};


	/// Random number generator used to determine which entry to displace from a full bucket when
	/// inserting a new cell.
	std::minstd_rand _rng {std::random_device{}()};
	/// The memory resource in which hash buckets and grid cells are dynamically allocated.
	pmr::memory_region* _memory {};
	/// Hash table buckets each containing multiple slots for cells.
	/// Length is a power of two.
	std::span<bucket_t> _buckets;
#if OTV_HASH_GRID_VALIDATION
	/// Test the implementation by mirroring operations with an STL container.
	struct {
		/// Function object hashing grid indices for use in STL containers.
		/// Uses a different hash function than the main table.
		struct index_hash_t {
			[[nodiscard]] constexpr auto operator()(index_t const& idx) const noexcept -> size_t
			{
				using index_bits = std::bitset<sizeof(index_t) * 8>;
				return std::hash<index_bits>{}(*reinterpret_cast<index_bits const*>(&idx));
			}
		};

		/// Stores the number of trajectory intervals in each grid cell.
		std::unordered_map<index_t, uint32_t, index_hash_t> cell_fill {};
	} _validation;
#endif
	/// Reciprocal of each grid cell's extent.
	cgv::vec4 _cell_size {};
	/// Multiply to map coordinates to cell (see `index`).
	coord_t _scale {};
	/// Counts cells stored in the grid for performance evaluation.
	uint32_t _num_cells {};
	/// Minimum arclength distance between sample points when inserting a segment into the grid.
	float _sample_step_space;
	/// Minimum timespan between sample points when inserting a segment into the grid.
	[[no_unique_address]] std::conditional_t<
		dimensions == dimensions::xyz,
		decltype(std::ignore),
		float
	> _sample_step_time;

	/// Allocate a contiguous span of buckets, initializing all slots as empty.
	[[nodiscard]] auto allocate_buckets (uint32_t count) -> std::span<bucket_t>;
	/// Deallocate buckets without freeing the cells they contain.
	void free_buckets (std::span<bucket_t>) noexcept;
	/// Find or create a cell by index, then insert a trajectory interval into that cell.
	void add_interval (index_t, interval_t);
	/// Access a cell by its index.
	/// If the cell is not yet stored in the grid, it is created.
	auto get (index_t) -> cell_t&;
	/// Find a cell in the hash table by its index.
	/// If there is no matching entry, insert `new_cell`, which must have the same index.
	/// If `new_cell` is null, it is allocated on demand.
	/// Returns a pointer to the queried entry in the table or nullptr if insertion failed.
	auto find_or_insert (index_t, cell_t new_cell = {}) -> cell_t*;

	/// Hash a cell index into a shorter signature with high entropy.
	[[nodiscard]] constexpr auto signature (index_t) const noexcept -> signature_t;
	/// The bucket in which a cell with the given signature is stored by `hash_fn`.
	[[nodiscard]] auto bucket (signature_t, uint8_t hash_fn) noexcept -> bucket_t&;
};

} // namespace otv
