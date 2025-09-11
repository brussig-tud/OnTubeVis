#pragma once

// C++ STL
#include <array>
#include <cstdint>
#include <random>
#include <vector>

#ifndef NDEBUG
// Required for optional validation.
#include <bitset>
#include <unordered_map>
#endif

// CGV framework
#include <cgv/math/fvec.h>

// local includes
#include "gpumem/heap.h"
#include "render/common.h"


namespace otv {

/// An acceleration data structure storing which trajectory intervals lie in each cell of an
/// infinite regular grid, stored as a hash map in GPU-accessible memory.
class traj_grid {
public:
	/// The coordinate system that defines a grid.
	enum class dimensions : uint8_t {
		xyz  = 3, // 3D spatial grid.
		xyzt = 4, // 4D spatio-temporal grid.
	};

	/// The number of hash functions mapping cells to buckets, i.e. how many candidate buckets there
	/// are to store each entry.
	/// More functions allow higher load factors at the cost of increased query time.
	constexpr static uint8_t num_hash_fns = 2;

	/// Create an empty grid.
	[[nodiscard]] traj_grid() = default;
	/// Create a grid with 2 ^ `order` hash buckets.
	[[nodiscard]] traj_grid(gpumem::heap&, dimensions, cgv::vec4 cell_size, uint8_t order);

	/// Update the grid with a new trajectory segment whose nodes are stored at `render_idcs` in the
	/// render buffer.
	void add_segment (node_attribs const& start, node_attribs const& end, cgv::uvec2 render_idcs);

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
	/// In a 3D grid, the fourth component is always 0.
	using index_t = cgv::ivec4;

	/// Hash of a cell index.
	/// Like in Mega-KV (Zhang et al. 2015), only the signature is stored directly in the buckets
	/// of the hash tables to reduce memory transfers.
	struct signature_t {uint32_t value;};

	/// Stores a grid cell's index and the trajectory intervals it contains as a dynamic array with
	/// pointer semantics, meaning copies are shallow and allocations are not automatically freed.
	struct cell_t {
		/// Format of the actual allocations instances point to.
		struct data_t {
			/// The index identifying the cell.
			alignas(16) index_t index;
			/// Number of trajectory intervals in the cell.
			uint32_t size;
			/// Maximum number of intervals that fit in the current allocation.
			uint32_t capacity;
			/// Trajectory intervals within the cell.
			interval_t intervals[];
		};

		/// Create a null handle.
		[[nodiscard]] constexpr cell_t() noexcept {};
		/// Allocate a new cell.
		/// All future operations on the cell must use the same memory resource.
		[[nodiscard]] cell_t(gpumem::heap& memory, index_t);

		/// Check if a handle is not null.
		[[nodiscard]] constexpr operator bool () const noexcept;
		/// Check if two handles point to the same data, assuming they use the same memory resource.
		[[nodiscard]] constexpr auto operator== (cell_t const&) const noexcept -> bool;

		/// Dereference the handle to access the cell data.
		[[nodiscard]] auto get (gpumem::span<std::byte> const& buffer) noexcept -> data_t&;
		/// Store a trajectory interval in the cell.
		void add_interval (gpumem::heap& memory, interval_t);

	private:
		/// Allocate data with enough memory to store the requested number of intervals.
		[[nodiscard]] static auto allocate (gpumem::heap&, uint32_t capacity) -> data_t&;

		/// Offset of the data allocation within the heap buffer.
		uint32_t _address = ~0;

		/// Point this instance to the given data.
		/// Does not free the current allocation.
		void set (gpumem::span<std::byte> const& buffer, data_t&);
	};

	/// A slot in one of the hash table's buckets, storing a cell pointer and its index signature.
	/// Iff the cell is null the slot is considered empty and the signature indeterminate.
	struct alignas(8) slot_t {
		signature_t signature;
		cell_t      cell;
	};
	/// A bucket of the hash table, dimensioned to match GPU memory transfers.
	using bucket_t = std::array<slot_t, 128 / sizeof(slot_t)>;


	/// The GPU buffer in which hash buckets and grid cells are stored.
	gpumem::heap* _memory {};
	/// Hash table buckets each containing multiple slots for cells.
	/// Length is a power of two.
	std::pmr::vector<bucket_t> _buckets {};
	/// Reciprocal of each grid cell's extent.
	/// Multiply to map coordinates to cell (see `index`).
	cgv::vec4 _scale {};
	/// Random number generator used to determine which entry to displace from a full bucket when
	/// inserting a new cell.
	std::minstd_rand _rng {std::random_device{}()};
	/// Counts cells stored in the grid for performance evaluation.
	uint32_t _num_cells {};
	/// Determines whether the grid partitions only space (3D) or also time (4D).
	dimensions _dimensions {};

	/// Access a cell by its index.
	/// If the cell is not yet stored in the grid, it is created.
	auto get (index_t) -> cell_t&;
	/// Find a cell in the hash table by its index.
	/// If there is no matching entry, insert `new_cell`, which must have the same index.
	/// If `new_cell` is null, it is allocated on demand.
	/// Returns a pointer to the queried entry in the table or nullptr if insertion failed.
	auto find_or_insert (index_t, cell_t new_cell = {}) -> cell_t*;

	/// Calculate the index of the cell containing the given node.
	[[nodiscard]] constexpr auto index (node_attribs const&) const noexcept -> index_t;
	/// Hash a cell index into a shorter signature with high entropy.
	[[nodiscard]] constexpr auto signature (index_t) const noexcept -> signature_t;
	/// The bucket in which a cell with the given signature is stored by `hash_fn`.
	[[nodiscard]] auto bucket (signature_t, uint8_t hash_fn) noexcept -> bucket_t&;

#ifndef NDEBUG
	/// Test the implementation by mirroring operations with an STL container in host memory.
	[[no_unique_address]] struct validation {
		/// If set to false, no state will be mirrored, avoiding all associated runtime and memory
		/// overhead.
		static constexpr auto enabled = true;

		/// Function object hashing grid indices for use in STL containers.
		/// Uses a different hash function than the GPU grid.
		struct index_hash_t {
			[[nodiscard]] constexpr auto operator()(index_t const& idx) const noexcept -> size_t
			{
				return std::hash<std::bitset<128>>{}(*reinterpret_cast<std::bitset<128> const*>(&idx));
			}
		};

		/// Stores the number of trajectory intervals in each grid cell.
		[[no_unique_address]] std::conditional_t<
			enabled,
			std::unordered_map<index_t, uint32_t, index_hash_t>,
			std::monostate
		> cell_load {};
	} _validation;
#endif
};

} // namespace otv
