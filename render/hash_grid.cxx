// C++ STL
#include <print>

// CGV framework
#include <cgv/render/context.h>
#include <cgv/render/shader_program.h>

// local includes
#include "gpumem/span.inl"

// implemented header
#include "render/hash_grid.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[hash_grid]\x1b[m"
/// Message severity.
#define LOG_ERROR   "\x1b[1;31m[error]\x1b[m"
#define LOG_WARNING "\x1b[1;33m[warning]\x1b[m"


namespace otv {
namespace {

using cgv::vec3;
using cgv::vec4;

/// Controls the amount of debug messages produced by the grid.
/// Ranges from 0 (no output) to 4 (full output).
constexpr auto log_level = 2u;

/// Print a message to `std::clog` using a C++ 20 format string.
/// Does not append a newline.
template <class... Args>
void log(std::format_string<Args...> fmt, Args&&... args)
{
	std::print(std::clog, fmt, std::forward<Args>(args)...);
}


/// In the shader, pointers are emulated with offsets into the grid buffer.
/// This value determines the size in bytes of their minimal addressable unit.
constexpr uint32_t shader_address_unit = 1u;

/// The number of hash functions mapping cells to buckets, i.e. how many candidate buckets can store
/// each entry.
/// More functions allow higher load factors at the cost of increased query time.
constexpr uint8_t num_hash_fns = 2;

/// Maximum iterations to try when inserting a cell into the grid.
/// Larger numbers allow for a higher load factor at the cost of longer worst-case insertion time.
constexpr auto max_cuckoo_chain = 50u;


/// Calculate the offset of `ptr` within the persistently mapped GPU `buffer` for use an address in
/// the shader.
[[nodiscard]] constexpr auto ptr_to_offset (gpumem::span<const std::byte> buffer, void const* ptr)
	-> uint32_t
{
	uint32_t const offset = (reinterpret_cast<std::byte const*>(ptr) - buffer.data())
		/ shader_address_unit;
	assert(&buffer[offset * shader_address_unit] == ptr);
	return offset;
}

/// Access memory at `offset` within `buffer` such that
/// `offset_to_ptr(b, ptr_to_offset(b, p)) == p`.
[[nodiscard]] constexpr auto offset_to_ptr (gpumem::span<std::byte> buffer, uint32_t offset)
	-> void*
{
	return &buffer[offset * shader_address_unit];
}

} // namespace


hash_grid::hash_grid(gpumem::heap* memory, vec4 cell_size, uint8_t order)
	: _memory            {memory}
	, _buckets           (1uz << order, gpumem::pmr_alloc{_memory})
	, _cell_size         {cell_size}
	, _scale             {coord_t{1} / coord_t{cell_size}} // index = coords / cell_size
	, _sample_step_space {cgv::math::min_value(vec3{cell_size}) * 0.05f}
{
	if constexpr (dimensions != dimensions::xyz) _sample_step_time = cell_size[3] * 0.05f;
	if constexpr (log_level > 0) {
		std::clog << LOG_TAG" Create instance.\n"
			"\tBuckets:           "  << _buckets.size()    <<  "\n"
			"\tCell size:         (" << cell_size          << ")\n"
			"\tSpatial sampling:  "  << _sample_step_space <<  "\n";

		if constexpr (dimensions != dimensions::xyz)
			log("\tTemporal sampling: {}\n", [&]() {
				if constexpr (dimensions == dimensions::xyz)
					return 0.0f;
				else return _sample_step_time;
			}());
	}
}

void hash_grid::add_segment (
	node_attribs const& start,
	node_attribs const& end,
	cgv::uvec2          node_idcs,
	cgv::mat4 const&    t_to_s
) {
	/// Beginning and end of the segment, in grid coordinates.
	auto const start_point = coord_t{vec4{vec3{start.pos_rad}, start.t[0]}} * _scale;
	auto const end_point   = coord_t{vec4{vec3{  end.pos_rad},   end.t[0]}} * _scale;
	/// Grid cell in which the segment begins.
	auto const start_index = round(start_point);

	if constexpr (log_level > 2)
		std::clog << LOG_TAG" Add segment (" << start_point << ") to (" << end_point << ")\n";

	/// Cubic Bézier curve defining the segment's spatial components, scaled to grid coordinates.
	auto const [b0, b1, b2, b3] = std::array{
		vec3{start_point},
		vec3{start.pos_rad + start.tangent} * vec3{_scale},
		vec3{  end.pos_rad -   end.tangent} * vec3{_scale},
		vec3{end_point},
	};
	// If all control points lie in the same cell, that cell contains the entire segment.
	if (
		start_index == round(end_point)
		// Tangent points only have spatial coordinates.
		&& vec3{start_index} == round(b1)
		&& vec3{start_index} == round(b2)
	) {
		add_interval(start_index, {node_idcs, {start.t[0], end.t[0]}});
		return;
	}

	/// Spatial and temporal extent of the segment.
	auto const arclen   = t_to_s[15] - t_to_s[0];
	auto const duration = end.t[0] - start.t[0];
	/// Bounds on how much the curve parameter is increased with every sample.
	// The actual maximum step size is max_step/2.
	auto const max_step = coord_t{1} / (coord_t{vec4{vec3{arclen}, duration}} * _scale);
	auto const min_step = std::min({
		min_value(max_step),
		// Assume a constant average speed.
		_sample_step_space / arclen,
		// Inline lambda required so the else branch compiles when dimensions == xyz.
		[this]() {
			if constexpr (dimensions == dimensions::xyz)
				return std::numeric_limits<float>::infinity();
			else return _sample_step_time;
		}() / duration
	});

	if constexpr (log_level > 3)
		std::clog <<
			"\tArclength:    "  << arclen          <<  "\n"
			"\tDuration:     "  << duration        <<  "\n"
			"\tMin sampling: "  << min_step        <<  "\n"
			"\tMax sampling: (" << max_step * 0.5f << ")\n";

	/// Start of the current interval.
	auto min_time = start.t[0];

	// Begin at the start node.
	auto prev_t     = 0.0f;
	auto prev_point = start_point;
	auto prev_index = start_index;
	// Sample the trajectory.
	do {
		/// Determine the next sample based on how close the last sample was to a grid plane.
		auto const t = std::min(
			prev_t + std::max(
				// Bigger steps from samples near the center of their grid cell.
				min_value(max_step * (coord_t{0.5f} - abs(prev_point - prev_index))),
				min_step // Ensure progress.
			),
			1.0f // Limit to segment.
		);

		// Evaluate the segment for the current curve parameter.
		auto const t2    = t*t;
		auto const s     = 1.0f - t;
		auto const s2    = s*s;
		auto const point = coord_t{vec4{
			// Spatial components: Evaluate cubic Bézier.
			s2*s*b0 + 3*s2*t*b1 + 3*s*t2*b2 + t*t2*b3,
			// Temporal component: Linear interpolation.
			dimensions == dimensions::xyz ? 0.0f : s*start_point[3] + t*end_point[3]
		}};

		if constexpr (log_level > 3) {
			log("{:5.1f}% {:f}", t * 100, (start.t[0] + t*duration));
			std::clog << " (" << point << ")\n";
		}

		/// Grid cell containing the current sample.
		auto const index = round(point);

		// If the trajectory has crossed into another grid cell:
		if (index != prev_index) {
			/// Average difference in curve parameter between the previous sample and every inter-
			/// section between trajectory and grid.
			auto isect_offset = 0.0f;
			auto isect_count  = -1; // Number of grid intersections - 1.

			// Indices differ in at least one component, maybe multiple.
			for (auto d = 0u; d < index.size(); ++d) {
				if (prev_index[d] == index[d]) continue;
				/// Coordinate of the hyper-plane separating the the two cells in dimension d.
				/// If the step size is small enough that there should only ever be one crossing per
				/// component.
				auto grid_plane = 0.5f*prev_index[d] + 0.5f*index[d];

				if (log_level > 1 && abs(index[d] - prev_index[d]) > 1)
					std::clog << LOG_WARNING LOG_TAG" Skipping cell between (" << prev_index
						<< ") and (" << index << ")\n";

				// Linearly approximate the intersection between trajectory and grid.
				isect_offset +=
					(t - prev_t) * (grid_plane - prev_point[d])/(point[d] - prev_point[d]);
				++isect_count;
			}
			assert(isect_count >= 0 && isect_count < index.size());
			/// End the interval at the average of all grid crossings.
			auto const max_time =
				start.t[0]
				+ (prev_t + isect_offset*std::array{1.0f, 0.5f, 1.0f/3, 0.25f}[isect_count])
				* duration;
			// Store the interval in the grid.
			add_interval(prev_index, {node_idcs, {min_time, max_time}});
			// The next interval begins where the previous one ends.
			min_time = max_time;
		}
		// Remember the current sample for the next iteration.
		prev_t     = t;
		prev_point = point;
		prev_index = index;
	} while (prev_t < 1.0f);
	// Create a final interval up to the end of the segment.
	add_interval(prev_index, {node_idcs, {min_time, end.t[0]}});
}

void hash_grid::set_defines (cgv::render::shader_define_map& d, GLuint buffer_binding) const
{
	using sc = cgv::render::shader_code;
	sc::set_define(d, "HASH_GRID_BUFFER_BINDING",   buffer_binding,                      {});
	sc::set_define(d, "HASH_GRID_ADDRESS_UNIT",     shader_address_unit,                 {});
	sc::set_define(d, "HASH_GRID_NUM_HASH_FNS",     uint32_t{num_hash_fns},              {});
	sc::set_define(d, "HASH_GRID_SLOTS_PER_BUCKET", bucket_t::num_slots,                 {});
	sc::set_define(d, "HASH_GRID_CELL_HEADER_SIZE", offsetof(cell_t::data_t, intervals), {});
	sc::set_define(d, "HASH_GRID_DIMENSIONS",       dimensions,                          {});
}

void hash_grid::set_uniforms (cgv::render::context& c, cgv::render::shader_program& p) const
{
	auto ok = true
	&& p.set_uniform(c, "hash_grid_cell_size",       _cell_size)
	&& p.set_uniform(c, "hash_grid_scale",           _scale)
	&& p.set_uniform(c, "hash_grid_buckets.address", ptr_to_offset(_memory->as_span().as_const(), _buckets.data()))
	&& p.set_uniform(c, "hash_grid_buckets_mask",    static_cast<uint32_t>(_buckets.size() - 1))
	;
	assert(ok);
}


void hash_grid::add_interval (index_t index, interval_t interval)
{
	if constexpr (log_level > 2)
		std::clog << LOG_TAG" Add interval (" << interval.time << ") to cell (" << index << ").\n";

	get(index).add_interval(*_memory, interval);

#if OTV_HASH_GRID_VALIDATION
	// Track the number of intervals in each cell.
	++_validation.cell_fill[index];
#endif
}

auto hash_grid::get (index_t index) -> cell_t&
{
	while (true) {
		// In the vast majority of cases, the cell is already stored in the table or can be
		// inserted into the existing buckets.
		if (auto const cell = find_or_insert(index)) return *cell;
		// In the unlikely event that a new cell cannot be inserted, more buckets must be allocated.

		/// Save the contents of the table.
		auto old_buckets = std::move(_buckets);
		/// Size of the current table.
		auto num_buckets  = old_buckets.size();
		auto const buffer = _memory->as_span();

		// Rebuild the table with double capacity.
		rehash:
			// Print the current load factor to determine efficiency.
			if constexpr (log_level > 0) {
				auto const num_slots = num_buckets * bucket_t::num_slots;
				log(LOG_TAG" Rehashing after insertion failed with {}/{} slots occupied "
					"(load factor {:.2}).\n",
					_num_cells,
					num_slots,
					static_cast<float>(_num_cells) / num_slots
				);
			}

			// Allocate new, empty buckets.
			num_buckets *= 2;
			_buckets     = decltype(_buckets)(num_buckets, gpumem::pmr_alloc{_memory});
			_num_cells   = 0;

			// Reinsert all entries in the new buckets.
			for (auto& bucket : old_buckets)
				for (auto& slot : bucket.slots) {
					// Buckets are filled front to back, all following slots are empty.
					if (!slot.cell) break;
					// If insertion fails during a rehash (extremely unlikely), abort the current
					// attempt and try again with yet more buckets.
					if (!find_or_insert(slot.cell.get(buffer).index, slot.cell)) goto rehash;
				}
		// Once all previous entries have been restored, reattempt to insert the new index.
	}
}

auto hash_grid::find_or_insert (index_t query, cell_t new_cell) -> cell_t*
{
	// Recurring constants.
	auto const buffer = _memory->as_span();

	/// Hash the query index into its shorter signature for quick comparison.
	auto const signature = this->signature(query);

	struct {
		bucket_t* data {};
		unsigned load  {~0u};
	}
	/// Tracks which of the buckets that the entry being inserted hashes to has the most free slots.
	dest_bucket {};

	// Check all buckets the queried cell hashes to.
	for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
		auto& bucket = this->bucket(signature, fn);
		/// Count the number of occupied slots in this bucket.
		auto load = 0u;

		// Check all occupied slots.
		for (auto& slot : bucket.slots) {
			// Buckets are filled front to back and entries are never deleted, so if we find a free
			// slot, all following ones must be free as well.
			if (!slot.cell) break;
			if (slot.signature.value == signature.value) {
				// Since many indices map to the same signature, we must load the index of the
				// possible match for an exact comparison.
				// In practice, a good hash makes signature collisions unlikely.
				auto const index = slot.cell.get(buffer).index;
			 	if (index == query) {
					// We have found the queried cell.
#if OTV_HASH_GRID_VALIDATION
					assert(slot.cell.get(buffer).size == _validation.cell_fill[query]);
#endif
					return &slot.cell;
				}
				// Report signature hash collisions.
				if constexpr (log_level > 1)
					std::clog << LOG_TAG" Signature collision for indices (" << index
						<< ") and (" << query << ")\n";
			}
			++load;
		}
		// Track the least occupied bucket.
		if (load < dest_bucket.load) dest_bucket = {&bucket, load};
	}
	// If the index cannot be found in the table, insert it.

	if (new_cell)
		// Check that the given cell actually matches the query index.
		assert(new_cell.get(buffer).index == query);
	else {
		// If no cell has been given to insert, allocate a new one.
		new_cell = cell_t{*_memory, query};
#if OTV_HASH_GRID_VALIDATION
		// Check that the index really does not exist in the table, then insert it into the
		// validation map with zero trajectory intervals.
		// This check is not performed when `new_cell` is given, since that only happens during
		// rehashing, which does not affect the validation map.
		if (_validation.cell_fill[query] != 0) {
			std::clog << LOG_ERROR LOG_TAG" Could not find cell (" << query << ") "
				"even though it has been added to the grid before.\n";
			std::exit(EXIT_FAILURE);
		}
#endif
	}

	/// The entry currently being inserted.
	/// Initialized to the new cell, updated with every cuckoo.
	auto insert_entry = slot_t{signature, new_cell};
	/// Hash function the entry being inserted was last stored with.
	/// For the new cell that was never in the table, it is initialized such that the next function
	/// will be zero.
	uint8_t prev_fn = ~0;

	/// Track the location of the newly created cell within the table for the return value.
	cell_t* new_entry = nullptr;

	// Move entries until we find a free slot or some maximum number of iterations is reached.
	for (auto i = 0u; i < max_cuckoo_chain; ++i) {
		// If one of the candidate buckets has a free slot, store the entry and return.
		if (dest_bucket.load < bucket_t::num_slots) {
			// Buckets are filled front to back.
			auto& dest = dest_bucket.data->slots[dest_bucket.load] = insert_entry;
			// The grid has grown by one cell.
			++_num_cells;
			// Ensure that the return value points to the new cell.
			if (insert_entry.cell == new_cell) new_entry = &dest.cell;
			else assert(*new_entry == new_cell);

			if constexpr (log_level > 2)
				std::clog << LOG_TAG" Inserted cell (" << query << ") after " << i << " cuckoos.\n";

			return new_entry;
		}

		// If all candidate buckets are full, pick one by cycling through hash functions, then store
		// the entry in a random slot.
		auto& cuckoo_bucket = this->bucket(insert_entry.signature, (prev_fn + 1) % num_hash_fns);
		auto& dest          = cuckoo_bucket.slots[
			std::uniform_int_distribution{0uz, bucket_t::num_slots - 1}(_rng)
		];
		// Remember where the new cell is stored.
		if (insert_entry.cell == new_cell) new_entry = &dest.cell;
		// Cuckoo the previous entry from the chosen slot.
		std::swap(dest, insert_entry);

		// Find the best bucket to store the displaced entry.
		dest_bucket = {};
		for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
			auto& bucket = this->bucket(insert_entry.signature, fn);

			// We know that the bucket the entry was displaced from is full already.
			// Remember the associated hsah function, so we know which one to try next if all
			// candidate buckets are full again.
			if (&bucket == &cuckoo_bucket) {
				prev_fn = fn;
				assert(bucket.slots.back().cell);
				continue;
			}

			// Count the occupied slots in the bucket.
			// Iterate back to front, since at this point buckets likely have only a few free slots.
			unsigned load = bucket_t::num_slots;
			while (load > 0 && !bucket.slots[load - 1].cell) --load;
			if (load < dest_bucket.load) dest_bucket = {&bucket, load};
		}
	}
	// If no free slot could be found in the given number of tries, the table must be resized.
	// All cells in the table before this function call are reinserted into the new buckets.
	// To this end, the last cell to be displaced is restored into the slot of the new cell.
	// It will be in the wrong bucket with the wrong signature, but for rehashing that is OK.
	// The new cell will be added to the expanded table after all previous entries have been moved.
	*new_entry = insert_entry.cell;
	return nullptr;
}

constexpr auto hash_grid::signature (index_t index) const noexcept -> signature_t
{
	// Signatures are generated using the variable length xxhash32 algorithm by Collet 2012 as
	// implemented by Jarzynski and Olano 2020.

	/// Prime constants.
	auto const [p2, p3, p4, p5] = std::to_array<uint32_t>({
		2246822519, 3266489917, 668265263, 374761393
	});

	// Compared to the implementation by Jarzynski and Olano, `index` is rotated by one component.
	uint32_t hash = std::bit_cast<uint32_t>(index[0]) + p5;

	for (auto d = 1u; d < index.size(); ++d) {
		hash += std::bit_cast<uint32_t>(index[d]) * p3;
		hash  = p4 * ((hash << 17) | (hash >> 15));
	}

	hash = p2 * (hash ^ (hash >> 15));
	hash = p3 * (hash ^ (hash >> 13));
	return {hash ^ (hash >> 16)};
}

auto hash_grid::bucket (signature_t signature, uint8_t hash_fn) noexcept -> bucket_t&
{
	uint32_t hash = signature.value;

	// Signatures are already hashed, so for the first function we use it unchanged.
	if (hash_fn != 0) {
		// Otherwise the signature is hashed again using the PCG hash function by O'Neill 2014,
		// as implemented by Jarzynski and Olano 2020.

		/// Sets of random 32 bit odd integers used to seed the hash function.
		static constexpr auto seeds = std::to_array<std::array<uint32_t, 3>>({
			{747796405, 2891336453, 277803737},
		});
		static_assert(seeds.size() >= num_hash_fns - 1);
		auto const& s = seeds[hash_fn - 1];

		/// Calculate the hash using the chosen seed.
		hash = signature.value * s[0] + s[1];
		hash = ((hash >> ((hash >> 28) + 4)) ^ hash) * s[2];
		hash = (hash >> 22) ^ hash;
	};

	// The number of buckets is a power of two, so the modulo simplifies to a bit-wise and.
	return _buckets[hash & _buckets.size() - 1];
}


hash_grid::cell_t::cell_t(gpumem::heap& memory, index_t index)
{
	// Start with the smallest capacity > 0 such that the total size of the allocation is a power of
	// two.
	auto const capacity =
		(std::bit_ceil(offsetof(data_t, intervals[1])) - offsetof(data_t, intervals))
		/ sizeof(interval_t);
	// Allocate and initialize cell data in the GPU buffer.
	_address = ptr_to_offset(
		memory.as_span().as_const(),
		&(allocate(memory, capacity) = {.index = index, .size = 0, .capacity = capacity})
	);
}

constexpr auto hash_grid::cell_t::operator== (cell_t const& other) const noexcept -> bool
{
	// Pointer semantics.
	return _address == other._address;
}

constexpr hash_grid::cell_t::operator bool () const noexcept
{
	// Zero is a valid offset, so null pointers instead have all bits set.
	return _address != ~0;
}

auto hash_grid::cell_t::get (gpumem::span<std::byte> const& buffer) noexcept -> data_t&
{
	return *reinterpret_cast<data_t*>(offset_to_ptr(buffer, _address));
}

void hash_grid::cell_t::add_interval (gpumem::heap& memory, interval_t interval)
{
	auto const buffer = memory.as_span();
	auto& data        = get(buffer);
	// Load header.
	auto const [index, size, capacity, _] = data;

	// If the allocation has spare capacity, append the new interval.
	if (size < capacity) {
		data.intervals[data.size++] = interval;
		return;
	}

	// Sanity check: Array length cannot exceed allocation size.
	assert(size == capacity);

	// If the current allocation is full, reallocate with twice as much memory.
	auto const new_capacity = capacity * 2
		+ static_cast<uint32_t>(offsetof(data_t, intervals) / sizeof(interval_t));

	if constexpr (log_level > 2)
		log(LOG_TAG" Growing cell from capacity {} to {}.\n", capacity, new_capacity);

	auto& new_data = allocate(memory, new_capacity) = {index, size + 1, new_capacity};
	// Copy previous intervals.
	std::memcpy(new_data.intervals, data.intervals, size * sizeof(interval_t));
	// Append the new interval.
	new_data.intervals[size] = interval;

	// Free the previous allocation.
	memory.deallocate(
		&data,
		offsetof(data_t, intervals) + capacity * sizeof(interval_t),
		alignof(data_t)
	);
	// Point to the new allocation.
	_address = ptr_to_offset(buffer.as_const(), &new_data);
}

auto hash_grid::cell_t::allocate (gpumem::heap& memory, uint32_t capacity) -> data_t&
{
	return *reinterpret_cast<data_t*>(memory.allocate(
		offsetof(data_t, intervals) + capacity*sizeof(interval_t),
		alignof(data_t)
	));
}

} // namespace otv
