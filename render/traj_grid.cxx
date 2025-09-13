// C++ STL
#include <print>

// local includes
#include "gpumem/span.inl"

// implemented header
#include "render/traj_grid.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[traj_grid]\x1b[m"


namespace otv {
namespace {

/// Controls the amount of debug messages produced by the grid.
/// Ranges from 0 (no output) to 3 (full output).
constexpr auto log_level = 2u;

/// Print a message to `std::clog` using a C++ 20 format string.
/// Does not append a newline.
template <class... Args>
void log(std::format_string<Args...> fmt, Args&&... args)
{
	std::print(std::clog, fmt, std::forward<Args>(args)...);
}


/// Maximum iterations to try when inserting a cell into the grid.
/// Larger numbers allow for a higher load factor at the cost of longer worst-case insertion time.
constexpr auto max_cuckoo_chain = 50u;

} // namespace


traj_grid::traj_grid(gpumem::heap* memory, dimensions dims, cgv::vec4 cell_size, uint8_t order)
	: _memory     {memory}
	, _buckets    (1uz << order, gpumem::pmr_alloc{_memory})
	, _scale      {cgv::vec4{1} / cell_size} // index = coords / cell_size
	, _dimensions {dims}
{}

void traj_grid::add_segment (
	node_attribs const& start,
	node_attribs const& end,
	cgv::uvec2          node_idcs
) {
	// TODO: Properly partition the segment into intervals.
	get(index(start)).add_interval(*_memory, {node_idcs, {start.t.x(), end.t.x()}});
#ifndef NDEBUG
	// Track the number of intervals in each cell.
	if constexpr (validation::enabled) ++_validation.cell_load[index(start)];
#endif
}


auto traj_grid::get (index_t index) -> cell_t&
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
				auto const num_slots = num_buckets * std::tuple_size_v<bucket_t>;
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
				for (auto& slot : bucket) {
					// Buckets are filled front to back, all following slots are empty.
					if (!slot.cell) break;
					// If insertion fails during a rehash (extremely unlikely), abort the current
					// attempt and try again with yet more buckets.
					if (!find_or_insert(slot.cell.get(buffer).index, slot.cell)) goto rehash;
				}
		// Once all previous entries have been restored, reattempt to insert the new index.
	}
}

auto traj_grid::find_or_insert (index_t query, cell_t new_cell) -> cell_t*
{
	// Recurring constants.
	auto const buffer              = _memory->as_span();
	constexpr unsigned bucket_size = std::tuple_size_v<bucket_t>;

	/// Hash the query index into its shorter signature for quick comparison.
	auto const signature = this->signature(query);

	struct {
		bucket_t* slots {};
		unsigned load   {~0u};
	}
	/// Tracks which of the buckets that the entry being inserted hashes to has the most free slots.
	dest_bucket {};

	// Check all buckets the queried cell hashes to.
	for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
		auto& bucket = this->bucket(signature, fn);
		/// Count the number of occupied slots in this bucket.
		auto load = 0u;

		// Check all occupied slots.
		for (auto& slot : bucket) {
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
#ifndef NDEBUG
					// Check that it contains the expected number of trajectory intervals.
					if constexpr (validation::enabled)
						assert(_validation.cell_load[query] == slot.cell.get(buffer).size);
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
#ifndef NDEBUG
		// Check that the index really does not exist in the table, then insert it into the
		// validation map with zero trajectory intervals.
		// This check is not performed when `new_cell` is given, since that only happens during
		// rehashing, which does not affect the validation map.
		if constexpr (validation::enabled) assert(_validation.cell_load[query] == 0);
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
		if (dest_bucket.load < bucket_size) {
			// Buckets are filled front to back.
			auto& dest = (*dest_bucket.slots)[dest_bucket.load] = insert_entry;
			// The grid has grown by one cell.
			++_num_cells;
			// Ensure that the return value points to the new cell.
			if (insert_entry.cell == new_cell) new_entry = &dest.cell;
			else assert(*new_entry == new_cell);

			if constexpr (log_level > 2)
				log(LOG_TAG" Inserted a cell after {} cuckoos.\n", i);

			return new_entry;
		}

		// If all candidate buckets are full, pick one by cycling through hash functions, then store
		// the entry in a random slot.
		auto& cuckoo_bucket = this->bucket(insert_entry.signature, (prev_fn + 1) % num_hash_fns);
		auto& dest          = cuckoo_bucket[std::uniform_int_distribution{0u, bucket_size-1}(_rng)];
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
				assert(bucket.back().cell);
				continue;
			}

			// Count the occupied slots in the bucket.
			// Iterate back to front, since at this point buckets likely have only a few free slots.
			unsigned load = bucket_size;
			while (load > 0 && !bucket[load - 1].cell) --load;
			if (load < dest_bucket.load) dest_bucket = {&bucket, load};
		}
	}

	// If no free slot could be found in the given number of tries, the table must be resized.
	return nullptr;
}

constexpr auto traj_grid::index (node_attribs const& node) const noexcept -> index_t
{
	// Round such that cells are centered on their index.
	return cgv::math::round(_scale * cgv::vec4{
		cgv::vec3{node.pos_rad},
		// 3D spatial grids ignore the last component.
		_dimensions == dimensions::xyz ? 0 : node.t.x()
	});
}

constexpr auto traj_grid::signature (index_t index) const noexcept -> signature_t
{
	// Signatures are generated using the variable length xxhash32 algorithm by Collet 2012 as
	// implemented by Jarzynski and Olano 2020.

	/// Prime constants.
	auto const [p2, p3, p4, p5] = std::to_array<uint32_t>({
		2246822519, 3266489917, 668265263, 374761393
	});

	// Compared to the implementation by Jarzynski and Olano, `index` is rotated by one component.
	uint32_t hash = index.x() + p5;

	for (auto d = 1; d < static_cast<uint8_t>(_dimensions); ++d) {
		hash += index[d] * p3;
		hash  = p4 * ((hash << 17) | (hash >> 15));
	}

	hash = p2 * (hash ^ (hash >> 15));
	hash = p3 * (hash ^ (hash >> 13));
	return {hash ^ (hash >> 16)};
}

auto traj_grid::bucket (signature_t signature, uint8_t hash_fn) noexcept -> bucket_t&
{
	uint32_t hash;

	switch (hash_fn) {
	case 0:
		// Signatures are already hashed, so for the first function we use it unchanged.
		hash = signature.value;
		break;
	default: {
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
		break;
	}};

	// The number of buckets is a power of two, so the modulo simplifies to a bit-wise and.
	return _buckets[hash & _buckets.size() - 1];
}


traj_grid::cell_t::cell_t(gpumem::heap& memory, index_t index)
{
	// Start with the smallest capacity > 0 such that the total size of the allocation is a power of
	// two.
	auto const capacity =
		(std::bit_ceil(offsetof(data_t, intervals[1])) - offsetof(data_t, intervals))
		/ sizeof(interval_t);
	// Allocate and initialize cell data on the GPU heap.
	set(memory.as_span(), allocate(memory, capacity) =
		{.index = index, .size = 0, .capacity = capacity});
}

constexpr auto traj_grid::cell_t::operator== (cell_t const& other) const noexcept -> bool
{
	// Pointer semantics.
	return _address == other._address;
}

constexpr traj_grid::cell_t::operator bool () const noexcept
{
	// Zero is a valid offset, so null pointers instead have all bits set.
	return _address != ~0;
}

auto traj_grid::cell_t::get (gpumem::span<std::byte> const& buffer) noexcept -> data_t&
{
	return *reinterpret_cast<data_t*>(&buffer[_address]);
}

void traj_grid::cell_t::add_interval (gpumem::heap& memory, interval_t interval)
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
	set(memory.as_span(), new_data);
}

void traj_grid::cell_t::set (gpumem::span<std::byte> const& buffer, data_t& data)
{
	auto const offset = reinterpret_cast<std::byte*>(&data) - buffer.data();
	_address          = offset;
	assert(_address == offset && _address < buffer.length());
}

auto traj_grid::cell_t::allocate (gpumem::heap& memory, uint32_t capacity) -> data_t&
{
	return *reinterpret_cast<data_t*>(memory.allocate(
		offsetof(data_t, intervals) + capacity*sizeof(interval_t),
		alignof(data_t)
	));
}

} // namespace otv
