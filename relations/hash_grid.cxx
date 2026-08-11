#include <format>
#include <memory>

#include <cgv/render/context.h>
#include <cgv/render/shader_program.h>

#include "hash_grid.h"
#include "render_layout.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[hash_grid]\x1b[m"
/// Message severity.
#define LOG_ERROR   "\x1b[1;31m[error]\x1b[m"
#define LOG_WARNING "\x1b[1;33m[warning]\x1b[m"


namespace {

/// Controls the amount of debug messages produced by the grid.
/// Ranges from 0 (no output) to 4 (full output).
constexpr auto log_level = 2u;


/// In the shader, pointers are emulated with offsets into the grid buffer.
/// This value determines the size in bytes of their minimal addressable unit.
constexpr uint32_t address_unit = 1u;

/// The number of hash functions mapping cells to buckets, i.e. how many candidate buckets can store
/// each entry. More functions allow higher load factors at the cost of increased query time.
constexpr uint8_t num_hash_fns = 2;

/// Maximum iterations to try when inserting a cell into the grid. Larger numbers allow for a higher
/// load factor at the cost of longer worst-case insertion time.
constexpr auto max_cuckoo_chain = 50u;

/// Generate a lookup table that maps every bit `b` in a byte to bit `spread_factor * b`, where
/// `b = 0` is the least significant bit. All other bits are zero.
[[nodiscard]] constexpr auto bit_spread_lut (uint8_t spread_factor) noexcept
	-> std::array<uint32_t, 256>
{
	std::array<uint32_t, 256> lut {}; // Zero-initialization.
	for (auto i = 0u; i < 256; ++i)
		for (auto b = 0u; b < 8u; ++b)
			lut[i] |= (i & 1<<b) << b*(spread_factor - 1);
	return lut;
}

// Lookup tables used to calculate 3D and 4D Z-order curve by interleaving bits.
constexpr auto spread3 = bit_spread_lut(3);
constexpr auto spread4 = bit_spread_lut(4);

} // namespace


hash_grid::hash_grid(params const& params, uint32_t initial_buckets)
	: _cell_size {params.layout == layout::xyz
		// A 3D grid is equivalent to a 4D grid with infinite cell extent in time.
		? vec4{vec3{params.cell_size}, std::numeric_limits<float>::infinity()}
		: params.cell_size
	}
	, _scale           {vec4{1} / _cell_size} // index = round(point / _cell_size)
	, _sample_step     {max(params.sample_step, vec2{0.001})}
	, _layout          {params.layout}
	, _signature_fn    {params.signature_fn}
	, _initial_buckets {initial_buckets}
{
	if (_layout != layout::t_xyz) _tables.emplace_back(_initial_buckets, 0);

	if constexpr (log_level > 0) {
		using std::operator""sv;
		auto const layout_name = std::array{"3D"sv, "Strided 3D"sv, "4D"sv}
			[static_cast<size_t>(_layout)];
		auto const signature_fn = std::array{"Multiply and XOR"sv, "xxHash32"sv, "Z-Order Curve"sv}
			[static_cast<size_t>(_signature_fn)];
		std::clog << LOG_TAG" Create instance.\n"
			"\tLayout:          " << layout_name      << "\n"
			"\tSignature:       " << signature_fn     << "\n"
			"\tCell size:       ("<< _cell_size       <<")\n"
			"\tSample step:     " << _sample_step[0]  << " (space), "
			                      << _sample_step[1]  << " (time)\n"
			"\tInitial buckets: " << _initial_buckets << "\n";
	}
}

void hash_grid::reorganize ()
{
	// Sort cells along a Z-order curve to improve data locality in the shader.
	std::sort(_cells.begin(), _cells.end(), [this](auto const& l, auto const& r) {
		auto const& lidx = l.index;
		auto const& ridx = r.index;

		// Compare the coordinates with the most significant differing bit. In case of a tie, prefer
		// t > z > y > x, i.e. prefer outer over inner loops in the shader.
		auto sortdim = 0u;
		auto min_lzeroes = ~0u;
		for (auto d : {3, 2, 1, 0}) {
			auto const lzeroes = std::countl_zero(
				std::bit_cast<uint32_t>(lidx[d]) ^ std::bit_cast<uint32_t>(ridx[d]));
			if (lzeroes >= min_lzeroes) continue;
			min_lzeroes = lzeroes;
			sortdim = d;
		}
		return lidx[sortdim] < ridx[sortdim];
	});

	// Replace tables with new, smaller ones, sized to match the actual number of entries plus a
	// small amount of spare capacity for hash collisions.
	for (auto& table : _tables) table = {
		std::min<uint32_t>(ceil(table.num_entries * 1.02f / Bucket::num_slots), table.length),
		table.timestep
	};

	// Reinsert all cells into the new tables.
	uint32_t idx = 0;
	for (auto const& cell : _cells) find_or_insert(cell.index, 1.05, idx++);
	}

auto hash_grid::stats () const -> std::string
{
	size_t num_buckets = 0;
	for (auto const& table : _tables) num_buckets += table.length;
	auto const num_slots = num_buckets * Bucket::num_slots;

	using std::operator""sv;
	auto const     bytes = buffer_size();
	constexpr auto units = std::array{"B"sv, "kiB"sv, "MiB"sv, "GiB"sv};

	unsigned ord_of_mag = 0;
	while (++ord_of_mag < units.size() && bytes >> (10*ord_of_mag));
	--ord_of_mag;

	return std::format(
		"Tables:      {}\n"
		"Buckets:     {} (avg. {:.3} per table)\n"
		"Slots:       {} ({} per bucket)\n"
		"Cells:       {} ({:.1f}%% of slots)\n"
		"Intervals:   {} (avg. {:.3} per cell)\n"
		"Buffer size: {:.3} {} ({} bytes)\n",
		_tables.size(),
		num_buckets, num_buckets * (1.f/_tables.size()),
		num_buckets * Bucket::num_slots, Bucket::num_slots,
		_cells.size(), _cells.size() * (1.f/num_slots) * 100.f,
		_num_intervals, _num_intervals * (1.f/_cells.size()),
		bytes * (1.f/(1 << 10*ord_of_mag)), units[ord_of_mag], bytes
	);
}

void hash_grid::add_segment (
	node_attribs const& start,
	node_attribs const& end,
	cgv::uvec2         node_idcs,
	cgv::mat4 const&   t_to_s
) {
	// Check that the arclength parametrization is plausible.
	assert(!isnan(t_to_s[0]) && t_to_s[0] <= t_to_s[15]);

	/// Beginning and end of the segment, in grid coordinates.
	auto const start_point = vec4{vec3{start.pos}, start.time} * _scale;
	auto const end_point   = vec4{vec3{  end.pos},   end.time} * _scale;
	/// Grid cell in which the segment begins.
	auto const start_index = round(start_point);

	if constexpr (log_level > 2)
		std::clog << LOG_TAG" Add segment ("<< start_point <<") -> ("<< end_point <<")\n";

	/// Cubic Bézier curve defining the segment's spatial components, scaled to grid coordinates.
	auto const [b0, b1, b2, b3] = std::to_array<vec3>({
		start_point,
		(start.pos + start.tangent*(1/3.0f)) * vec3{_scale},
		(  end.pos -   end.tangent*(1/3.0f)) * vec3{_scale},
		end_point,
	});
	// If all control points lie in the same cell, that cell contains the entire segment.
	if (
		start_index == round(end_point)
		// Tangent points only have spatial coordinates.
		&& vec3{start_index} == round(b1)
		&& vec3{start_index} == round(b2)
	) {
		add_interval(start_index, node_idcs[0], {0, 1});
		return;
	}

	// TODO: Calculate intersections analytically instead of sampling.

	/// Spatial and temporal extent of the segment.
	auto const arclen   = t_to_s[15] - t_to_s[0];
	auto const duration = end.time - start.time;
	/// Bounds on how much the curve parameter is increased with every sample.
	// The actual maximum step size is max_step/2.
	auto const max_step = _cell_size / vec4{vec3{arclen}, duration};
	auto const min_step = std::min(
		min_value(max_step),
		// Assume a constant average speed.
		min_value(_sample_step / vec2{arclen, duration})
	);
	// Check plausibility.
	assert(min_step > 1e-6 && min_step < 1e6);

	if constexpr (log_level > 3) std::clog << 
		"\tArclength:    " << arclen          << "\n"
		"\tDuration:     " << duration        << "\n"
		"\tMin sampling: " << min_step        << "\n"
		"\tMax sampling: ("<< max_step * 0.5f <<")\n";

	// Begin at the start node.
	auto min_t      = 0.0f;
	auto prev_t     = 0.0f;
	auto prev_point = start_point;
	auto prev_index = start_index;
	// Sample the trajectory.
	do {
		/// Determine the next sample based on how close the last sample was to a grid plane.
		auto const t = std::min(
			prev_t + std::max(
				// Bigger steps from samples near the center of their grid cell.
				min_value(max_step * (vec4{0.5f} - abs(prev_point - prev_index))),
				min_step // Ensure progress.
			),
			1.0f // Limit to segment.
		);

		// Evaluate the segment for the current curve parameter using de Casteljau's algorithm.
		vec4 point;
		{
		auto const p01 = lerp(b0, b1, t);
		auto const p12 = lerp(b1, b2, t);
		auto const p23 = lerp(b2, b3, t);

		auto const p02 = lerp(p01, p12, t);
		auto const p13 = lerp(p12, p23, t);

		point = lerp(vec4{p02, start_point[3]}, {p13, end_point[3]}, t);
		}

		if constexpr (log_level > 3)
			std::clog << std::format(
				"{:5.1f}% {} ({}, {}, {}, {})\n",
				t * 100, start.time + t*duration, point[0], point[1], point[2], point[3]
			);

		/// Grid cell containing the current sample.
		auto const index = round(point);

		// If the trajectory has crossed into another grid cell:
		if (index != prev_index) {
			/// Average difference in curve parameter between the previous sample and every inter-
			/// section between trajectory and grid.
			auto isect_offset = 0.0f;
			auto isect_count  = -1; // Number of grid intersections - 1.

			// Indices differ in at least one component, maybe multiple.
			for (auto d = 0u; d < (_layout == layout::xyz ? 3u : 4u); ++d) {
				if (prev_index[d] == index[d]) continue;
				/// Coordinate of the hyper-plane separating the the two cells in dimension d.
				/// If the step size is small enough that there should only ever be one crossing per
				/// component.
				auto grid_plane = 0.5f*prev_index[d] + 0.5f*index[d];

				if (log_level > 1 && abs(index[d] - prev_index[d]) > 1)
					std::clog << LOG_WARNING LOG_TAG" Skipping cell between ("<< prev_index
						<<") and ("<< index <<")\n";

				// Linearly approximate the intersection between trajectory and grid.
				isect_offset +=
					(t - prev_t) * (grid_plane - prev_point[d])/(point[d] - prev_point[d]);
				++isect_count;
			}
			assert(isect_count >= 0 && isect_count < index.size());
			/// End the interval at the average of all grid crossings.
			auto const max_t =
				prev_t + isect_offset*std::array{1.0f, 0.5f, 1.0f/3, 0.25f}[isect_count];
			// Store the interval in the grid.
			add_interval(prev_index, node_idcs[0], {min_t, max_t});
			// The next interval begins where the previous one ends.
			min_t = max_t;
		}
		// Remember the current sample for the next iteration.
		prev_t     = t;
		prev_point = point;
		prev_index = index;
	} while (prev_t < 1.0f);
	// Create a final interval up to the end of the segment.
	add_interval(prev_index, node_idcs[0], {min_t, 1});
}

auto hash_grid::upload () -> gl_buffer
{
	auto const buffer_size = this->buffer_size();

	// Linearize the grid in a staging buffer in host memory.
	auto const staging = std::make_unique_for_overwrite<std::byte[]>(buffer_size);
	auto const base    = staging.get();
	auto       buckets = reinterpret_cast<Bucket*>(base);

	// For a strided grid, store timestep, address, and size of each table.
	if (_layout == layout::t_xyz) {
		auto const timesteps  = reinterpret_cast<int32_t*>(base);
		auto const tables     = reinterpret_cast<cgv::uvec2*>(timesteps + _tables.size());
		           buckets    = reinterpret_cast<Bucket*>(tables + _tables.size());
		size_t     table_addr = reinterpret_cast<std::byte*>(buckets) - base;

		for (auto i = 0u; i < _tables.size(); ++i) {
			timesteps[i] = _tables[i].timestep;
			tables[i]    = {static_cast<uint32_t>(table_addr / address_unit), _tables[i].length};
			table_addr  += _tables[i].length * sizeof(Bucket);
		}
	}

	// Store the cells after the tables and write table entries.
	auto       head  = reinterpret_cast<std::byte*>(buckets + num_buckets());
	auto const write = [&head]<class T>(T const* data, size_t count) {
		head = reinterpret_cast<std::byte*>(std::uninitialized_copy_n(
			data, count, reinterpret_cast<T*>(head)
		));
	};
	for (auto const& table : _tables) {
		for (auto bucket_idx = 0u; bucket_idx < table.length; ++bucket_idx) {
			auto const& slots = table.buckets()[bucket_idx].slots;
			auto& dest_slots = buckets[bucket_idx].slots;
			uint32_t slot_idx {};

			// Filled slots.
			for (; slot_idx < Bucket::num_slots && slots[slot_idx].cell != no_cell; ++slot_idx) {
				// Write the slot, replacing the cell's host index with its offset in the buffer.
				dest_slots[slot_idx] = {
					slots[slot_idx].signature,
					static_cast<uint32_t>((head - base) / address_unit)
				};
				// Append the cell to the buffer. First write a header consisting of the cell's
				// index and number of contained intervals, then copy the intervals themselves.
				auto const& cell = _cells[slots[slot_idx].cell];
				auto const num_intervals = static_cast<uint32_t>(cell.intervals.size());
				write(&cell.index, 1);
				write(&num_intervals, 1);
				write(cell.intervals.data(), num_intervals);
			}
			/// Mark the remaining slots as empty.
			for (; slot_idx < Bucket::num_slots;) dest_slots[slot_idx++] = {};
		}
		buckets += table.length;
	}
	assert(head - base == buffer_size);

	// Create a GL buffer and upload the linearized data.
	auto buffer = gl_buffer::create();
	glNamedBufferStorage(buffer.handle(), buffer_size, base, 0);
	gl_errors("in hash_grid::upload");
	return buffer;
}

void hash_grid::set_shader_opts (cgv::render::shader_compile_options& opts) const {
	opts.define_macro("HASH_GRID_BUFFER_BINDING",   ssbo_idx::grid_memory );
	opts.define_macro("HASH_GRID_ADDRESS_UNIT",     address_unit          );
	opts.define_macro("HASH_GRID_NUM_HASH_FNS",     uint32_t{num_hash_fns});
	opts.define_macro("HASH_GRID_SLOTS_PER_BUCKET", Bucket::num_slots     );
	opts.define_macro("HASH_GRID_LAYOUT",           _layout               );
	opts.define_macro("HASH_GRID_SIGNATURE_FN",     _signature_fn         );
}

void hash_grid::set_uniforms (cgv::render::context& c, cgv::render::shader_program& p) const
{
	p.set_uniform(c, "hash_grid_cell_size", _cell_size);
	p.set_uniform(c, "hash_grid_scale",     _scale);
	p.set_uniform(c, "hash_grid_data_len",
		_layout == layout::t_xyz ? static_cast<uint32_t>(_tables.size()) : _tables[0].length);
}

auto hash_grid::num_buckets() const -> size_t
{
	size_t num_buckets = 0;
	for (auto& table : _tables) num_buckets += table.length;
	return num_buckets;
}

auto hash_grid::buffer_size () const -> size_t
{
	return (_layout == layout::t_xyz
			? _tables.size() * 3 * sizeof(uint32_t) // address, size, timestep
			: 0 // only one table, no additional info required
		)
		+ num_buckets()*sizeof(Bucket)
		+ _cells.size()*(sizeof(Index) + 4)
		+ _num_intervals*sizeof(Interval);
}


void hash_grid::add_interval (Index index, uint32_t start_node, vec2 range) {
	if constexpr (log_level > 2)
		std::clog << LOG_TAG" Add interval ("<< range <<") to cell ("<< index <<").\n";

	// Convert range into normalized integers and pack them.
	auto const irange = cgv::uvec2{round(range * 0xffff)};
	find_or_insert(index, 2, no_cell)
		.intervals.emplace_back(start_node, irange[0] | (irange[1] << 16));

	++_num_intervals;
#if OTV_HASH_GRID_VALIDATION
	// Track the number of intervals in each cell.
	++_validation.cell_fill[index];
#endif
}

auto hash_grid::find_or_insert (Index index, float growth_factor, uint32_t new_cell) -> Cell&
{
	// Find or create the table for this timestep.
	auto& table = this->table(index[3]);

	while (true) {
		// In the vast majority of cases, the cell is already stored in the table or can be
		// inserted into the existing buckets.
		if (auto const cell = try_find_or_insert(table, index, new_cell)) return *cell;

		// In the unlikely event that a new cell cannot be inserted, more buckets must be allocated.
		auto const old_table = std::move(table);
		auto new_length = old_table.length;
		uint32_t num_cells;

		// Rebuild the table with double capacity.
		grow: {
			new_length = static_cast<uint32_t>(ceil(new_length * growth_factor));

			if constexpr (log_level > 0) {
				// Print the load factor before resizing to determine efficiency.
				auto const num_slots = old_table.length * Bucket::num_slots;
				std::clog << std::format(LOG_TAG" Insert failed with {}/{} slots occupied "
					"({:.1f}%), growing to {} slots.\n",
					old_table.num_entries, num_slots,
					float(old_table.num_entries) / num_slots * 100.f,
					new_length * Bucket::num_slots
				);
			}

			// Prevent excessive memory use in case of bugs in the resize implementation.
			if (new_length > 1 << 20) std::abort();

			// Reinsert all entries into the new buckets.
			table = {new_length, table.timestep};
			num_cells = 0;
			for (auto const& bucket : old_table.buckets()) for (auto const slot : bucket.slots) {
				if (slot.cell == no_cell) break; // next bucket
				++num_cells;

				// If insertion fails during a rehash (extremely unlikely), abort the current
				// attempt and try again with yet more buckets.
				if (!try_find_or_insert(table, _cells[slot.cell].index, slot.cell))
					goto grow;
			}
		}

		assert(num_cells = old_table.num_entries);

#if OTV_HASH_GRID_VALIDATION > 1
		// Check that no entries have been lost.
		for (auto const& bucket : old_table.buckets()) for (auto const slot : bucket.slots) {
			if (slot.cell == no_cell) continue;
			auto const& idx = _cells[slot.cell].index;
			assert(try_find_or_insert(table, idx)->intervals.size() == _validation.cell_fill[idx]);
		}
#endif
		// Once all previous entries have been restored, reattempt to insert the new index.
	}
}

auto hash_grid::table (int32_t timestep) -> Table&
{
	// Only strided grids have more than one table.
	if (_layout != layout::t_xyz) return _tables[0];

	// Search for the requested timestep.
	auto const table = std::ranges::lower_bound(_tables, timestep, {}, [](auto const& table) {
		return table.timestep;
	});
	// If it exists, return the corresponding buckets.
	if (table < _tables.end() && table->timestep == timestep) return *table;

	// Otherwise, a new table must be allocated.
	if (log_level > 1)
		std::clog << LOG_TAG" Allocate "<< _initial_buckets <<" buckets for timestep "<< timestep <<".\n";

	return *_tables.emplace(table, _initial_buckets, timestep);
}

auto hash_grid::try_find_or_insert (Table& table, Index query, uint32_t new_cell) -> Cell*
{
	/// Hash the query index into its shorter signature for quick comparison.
	auto const signature = this->signature(query);
	auto const buckets = table.buckets();

	struct {
		Bucket*  data {};
		unsigned load {~0u};
	}
	/// Tracks which of the buckets that the entry being inserted hashes to has the most free slots.
	dest_bucket {};

	// Check all buckets the queried cell hashes to.
	for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
		auto& bucket = this->bucket(buckets, signature, fn);
		/// Count the number of occupied slots in this bucket.
		auto load = 0u;

		// Check all occupied slots.
		for (auto& slot : bucket.slots) {
			// Buckets are filled front to back and entries are never deleted, so if we find a free
			// slot, all following ones must be free as well.
			if (slot.cell == no_cell) break;
			if (slot.signature.value == signature.value) {
				auto& cell = _cells[slot.cell];
				// Since many indices map to the same signature, we must load the index of the
				// possible match for an exact comparison.
				// In practice, a good hash makes signature collisions unlikely.
			 	if (cell.index == query) {
					// We have found the queried cell.
#if OTV_HASH_GRID_VALIDATION
					assert(cell.intervals.size() == _validation.cell_fill[query]);
#endif
					return &cell;
				}
				// Report signature hash collisions.
				if constexpr (log_level > 1)
					std::clog << LOG_TAG" Signature collision for indices ("<< cell.index <<")"
						"and (" << query <<")\n";
			}
			++load;
		}
		// Track the least occupied bucket.
		if (load < dest_bucket.load) dest_bucket = {&bucket, load};
	}
	// If the index cannot be found in the table, insert it.

	auto const new_cell_given = new_cell != no_cell;
	if (new_cell_given)
		// Check that the given cell actually matches the query index.
		assert(_cells[new_cell].index == query);
	else {
		// If no cell has been given to insert, allocate a new one.
		new_cell = _cells.size();
		_cells.emplace_back(query);
#if OTV_HASH_GRID_VALIDATION
		// Check that the index really does not exist in the table, then insert it into the
		// validation map with zero trajectory intervals.
		// This check is not performed when `new_cell` is given, since that only happens during
		// rehashing, which does not affect the validation map.
		if (_validation.cell_fill[query] != 0) {
			std::clog << LOG_ERROR LOG_TAG" Could not find cell ("<< query <<") "
				"even though it has been added to the grid before.\n";
			std::abort();
		}
#endif
	}

	/// The entry currently being inserted. Initialized to the new cell, updated with every cuckoo.
	auto floating_entry = Slot{signature, new_cell};
	/// Hash function the entry being inserted was last stored with. For the new cell that was never
	/// in the table, it is initialized such that the next function will be zero.
	uint8_t prev_fn = -1;

	/// Track the location of the newly created cell within the table for the return value.
	Slot* new_entry = nullptr;

	// Move entries until we find a free slot or some maximum number of iterations is reached.
	auto cuckoo_chain = 0u;
	while (true) {
		// If one of the candidate buckets has a free slot, store the entry and return.
		if (dest_bucket.load < Bucket::num_slots) {
			// Buckets are filled front to back.
			auto& dest = dest_bucket.data->slots[dest_bucket.load] = floating_entry;
			// Ensure that the return value points to the new cell.
			if (floating_entry.cell == new_cell) new_entry = &dest;
			else assert(new_entry->cell == new_cell);

			if constexpr (log_level > 2)
				std::clog << LOG_TAG" Inserted cell ("<< query <<") after "
					<< cuckoo_chain <<" cuckoos.\n";

			++table.num_entries;
			return &_cells[new_cell];
		}

		// Count iterations and abort insertion after maximum.
		if (cuckoo_chain++ >= max_cuckoo_chain) break;

		// If all candidate buckets are full, pick one by cycling through hash functions, then store
		// the entry in a random slot.
		auto& cuckoo_bucket = this->bucket(
			buckets,
			floating_entry.signature,
			(prev_fn + 1) % num_hash_fns
		);
		auto& dest = cuckoo_bucket.slots[
			std::uniform_int_distribution{size_t{0}, Bucket::num_slots - 1}(_rng)
		];
		// Remember where the new cell is stored.
		if (floating_entry.cell == new_cell) new_entry = &dest;
		// Cuckoo the previous entry from the chosen slot.
		std::swap(dest, floating_entry);

		// Find the best bucket to store the displaced entry.
		dest_bucket.load = ~0u;
		for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
			auto& bucket = this->bucket(buckets, floating_entry.signature, fn);

			// We know that the bucket the entry was displaced from is full already.
			// Remember the associated hash function, so we know which one to try next if all
			// candidate buckets are full again.
			if (&bucket == &cuckoo_bucket) {
				prev_fn = fn;
				assert(bucket.slots.back().cell != no_cell);
				continue;
			}

			// Count the occupied slots in the bucket.
			// Iterate back to front, since at this point buckets likely have only a few free slots.
			unsigned load = Bucket::num_slots;
			while (load > 0 && bucket.slots[load - 1].cell == no_cell) --load;
			if (load < dest_bucket.load) dest_bucket = {&bucket, load};
		}
	}
	if constexpr (log_level > 2)
		std::clog << LOG_TAG" Could not insert cell ("<< query <<") after "
			<< max_cuckoo_chain <<" cuckoos.\n";
	// If no free slot could be found in the given number of tries, the table must be resized.
	// All cells in the table before this function call are reinserted into the new buckets.
	// To this end, the last cell to be displaced is restored into the slot of the new cell.
	// It will be in the wrong bucket with the wrong signature, but for rehashing that is OK.
	// The new cell will be added to the expanded table after all previous entries have been moved.
	if (floating_entry.cell != new_cell) *new_entry = floating_entry;
	if (!new_cell_given) _cells.pop_back();
	return nullptr;
}

auto hash_grid::signature (Index index) const noexcept -> Signature
{
	auto const dims = _layout == layout::xyzt ? 4u : 3u;
	/// Treat the indices as unsigned integers.
	auto const uidx = *reinterpret_cast<cgv::uvec4 const*>(&index);

	switch (_signature_fn) {
	case signature_fn::mult_xor: {
		/// Prime constants taken from Teschner et al., except for `c0 = 1` as in Müller et al.
		auto const c = std::to_array<uint32_t>({1, 73856093, 19349663, 83492791});
		auto sig     = uint32_t{};
		for (auto d = 0u; d < dims; ++d) sig ^= c[d]*uidx[d];
		return {sig};
	}
	case signature_fn::xxhash32: {
		// Multibyte xxHash32 algorithm by Collet 2012, as implemented by Jarzynski and Olano 2020.

		/// Prime constants.
		auto const [c2, c3, c4, c5] = std::to_array<uint32_t>({
			2246822519, 3266489917, 668265263, 374761393
		});
		// Compared to the implementation by Jarzynski and Olano, `index` is rotated by one
		// component.
		auto sig = uidx[0] + c5;
		for (auto d = 1u; d < dims; ++d) {
			sig += c3 * uidx[d];
			sig  = c4 * ((sig << 17) | (sig >> 15));
		}
		sig = c2 * (sig ^ (sig >> 15));
		sig = c3 * (sig ^ (sig >> 13));
		return {sig ^ (sig >> 16)};
	}
	case signature_fn::z_order: {
		/// Lookup table for spreading out bits to make room for other dimensions.
		auto const& lut = _layout == layout::xyzt ? spread4 : spread3;
		auto sig        = uint32_t{};
		for (auto d = 0u; d < dims; ++d)
			// Only the least significant `32 / dims` bits affect the result.
			for (auto b = 0u; b <= (_layout == layout::xyzt ? 0 : 8); b += 8)
				// Get byte, scatter bits, then offset by one bit per dimension and shift back to
				// the byte's original offset.
				sig |= lut[uidx[d]>>b & 0xff] << (b*dims + d);
		return {sig};
	}
	};
}

auto hash_grid::bucket (std::span<Bucket> buckets, Signature signature, uint8_t hash_fn)
	noexcept -> Bucket&
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
	return buckets[hash % buckets.size()];
}


hash_grid::Table::Table(uint32_t num_buckets, int32_t timestep)
	: data     {std::make_unique<Bucket[]>(num_buckets)}
	, length   {num_buckets}
	, timestep {timestep}
{}

constexpr auto hash_grid::Table::buckets () noexcept -> std::span<Bucket>
{
	return {data.get(), length};
}
constexpr auto hash_grid::Table::buckets () const noexcept -> std::span<const Bucket>
{
	return {data.get(), length};
}
