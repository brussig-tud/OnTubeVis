// CGV framework
#include <cgv/render/context.h>
#include <cgv/render/shader_program.h>

// local includes
#include <util.h>

// implemented header
#include "render/hash_grid.h"


/// Marks log messages from this file.
#define LOG_TAG "\x1b[1m[hash_grid]\x1b[m"
/// Message severity.
#define LOG_ERROR   "\x1b[1;31m[error]\x1b[m"
#define LOG_WARNING "\x1b[1;33m[warning]\x1b[m"


namespace otv {
namespace {

using cgv::vec2;
using cgv::vec3;
using cgv::vec4;

/// Controls the amount of debug messages produced by the grid.
/// Ranges from 0 (no output) to 4 (full output).
constexpr auto log_level = 2u;

/// Stream arguments to `std::clog`.
template<typename... Args>
void log(Args&&... args)
{
	(std::clog << ... << std::forward<Args>(args));
}


/// In the shader, pointers are emulated with offsets into the grid buffer.
/// This value determines the size in bytes of their minimal addressable unit.
constexpr uint32_t address_unit = 1u;

/// The number of hash functions mapping cells to buckets, i.e. how many candidate buckets can store
/// each entry.
/// More functions allow higher load factors at the cost of increased query time.
constexpr uint8_t num_hash_fns = 2;

/// Maximum iterations to try when inserting a cell into the grid.
/// Larger numbers allow for a higher load factor at the cost of longer worst-case insertion time.
constexpr auto max_cuckoo_chain = 50u;

} // namespace


template <class T>
constexpr hash_grid::ptr_t<T>::ptr_t(memory_region region, T* ptr) noexcept
	: address {
		static_cast<uint32_t>(reinterpret_cast<std::byte*>(ptr) - region.data())
		/ address_unit
	}
{
	// Check that the pointer lies within the region and its offset can be represented in 32 bits.
	assert(get(region) == ptr);
}

template <class T>
constexpr hash_grid::ptr_t<T>::operator bool() const noexcept
{
	return address != ~0u;
}

template <class T>
constexpr auto hash_grid::ptr_t<T>::operator== (ptr_t other) const noexcept -> bool
{
	return address == other.address;
}

template <class T>
constexpr auto hash_grid::ptr_t<T>::get (memory_region region) const noexcept -> T*
{
	assert(address * address_unit < region.size());
	return reinterpret_cast<T*>(&region[address * address_unit]);
}


template <class T>
constexpr hash_grid::span_t<T>::span_t(memory_region region, std::span<T> span) noexcept
	: start {region, span.data()}
	, len   {static_cast<uint32_t>(span.size())}
{
	// Check that the length can be represented with 32 bits.
	assert(len == span.size());
}

template <class T>
constexpr auto hash_grid::span_t<T>::get (memory_region region) const noexcept -> std::span<T>
{
	return {start.get(region), len};
}


hash_grid::cell_t::cell_t(pmr::memory_region& memory, index_t index)
{
	// Start with the smallest capacity > 0 such that the total size of the allocation is a power of
	// two.
	auto const capacity =
		(std::bit_ceil(offsetof(data_t, intervals[1])) - offsetof(data_t, intervals))
		/ sizeof(interval_t);
	// Allocate and initialize cell data.
	data = {
		memory.span(),
		&(*allocate(memory, capacity) = {.index = index, .size = 0, .capacity = capacity})
	};
}

void hash_grid::cell_t::add_interval (pmr::memory_region& memory, interval_t interval)
{
	auto const region = memory.span();
	auto& data        = *this->data.get(region);
	// Load header.
	auto const [index, size, capacity, _] = data;

	// If the allocation has spare capacity, append the new interval.
	if (size < capacity) {
		std::construct_at(&data.intervals[data.size++], interval);
		return;
	}

	// Sanity check: Array length cannot exceed allocation size.
	assert(size == capacity);

	// If the current allocation is full, reallocate with twice as much memory.
	auto const new_capacity = capacity * 2
		+ static_cast<uint32_t>(offsetof(data_t, intervals) / sizeof(interval_t));

	if constexpr (log_level > 2)
		log(LOG_TAG" Grow cell from capacity ",capacity," to ",new_capacity,".\n");

	auto& new_data = *allocate(memory, new_capacity) = {index, size + 1, new_capacity};
	// Copy previous intervals.
	std::uninitialized_move_n(data.intervals, size, new_data.intervals);
	// Append the new interval.
	std::construct_at(&new_data.intervals[size], interval);

	// Free the previous allocation.
	memory.deallocate(
		&data,
		offsetof(data_t, intervals) + capacity * sizeof(interval_t),
		alignof(data_t)
	);
	// Point to the new allocation.
	this->data = {region, &new_data};
}

void hash_grid::cell_t::free (pmr::memory_region& memory) noexcept
{
	auto& data = *this->data.get(memory.span());
	memory.deallocate(
		&data,
		offsetof(data_t, intervals) + data.capacity * sizeof(interval_t),
		alignof(data_t)
	);
}

auto hash_grid::cell_t::allocate (pmr::memory_region& memory, uint32_t capacity) -> data_t*
{
	return std::construct_at(static_cast<data_t*>(memory.allocate(
		offsetof(data_t, intervals) + capacity*sizeof(interval_t),
		alignof(data_t)
	)));
}


hash_grid::hash_grid(pmr::memory_region* memory, params const& params, uint32_t initial_buckets)
	: _memory    {memory}
	, _cell_size {params.layout == layout::xyz
		// A 3D grid is equivalent to a 4D grid with infinite cell extent in time.
		? vec4{vec3{params.cell_size}, std::numeric_limits<float>::infinity()}
		: params.cell_size
	}
	, _scale           {vec4{1} / _cell_size} // index = round(point / _cell_size)
	, _sample_step     {max(params.sample_step, vec2{0.001})}
	, _layout          {params.layout}
	, _initial_buckets {static_cast<uint8_t>(
		std::bit_width(std::max(initial_buckets, 1u) - 1)) // ceil(log2)
	}
{
	auto const region = memory->span();

	// Check that every address allocated by the memory resource can be represented as a 32 bit
	// offset.
	auto const address_limit = (static_cast<size_t>(address_unit) << 32) - 1;
	if (region.size() >= address_limit)
		throw std::runtime_error{concat(
			region.size()," byte memory region exceeds the hash grid's address limit of ",
			address_limit," bytes."
		)};

	// Initialize tables.
	if (_layout == layout::t_xyz) {
		auto const capacity = 16;
		_data.tables = {
			{region, allocate<span_t<bucket_t>>(capacity).first(0)}, // Length 0.
			ptr_t{region, allocate<int32_t>(capacity).data()},
			capacity
		};
	} else {
		auto const buckets = allocate<bucket_t>(1u << _initial_buckets);
		std::ranges::uninitialized_default_construct(buckets);
		_data.buckets = {region, buckets};
	}

	if constexpr (log_level > 0) {
		using std::operator""sv;
		auto const layout_name = std::array{"3D"sv, "Strided 3D"sv, "4D"sv}
			[static_cast<size_t>(_layout)];
		log(LOG_TAG" Create instance.\n"
			"\tLayout:          " ,layout_name             , "\n"
			"\tCell size:       (",_cell_size              ,")\n"
			"\tSample step:     " ,_sample_step[0]         , " (space), "
			                      ,_sample_step[1]         , " (time)\n"
			"\tInitial buckets: " ,(1u << _initial_buckets), "\n"
		);
	}
}

hash_grid::hash_grid(hash_grid&& src) noexcept
	: _rng     {std::move(src._rng)}
	, _memory  {std::move(src._memory)}
	, _data    {std::move(src._data)}
#if OTV_HASH_GRID_VALIDATION
	, _validation {std::move(src._validation)}
#endif
	, _cell_size       {std::move(src._cell_size)}
	, _scale           {std::move(src._scale)}
	, _sample_step     {std::move(src._sample_step)}
	, _layout          {std::move(src._layout)}
	, _initial_buckets {std::move(src._initial_buckets)}
{
	// Take ownership of tables.
	if (_layout == layout::t_xyz) src._data.tables = {};
	else src._data.buckets = {};
}

auto hash_grid::operator= (hash_grid&& src) noexcept -> hash_grid&
{
	// Self-assignment is a NOP.
	if (&src == this) return *this;

	// Free any current tables.
	free();

	// Move member variables.
	_rng    = std::move(src._rng);
	_memory = std::move(src._memory);
	_data   = std::move(src._data);
#if OTV_HASH_GRID_VALIDATION
	_validation = std::move(src._validation);
#endif
	_cell_size       = std::move(src._cell_size);
	_scale           = std::move(src._scale);
	_sample_step     = std::move(src._sample_step);
	_layout          = std::move(src._layout);
	_initial_buckets = std::move(src._initial_buckets);

	// Take ownership of tables.
	if (_layout == layout::t_xyz) src._data.tables = {};
	else src._data.buckets = {};
	return *this;
}

hash_grid::~hash_grid() noexcept
{
	free();
}

void hash_grid::add_segment (
	node_attribs const& start,
	node_attribs const& end,
	cgv::uvec2          node_idcs,
	cgv::mat4 const&    t_to_s
) {
	// Check that the arclength parametrization is plausible.
	assert(!isnan(t_to_s[0]) && t_to_s[0] <= t_to_s[15]);

	/// Beginning and end of the segment, in grid coordinates.
	auto const start_point = vec4{vec3{start.pos_rad}, start.t[0]} * _scale;
	auto const end_point   = vec4{vec3{  end.pos_rad},   end.t[0]} * _scale;
	/// Grid cell in which the segment begins.
	auto const start_index = round(start_point);

	if constexpr (log_level > 2)
		log(LOG_TAG" Add segment (",start_point,") to (",end_point,")\n");

	/// Cubic Bézier curve defining the segment's spatial components, scaled to grid coordinates.
	auto const [b0, b1, b2, b3] = std::to_array<vec3>({
		start_point,
		(start.pos_rad + start.tangent*(1/3.0f)) * _scale,
		(  end.pos_rad -   end.tangent*(1/3.0f)) * _scale,
		end_point,
	});
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
	auto const max_step = _cell_size / vec4{vec3{arclen}, duration};
	auto const min_step = std::min(
		min_value(max_step),
		// Assume a constant average speed.
		min_value(_sample_step / vec2{arclen, duration})
	);
	// Check plausibility.
	assert(min_step > 1e-6 && min_step < 1e6);

	if constexpr (log_level > 3) log(
		"\tArclength:    " ,arclen         , "\n"
		"\tDuration:     " ,duration       , "\n"
		"\tMin sampling: " ,min_step       , "\n"
		"\tMax sampling: (",max_step * 0.5f,")\n"
	);

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

		if constexpr (log_level > 3) {
			auto const prec = std::clog.precision();
			log(std::setw(5),std::fixed,std::setprecision(1),t * 100,"% ",std::defaultfloat,prec,
				start.t[0] + t*duration," (",point,")\n");
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
			for (auto d = 0u; d < (_layout == layout::xyz ? 3u : 4u); ++d) {
				if (prev_index[d] == index[d]) continue;
				/// Coordinate of the hyper-plane separating the the two cells in dimension d.
				/// If the step size is small enough that there should only ever be one crossing per
				/// component.
				auto grid_plane = 0.5f*prev_index[d] + 0.5f*index[d];

				if (log_level > 1 && abs(index[d] - prev_index[d]) > 1)
					log(LOG_WARNING LOG_TAG" Skipping cell between (",prev_index,") and (",index,
						")\n");

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
	sc::set_define(d, "HASH_GRID_ADDRESS_UNIT",     address_unit,                        {});
	sc::set_define(d, "HASH_GRID_NUM_HASH_FNS",     uint32_t{num_hash_fns},              {});
	sc::set_define(d, "HASH_GRID_SLOTS_PER_BUCKET", bucket_t::num_slots,                 {});
	sc::set_define(d, "HASH_GRID_CELL_HEADER_SIZE", offsetof(cell_t::data_t, intervals), {});
	sc::set_define(d, "HASH_GRID_LAYOUT",           _layout,                             {});
}

void hash_grid::set_uniforms (cgv::render::context& c, cgv::render::shader_program& p) const
{
	auto const memory = _memory->span();
	auto ok = true
	&& p.set_uniform(c, "hash_grid_cell_size", _cell_size)
	&& p.set_uniform(c, "hash_grid_scale",     _scale)
	// All members of `_data` have a common `span_t` subsequence, so reading `buckets` is always
	// valid.
	&& p.set_uniform(c, "hash_grid_data.start.address", _data.buckets.start.address)
	&& p.set_uniform(c, "hash_grid_data.len",           _data.buckets.len)
	;
	if (_layout == layout::t_xyz)
		ok &= p.set_uniform(c, "hash_grid_timesteps.address", _data.tables.timesteps.address);
	assert(ok);
}

void hash_grid::free () noexcept
{
	// No tables allocated, nothing to do.
	if (!_data.buckets.start) {
#if OTV_HASH_GRID_VALIDATION
		assert(_validation.cell_fill.empty());
#endif
		return;
	}

	// Traverse tables to free cells.
	auto const region = _memory->span();
	auto const tables = _layout == layout::t_xyz
		? _data.tables.buckets.get(region)
		: std::span{&_data.buckets, 1};

	for (auto const& table : tables) {
		auto const buckets = table.get(region);
		for (auto& bucket : buckets)
			for (auto& slot : bucket.slots) {
				if (!slot.cell.data) break;
				slot.cell.free(*_memory);
			}
		free(buckets);
	}

	// Mark as empty.
	leak();
}

void hash_grid::leak () noexcept
{
	// Forget tables.
	if (_layout == layout::t_xyz) _data.tables = {};
	else _data.buckets = {};

	// The grid is now empty.
#if OTV_HASH_GRID_VALIDATION
	_validation.cell_fill = {};
#endif
}


template <class T>
auto hash_grid::allocate (uint32_t count) -> std::span<T>
{
	return std::span{
		static_cast<T*>(
			_memory->allocate(count * sizeof(T),
			alignof(T))
		),
		count
	};
}

template <class T>
void hash_grid::free (std::span<T> alloc) noexcept
{
	_memory->deallocate(alloc.data(), alloc.size_bytes(), alignof(T));
}

void hash_grid::add_interval (index_t index, interval_t interval)
{
	if constexpr (log_level > 2)
		log(LOG_TAG" Add interval (",interval.time,") to cell (",index,").\n");

	cell(index).add_interval(*_memory, interval);

#if OTV_HASH_GRID_VALIDATION
	// Track the number of intervals in each cell.
	++_validation.cell_fill[index];
#endif
}

auto hash_grid::cell (index_t index) -> cell_t&
{
	auto const region = _memory->span();
	// Find or create the table for this timestep.
	auto& table = this->table(index[3]);
	// Access its buckets.
	auto buckets = table.get(region);

	while (true) {
		// In the vast majority of cases, the cell is already stored in the table or can be
		// inserted into the existing buckets.
		if (auto const cell = find_or_insert(buckets, index)) return *cell;
		// In the unlikely event that a new cell cannot be inserted, more buckets must be allocated.

		/// Save the contents of the table.
		auto const old_buckets = buckets;

		// Rebuild the table with double capacity.
		rehash:
			auto const new_size = buckets.size() * 2;
			if constexpr (log_level > 2)
				log(LOG_TAG" Grow table from ",buckets.size()," to ",new_size," buckets.\n");

			// Allocate new buckets and initialize them as empty.
			buckets = allocate<bucket_t>(new_size);
			std::ranges::uninitialized_default_construct(buckets);
			// Store the new pointer.
			table = {region, buckets};

			// Reinsert all entries in the new buckets.
			auto num_cells = size_t{0};
			for (auto& bucket : old_buckets)
				for (auto& slot : bucket.slots) {
					// Count cells to determine load factor.
					++num_cells;
					// Buckets are filled front to back, all following slots are empty.
					if (!slot.cell.data) break;
					// If insertion fails during a rehash (extremely unlikely), abort the current
					// attempt and try again with yet more buckets.
					if (!find_or_insert(buckets, slot.cell.data.get(region)->index, slot.cell)) {
						free(buckets);
						goto rehash;
					}
				}

		if constexpr (log_level > 0) {
			// Print the load factor before resizing to determine efficiency.
			auto const num_slots = old_buckets.size() * bucket_t::num_slots;
			auto const prec      = std::clog.precision();
			log(LOG_TAG" Rehashed table after insertion failed with ",num_cells,"/",num_slots,
				" slots occupied (load factor ",std::setprecision(3),
				static_cast<float>(num_cells) / num_slots,").\n",prec);
		}

		// Free the previous allocation.
		free(old_buckets);

#if OTV_HASH_GRID_VALIDATION > 1
		// Check that no entries have been lost.
		for (auto const& [idx, fill] : _validation.cell_fill) {
			if (_layout == layout::t_xyz && idx[3] != index[3]) continue;
			assert(find_or_insert(buckets, idx)->data.get(region)->size == fill);
		}
#endif
		// Once all previous entries have been restored, reattempt to insert the new index.
	}
}

auto hash_grid::table (int32_t timestep) -> span_t<bucket_t>&
{
	// Only strided grids have more than one table.
	if (_layout != layout::t_xyz) return _data.buckets;

	auto const region = _memory->span();

	// Obtain a view over all allocated tables, not just valid ones.
	auto bucket_vec   = std::span{_data.tables.buckets.start.get(region), _data.tables.capacity};
	auto timestep_vec = std::span{_data.tables.timesteps.get(region),     _data.tables.capacity};
	// Number of tables that have actually been initialized.
	auto const num_tables = _data.tables.buckets.len;

	// Search for the requested timestep.
	auto const ts_iter = std::ranges::lower_bound(timestep_vec.first(num_tables), timestep);
	auto const ts_idx  = std::distance(timestep_vec.begin(), ts_iter);
	// If it exists, return the corresponding buckets.
	if (ts_idx < num_tables && *ts_iter == timestep) return bucket_vec[ts_idx];

	// Otherwise, a new table must be allocated.
	auto const num_buckets = 1u << _initial_buckets;
	if (log_level > 1) log(LOG_TAG" Allocate ",num_buckets," buckets for timestep ",timestep,".\n");

	auto const new_buckets = allocate<bucket_t>(num_buckets);
	std::ranges::uninitialized_default_construct(new_buckets);
	++_data.tables.buckets.len;

	// The first table must be handled specially,
	if (num_tables == 0) {
		std::construct_at(&timestep_vec[0], timestep);
		return *std::construct_at(&bucket_vec[0], region, new_buckets);
	}

	// If there is free capacity, shift elements to make room for the new table.
	if (num_tables < _data.tables.capacity) {
		std::construct_at(&timestep_vec[num_tables], std::move(timestep_vec[num_tables - 1]));
		std::shift_right(ts_iter, timestep_vec.begin() + num_tables, 1);
		*ts_iter = timestep;

		std::construct_at(&bucket_vec[num_tables], std::move(bucket_vec[num_tables - 1]));
		std::shift_right(bucket_vec.begin() + ts_idx, bucket_vec.begin() + num_tables, 1);
		return bucket_vec[ts_idx] = {region, new_buckets};
	}

	// Otherwise, reallocate the table vector with double capacity.
	_data.tables.capacity *= 2;

	if (log_level > 1)
		log(LOG_TAG" Grow table vector from capacity ",bucket_vec.size()," to ",
			_data.tables.capacity,".\n");

	// Move timesteps, splicing the new entry inbetween.
	auto const new_timestep_vec = allocate<int32_t>(_data.tables.capacity);
	std::uninitialized_move_n(timestep_vec.begin(), ts_idx, new_timestep_vec.begin());
	std::construct_at(&new_timestep_vec[ts_idx], timestep);
	std::uninitialized_move(
		timestep_vec.begin() + ts_idx,
		timestep_vec.end(),
		new_timestep_vec.begin() + ts_idx + 1
	);
	_data.tables.timesteps = {region, new_timestep_vec.data()};

	// Move tables, leaving room for the new one.
	auto const new_bucket_vec = allocate<span_t<bucket_t>>(_data.tables.capacity);
	std::uninitialized_move_n(bucket_vec.begin(), ts_idx, new_bucket_vec.begin());
	auto& result = *std::construct_at(&new_bucket_vec[ts_idx], span_t{region, new_buckets});
	std::uninitialized_move(
		bucket_vec.begin() + ts_idx,
		bucket_vec.end(),
		new_bucket_vec.begin() + ts_idx + 1
	);
	_data.tables.buckets.start = {region, new_bucket_vec.data()};

	return result;
}

auto hash_grid::find_or_insert (std::span<bucket_t> buckets, index_t query, cell_t new_cell)
	-> cell_t*
{
	// Recurring constants.
	auto const region = _memory->span();

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
		auto& bucket = this->bucket(buckets, signature, fn);
		/// Count the number of occupied slots in this bucket.
		auto load = 0u;

		// Check all occupied slots.
		for (auto& slot : bucket.slots) {
			// Buckets are filled front to back and entries are never deleted, so if we find a free
			// slot, all following ones must be free as well.
			if (!slot.cell.data) break;
			if (slot.signature.value == signature.value) {
				// Since many indices map to the same signature, we must load the index of the
				// possible match for an exact comparison.
				// In practice, a good hash makes signature collisions unlikely.
				auto const index = slot.cell.data.get(region)->index;
			 	if (index == query) {
					// We have found the queried cell.
#if OTV_HASH_GRID_VALIDATION
					assert(slot.cell.data.get(region)->size == _validation.cell_fill[query]);
#endif
					return &slot.cell;
				}
				// Report signature hash collisions.
				if constexpr (log_level > 1)
					log(LOG_TAG" Signature collision for indices (",index,") and (",query,")\n");
			}
			++load;
		}
		// Track the least occupied bucket.
		if (load < dest_bucket.load) dest_bucket = {&bucket, load};
	}
	// If the index cannot be found in the table, insert it.

	if (new_cell.data)
		// Check that the given cell actually matches the query index.
		assert(new_cell.data.get(region)->index == query);
	else {
		// If no cell has been given to insert, allocate a new one.
		new_cell = cell_t{*_memory, query};
#if OTV_HASH_GRID_VALIDATION
		// Check that the index really does not exist in the table, then insert it into the
		// validation map with zero trajectory intervals.
		// This check is not performed when `new_cell` is given, since that only happens during
		// rehashing, which does not affect the validation map.
		if (_validation.cell_fill[query] != 0) {
			log(LOG_ERROR LOG_TAG" Could not find cell (",query,") "
				"even though it has been added to the grid before.\n");
			std::abort();
		}
#endif
	}

	/// The entry currently being inserted.
	/// Initialized to the new cell, updated with every cuckoo.
	auto floating_entry = slot_t{signature, new_cell};
	/// Hash function the entry being inserted was last stored with.
	/// For the new cell that was never in the table, it is initialized such that the next function
	/// will be zero.
	uint8_t prev_fn = ~0;

	/// Track the location of the newly created cell within the table for the return value.
	cell_t* new_entry = nullptr;

	// Move entries until we find a free slot or some maximum number of iterations is reached.
	auto cuckoo_chain = 0u;
	while (true) {
		// If one of the candidate buckets has a free slot, store the entry and return.
		if (dest_bucket.load < bucket_t::num_slots) {
			// Buckets are filled front to back.
			auto& dest = dest_bucket.data->slots[dest_bucket.load] = floating_entry;
			// Ensure that the return value points to the new cell.
			if (floating_entry.cell.data == new_cell.data) new_entry = &dest.cell;
			else assert(new_entry->data == new_cell.data);

			if constexpr (log_level > 2)
				log(LOG_TAG" Inserted cell (",query,") after ",cuckoo_chain," cuckoos.\n");

			return new_entry;
		}

		// Count iterations and abort insertion after maximum.
		if (cuckoo_chain >= max_cuckoo_chain) break;
		++cuckoo_chain;

		// If all candidate buckets are full, pick one by cycling through hash functions, then store
		// the entry in a random slot.
		auto& cuckoo_bucket = this->bucket(
			buckets,
			floating_entry.signature,
			(prev_fn + 1) % num_hash_fns
		);
		auto& dest = cuckoo_bucket.slots[
			std::uniform_int_distribution{size_t{0}, bucket_t::num_slots - 1}(_rng)
		];
		// Remember where the new cell is stored.
		if (floating_entry.cell.data == new_cell.data) new_entry = &dest.cell;
		// Cuckoo the previous entry from the chosen slot.
		std::swap(dest, floating_entry);

		// Find the best bucket to store the displaced entry.
		dest_bucket = {};
		for (uint8_t fn = 0; fn < num_hash_fns; ++fn) {
			auto& bucket = this->bucket(buckets, floating_entry.signature, fn);

			// We know that the bucket the entry was displaced from is full already.
			// Remember the associated hsah function, so we know which one to try next if all
			// candidate buckets are full again.
			if (&bucket == &cuckoo_bucket) {
				prev_fn = fn;
				assert(bucket.slots.back().cell.data);
				continue;
			}

			// Count the occupied slots in the bucket.
			// Iterate back to front, since at this point buckets likely have only a few free slots.
			unsigned load = bucket_t::num_slots;
			while (load > 0 && !bucket.slots[load - 1].cell.data) --load;
			if (load < dest_bucket.load) dest_bucket = {&bucket, load};
		}
	}
	if constexpr (log_level > 2)
		log(LOG_TAG" Could not insert cell (",query,") after ",max_cuckoo_chain," cuckoos.\n");
	// If no free slot could be found in the given number of tries, the table must be resized.
	// All cells in the table before this function call are reinserted into the new buckets.
	// To this end, the last cell to be displaced is restored into the slot of the new cell.
	// It will be in the wrong bucket with the wrong signature, but for rehashing that is OK.
	// The new cell will be added to the expanded table after all previous entries have been moved.
	if (floating_entry.cell.data != new_cell.data) *new_entry = floating_entry.cell;
	return nullptr;
}

auto hash_grid::signature (index_t index) const noexcept -> signature_t
{
	// Signatures are generated using the variable length xxhash32 algorithm by Collet 2012 as
	// implemented by Jarzynski and Olano 2020.

	/// Prime constants.
	auto const [p2, p3, p4, p5] = std::to_array<uint32_t>({
		2246822519, 3266489917, 668265263, 374761393
	});

	// Compared to the implementation by Jarzynski and Olano, `index` is rotated by one component.
	uint32_t hash = std::bit_cast<uint32_t>(index[0]) + p5;

	for (auto d = 1u; d < (_layout == layout::xyzt ? 4 : 3); ++d) {
		hash += std::bit_cast<uint32_t>(index[d]) * p3;
		hash  = p4 * ((hash << 17) | (hash >> 15));
	}

	hash = p2 * (hash ^ (hash >> 15));
	hash = p3 * (hash ^ (hash >> 13));
	return {hash ^ (hash >> 16)};
}

auto hash_grid::bucket (std::span<bucket_t> buckets, signature_t signature, uint8_t hash_fn)
	noexcept -> bucket_t&
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
	return buckets[hash & buckets.size() - 1];
}

} // namespace otv
