#pragma once

#include <cassert>
#include <cstdint>
#include <ostream>
#include <numeric>
#include <algorithm>
#include <chrono>
#include <optional>


/// The duration type for the platform high-resolution clock
typedef decltype(
	std::chrono::high_resolution_clock::now() - std::chrono::high_resolution_clock::now()
) hires_duration_type;


/// A right-open range [begin, end).
template <class Iter>
struct ro_range
{
	Iter begin {};
	Iter end   {};

	[[nodiscard]] constexpr bool is_empty () const noexcept {
		return begin == end;
	}

	[[nodiscard]] constexpr std::size_t length () const noexcept {
		return end - begin;
	}

	[[nodiscard]] constexpr bool contains (const Iter &elem) const noexcept {
		return elem >= begin && elem < end;
	}

	/*ro_range(const ro_range &other) = default;
	inline ro_range& operator = (const ro_range &other) = default;*/

	inline ro_range& operator += (const std::size_t offset) noexcept {
		begin += offset;
		end += offset;
		return *this;
	}
	inline void safe_advance (const std::size_t offset, const Iter &max) noexcept {
		begin += offset;
		const std::size_t safe_offset = max - end;
		end += std::min(offset, safe_offset);
	}
	inline void safe_unadvance(const std::size_t offset) noexcept {
		const std::size_t safe_offset = begin != end ? offset : 0;
		begin -= offset;
		end -= safe_offset;
	}

	[[nodiscard]] inline ro_range operator + (const std::size_t offset) noexcept {
		return ro_range{begin+offset, end+offset};
	}

	inline ro_range& operator -= (const std::size_t offset) noexcept {
		begin -= offset;
		end -= offset;
		return *this;
	}
	inline void safe_retreat (const std::size_t offset, const Iter &min) noexcept {
		const std::size_t safe_offset = begin - min;
		begin -= std::min(offset, safe_offset);
		end -= offset;
	}
	inline void safe_unretreat (const std::size_t offset) noexcept {
		const std::size_t safe_offset = begin != end ? offset : 0;
		begin += safe_offset;
		end += offset;
	}

	[[nodiscard]] inline ro_range operator - (const std::size_t offset) noexcept {
		return ro_range{begin-offset, end-offset};
	}
};

template <class Iter>
ro_range(Iter, Iter) -> ro_range<Iter>;

////
// Various typedefs for iterator-like objects that can be used to greatly shorten explicit template instantiations for
// all functions that have to work with ro_ranges
#if defined(_STL_VECTOR_H) || (defined(_MSC_VER) && defined(_VECTOR_))
	typedef std::vector<float>::iterator std_vector_float_iter;
#endif
#if defined(_STL_DEQUE_H) || (defined(_MSC_VER) && defined(_DEQUE_))
	typedef std::deque<float>::iterator std_deque_float_iter;
#endif


/// A map() operator for optionals, enabling functional programming-style handling of std::optional.
template <typename O, typename F>
auto map_optional (O &&o, F &&f) -> std::optional<decltype(f(*std::forward<O>(o)))> {
	if (!o.has_value())
		return std::nullopt;
	return {f(*std::forward<O>(o))};
}


template <typename T, std::size_t ... Is>
constexpr std::array<T, sizeof...(Is)> make_array_helper(T value, std::index_sequence<Is...>) {
	return {{(static_cast<void>(Is), value)...}};
}

/// Create an @c std::array of the given type @c T containing @a N elements initialized to @a value.
template <std::size_t N, class T>
constexpr std::array<T, N> make_array(const T &value) {
	return make_array_helper(value, std::make_index_sequence<N>());
}


/// A simple helper struct for storing both the index of and a reference to some object typically residing in some
/// index-able container.
template <class T>
struct ref_with_id {
	/// The index of the referenced object.
	unsigned id;

	/// Reference to the object itself.
	T &ref;
};


/// A simple RAII-style finalizer, making sure the finalizer code is called when the helper gets destroyed. In case
/// the finalizer contains code that should not be called under some circumstances (e.g. when the parent function as
/// a whole succeeds and allocated resources should thus not be destroyed), it can be programmatically disarmed.
template <class Finalizer>
struct finalizer
{
	bool armed = true;
	Finalizer f;

	/// Construct for the provided finalizer.
	[[nodiscard]] explicit finalizer(Finalizer &&f) : f(std::move(f))
	{}

	~finalizer() {
		if (armed)
			f();
	}

	/// Disarm iff the provided @a disarmed flag is @c true . Disarming means that when the finalizer is triggered, it
	/// will not actually be executed. Forwards the passed in boolean as return.
	bool disarm (bool disarmed=true) {
		armed = !disarmed;
		return disarmed;
	}
};


/// Create a default-initialized instance of a type (does not need to be a class).
template <class T>
T default_construct()
{
	return {};
}

/// A simple RAII-wrapper for some resource. Calls the specified cleanup function on the wrapped resource in its
/// destructor.
template <
	class ResourceHandle,
	void(*finalizer)(ResourceHandle),
	ResourceHandle(*none)() = default_construct<ResourceHandle>
> struct RAII
{
	/// Handle for the wrapped resource.
	ResourceHandle handle = none();

	/// Default-construct with a `none` handle.
	constexpr RAII() = default;
	/// Construct by taking ownership of the given resource.
	constexpr RAII(ResourceHandle handle) : handle(std::move(handle))
	{}

	// No copy construction/assignment.
	constexpr RAII(const RAII&) = delete;
	constexpr RAII& operator= (const RAII&) = delete;

	/// The move constructor.
	constexpr RAII(RAII &&other)
		: handle {std::move(other.handle)}
	{
		// Clear the previous owner to avoid double-free.
		other.handle = none();
	}

	/// Move assignment.
	RAII& operator= (RAII &&other)
	{
		// Make self-assignment a NOP.
		if (&other == this) return *this;
		// Release any currently held resource.
		drop();
		// Take the handle from its previous owner.
		handle = std::move(other.handle);
		other.handle = none();
		return *this;
	}

	/// The destructor. Calls the finalizer on the wrapped resource.
	inline ~RAII() {
		drop();
	}

	/// Explicitly call the finalizer on the wrapped resource. Leaves the RAII wrapper in an undefined state.
	inline void drop (void) {
		if (handle != none()) {
			finalizer(handle);
			handle = none();
		}
	}
};


#if defined(OTV_API_INCLUDED) && defined(CGV_MATH_FVEC_DECLARED) && defined(CGV_MATH_FMAT_DECLARED)
/// Return a version of the given hermite @a node that has the given transformation applied.
inline OTV_HermiteNode transform_hermite_node (
	const OTV_HermiteNode &node, const cgv::mat4 &trans, const cgv::mat4 &tangents_trans
){
	const auto pos = trans * cgv::vec4(cgv::vec3(3, (float*)&node.position), 1);
	const auto tan = tangents_trans * cgv::vec4(cgv::vec3(3, (float*)&node.tangent), 0);
	return {
		node.time, otv__Vec3(pos.x()/pos.w(), pos.y()/pos.w(), pos.z()/pos.w()), *(OTV_Vec3*)&tan
	};
}
#endif


/// Calculate the median of the given range of elements as well as its index, and return both as a pair.
template <class It>
auto median_of_range (It begin, It end)
{
	// The type of the quantity pointed to by the iterators
	typedef std::remove_reference_t<decltype(*begin)> Quantity;

	const size_t size = std::distance(begin, end);
	if (size == 0)
		return std::make_pair(Quantity(0), 0); // Handle empty range

	const auto size_half = size/2;
	if (size%2 == 0) {
		const auto mid_right = begin + size_half;
		const auto mid_left = mid_right - 1;
		return std::make_pair((*mid_left + *mid_right)/2, (int)size_half-1);
	}
	return  std::make_pair(*(begin + size_half), (int)size_half);
}

/// A simple statistics collector.
template <class Quantity>
struct stats_collector
{
	typedef Quantity quantity_type;
	std::vector<Quantity> measurements;
	double avg;
	Quantity median, min, max, lower_quartile, upper_quartile;


	static inline double to_number (const Quantity &quantity) {
		return (double)quantity;
	}
	static inline auto number_format (void) {
		return std::dec;
	}
	static inline const std::string& quantity_unit (void) {
		static const std::string unit = "";
		return unit;
	}


	stats_collector(unsigned expected_num_measurements=86400 /* <- 144Hz sample rate for 10 minutes */) {
		measurements.reserve(expected_num_measurements);
	}


	void add_measurement (const Quantity &measurement) {
		measurements.emplace_back(measurement);
	}
	void add_measurement (Quantity &&measurement) {
		measurements.emplace_back(std::move(measurement));
	}

	void process (void)
	{
		// Don't crash if no data was collected.
		if (measurements.empty())
			return;

		// Sort measurments for robust statistics
		std::sort(measurements.begin(), measurements.end());

		// Exclude zeros
		ro_range calc_range{measurements.begin(), measurements.end()};
		if (*calc_range.begin == Quantity(0) && calc_range.length() > 5)
			++calc_range.begin;

		// Calculate average.
		avg = to_number(
			std::accumulate(calc_range.begin, calc_range.end, Quantity(0)) / calc_range.length()
		);

		// Get min/max
		min = *std::min_element(calc_range.begin, calc_range.end);
		max = *std::max_element(calc_range.begin, calc_range.end);

		// Get the median.
		const auto &[median, middle_idx] = median_of_range(
			calc_range.begin, calc_range.end
		);
		this->median = median;

		// Get the quartiles
		// - Q1: median of first half
		calc_range.end = calc_range.begin + middle_idx+1;
		lower_quartile = median_of_range(calc_range.begin, calc_range.end).first;
		// - Q3: median of second half
		calc_range = ro_range{calc_range.end-1, measurements.end()};
		upper_quartile = median_of_range(calc_range.begin, calc_range.end).first;
	}

	void print (std::ostream &os, const std::string &label) const
	{
		const auto flags_bak = os.flags();
		os << '['<<label<<"]:\n"
		   << "\tavg:    "<<avg<<number_format()<<' '<<quantity_unit() << "\n"
		   << "\tmin:    "<<to_number(min)<<' '<<quantity_unit() << "\n"
		   << "\tmax:    "<<to_number(max)<<' '<<quantity_unit() << "\n"
		   << "\tmedian: "<<to_number(median)<<' '<<quantity_unit() << "\n"
		   << "\tQ_25:   "<<to_number(lower_quartile)<<' '<<quantity_unit() << "\n"
		   << "\tQ_75:   "<<to_number(upper_quartile)<<' '<<quantity_unit() << std::endl;
		os.flags(flags_bak);
	}
};
template<>
inline double stats_collector<std::chrono::duration<double, std::nano>>::to_number(const quantity_type &quantity) {
	return quantity.count();
}
template<>
inline const std::string& stats_collector<std::chrono::duration<double, std::nano>>::quantity_unit(void) {
	static const std::string unit = "ns";
	return unit;
}
template<>
inline double stats_collector<std::chrono::duration<double, std::micro>>::to_number(const quantity_type &quantity) {
	return quantity.count();
}
template<>
inline const std::string& stats_collector<std::chrono::duration<double, std::micro>>::quantity_unit(void) {
	#ifdef _MSC_VER
		static const std::string unit = "us";
	#else
		static const std::string unit = "µs";
	#endif
	return unit;
}
template<>
inline double stats_collector<std::chrono::duration<double, std::milli>>::to_number(const quantity_type &quantity) {
	return quantity.count();
}
template<>
inline const std::string& stats_collector<std::chrono::duration<double, std::milli>>::quantity_unit(void) {
	static const std::string unit = "ms";
	return unit;
}
template<>
inline double stats_collector<std::chrono::duration<double>>::to_number(const quantity_type &quantity) {
	return quantity.count();
}
template<>
inline const std::string& stats_collector<std::chrono::duration<double>>::quantity_unit(void) {
	static const std::string unit = "s";
	return unit;
}
