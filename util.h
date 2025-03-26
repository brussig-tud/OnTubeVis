#pragma once

#include <cstdint>
#include <optional>


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


/// A simple RAII-wrapper for some resource. Calls the specified cleanup function on the wrapped resource in its
/// destructor.
template <class ResourceHandle, void(*finalizer)(ResourceHandle)>
struct RAII
{
	/// Handle for the wrapped resource.
	ResourceHandle handle;

	// No default and copy construction/assignment.
	RAII() = delete;
	RAII(const ResourceHandle&) = delete;
	RAII& operator= (const ResourceHandle&) = delete;

	/// The move constructor.
	inline RAII(RAII &&other) = default;

	/// Construct by moving in the given resource.
	inline RAII(ResourceHandle &&handle) : handle(std::move(handle))
	{}

	/// The destructor. Calls the finalizer on the wrapped resource.
	inline ~RAII() {
		drop();
	}

	/// Explicitly call the finalizer on the wrapped resource. Leaves the RAII wrapper in an undefined state.
	inline void drop (void) {
		if (handle) {
			finalizer(handle);
			handle = nullptr;
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
