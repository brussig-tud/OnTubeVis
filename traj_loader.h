#pragma once

// C++ STL
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <functional>

// CGV framework core
#include <cgv/base/base.h>
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/media/color.h>
#include <cgv/media/color_scale.h>


/// forward declaration of the attribute mapper class
template <class flt_type>
class visual_attribute_mapping;

/// forward declaration of the trajectory data storage class
template <class flt_type>
class traj_dataset;

/// forward declaration of the trajectory format handler interface
template <class flt_type>
class traj_format_handler;

/// forward declaration of the main trajectory loader class
template <class flt_type>
class traj_manager;


/// enumeration of known visual attributes
enum class VisualAttrib
{
	POSITION, TANGENT, RADIUS, COLOR
};


/// enumeration of all supported attribute types
enum class AttribType
{
	SCALAR, VEC2, VEC3, VEC4
};


/// RAII-type helper that trajectory format handlers can use to ensure reset of the stream position after
/// can_handle() queries
struct stream_pos_guard
{
	/// the stream to guard
	std::istream &stream;

	/// the position the stream will be reset to
	const std::istream::pos_type g;

	/// construct the guard for the given stream
	stream_pos_guard(std::istream& stream) : stream(stream), g(stream.tellg())
	{}

	/// the destructor
	~stream_pos_guard() { stream.seekg(g); }
};


/// class encapsulating an attribute type
template<class flt_type>
class traj_attribute
{
	friend class traj_dataset<flt_type>;
	friend class traj_format_handler<flt_type>;

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef cgv::math::fvec<real, 2> Vec2;

	/// 3D vector type
	typedef cgv::math::fvec<real, 3> Vec3;

	/// 4D vector type
	typedef cgv::math::fvec<real, 4> Vec4;

	/// struct referencing some datapoint of this attribute
	template <class T>
	struct datapoint
	{
		/// construct with the given references
		datapoint(T &val, flt_type &t) : val(val), t(t) {}

		/// reference to the attribute value
		T &val;

		/// datapoint timestamp
		flt_type &t;
	};

	/// struct representing the magnitude of a datapoint (and its timestamp)
	struct datapoint_mag
	{
		/// attribute value magnitude
		flt_type val;

		/// datapoint timestamp
		flt_type t;
	};

	/// base container class for having a common cleanup hook
	struct container_base
	{
		/// virtual base destructor - causes vtable creation
		virtual ~container_base();

		/// query the dimensionality (number of components) of the data points in the container
		virtual unsigned dims (void) const = 0;

		/// query the number of data points in the container
		virtual unsigned num (void) const = 0;

		/// returns the smallest attribute value in the series (in case of vector-valued attributes, the smallest magnitude), optionally
		/// reporting the index of the corresponding datapoint
		virtual real min (unsigned *index=nullptr) const = 0;

		/// returns the largest attribute value in the series (in case of vector-valued attributes, the largest magnitude), optionally
		/// reporting the index of the corresponding datapoint
		virtual real max (unsigned *index=nullptr) const = 0;

		/// obtain raw pointer to the contained data
		virtual void* get_pointer (void) = 0;

		/// obtain raw const pointer to the contained data
		virtual const void* get_pointer (void) const = 0;

		/// obtain access to the data point timestamps
		virtual std::vector<flt_type>& get_timestamps (void) = 0;

		/// obtain const access to the data point timestamps
		virtual const std::vector<flt_type>& get_timestamps (void) const = 0;

		/// return a representation of the datapoint at the given index that contains its magnitude instead of the actual value
		virtual datapoint_mag magnitude_at (unsigned index) const = 0;

		/// return a representation of the datapoint at the given index that contains its signed magnitude instead of the actual value
		virtual datapoint_mag signed_magnitude_at(unsigned index) const = 0;

		/// utility for returning the magnitude of a vector or a scalar
		template <class T>
		static flt_type mag(const T& value) { return value.length(); }
		template <>
		static flt_type mag<flt_type>(const flt_type &value) { return std::abs(value); }

		/// utility for returning the magnitude of a vector or a scalar, preserving the sign of the latter
		template <class T>
		static flt_type smag(const T &value) { return value.length(); }
		template <>
		static flt_type smag<flt_type>(const flt_type &value) { return value; }
	};

	/// generic container type for storing the actual attribute data
	template <class T>
	struct container : public container_base
	{
		/// actual element type
		typedef T elem_type;

		/// alias to prevent STL errors. ToDo: investigate!
		typedef elem_type value_type;

		/// constant indicating the number of data point components
		static constexpr unsigned num_components = sizeof(T) / sizeof(flt_type);

		/// attribute data vector
		std::vector<elem_type> values;

		/// attribute timestamps
		std::vector<flt_type> timestamps;

		/// default constructor
		container() {}

		/// copy constructor
		container(const container &other) : values(other.values), timestamps(other.timestamps)
		{}

		/// move constructor
		container(container &&other) : values(std::move(other.values)), timestamps(std::move(other.timestamps))
		{}

		/// construct with the given actual data
		container(const std::vector<elem_type> &values, const std::vector<flt_type> &timestamps)
			: values(values), timestamps(timestamps)
		{}

		/// construct with the given actual data, moving in the values only
		container(std::vector<elem_type> &&values, const std::vector<flt_type> &timestamps)
			: values(std::move(values)), timestamps(timestamps)
		{}

		/// construct with the given actual data, moving in the timestamps only
		container(const std::vector<elem_type> &values, std::vector<flt_type> &&timestamps)
			: values(values), timestamps(std::move(timestamps))
		{}

		/// construct with the given actual data, moving in both values and timestamps
		container(std::vector<elem_type> &&values, std::vector<flt_type> &&timestamps)
			: values(std::move(values)), timestamps(std::move(timestamps))
		{}

		/// write-access the attribute data
		datapoint<elem_type> operator[] (unsigned index) { return datapoint<elem_type>(values[index], timestamps[index]); }

		/// read-only access the attribute data
		const datapoint<elem_type> operator[] (unsigned index) const {
			return datapoint<elem_type>(const_cast<elem_type&>(values[index]), const_cast<flt_type&>(timestamps[index]));
		}

		/// internally reserve memory for at least the given amount of datapoints to accelerate subsequent calls to \ref append
		void reserve (unsigned num_elements) {
			values.reserve(num_elements); timestamps.reserve(num_elements);
		}

		/// append a datapoint, returning its resulting index
		unsigned append (const elem_type& value, flt_type timestamp)
		{
			values.emplace_back(value); timestamps.emplace_back(timestamp);
			return (unsigned)values.size()-1;
		}

		/// append a datapoint, returning its resulting index
		unsigned append (const datapoint<elem_type> &new_datapoint)
		{
			values.emplace_back(new_datapoint.val); timestamps.emplace_back(new_datapoint.t);
			return (unsigned)values.size()-1;
		}

		/// obtain an iterator over the attribute values pointing at the first datapoint
		typename std::vector<elem_type>::iterator begin (void) { return values.begin(); }

		/// obtain a const-iterator over the attribute values pointing at the first datapoint
		typename std::vector<elem_type>::const_iterator begin (void) const { return values.cbegin(); }

		/// obtain an iterator over the attribute values pointing behind the last datapoint
		typename std::vector<elem_type>::iterator end (void) { return values.end(); }

		/// obtain a const-iterator over the attribute values pointing behind the last datapoint
		typename std::vector<elem_type>::const_iterator end (void) const { return values.cend(); }

		/// obtain an iterator over the attribute timestamps pointing at the first datapoint
		typename std::vector<flt_type>::iterator begin_ts (void) { return timestamps.begin(); }

		/// obtain a const-iterator over the attribute timestamps pointing at the first datapoint
		typename std::vector<flt_type>::const_iterator begin_ts (void) const { return timestamps.cbegin(); }

		/// obtain an iterator over the attribute timestamps pointing behind the last datapoint
		typename std::vector<flt_type>::iterator end_ts (void) { return timestamps.end(); }

		/// obtain a const-iterator over the attribute timestamps pointing behind the last datapoint
		typename std::vector<flt_type>::const_iterator end_ts (void) const { return timestamps.cend(); }

		/// query the dimensionality (number of components) of the data points in the container
		virtual unsigned dims (void) const { return num_components; };

		/// query the number of data points in the container
		virtual unsigned num (void) const { return (unsigned)values.size(); };

		/// returns the smallest attribute value in the series (in case of vector-valued attributes, the smallest magnitude), optionally
		/// reporting the index of the corresponding datapoint
		virtual real min (unsigned *index=nullptr) const
		{
			auto it = std::min_element(
				values.begin(), values.end(),
				[](const elem_type &v1, const elem_type &v2) -> bool {
					return smag(v1) < smag(v2);
				}
			);
			if (index)
				*index = (unsigned)(it-values.begin());
			return smag(*it);
		}

		/// returns the largest attribute value in the series (in case of vector-valued attributes, the largest magnitude), optionally
		/// reporting the index of the corresponding datapoint
		virtual real max (unsigned *index=nullptr) const
		{
			auto it = std::max_element(
				values.begin(), values.end(),
				[](const elem_type &v1, const elem_type &v2) -> bool {
					return smag(v1) < smag(v2); // NOTE: <-- don't reverse comparison here for maximum search...
				}
			);
			if (index)
				*index = (unsigned)(it-values.begin());
			return smag(*it);
		}

		/// obtain raw pointer to the contained data
		virtual void* get_pointer (void) { return values.data(); };

		/// obtain raw const pointer to the contained data
		virtual const void* get_pointer (void) const { return values.data(); };

		/// obtain pointer to the data point timestamps
		virtual std::vector<flt_type>& get_timestamps (void) { return timestamps; };

		/// obtain const pointer to the data point timestamps
		virtual const std::vector<flt_type>& get_timestamps (void) const { return timestamps; };

		/// return a representation of the datapoint at the given index that contains its magnitude instead of the actual value
		virtual datapoint_mag magnitude_at (unsigned index) const {
			return { mag(const_cast<elem_type&>(values[index])), const_cast<flt_type&>(timestamps[index]) };
		}

		/// return a representation of the datapoint at the given index that contains its signed magnitude instead of the actual value
		virtual datapoint_mag signed_magnitude_at(unsigned index) const {
			return { smag(const_cast<elem_type&>(values[index])), const_cast<flt_type&>(timestamps[index]) };
		}
	};


protected:

	/// type of the represented attribute
	AttribType _type;

	/// opaque data
	container_base *_data;

	/// unique id of this attribute
	unsigned _id;


public:

	/// copy constructor
	traj_attribute(const traj_attribute &other);

	/// move constructor
	traj_attribute(traj_attribute &&other);

	/// construct with the given number of components (i.e. scalar, 2d, 3d, 4d)
	traj_attribute(unsigned components);

	/// construct with scalar attribute data, generating uniformly spaced timestamps for each item
	traj_attribute(const std::vector<real> &source, float tstart=0, float dt=1);

	/// construct with scalar attribute data (moved in), generating uniformly spaced timestamps for each item
	traj_attribute(std::vector<real> &&source, float tstart=0, float dt=1);

	/// construct with scalar attribute data (moved in) and timestamps (copied)
	traj_attribute(std::vector<real> &&source, const std::vector<real> &timestamps);

	/// construct with scalar attribute data and timestamps (both moved in)
	traj_attribute(std::vector<real> &&source, std::vector<real> &&timestamps);

	/// construct with 2D vector attribute data, generating uniformly spaced timestamps for each item
	traj_attribute(const std::vector<Vec2> &source, float tstart=0, float dt=1);

	/// construct with 2D vector attribute data (moved in), generating uniformly spaced timestamps for each item
	traj_attribute(std::vector<Vec2> &&source, float tstart=0, float dt=1);

	/// construct with 2D vector attribute data (moved in) and timestamps (copied)
	traj_attribute(std::vector<Vec2> &&source, const std::vector<real> &timestamps);

	/// construct with 2D vector attribute data and timestamps (both moved in)
	traj_attribute(std::vector<Vec2> &&source, std::vector<real> &&timestamps);

	/// construct with 3D vector attribute data, generating uniformly spaced timestamps for each item
	traj_attribute(const std::vector<Vec3> &source, float tstart=0, float dt=1);

	/// construct with 3D vector attribute data (moved in), generating uniformly spaced timestamps for each item
	traj_attribute(std::vector<Vec3> &&source, float tstart=0, float dt=1);

	/// construct with 3D vector attribute data (moved in) and timestamps (copied)
	traj_attribute(std::vector<Vec3> &&source, const std::vector<real> &timestamps);

	/// construct with 3D vector attribute data and timestamps (both moved in)
	traj_attribute(std::vector<Vec3> &&source, std::vector<real> &&timestamps);

	/// construct with 4D vector attribute data, generating uniformly spaced timestamps for each item
	traj_attribute(const std::vector<Vec4> &source, float tstart=0, float dt=1);

	/// construct with 4D vector attribute data (moved in), generating uniformly spaced timestamps for each item
	traj_attribute(std::vector<Vec4> &&source, float tstart=0, float dt=1);

	/// construct with 4D vector attribute data (moved in) and timestamps (copied)
	traj_attribute(std::vector<Vec4> &&source, const std::vector<real> &timestamps);

	/// construct with 4D vector attribute data and timestamps (both moved in)
	traj_attribute(std::vector<Vec4> &&source, std::vector<real> &&timestamps);

	/// the destructor
	~traj_attribute();

	/// copy assignment
	traj_attribute& operator= (const traj_attribute &other);

	/// move assignment
	traj_attribute& operator= (traj_attribute &&other);

	/// report type of the stored attribute
	AttribType type (void) const { return _type; }

	/// return the unique ID of this attribute
	unsigned id (void) const { return _id; }

	/// query the number of datapoints for this attribute
	unsigned num (void) const { return _data->num(); }

	/// access the attribute data as if it was of the specified type
	template <class T>
	container<T>& get_data (void);

	/// access the attribute data as if it was of the specified type (read-only)
	template <class T>
	const container<T>& get_data (void) const;

	/// access the attribute data as if it was of scalar type
	template <>
	container<real>& get_data<real>(void) { return *dynamic_cast<container<real>*>(_data); }

	/// access the attribute data as if it was of scalar type (read-only)
	template <>
	const container<real>& get_data<real>(void) const { return *dynamic_cast<container<real>*>(_data); }

	/// access the attribute data as if it was of type Vec2
	template <>
	container<Vec2>& get_data<Vec2>(void) { return *dynamic_cast<container<Vec2>*>(_data); }

	/// access the attribute data as if it was of type Vec2 (read-only)
	template <>
	const container<Vec2>& get_data<Vec2>(void) const { return *dynamic_cast<container<Vec2>*>(_data); }

	/// access the attribute data as if it was of type Vec3
	template <>
	container<Vec3>& get_data<Vec3>(void) { return *dynamic_cast<container<Vec3>*>(_data); }

	/// access the attribute data as if it was of type Vec3 (read-only)
	template <>
	const container<Vec3>& get_data<Vec3>(void) const { return *dynamic_cast<container<Vec3>*>(_data); }

	/// access the attribute data as if it was of type Vec4
	template <>
	container<Vec4>& get_data<Vec4>(void) { return *dynamic_cast<container<Vec4>*>(_data); }

	/// access the attribute data as if it was of type Vec4 (read-only)
	template <>
	const container<Vec4>& get_data<Vec4>(void) const { return *dynamic_cast<container<Vec4>*>(_data); }

	/// return a representation of the attribute datapoint at the given index that contains its magnitude instead of the actual value
	datapoint_mag magnitude_at (unsigned index) const { return _data->magnitude_at(index); }

	/// return a representation of the attribute datapoint at the given index that contains its signed magnitude instead of the actual value
	datapoint_mag signed_magnitude_at(unsigned index) const { return _data->signed_magnitude_at(index); }

	/// returns the smallest attribute value in the series (in case of vector-valued attributes, the smallest magnitude), optionally
	/// reporting the index of the corresponding datapoint
	real min (unsigned *index=nullptr) const { return _data->min(index); }

	/// returns the largest attribute value in the series (in case of vector-valued attributes, the largest magnitude), optionally
	/// reporting the index of the corresponding datapoint
	real max (unsigned *index=nullptr) const { return _data->max(index); }

	/// obtain raw pointer to the attribute data
	void* get_pointer (void) { return _data->get_pointer(); };

	/// obtain raw const pointer to the attribute data
	const void* get_pointer (void) const { return _data->get_pointer(); };

	/// obtain pointer to the data point timestamps
	virtual std::vector<flt_type>& get_timestamps (void) { return _data->get_timestamps(); };

	/// obtain const pointer to the data point timestamps
	virtual const std::vector<flt_type>& get_timestamps (void) const { return _data->get_timestamps(); };

	/// return a string representing the attribute data type
	const std::string& type_string (void) const;

	/// shallow consistency diagnostics for data point values and timestamps (essentially checks for equal amount)
	bool check_timestamps_shallow (void) const;
};

/// a map of attribute names to their data
template <class flt_type>
using attribute_map = std::unordered_map<std::string, traj_attribute<flt_type> >;


/// represents any sort of transformation that can be applied to attribute data
template <class flt_type>
class attrib_transform
{
	// interfacing
	friend class visual_attribute_mapping<flt_type>;
	friend struct visual_attribute_mapping<flt_type>::attrib_reference;


public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef typename traj_attribute<real>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_attribute<real>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_attribute<real>::Vec4 Vec4;

	/// 2D matrix type
	typedef typename cgv::math::fmat<real, 2, 2> Mat2;

	/// 3D matrix type
	typedef typename cgv::math::fmat<real, 3, 3> Mat3;

	/// 4D matrix type
	typedef typename cgv::math::fmat<real, 4, 4> Mat4;


private:

	/// opaque transform pointer
	void *t_ptr;

	/// target type of the transformation
	AttribType tgt_type;

	/// source type of the transformation
	AttribType src_type;

	/// default constructor - essentially creates an identity transform.
	attrib_transform();


public:

	/// copy constructor
	attrib_transform(const attrib_transform &other);

	/// move constructor
	attrib_transform(attrib_transform &&other);

	typedef std::function<void(Vec4&,const Vec4&)> vec4_to_vec4;
	// construct with the given functor that transforms a \c Vec4 to a \c Vec4 .
	attrib_transform(const vec4_to_vec4 &transform_func);

	typedef std::function<void(Vec3&,const Vec3&)> vec3_to_vec3;
	/// construct with the given functor that transforms a \c Vec3 to a \c Vec3 .
	attrib_transform(const vec3_to_vec3 &transform_func);

	typedef std::function<void(real&,const Vec3&)> vec3_to_real;
	/// construct with the given functor that transforms a \c Vec3 to a \c real scalar .
	attrib_transform(const vec3_to_real &transform_func);

	typedef std::function<void(real&,const real&)> real_to_real;
	/// construct with the given functor that transforms a \c real scalar into another.
	attrib_transform(const real_to_real &transform_func);

	/// the destructor
	~attrib_transform();

	/// copy assignment
	attrib_transform& operator= (const attrib_transform &other);

	/// move assignment
	attrib_transform& operator= (attrib_transform &&other);

	/// check if the transform does nothing
	bool is_identity(void) const { return !t_ptr; }

	/// query source type
	inline AttribType get_src_type (void) const { return src_type; }

	/// query source type
	inline AttribType get_tgt_type(void) const { return tgt_type; }

	/// transform execution hook
	template <class T_tgt, class T_src>
	inline void exec (T_tgt &target, const T_src &source) const
	{
		(*(std::function<void(T_tgt&, const T_src&)>*)t_ptr)(target, source);
	}
};


/// struct describing a color mapping that can be either framework-builtin, reference the named color
/// scale registry or completely user-defined
class colormap
{
public:

	/// Enumeration of possible sources of a color map
	enum class Source { BUILTIN, NAMED, USER };
	typedef Source Src;

	/// Framework-type for color scales
	typedef std::vector<cgv::media::color<float, cgv::media::RGB>> clr_scale_type;


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


public:

	/// default constructor - undefined color map
	colormap();

	/// copy constructor
	colormap(const colormap &other);

	/// move constructor
	colormap(colormap &&other);

	/// use a built-in color map
	colormap(cgv::media::ColorScale built_in);

	/// use a named color map from the registry
	colormap(const std::string &named);

	/// use a new color map based on a sampled transfer function
	colormap(const clr_scale_type &samples);

	/// the destructor
	~colormap();

	/// copy assignment
	colormap &operator= (const colormap &other);

	/// move assignment
	colormap &operator= (colormap &&other);

	/// checks if the color map refers to something or not
	bool is_defined (void) const;

	/// Report the source of this color map
	Source source (void) const;

	/// return a color scale for use with framework functions
	const clr_scale_type& get_color_scale (void) const;
};


/// descriptor for mapping data attributes to visual attributes
template <class flt_type>
class visual_attribute_mapping
{

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef typename traj_attribute<real>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_attribute<real>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_attribute<real>::Vec4 Vec4;

	/// color type
	typedef cgv::media::color<float, cgv::media::RGB> Color;

	/// helper for referring to an attribute when using the mapping constructor; enables making the specification of
	/// transforms optional by means of the two constructors provided by this helper
	struct attrib_reference
	{
		/// name of the data attribute to reference
		std::string name;

		/// the transform to apply to the attribute
		attrib_transform<real> transform;

		/// copy constructor
		attrib_reference(const attrib_reference &other)
			: name(other.name), transform(other.transform)
		{}

		/// move constructor
		attrib_reference(attrib_reference &&other)
			: name(std::move(other.name)), transform(std::move(other.transform))
		{}

		/// construct referring to the attribute of the given name
		attrib_reference(const std::string &name) : name(name)
		{}

		/// construct referring to the attribute of the given name being transformed by the given transformation
		attrib_reference(const std::string &name, const attrib_transform<real> &transform)
			: name(name), transform(transform)
		{}
	};

	/// map type
	typedef std::map<VisualAttrib, attrib_reference> map_type;


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


public:

	/// default constructor
	visual_attribute_mapping();

	/// copy constructor
	visual_attribute_mapping(const visual_attribute_mapping &other);

	/// move constructor
	visual_attribute_mapping(visual_attribute_mapping &&other);

	/// construct with the given intial mapping definition
	visual_attribute_mapping(const std::map<VisualAttrib, attrib_reference> &initial_mapping);

	/// construct with the given intial mapping definition (move semantics)
	visual_attribute_mapping(std::map<VisualAttrib, attrib_reference> &&initial_mapping);

	/// construct with the given intial mapping definition including color map
	visual_attribute_mapping(const std::map<VisualAttrib, attrib_reference> &initial_mapping, const colormap &cm);

	/// construct with the given intial mapping definition (move semantics) including color map
	visual_attribute_mapping(std::map<VisualAttrib, attrib_reference> &&initial_mapping, const colormap &cm);

	/// the destructor
	~visual_attribute_mapping();

	/// copy assignment
	visual_attribute_mapping& operator= (const visual_attribute_mapping &other);

	/// move assignment
	visual_attribute_mapping& operator= (visual_attribute_mapping &&other);

	/// map an attribute
	void map_attribute (VisualAttrib visual_attrib, const std::string &name);

	/// map an attribute subject to the specified pre-transformation
	void map_attribute (VisualAttrib visual_attrib, const std::string &name, const attrib_transform<real> &transform);

	/// unmap a visual attribute
	void unmap_attribute (VisualAttrib visual_attrib);

	/// apply a pre-transformation to an attribute
	void transform_attribute (VisualAttrib visual_attrib, const attrib_transform<real> &transform);

	/// set up color mapping
	void setup_colormap (bool enable, const colormap &color_map);

	/// access the current mapping information
	const map_type& map (void) const;

	/// query the current color map
	const colormap& get_colormap (void) const;

	/// reports whether the given visual attribute is being mapped by this mapping
	bool is_mapped (VisualAttrib visual_attrib) const;

	/// reports whether a color scale is used for mapping the color attribute
	bool uses_colormap (void) const;

	/// explicitly clear everything
	void clear (void);
};


/// struct encapsulating a range of samples (single trajectory, whole dataset) via the first index of the
/// range and the number of sampels spanned by the range
struct range
{
	/// start index
	unsigned i0;

	/// number of samples in the range
	unsigned n;

	/// median of node radii
	float med_radius;
};


/// trajectory data storage
template <class flt_type>
class traj_dataset
{
	// interfacing
	friend class traj_format_handler<flt_type>;
	friend class traj_manager<flt_type>;

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef typename traj_attribute<real>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_attribute<real>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_attribute<real>::Vec4 Vec4;

	/// color type
	typedef typename visual_attribute_mapping<float>::Color Color;

	/// convenience struct that \ref add_attribute uses to return a reference to both the attribute interface and the data container
	template <class T>
	struct attrib_info
	{
		/// the abstract interface of the attribute
		traj_attribute<real> &attrib;

		/// the actual data container of the attribute
		traj_attribute<real>::container<T> &data;

		/// construct for given attribute
		attrib_info(traj_attribute<real>& attribute) : attrib(attribute), data(attribute.get_data<T>()) {}
	};


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


protected:

	/// write-access the source name (filename, description, etc.) the dataset was loaded / originated from
	std::string& data_source (void);

	/// write-access the "special" positions attribute data (for use by trajectory format handlers)
	attrib_info<Vec3> positions (void);

	/// write-access the timestamps at each position (for use by trajectory format handlers)
	flt_type* timestamps (void);

	/// Helper for derived classes to create an attribute of given name and type and obtain a reference for easy immediate
	/// access
	template <class T>
	attrib_info<T> add_attribute (const std::string &name)
	{
		return attributes().emplace(name, std::vector<T>()).first->second;
	}

	/// Helper for derived classes to create an attribute of given name and type and obtain a reference for easy immediate
	/// access
	template <class T, typename ...Args>
	attrib_info<T> add_attribute (const std::string &name, Args&& ...args)
	{
		return attributes().emplace(name, std::forward<Args>(args)...).first->second;
	}

/// write-access the map containing all generic attributes (for use by trajectory format handlers)
	attribute_map<flt_type>& attributes (void);

	/// set the average segment length (for use by trajectory format handlers)
	void set_avg_segment_length (real length);

	/// write-access the list of individual trajectory ranges for the given attribute. Note: if no trajectory
	/// information for the given attribute exists yet, it will be created!
	std::vector<range>& trajectories (const traj_attribute<real> &attribute);


public:

	/// default constructor
	traj_dataset();

	/// construct with the given name and data source
	traj_dataset(const std::string &name, const std::string &data_source);

	/// copy constructor
	traj_dataset(const traj_dataset &other);

	/// move constructor
	traj_dataset(traj_dataset &&other);

	/// the destructor
	~traj_dataset();

	/// copy assignment
	traj_dataset& operator= (const traj_dataset &other);

	/// move assignment
	traj_dataset& operator= (traj_dataset &&other);

	/// explicitely clear everything from storage
	void clear (void);

	/// report if the dataset contains at least indices and a position attribute
	bool has_data (void) const;

	/// write-access the dataset name
	std::string& name (void);

	/// access the source name (filename, description, etc.) the dataset was loaded / originated from
	const std::string& data_source (void) const;

	/// access the "special" positions attribute data.
	const traj_dataset<real>::attrib_info<Vec3> positions (void) const;

	/// access the timestamps at each position
	const flt_type* timestamps (void) const;

	/// return the names of all attributes in the dataset
	std::vector<std::string> get_attribute_names (void) const;

	/// checks if an attribute of the given name is in the dataset
	bool has_attribute (const std::string &name) const;

	/// returns an interface to the attribute of the given name if it exists in the dataset. If it doesn't exist, returns an explicitly
	/// invalid attribute interface that acts "empty" on all relevant queries. To explicitely check if an attribute of some name
	/// exists, use \ref has_attribute .
	const traj_attribute<real>& attribute (const std::string &name) const;

	/// report the average length of line segments
	real avg_segment_length (void) const;

	/// access the list of individual trajectory ranges for the given attribute. In case the dataset has no trajectory information for
	/// this attribute, invalid range will be returned which is generally "safe to use" as it indicates 0 elements in the range.
	const std::vector<range>& trajectories (const traj_attribute<real> &attribute) const;

	/// set the visual attribute mapping, reporting whether the crucial position mapping could be resolved. In case it couldn't, the
	/// method will return false and the current mapping will remain unchanged.
	bool set_mapping (const visual_attribute_mapping<real> &visual_attrib_mapping);

	/// set the visual attribute mapping (move semantics), reporting whether the crucial position mapping could be resolved. In case
	/// it couldn't, the method will return false and the current mapping as well as the to-be-moved in argument will remain unchanged.
	bool set_mapping (visual_attribute_mapping<real> &&visual_attrib_mapping);

	/// access the current visual attribute mapping
	const visual_attribute_mapping<real>& mapping (void) const;
};


/// abstract handler interface that can be implemented to support specific trajectory file formats
template <class flt_type>
class traj_format_handler : public cgv::base::base
{

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef typename traj_dataset<real>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_dataset<real>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_dataset<real>::Vec4 Vec4;

	/// color type
	typedef typename traj_dataset<real>::Color Color;


protected:

	/// Proxy for derived classes to gain write-access the "special" positions attribute
	static traj_dataset<real>::attrib_info<Vec3> positions (traj_dataset<real> &dataset);

	/// Proxy for derived classes to gain write-access to the \link traj_dataset::attributes generic attributes \endlink .
	static attribute_map<flt_type>& attributes (traj_dataset<real> &dataset);

	/// Proxy for derived classes to set the average segment length in a dataset.
	static void set_avg_segment_length (traj_dataset<real> &dataset, real length);

	/// Proxy for derived classes to gain write-access the list of individual trajectory ranges for the given attribute
	static std::vector<range>& trajectories (traj_dataset<real> &dataset, const traj_attribute<real> &attribute);

	/// Helper for derived classes to create an attribute of given name and type and obtain a reference for easy immediate
	/// access
	template <class T>
	static typename traj_dataset<real>::attrib_info<T> add_attribute (traj_dataset<real> &dataset, const std::string &name)
	{
		return dataset.add_attribute<T>(name);
	}

	/// Helper for derived classes to create an attribute of given name and type and obtain a reference for easy immediate
	/// access
	template <class T, typename ... Args>
	static typename traj_dataset<real>::attrib_info<T> add_attribute (
		traj_dataset<real> &dataset, const std::string &name, Args&& ... args
	)
	{
		return dataset.add_attribute<T>(name, std::forward<Args>(args)...);
	}

public:

	/// default constructor
	traj_format_handler() {};

	/// virtual base destructor - causes vtable creation
	virtual ~traj_format_handler() {};

	/// Test if the given data stream can be handled by this handler. At the minimum, the handler must be able to extract sample
	/// positions from the data stream when reporting true.
	virtual bool can_handle (std::istream &contents) const = 0;

	/// parse the given stream containing the file contents to load trajectories stored in it (optionally offsetting sample indices by
	/// the specified amount) and report whether any data was loaded
	virtual traj_dataset<real> read (std::istream &contents) = 0;
};


/// common trajectory data loader class
template <class flt_type>
class traj_manager
{

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef typename traj_format_handler<real>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_format_handler<real>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_format_handler<real>::Vec4 Vec4;

	/// color type
	typedef typename traj_format_handler<real>::Color Color;

	/// encapsulates data for all visual attributes for use by renderers
	struct render_data
	{
		typedef flt_type real;
		typedef typename traj_manager<real>::Vec2 Vec2;
		typedef typename traj_manager<real>::Vec3 Vec3;
		typedef typename traj_manager<real>::Vec4 Vec4;
		typedef typename traj_manager<real>::Color Color;

		struct dataset
		{
			range irange;
			std::vector<range> trajs;
		};

		std::vector<Vec3> positions;
		std::vector<Vec4> tangents;
		std::vector<real> radii;
		std::vector<Color> colors;
		std::vector<unsigned> indices;
		std::vector<dataset> datasets;
	};

	/// convenience helper for iterating over datasets in the manager using range-based for loops
	struct dataset_range
	{
		friend class traj_manager;

	public:
		/// dereference to current dataset
		inline const traj_dataset<real>& operator* (void) const { return mgr.dataset(index); }

		/// start iterating
		inline dataset_range begin (void) { return dataset_range(mgr); }

		/// end iterating
		inline dataset_range end (void) { return dataset_range(mgr, mgr.num_datasets()); }

		/// iterate forward
		inline dataset_range& operator++ (void) { index++; return *this; }

		/// iterate forward
		inline dataset_range operator++ (int) { dataset_range copy(*this); index++; return copy; }

		/// equality comparison
		inline bool operator== (const dataset_range &other) const { return index == other.index; }

		/// inequality comparison
		inline bool operator!= (const dataset_range &other) const { return index != other.index; }

	protected:
		/// construct for the given manager and start index
		inline dataset_range(const traj_manager<real> &mgr, unsigned start_index=0) : mgr(mgr), index(start_index) {}

	private:
		/// trajectory  manager back-reference
		const traj_manager<real> &mgr;

		/// current index counter
		unsigned index;
	};


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


public:

	/// default constructor
	traj_manager();

	/// the destructor
	~traj_manager();

	/// test if the given file or directory can be loaded
	bool can_load (const std::string &path) const;

	/// load trajectories from a file or directory and add them to the internal database, returning the index of the loaded dataset
	/// if successfull, and -1 otherwise.
	unsigned load (const std::string &path);

	/// copy a custom dataset to the internally managed list, returning the resulting access index
	unsigned add_dataset (const traj_dataset<real> &dataset);

	/// move a custom dataset into the internally managed list (i.e. takes ownership of all memory etc), returning the resulting
	/// access index
	unsigned add_dataset (traj_dataset<real> &&dataset);

	/// query the number of currently loaded datasets
	unsigned num_datasets (void) const;

	/// read-only reference the dataset of the given index
	const traj_dataset<real>& dataset (unsigned index) const;

	/// return a range over all datasets of the manager
	dataset_range datasets (void) const;

	/// reset internal trajectory database (e.g. for loading a new, unrelated set of trajectories into this existing instance)
	void clear (void);

	/// check if the manager currently stores valid loaded data
	bool has_data (void) const { return num_datasets() > 0; }

	/// returns the visual attributes of all dataset laid out in a way suitable for rendering
	/// \param auto_tangents
	///		whether to auto-derive tangents from curve geometry when they don't have a mapped data attribute
	/// ToDo: make auto-tangent generation configurable and preset-able (e.g. simple forward diff, catmul-rom, etc.)
	const render_data& get_render_data (bool auto_tangents=true);

	/// returns the visual attributes of all dataset laid out in a way suitable for rendering. since this is for read-only access,
	/// the method will fail if the render data is out-of-date!
	const render_data& get_render_data (void) const;
};


/// converts (1-based) 3-vectors to float RGB colors
template <class vec_type>
inline typename cgv::media::color<float, cgv::media::RGB, cgv::media::NO_ALPHA> vec3_to_rgb (const vec_type &v)
{
	return traj_format_handler<float>::Color((float)v.x(), (float)v.y(), (float)v.z());
}

/// truncates the last component from a 4-vector to create a 3-vector
template <class vec4_type>
inline cgv::math::fvec<typename vec4_type::value_type, 3> vec3_from_vec4(const vec4_type& v)
{
	return cgv::math::fvec<typename vec4_type::value_type, 3>(v.x(), v.y(), v.z());
}

/// dehomogenizes a 4-vector to obtain an equivalent 3-vector
template <class vec4_type>
inline cgv::math::fvec<typename vec4_type::value_type, 3> vec3_from_vec4h(const vec4_type& v)
{
	return cgv::math::fvec<typename vec4_type::value_type, 3>(v.x()/v.w(), v.y()/v.w(), v.z()/v.w());
}
