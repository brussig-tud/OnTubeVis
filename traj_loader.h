#pragma once

// C++ STL
#include <iostream>
#include <string>
#include <vector>
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
	REAL, VEC2, VEC3, VEC4
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

public:

	/// real number type
	typedef flt_type real;

	/// 2D vector type
	typedef cgv::math::fvec<real, 2> Vec2;

	/// 3D vector type
	typedef cgv::math::fvec<real, 3> Vec3;

	/// 4D vector type
	typedef cgv::math::fvec<real, 4> Vec4;

	/// color type
	typedef cgv::media::color<float, cgv::media::RGB> Color;

	/// base container class for having a common cleanup hook
	struct container_base
	{
		/// virtual base destructor - causes vtable creation
		virtual ~container_base();

		/// obtain raw pointer to the contained data
		virtual void* get_pointer (void) = 0;

		/// obtain raw const pointer to the contained data
		virtual const void* get_pointer (void) const = 0;
	};

	/// generic container type for storing the actual attribute data
	template <class T>
	struct container : public container_base
	{
		/// actual element type
		typedef T elem_type;

		/// attribute data vector
		std::vector<elem_type> data;

		/// default constructor
		container() {}

		/// construct with the given actual data
		container(const std::vector<elem_type> &data) : data(data)
		{}

		/// construct with the given actual data (move semantics)
		container(std::vector<elem_type> &&data) : data(std::move(data))
		{}

		/// obtain raw pointer to the contained data
		virtual void* get_pointer (void) { return &data; };

		/// obtain raw const pointer to the contained data
		virtual const void* get_pointer (void) const { return &data; };
	};


protected:

	/// type of the represented attribute
	AttribType _type;

	/// opaque data
	container_base *data;


public:

	/// copy constructor
	traj_attribute(const traj_attribute &other);

	/// move constructor
	traj_attribute(traj_attribute &&other);

	/// construct with the given number of components (i.e. scalar, 2d, 3d, 4d)
	traj_attribute(unsigned components);

	/// construct with scalar attribute data
	traj_attribute(std::vector<real> &&source);

	/// construct with 2D vector attribute data
	traj_attribute(std::vector<Vec2> &&source);

	/// construct with 3D vector attribute data
	traj_attribute(std::vector<Vec3> &&source);

	/// construct with 4D vector attribute data
	traj_attribute(std::vector<Vec4> &&source);

	/// the destructor
	~traj_attribute();

	/// copy assignment
	traj_attribute& operator= (const traj_attribute &other);

	/// move assignment
	traj_attribute& operator= (traj_attribute &&other);

	/// report type of the stored attribute
	AttribType type(void) const { return _type; }

	/// access the attribute data as if it was of the specified type
	template <class T>
	std::vector<T>& get_data (void);

	/// access the attribute data as if it was of the specified type
	template <class T>
	const std::vector<T>& get_data (void) const;

	/// access the attribute data as if it was of scalar type
	template <>
	std::vector<real>& get_data<real>(void) { return dynamic_cast<container<real>*>(data)->data; }

	/// access the attribute data as if it was of scalar type (read-only)
	template <>
	const std::vector<real>& get_data<real>(void) const { return dynamic_cast<container<real>*>(data)->data; }

	/// access the attribute data as if it was of type Vec2
	template <>
	std::vector<Vec2>& get_data<Vec2>(void) { return dynamic_cast<container<Vec2>*>(data)->data; }

	/// access the attribute data as if it was of type Vec2 (read-only)
	template <>
	const std::vector<Vec2>& get_data<Vec2>(void) const { return dynamic_cast<container<Vec2>*>(data)->data; }

	/// access the attribute data as if it was of type Vec3
	template <>
	std::vector<Vec3>& get_data<Vec3>(void) { return dynamic_cast<container<Vec3>*>(data)->data; }

	/// access the attribute data as if it was of type Vec3 (read-only)
	template <>
	const std::vector<Vec3>& get_data<Vec3>(void) const { return dynamic_cast<container<Vec3>*>(data)->data; }

	/// access the attribute data as if it was of type Vec4
	template <>
	std::vector<Vec4>& get_data<Vec4>(void) { return dynamic_cast<container<Vec4>*>(data)->data; }

	/// access the attribute data as if it was of type Vec4 (read-only)
	template <>
	const std::vector<Vec4>& get_data<Vec4>(void) const { return dynamic_cast<container<Vec4>*>(data)->data; }

	/// obtain raw pointer to the attribute data
	void* get_pointer (void) { return data->get_pointer(); };

	/// obtain raw const pointer to the attribute data
	const void* get_pointer (void) const { return data->get_pointer(); };

	/// return a string representing the attribute data type
	const std::string& type_string (void) const;
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
	typedef typename traj_attribute<real>::Color Color;

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
	typedef typename traj_attribute<real>::Color Color;

	/// struct encapsulating the start attribute index and sample count of an individual trajectory
	struct trajectory
	{
		/// start index
		unsigned i0;

		/// number of samples in the trajectory
		unsigned n;
	};


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


protected:

	/// write-access the source name (filename, description, etc.) the dataset was loaded / originated from
	std::string& data_source (void);

	/// write-access the "special" positions attribute (for use by trajectory format handlers)
	std::vector<Vec3>& positions (void);

	/// write-access the map containing all generic attributes (for use by trajectory format handlers)
	attribute_map<flt_type>& attributes (void);

	/// write-access the indices, which store connectivity with line-list semantics (i.e. pairs of indices form a trajectory segment)
	/// (for use by trajectory format handlers)
	std::vector<unsigned>& indices (void);

	/// set the average segment length (for use by trajectory format handlers)
	void set_avg_segment_length (real length);

	/// write-access the list of individual trajectories in the dataset
	std::vector<trajectory>& trajectories (void);


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

	/// access the "special" positions attribute
	const std::vector<Vec3>& positions (void) const;

	/// access the map containing all generic attributes
	const attribute_map<flt_type>& attributes (void) const;

	/// access the indices, which store connectivity with line-list semantics (i.e. pairs of indices form a trajectory segment)
	const std::vector<unsigned>& indices (void) const;

	/// report the average length of line segments
	real avg_segment_length (void) const;

	/// access the list of individual trajectories in the dataset
	const std::vector<trajectory>& trajectories (void) const;

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

	/// Proxy for derived classes to gain write-access to the \ref traj_dataset::positions attribute.
	static std::vector<Vec3>& positions (traj_dataset<real> &dataset);

	/// Proxy for derived classes to gain write-access to the \link traj_dataset::attributes generic attributes \endlink .
	static attribute_map<flt_type>& attributes (traj_dataset<real> &dataset);

	/// Proxy for derived classes to gain write-access to the \ref traj_dataset::indices .
	static std::vector<unsigned>& indices (traj_dataset<real> &dataset);

	/// Proxy for derived classes to set the average segment length in a dataset.
	static void set_avg_segment_length (traj_dataset<real> &dataset, real length);

	/// Proxy for derived classes to gain write-access the list of individual trajectories in the dataset.
	static std::vector<typename traj_dataset<real>::trajectory>& trajectories (traj_dataset<real> &dataset);


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

		std::vector<Vec3> positions;
		std::vector<Vec4> tangents;
		std::vector<real> radii;
		std::vector<Color> colors;
		std::vector<unsigned> indices;
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

	// reference the dataset of the given index
	traj_dataset<real>& dataset (unsigned index);

	/// reset internal trajectory database (e.g. for loading a new, unrelated set of trajectories into this existing instance)
	void clear (void);

	/// check if the manager currently stores valid loaded data
	bool has_data (void) const;

	/// returns the visual attributes of all dataset laid out in a way suitable for rendering
	/// \param auto_tangents
	///		whether to auto-derive tangents from curve geometry when they don't have a mapped data attribute
	/// ToDo: make auto-tangent generation configurable and preset-able (e.g. simple forward diff, catmul-rom, etc.)
	const render_data& get_render_data (bool auto_tangents=true);
};


/// converts (1-based) 3-vectors to float RGB colors
template <class vec_type>
inline typename cgv::media::color<float, cgv::media::RGB, cgv::media::NO_ALPHA> vec3_to_rgb (const vec_type &v)
{
	return traj_format_handler<float>::Color((float)v.x(), (float)v.y(), (float)v.z());
}

/// constructs a 4-vector from a 3-vector and a scalar
template <class vec3_type, class scalar_type>
inline cgv::math::fvec<typename vec3_type::value_type, 4> vec4_from_vec3s (const vec3_type& v, scalar_type s)
{
	return cgv::math::fvec<typename vec3_type::value_type, 4>(v.x(), v.y(), v.z(), vec3_type::value_type(s));
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
