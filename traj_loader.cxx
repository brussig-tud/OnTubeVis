
// C++ STL
#include <iostream>
#include <fstream>
#include <algorithm>
#include <atomic>
#include <utility>

// CGV framework core
#include <cgv/base/base.h>
#include <cgv/base/register.h>
#include <cgv/utils/file.h>
#include <cgv/utils/dir.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>

// implemented header
#include "traj_loader.h"


////
// Local types and variables

// anonymous namespace begin
namespace {

// attribute datatype names
namespace type_str {
	static const std::string SCALAR       = "SCALAR";
	static const std::string VEC2       = "VEC2";
	static const std::string VEC3       = "VEC3";
	static const std::string VEC4       = "VEC4";
	static const std::string ERROR_TYPE = "ERROR_TYPE";
};

// unique attribute id generation
unsigned get_unique_id(void)
{
	static std::atomic<unsigned> id(0);
	return id++;
}


// trajectory handler registry
template <class flt_type>
struct trajectory_handler_registry : public cgv::base::base, public cgv::base::registration_listener
{
	static std::vector<cgv::base::base_ptr>& handlers (void)
	{
		static std::vector<cgv::base::base_ptr> _handlers;
		return _handlers;
	}
	std::string get_type_name() const { return "trajectory_handler_registry"; }

	void register_object (cgv::base::base_ptr object, const std::string &)
	{
		if (object->get_interface<traj_format_handler<flt_type> >())
			handlers().push_back(object);
	}
	void unregister_object(cgv::base::base_ptr object, const std::string &)
	{
		for (unsigned i=0; i<handlers().size(); i++)
			if (object == handlers()[i])
				handlers().erase(handlers().begin()+i);
	}
};

// maximum ID of all visual attributes
constexpr const unsigned va_max = (const unsigned)VisualAttrib::COLOR;

// Anonymous namespace end
}


////
// Class implementation - traj_attribute

// helper class used in invalid attributes
template <class flt_type>
struct invalid_container : traj_attribute<flt_type>::container_base
{
	virtual unsigned dims (void) const { return 0; }
	virtual unsigned num (void) const { return 0; }
	virtual flt_type min(unsigned *index) const { return 0; }
	virtual flt_type max(unsigned *index) const { return 0; }
	virtual void* get_pointer (void) { return nullptr; }
	virtual const void* get_pointer (void) const { return nullptr; }
	virtual flt_type* get_timestamps (void) { return nullptr; }
	virtual const flt_type* get_timestamps (void) const { return nullptr; }
	virtual typename traj_attribute<flt_type>::datapoint_mag magnitude_at (unsigned index) const {
		return { -1.f, 0.f };
	}
	virtual typename traj_attribute<flt_type>::datapoint_mag signed_magnitude_at(unsigned index) const {
		return { -1.f, 0.f };
	}
};

template <class flt_type>
traj_attribute<flt_type>::container_base::~container_base()
{}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const traj_attribute &other)
	: _data(nullptr), _type(other._type), id(get_unique_id())
{
	switch (other._type)
	{
		case AttribType::SCALAR:
			_data = new container<flt_type>(*(container<flt_type>*)other._data);
			return;
		case AttribType::VEC2:
			_data = new container<Vec2>(*(container<Vec2>*)other._data);
			return;
		case AttribType::VEC3:
			_data = new container<Vec3>(*(container<Vec3>*)other._data);
			return;
		case AttribType::VEC4:
			_data = new container<Vec4>(*(container<Vec4>*)other._data);
			return;

		default:
			_data = new invalid_container<real>;
	}
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (traj_attribute &&other)
	: _type(other._type), _data(other._data), id(other.id)
{
	other._data = nullptr;
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (unsigned components) : _data(nullptr), id(get_unique_id())
{
	switch (components)
	{
		case 1:
			_type = AttribType::SCALAR;
			_data = new container<real>();
			return;
		case 2:
			_type = AttribType::VEC2;
			_data = new container<Vec2>();
			return;
		case 3:
			_type = AttribType::VEC3;
			_data = new container<Vec3>();
			return;
		case 4:
			_type = AttribType::VEC4;
			_data = new container<Vec4>();
			return;

		default:
			_type = (AttribType)-1;
			_data = new invalid_container<real>;
	}
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const std::vector<real> &source, float tstart, float dt)
	: _type(AttribType::SCALAR), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<real>(source, std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<real> &&source, float tstart, float dt)
	: _type(AttribType::SCALAR), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<real>(std::move(source), std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<real> &&source, const std::vector<real> &timestamps)
	: _type(AttribType::SCALAR), _data(nullptr), id(get_unique_id())
{
	_data = new container<real>(std::move(source), timestamps);
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<real> &&source, std::vector<real> &&timestamps)
	: _type(AttribType::SCALAR), _data(nullptr), id(get_unique_id())
{
	_data = new container<real>(std::move(source), std::move(timestamps));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const std::vector<Vec2> &source, float tstart, float dt)
	: _type(AttribType::VEC2), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec2>(source, std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec2> &&source, float tstart, float dt)
	: _type(AttribType::VEC2), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec2>(std::move(source), std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec2> &&source, const std::vector<real> &timestamps)
	: _type(AttribType::VEC2), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec2>(std::move(source), timestamps);
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec2> &&source, std::vector<real> &&timestamps)
	: _type(AttribType::VEC2), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec2>(std::move(source), std::move(timestamps));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const std::vector<Vec3> &source, float tstart, float dt)
	: _type(AttribType::VEC3), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec3>(source, std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec3> &&source, float tstart, float dt)
	: _type(AttribType::VEC3), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec3>(std::move(source), std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec3> &&source, const std::vector<real> &timestamps)
	: _type(AttribType::VEC3), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec3>(std::move(source), timestamps);
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec3> &&source, std::vector<real> &&timestamps)
	: _type(AttribType::VEC3), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec3>(std::move(source), std::move(timestamps));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const std::vector<Vec4> &source, float tstart, float dt)
	: _type(AttribType::VEC4), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec4>(source, std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec4> &&source, float tstart, float dt)
	: _type(AttribType::VEC4), _data(nullptr), id(get_unique_id())
{
	// generate timestamps
	std::vector<real> ts; ts.reserve(source.size());
	for (unsigned i=0; i<(unsigned)source.size(); i++)
		ts.emplace_back(tstart + dt*i);
	// construct data container
	_data = new container<Vec4>(std::move(source), std::move(ts));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec4> &&source, const std::vector<real> &timestamps)
	: _type(AttribType::VEC4), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec4>(std::move(source), timestamps);
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec4> &&source, std::vector<real> &&timestamps)
	: _type(AttribType::VEC4), _data(nullptr), id(get_unique_id())
{
	_data = new container<Vec4>(std::move(source), std::move(timestamps));
}

template <class flt_type>
traj_attribute<flt_type>::~traj_attribute()
{
	if (_data)
	{
		delete _data;
		_data = nullptr;
	}
}

template <class flt_type>
traj_attribute<flt_type>& traj_attribute<flt_type>::operator= (const traj_attribute &other)
{
	this->~traj_attribute();
	id = get_unique_id();
	_type = other._type;
	_data = other._data;
	return *this;
}

template <class flt_type>
traj_attribute<flt_type>& traj_attribute<flt_type>::operator= (traj_attribute &&other)
{
	this->~traj_attribute();
	id = other.id;
	_type = other._type;
	std::swap(_data, other._data);
	return *this;
}

template <class flt_type>
const std::string& traj_attribute<flt_type>::type_string (void) const
{
	switch (_type)
	{
		case AttribType::SCALAR:
			return type_str::SCALAR;
		case AttribType::VEC2:
			return type_str::VEC2;
		case AttribType::VEC3:
			return type_str::VEC3;
		case AttribType::VEC4:
			return type_str::VEC4;

		default:
			return type_str::ERROR_TYPE;
	}
}


////
// Class implementation - colormap_suggestion

struct colormap::Impl
{
	// fields
	Source source;
	cgv::media::ColorScale builtin;
	std::string named;
	clr_scale_type samples;
};

colormap::colormap() : pimpl(nullptr) {}

colormap::colormap(const colormap &other) : pimpl(nullptr)
{
	if (other.pimpl)
	{
		pimpl = new Impl;
		auto &impl = *pimpl; const auto &other_impl = *other.pimpl;
		impl.builtin = other_impl.builtin;
		impl.named = other_impl.named;
		impl.source = other_impl.source;
		impl.samples = other_impl.samples;
	}
}

colormap::colormap(colormap &&other) : pimpl(other.pimpl)
{
	other.pimpl = nullptr;
}

colormap::colormap(cgv::media::ColorScale built_in) : pimpl(nullptr)
{
	pimpl = new Impl;
	auto &impl = *pimpl;
	impl.builtin = built_in;
	impl.named = cgv::media::get_color_scale_name(built_in);
	impl.source = Src::BUILTIN;
}

colormap::colormap(const std::string &named) : pimpl(nullptr)
{
	pimpl = new Impl;
	auto &impl = *pimpl;
	impl.builtin = cgv::media::CS_NAMED;
	impl.named = named;
	impl.source = Src::NAMED;
}

colormap::colormap(const clr_scale_type &samples) : pimpl(nullptr)
{
	pimpl = new Impl;
	auto &impl = *pimpl;
	impl.builtin = cgv::media::CS_NAMED;
	impl.samples = samples;
	// Note: impl.named remains empty to signify new user-defined scale
	impl.source = Src::USER;
}

colormap::~colormap()
{
	if (pimpl)
	{
		delete pimpl;
		pimpl = nullptr;
	}
}

colormap& colormap::operator= (const colormap &other)
{
	if (pimpl == other.pimpl)
		return *this;
	if (!pimpl)
		pimpl = new Impl;

	auto &impl = *pimpl; const auto &other_impl = *other.pimpl;
	impl.builtin = other_impl.builtin;
	impl.named = other_impl.named;
	impl.source = other_impl.source;
	impl.samples = other_impl.samples;
	return *this;
}

colormap& colormap::operator= (colormap &&other)
{
	this->~colormap();
	std::swap(pimpl, other.pimpl);
	return *this;
}

bool colormap::is_defined (void) const
{
	if (!pimpl)
		return false;
	auto &impl = *pimpl;
	if (impl.source == Src::BUILTIN && unsigned(impl.builtin) < unsigned(cgv::media::CS_NAMED))
		return true;
	if (impl.source == Src::NAMED && impl.builtin == cgv::media::CS_NAMED)
	{
		const auto &registered = cgv::media::query_color_scale_names();
		const auto &it = std::find(registered.begin(), registered.end(), impl.named);
		return it != registered.end();
	}
	if (impl.source == Src::USER && impl.builtin == cgv::media::CS_NAMED && !impl.samples.empty())
		return true;
	return false;
}

colormap::Source colormap::source (void) const
{
	return pimpl->source;
}

const colormap::clr_scale_type& colormap::get_color_scale (void) const
{
	auto &impl = *pimpl;
	if (impl.source == Src::BUILTIN || impl.source == Src::NAMED)
		return cgv::media::query_named_color_scale(impl.named);
	else
		return impl.samples;
}


////
// Class implementation - attrib_transform

template <class flt_type>
attrib_transform<flt_type>::attrib_transform()
	: t_ptr(nullptr), tgt_type((AttribType)-1), src_type((AttribType)-1)
{}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const attrib_transform &other)
	: t_ptr(nullptr), tgt_type((AttribType)-1), src_type((AttribType)-1)
{
	*this = other;
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(attrib_transform &&other)
	: t_ptr(other.t_ptr), tgt_type(other.tgt_type), src_type(other.src_type)
{
	other.t_ptr = nullptr;
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const vec4_to_vec4 &transform_func)
	: tgt_type(AttribType::VEC4), src_type(AttribType::VEC4)
{
	t_ptr = new vec4_to_vec4(transform_func);
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const vec3_to_vec3 &transform_func)
	: tgt_type(AttribType::VEC3), src_type(AttribType::VEC3)
{
	t_ptr = new vec3_to_vec3(transform_func);
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const vec3_to_real &transform_func)
	: tgt_type(AttribType::SCALAR), src_type(AttribType::VEC3)
{
	t_ptr = new vec3_to_real(transform_func);
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const real_to_real &transform_func)
	: tgt_type(AttribType::SCALAR), src_type(AttribType::SCALAR)
{
	t_ptr = new real_to_real(transform_func);
}

template <class flt_type>
attrib_transform<flt_type>::~attrib_transform()
{
	// check if we even need to clean up anything
	if (!t_ptr)
		return;

	// perform appropriate functor deletion
	switch (src_type)
	{
		case AttribType::VEC4: switch (tgt_type) {
			case AttribType::VEC4:
				delete (std::function<void(Vec4&, const Vec4&)>*)t_ptr;
				goto _done;
			case AttribType::VEC3:
				delete (std::function<void(Vec3&, const Vec4&)>*)t_ptr;
				goto _done;
			case AttribType::VEC2:
				delete (std::function<void(Vec2&, const Vec4&)>*)t_ptr;
				goto _done;
			case AttribType::SCALAR:
				delete (std::function<void(real&, const Vec4&)>*)t_ptr;
				goto _done;
		}
		case AttribType::VEC3: switch (tgt_type) {
			case AttribType::VEC4:
				delete (std::function<void(Vec4&, const Vec3&)>*)t_ptr;
				goto _done;
			case AttribType::VEC3:
				delete (std::function<void(Vec3&, const Vec3&)>*)t_ptr;
				goto _done;
			case AttribType::VEC2:
				delete (std::function<void(Vec2&, const Vec3&)>*)t_ptr;
				goto _done;
			case AttribType::SCALAR:
				delete (std::function<void(real&, const Vec3&)>*)t_ptr;
				goto _done;
		}
		case AttribType::VEC2: switch (tgt_type) {
			case AttribType::VEC4:
				delete (std::function<void(Vec4&, const Vec2&)>*)t_ptr;
				goto _done;
			case AttribType::VEC3:
				delete (std::function<void(Vec3&, const Vec2&)>*)t_ptr;
				goto _done;
			case AttribType::VEC2:
				delete (std::function<void(Vec2&, const Vec2&)>*)t_ptr;
				goto _done;
			case AttribType::SCALAR:
				delete (std::function<void(real&, const Vec2&)>*)t_ptr;
				goto _done;
		}
		case AttribType::SCALAR: switch (tgt_type) {
			case AttribType::VEC4:
				delete (std::function<void(Vec4&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::VEC3:
				delete (std::function<void(Vec3&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::VEC2:
				delete (std::function<void(Vec2&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::SCALAR:
				delete (std::function<void(real&, const real&)>*)t_ptr;
		}
	}
_done:
	src_type = tgt_type = (AttribType)-1;
}

template <class flt_type>
attrib_transform<flt_type>& attrib_transform<flt_type>::operator= (const attrib_transform &other)
{
	// check if no-op / identity is indicated
	if (!other.t_ptr)
	{
		this->~attrib_transform();
		return *this;
	}

	// copy type info
	tgt_type = other.tgt_type;
	src_type = other.src_type;

	// only copy-construct new functor when not self assigning
	if (!t_ptr && this != &other) switch (src_type)
	{
		case AttribType::VEC4: switch (tgt_type) {
			case AttribType::VEC4:
				t_ptr = new std::function<void(Vec4&, const Vec4&)>(*(std::function<void(Vec4&, const Vec4&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC3:
				t_ptr = new std::function<void(Vec3&, const Vec4&)>(*(std::function<void(Vec3&, const Vec4&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC2:
				t_ptr = new std::function<void(Vec2&, const Vec4&)>(*(std::function<void(Vec2&, const Vec4&)>*)other.t_ptr);
				return *this;
			case AttribType::SCALAR:
				t_ptr = new std::function<void(real&, const Vec4&)>(*(std::function<void(real&, const Vec4&)>*)other.t_ptr);
				return *this;
		}
		case AttribType::VEC3: switch (tgt_type) {
			case AttribType::VEC4:
				t_ptr = new std::function<void(Vec4&, const Vec3&)>(*(std::function<void(Vec4&, const Vec3&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC3:
				t_ptr = new std::function<void(Vec3&, const Vec3&)>(*(std::function<void(Vec3&, const Vec3&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC2:
				t_ptr = new std::function<void(Vec2&, const Vec3&)>(*(std::function<void(Vec2&, const Vec3&)>*)other.t_ptr);
				return *this;
			case AttribType::SCALAR:
				t_ptr = new std::function<void(real&, const Vec3&)>(*(std::function<void(real&, const Vec3&)>*)other.t_ptr);
				return *this;
		}
		case AttribType::VEC2: switch (tgt_type) {
			case AttribType::VEC4:
				t_ptr = new std::function<void(Vec4&, const Vec2&)>(*(std::function<void(Vec4&, const Vec2&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC3:
				t_ptr = new std::function<void(Vec3&, const Vec2&)>(*(std::function<void(Vec3&, const Vec2&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC2:
				t_ptr = new std::function<void(Vec2&, const Vec2&)>(*(std::function<void(Vec2&, const Vec2&)>*)other.t_ptr);
				return *this;
			case AttribType::SCALAR:
				t_ptr = new std::function<void(real&, const Vec2&)>(*(std::function<void(real&, const Vec2&)>*)other.t_ptr);
				return *this;
		}
		case AttribType::SCALAR: switch (tgt_type) {
			case AttribType::VEC4:
				t_ptr = new std::function<void(Vec4&, const real&)>(*(std::function<void(Vec4&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC3:
				t_ptr = new std::function<void(Vec3&, const real&)>(*(std::function<void(Vec3&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC2:
				t_ptr = new std::function<void(Vec2&, const real&)>(*(std::function<void(Vec2&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::SCALAR:
				t_ptr = new std::function<void(real&, const real&)>(*(std::function<void(real&, const real&)>*)other.t_ptr);
		}
	}
	return *this;
}

template <class flt_type>
attrib_transform<flt_type>& attrib_transform<flt_type>::operator= (attrib_transform &&other)
{
	// destroy self if not self-assigning
	if (this == &other)
		return *this;
	this->~attrib_transform();

	// steal other's fields
	std::swap(t_ptr, other.t_ptr);
	std::swap(tgt_type, other.tgt_type);
	std::swap(src_type, other.src_type);
	return *this;
}

/*template <class flt_type>
void attrib_transform<flt_type>::exec (void *target, const void *source) const
{
	// check if no-op / identity is indicated
	if (!t_ptr)
		return;

	switch (src_type)
	{
		case AttribType::VEC4: switch (tgt_type)
		{
			case AttribType::VEC4:
			{
				auto &tgt = *(std::vector<Vec4>*)target;
				auto &src = *(std::vector<Vec4>*)source;
				auto &t = *(std::function<void(Vec4&, const Vec4&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC3:
			{
				auto &tgt = *(std::vector<Vec3>*)target;
				auto &src = *(std::vector<Vec4>*)source;
				auto &t = *(std::function<void(Vec3&, const Vec4&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC2:
			{
				auto &tgt = *(std::vector<Vec2>*)target;
				auto &src = *(std::vector<Vec4>*)source;
				auto &t = *(std::function<void(Vec2&, const Vec4&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::SCALAR:
			{
				auto &tgt = *(std::vector<real>*)target;
				auto &src = *(std::vector<Vec4>*)source;
				auto &t = *(std::function<void(real&, const Vec4&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
		}
		case AttribType::VEC3: switch (tgt_type)
		{
			case AttribType::VEC4:
			{
				auto &tgt = *(std::vector<Vec4>*)target;
				auto &src = *(std::vector<Vec3>*)source;
				auto &t = *(std::function<void(Vec4&, const Vec3&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC3:
			{
				auto &tgt = *(std::vector<Vec3>*)target;
				auto &src = *(std::vector<Vec3>*)source;
				auto &t = *(std::function<void(Vec3&, const Vec3&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC2:
			{
				auto &tgt = *(std::vector<Vec2>*)target;
				auto &src = *(std::vector<Vec3>*)source;
				auto &t = *(std::function<void(Vec2&, const Vec3&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::SCALAR:
			{
				auto &tgt = *(std::vector<real>*)target;
				auto &src = *(std::vector<Vec3>*)source;
				auto &t = *(std::function<void(real&, const Vec3&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
		}
		case AttribType::VEC2: switch (tgt_type)
		{
			case AttribType::VEC4:
			{
				auto &tgt = *(std::vector<Vec4>*)target;
				auto &src = *(std::vector<Vec2>*)source;
				auto &t = *(std::function<void(Vec4&, const Vec2&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC3:
			{
				auto &tgt = *(std::vector<Vec3>*)target;
				auto &src = *(std::vector<Vec2>*)source;
				auto &t = *(std::function<void(Vec3&, const Vec2&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC2:
			{
				auto &tgt = *(std::vector<Vec2>*)target;
				auto &src = *(std::vector<Vec2>*)source;
				auto &t = *(std::function<void(Vec2&, const Vec2&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::SCALAR:
			{
				auto &tgt = *(std::vector<real>*)target;
				auto &src = *(std::vector<Vec2>*)source;
				auto &t = *(std::function<void(real&, const Vec2&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
		}
		case AttribType::SCALAR: switch (tgt_type)
		{
			case AttribType::VEC4:
			{
				auto &tgt = *(std::vector<Vec4>*)target;
				auto &src = *(std::vector<real>*)source;
				auto &t = *(std::function<void(Vec4&, const real&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC3:
			{
				auto &tgt = *(std::vector<Vec3>*)target;
				auto &src = *(std::vector<real>*)source;
				auto &t = *(std::function<void(Vec3&, const real&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::VEC2:
			{
				auto &tgt = *(std::vector<Vec2>*)target;
				auto &src = *(std::vector<real>*)source;
				auto &t = *(std::function<void(Vec2&, const real&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
			case AttribType::SCALAR:
			{
				auto &tgt = *(std::vector<real>*)target;
				auto &src = *(std::vector<real>*)source;
				auto &t = *(std::function<void(real&, const real&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
		}
	}
}*/


////
// Class implementation - visual_attribute_mapping

template <class flt_type>
struct visual_attribute_mapping<flt_type>::Impl
{
	// fields
	std::map<VisualAttrib, attrib_reference> attrmap;
	bool use_color_mapping;
	colormap cm;
	unsigned mapped_flags;

	/// helper methods
	Impl() : use_color_mapping(false), mapped_flags(NULL) {}
	Impl(const Impl *other)
		: attrmap(other->attrmap), use_color_mapping(other->use_color_mapping), cm(other->cm),
		  mapped_flags(other->mapped_flags)
	{}
	Impl(const std::map<VisualAttrib, attrib_reference> &mapping)
		: attrmap(mapping), use_color_mapping(false), mapped_flags(NULL)
	{
		setup_mapped_flags(mapping);
	}
	Impl(std::map<VisualAttrib, attrib_reference> &&mapping)
		: attrmap(std::move(mapping)), use_color_mapping(false), mapped_flags(NULL)
	{
		setup_mapped_flags(attrmap);
	}
	Impl(const std::map<VisualAttrib, attrib_reference> &mapping, const colormap &cm)
		: attrmap(mapping), use_color_mapping(true), cm(cm), mapped_flags(NULL)
	{
		setup_mapped_flags(mapping);
	}
	Impl(std::map<VisualAttrib, attrib_reference> &&mapping, const colormap &cm)
		: attrmap(std::move(mapping)), use_color_mapping(true), cm(cm), mapped_flags(NULL)
	{
		setup_mapped_flags(attrmap);
	}
	void setup_mapped_flags (const std::map<VisualAttrib, attrib_reference> &mapping_source)
	{
		for (unsigned i=(unsigned)VisualAttrib::POSITION; i<=va_max; i++)
		{
			const auto it = mapping_source.find((VisualAttrib)i);
			mapped_flags |= (it!=mapping_source.end()) << i;
		}
	}
};

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping() : pimpl(nullptr)
{
	pimpl = new Impl;
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(const visual_attribute_mapping &other) : pimpl(nullptr)
{
	pimpl = new Impl(other.pimpl);
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(visual_attribute_mapping &&other) : pimpl(other.pimpl)
{
	other.pimpl = nullptr;
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(const std::map<VisualAttrib, attrib_reference> &mapping) : pimpl(nullptr)
{
	pimpl = new Impl(mapping);
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(std::map<VisualAttrib, attrib_reference> &&mapping) : pimpl(nullptr)
{
	pimpl = new Impl(std::move(mapping));
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(const std::map<VisualAttrib, attrib_reference> &mapping, const colormap &cm) : pimpl(nullptr)
{
	pimpl = new Impl(mapping, cm);
}

template <class flt_type>
visual_attribute_mapping<flt_type>::visual_attribute_mapping(std::map<VisualAttrib, attrib_reference> &&mapping, const colormap &cm) : pimpl(nullptr)
{
	pimpl = new Impl(std::move(mapping), cm);
}

template <class flt_type>
visual_attribute_mapping<flt_type>::~visual_attribute_mapping()
{
	if (pimpl)
	{
		delete pimpl;
		pimpl = nullptr;
	}
}

template <class flt_type>
visual_attribute_mapping<flt_type>& visual_attribute_mapping<flt_type>::operator= (const visual_attribute_mapping &other)
{
	const auto &other_impl = *(other.pimpl);
	if (!pimpl)
		pimpl = new Impl;
	auto &impl = *pimpl;
	impl.attrmap = other_impl.attrmap;
	impl.use_color_mapping = other_impl.use_color_mapping;
	impl.cm = other_impl.cm;
	impl.mapped_flags = other_impl.mapped_flags;
	return *this;
}

template <class flt_type>
visual_attribute_mapping<flt_type>& visual_attribute_mapping<flt_type>::operator= (visual_attribute_mapping &&other)
{
	if (pimpl)
		delete pimpl;
	pimpl = other.pimpl;
	other.pimpl = nullptr;
	return *this;
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::map_attribute (VisualAttrib visual_attrib, const std::string &name)
{
	auto it = pimpl->attrmap.emplace(visual_attrib, name);
	it.first->second.name = name;
	if (it.second)
		// the mapping didn't exist before, so update mapping flags
		pimpl->setup_mapped_flags(pimpl->attrmap);
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::map_attribute (VisualAttrib visual_attrib, const std::string &name, const attrib_transform<real> &transform)
{
	auto it = pimpl->attrmap.emplace(visual_attrib, name);
	auto &attrib = it.first->second;
	attrib.name = name;
	attrib.transform = transform;
	if (it.second)
		// the mapping didn't exist before, so update mapping flags
		pimpl->setup_mapped_flags(pimpl->attrmap);
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::unmap_attribute (VisualAttrib visual_attrib)
{
	pimpl->attrmap.erase(visual_attrib);
	pimpl->setup_mapped_flags(pimpl->attrmap);
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::transform_attribute (VisualAttrib visual_attrib, const attrib_transform<real> &transform)
{
	auto it = pimpl->attrmap.find(visual_attrib);
	if (it != pimpl->attrmap.end())
	{
		auto& attrib = it->second;
		attrib.transform = transform;
	}
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::setup_colormap (bool enable, const colormap &color_map)
{
	pimpl->use_color_mapping = enable;
	if (enable)
		pimpl->cm = color_map;
}

template <class flt_type>
const std::map<VisualAttrib, typename visual_attribute_mapping<flt_type>::attrib_reference>& visual_attribute_mapping<flt_type>::map (void) const
{
	return pimpl->attrmap;
}

template <class flt_type>
const colormap& visual_attribute_mapping<flt_type>::get_colormap (void) const
{
	return pimpl->cm;
}

template <class flt_type>
bool visual_attribute_mapping<flt_type>::is_mapped (VisualAttrib visual_attrib) const
{
	return (pimpl->mapped_flags >> (unsigned)visual_attrib) & 0x1;
}

template <class flt_type>
bool visual_attribute_mapping<flt_type>::uses_colormap (void) const
{
	return pimpl->use_color_mapping;
}

template <class flt_type>
void visual_attribute_mapping<flt_type>::clear (void)
{
	pimpl->attrmap.clear();
	pimpl->use_color_mapping = false;
}


////
// Class implementation - visual_attribute_mapping

template <class flt_type>
struct traj_dataset<flt_type>::Impl
{
	// types
	typedef typename traj_dataset::Vec3 Vec3;

	// fields
	std::string name, data_source;
	typename traj_attribute<flt_type> *positions;
	attribute_map<flt_type> attribs;
	std::unordered_map<unsigned, std::vector<range>> trajs;
	std::vector<range> empty_default_trajectories;
	visual_attribute_mapping<flt_type> attrmap;
	real avg_seg_len;

	/// helper methods
	Impl() : positions(nullptr), avg_seg_len(0) {}
	Impl(const std::string &name, const std::string &data_source)
		: name(name), data_source(data_source), positions(nullptr), avg_seg_len(0)
	{}
	Impl(const Impl *other)
		: name(other->name), data_source(other->data_source), positions(other->positions),
		  attribs(other->attribs), attrmap(other->attrmap), avg_seg_len(other->avg_seg_len)
	{}
	void operator= (const Impl *other)
	{
		name = other->name;
		data_source = other->data_source;
		positions = other->positions;
		attribs = other->attribs;
		attrmap = other->attrmap;
		avg_seg_len = other->avg_seg_len;
	}
	void clear (void)
	{
		name.clear();
		data_source.clear();
		positions = nullptr;
		attribs.clear();
		attrmap.clear();
	}
};

template <class flt_type>
traj_dataset<flt_type>::traj_dataset() : pimpl(nullptr)
{
	pimpl = new Impl;
}

template <class flt_type>
traj_dataset<flt_type>::traj_dataset(const std::string &name, const std::string &data_source) : pimpl(nullptr)
{
	pimpl = new Impl(name, data_source);
}

template <class flt_type>
traj_dataset<flt_type>::traj_dataset(const traj_dataset &other) : pimpl(nullptr)
{
	pimpl = new Impl(other.pimpl);
}

template <class flt_type>
traj_dataset<flt_type>::traj_dataset(traj_dataset &&other) : pimpl(other.pimpl)
{
	other.pimpl = nullptr;
}

template <class flt_type>
traj_dataset<flt_type>::~traj_dataset()
{
	if (pimpl)
	{
		delete pimpl;
		pimpl = nullptr;
	}
}

template <class flt_type>
traj_dataset<flt_type>& traj_dataset<flt_type>::operator= (const traj_dataset &other)
{
	*pimpl = other.pimpl;
	return *this;
}

template <class flt_type>
traj_dataset<flt_type>& traj_dataset<flt_type>::operator= (traj_dataset &&other)
{
	if (pimpl)
		delete pimpl;
	pimpl = other.pimpl;
	other.pimpl = nullptr;
	return *this;
}

template <class flt_type>
void traj_dataset<flt_type>::clear (void)
{
	pimpl->clear();
}

template <class flt_type>
bool traj_dataset<flt_type>::has_data (void) const
{
	const auto &impl = *pimpl;
	return impl.positions && impl.trajs.find(impl.positions->id) != impl.trajs.end();
}

template <class flt_type>
std::string& traj_dataset<flt_type>::data_source (void)
{
	return pimpl->data_source;
}

template <class flt_type>
traj_dataset<flt_type>::attrib_info<typename traj_dataset<flt_type>::Vec3> traj_dataset<flt_type>::positions (void)
{
	return attrib_info<Vec3>(*pimpl->positions);
}

template <class flt_type>
flt_type* traj_dataset<flt_type>::timestamps (void)
{
	return pimpl->positions->get_timestamps();
}

template <class flt_type>
typename attribute_map<flt_type>& traj_dataset<flt_type>::attributes (void)
{
	return pimpl->attribs;
}

template <class flt_type>
void traj_dataset<flt_type>::set_avg_segment_length (real length)
{
	pimpl->avg_seg_len = length;
}

template <class flt_type>
std::vector<range>& traj_dataset<flt_type>::trajectories (const traj_attribute<real> &attribute)
{
	// Note: we blindly invoke the element-access operator here (compare to the const version of this function) since we assume that the
	//       caller intends to create the trajectory entry if it doesn't yet exist and plans to commit the actual attribute data later on
	return pimpl->trajs[attribute.id];
}

template <class flt_type>
std::string& traj_dataset<flt_type>::name (void)
{
	return pimpl->name;
}

template <class flt_type>
const std::string& traj_dataset<flt_type>::data_source (void) const
{
	return pimpl->data_source;
}

template <class flt_type>
const traj_dataset<flt_type>::attrib_info<typename traj_dataset<flt_type>::Vec3> traj_dataset<flt_type>::positions (void) const
{
	return typename traj_dataset<flt_type>::attrib_info<Vec3>(*pimpl->positions);
}

template <class flt_type>
const flt_type* traj_dataset<flt_type>::timestamps (void) const
{
	return pimpl->positions->get_timestamps();
}

template <class flt_type>
std::vector<std::string> traj_dataset<flt_type>::get_attribute_names (void) const
{
	auto &impl = *pimpl;
	std::vector<std::string> names;
	std::transform(
		impl.attribs.begin(), impl.attribs.end(), std::back_inserter(names),
		[](const attribute_map<real>::value_type &elem) { return elem.first; }
	);
	return names;
}

template <class flt_type>
bool traj_dataset<flt_type>::has_attribute (const std::string &name) const
{
	const auto &impl = *pimpl;
	return impl.attribs.find(name) != impl.attribs.end();
}

template <class flt_type>
const traj_attribute<flt_type>& traj_dataset<flt_type>::attribute (const std::string &name) const
{
	static const traj_attribute<real> error_attrib(0);
	const auto &impl = *pimpl;
	const auto it = impl.attribs.find(name);
	return it != impl.attribs.end() ? it->second : error_attrib;
}

template <class flt_type>
flt_type traj_dataset<flt_type>::avg_segment_length (void) const
{
	return pimpl->avg_seg_len;
}

template <class flt_type>
const std::vector<range>& traj_dataset<flt_type>::trajectories (const traj_attribute<real> &attribute) const
{
	// find index range and return it
	const auto &impl = *pimpl;
	const auto it = impl.trajs.find(attribute.id);
	return it != impl.trajs.end() ? it->second : impl.empty_default_trajectories;
}

template <class flt_type>
bool traj_dataset<flt_type>::set_mapping (const visual_attribute_mapping<real> &visual_attrib_mapping)
{
	const auto &mapping = visual_attrib_mapping.map();
	const auto it_visual = mapping.find(VisualAttrib::POSITION);
	if (it_visual != mapping.end())
	{
		auto &impl = *pimpl;
		const auto it_data = impl.attribs.find(it_visual->second.name);
		if (it_data != impl.attribs.end())
		{
			impl.attrmap = visual_attrib_mapping;
			impl.positions = &it_data->second;
			return true;
		}
	}
	return false; 
}

template <class flt_type>
bool traj_dataset<flt_type>::set_mapping (visual_attribute_mapping<real> &&visual_attrib_mapping)
{
	const auto &map = visual_attrib_mapping.map();
	const auto it_visual = map.find(VisualAttrib::POSITION);
	if (it_visual != map.end())
	{
		auto &impl = *pimpl;
		const auto it_data = impl.attribs.find(it_visual->second.name);
		if (it_data != impl.attribs.end())
		{
			impl.attrmap = std::move(visual_attrib_mapping);
			impl.positions = &it_data->second;
			return true;
		}
	}
	return false;
}

template <class flt_type>
const visual_attribute_mapping<flt_type>& traj_dataset<flt_type>::mapping (void) const
{
	return pimpl->attrmap;
}


////
// Class implementation - traj_format_handler

template <class flt_type>
traj_dataset<flt_type>::attrib_info<typename traj_dataset<flt_type>::Vec3> traj_format_handler<flt_type>::positions (traj_dataset<real> &dataset)
{
	return dataset.positions();
}

template <class flt_type>
attribute_map<flt_type>& traj_format_handler<flt_type>::attributes (traj_dataset<real> &dataset)
{
	return dataset.attributes();
}

template <class flt_type>
void traj_format_handler<flt_type>::set_avg_segment_length (traj_dataset<real> &dataset, real length)
{
	return dataset.set_avg_segment_length(length);
}

template <class flt_type>
std::vector<range>& traj_format_handler<flt_type>::trajectories (traj_dataset<real> &dataset, const traj_attribute<real> &attribute)
{
	return dataset.trajectories(attribute);
}


////
// Class implementation - traj_manager

template <class flt_type>
struct traj_manager<flt_type>::Impl
{
	// types
	typedef typename traj_manager::real real;
	typedef typename traj_manager::Vec2 Vec2;
	typedef typename traj_manager::Vec3 Vec3;
	typedef typename traj_manager::Vec4 Vec4;
	typedef typename traj_manager::Color Color;

	// for returning a matching entry in visual attribute mappings
	struct visual_attrib_match {
		bool found;
		const std::string &attrib_name;
		typename visual_attribute_mapping<real>::map_type::const_iterator it;
	};

	// fields
	std::vector<std::unique_ptr<traj_dataset<real>>> datasets;
	render_data rd;
	bool dirty = true;

	// helper methods
	Impl()
	{}
	~Impl()
	{}
	static void ensure_dataset_defaults (traj_dataset<real> &dataset)
	{
		auto &ds_impl = *dataset.pimpl;
		const auto &pos_trajs = dataset.trajectories(*ds_impl.positions);

		// make sure querying trajectories for non-existing attributes (or for attributes that don't
		// have trajectory information) works
		if (ds_impl.empty_default_trajectories.size() < 1)
			ds_impl.empty_default_trajectories = std::vector<range>(pos_trajs.size(), range{(unsigned)-1, 0, 0});

#ifdef _DEBUG
		// Sanity checks
		// - per-attribute
		for (const auto &a : ds_impl.attribs)
		{
			// all attributes are named
			assert(!a.first.empty());
			// consistent trajectory information
			assert(dataset.trajectories(a.second).size() == pos_trajs.size());
		}
#endif
	}
	static visual_attrib_match find_visual_attrib (const visual_attribute_mapping<real> &mapping,
	                                               VisualAttrib visual_attrib)
	{
		const auto &map = mapping.map();
		const auto it = map.find(visual_attrib);
		if (it != map.end())
		{
			return {true, it->second.name, it};
		}
		else
			return {false, "", it};
	}
	template <class T>
	void transform_attrib_old (
		std::vector<T> *out, VisualAttrib visual_attrib, const traj_dataset<real> &dataset, int auto_tangents_ds=-1
	)
	{
		const auto match = Impl::find_visual_attrib(dataset.mapping(), visual_attrib);
		if (match.found)
		{
			const auto &ref = match.it->second;
			const auto &attrib = dataset.attribute(match.attrib_name);
			if (ref.transform.is_identity())
			{
				const auto &data = attrib.get_data<T>();
				out->reserve(out->size() + data.num());
				out->insert(out->end(), data.begin(), data.end());
			}
			else switch(ref.transform.get_src_type())
			{
				case AttribType::SCALAR:
				{
					const auto &data = attrib.get_data<real>();
					out->reserve(out->size() + data.num());
					for (unsigned i=0; i<data.num(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data.values[i]);
					}
					return;
				}
				case AttribType::VEC2:
				{
					const auto &data = attrib.get_data<Vec2>();
					out->reserve(out->size() + data.num());
					for (unsigned i=0; i<data.num(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data.values[i]);
					}
					return;
				}
				case AttribType::VEC3:
				{
					const auto &data = attrib.get_data<Vec3>();
					out->reserve(out->size() + data.num());
					for (unsigned i=0; i<data.num(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data.values[i]);
					}
					return;
				}
				case AttribType::VEC4:
				{
					const auto &data = attrib.get_data<Vec4>();
					out->reserve(out->size() + data.num());
					for (unsigned i=0; i<data.num(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data.values[i]);
					}
				}
			}
			return;
		}
		if (visual_attrib == VisualAttrib::RADIUS)
		{
			// default to 1/4th the average segment length
			const real r = dataset.avg_segment_length() / real(4);
			const size_t num = dataset.positions().data.num();
			out->reserve(out->size() + num);
			for (size_t i=0; i<num; i++)
				out->emplace_back(r);
			// ^ ToDo: why does trying to do this simple fill operation with std::fill_n throw a runtime error
			//         about non-seekable vectors???
			return;
		}
		if (visual_attrib == VisualAttrib::TANGENT)
		{
			if (auto_tangents_ds > -1)
			{
				// get all necessary pointers, references and metrics
				unsigned idx_offset = (unsigned)out->size();
				const auto *I = rd.indices.data();
				const auto *P = rd.positions.data();
				const auto *R = rd.radii.data();
				const auto &trajs = rd.datasets[auto_tangents_ds].trajs;

				// generate tangents on per-trajectory basis (currently hard-coded as radius- and segment length-
				// adaptive variant of central differences)
				out->resize(idx_offset + dataset.positions().data.num());
				auto *o = ((std::vector<Vec4>*)out)->data();
				for (const auto &traj : trajs)
				{
					// skip single-sample trajectories (they will just retain a 0-length tangent)
					if (traj.n < 2) continue;

					// determine tangent of first sample in trajectory
					auto id_1st = I[traj.i0],
					     id_2nd = I[traj.i0+1];
					Vec3 tangent = P[id_2nd] - P[id_1st];
					o[id_1st] = Vec4(tangent, 0);

					// determine all inner tangents
					for (unsigned j=1; j<traj.n-2; j+=2)
					{
						id_1st = I[traj.i0+j-1];
						id_2nd = I[traj.i0+j+2];
						const auto id_mid = I[traj.i0+j];
						Vec3 diff0 = P[id_mid] - P[id_1st], diff1 = P[id_2nd] - P[id_mid];
						real len0 = diff0.length(), len1 = diff1.length(),
						     len = std::max(R[id_mid]/real(2), std::min(len0, len1));
						tangent = cgv::math::normalize(diff0 + diff1) * len;
						o[id_mid] = Vec4(tangent, 0);
					}

					// determine tangent of last sample in trajectory
					id_1st = I[traj.i0 + traj.n-2];
					id_2nd = I[traj.i0 + traj.n-1];
					tangent = (P[id_2nd] - P[id_1st]);
					o[id_2nd] = Vec4(tangent, 0);
				}
				return;
			}
			// default to 0-length tangents (i.e. a poly-line)
			const size_t num = dataset.positions().data.num();
			auto &o = *(std::vector<Vec4>*)out;
			o.reserve(o.size() + num);
			for (size_t i=0; i<num; i++)
				o.emplace_back(Vec4{0, 0, 0, 0});
			// ^ ToDo: why does trying to do this simple fill operation with std::fill_n throw a runtime error
			//         about non-seekable vectors???
		}
	}
	// ToDo: This specialization for the color attribute is not really ideal, should be handled more generically
	template <>
	void transform_attrib_old<Color> (
		std::vector<Color> *out, VisualAttrib visual_attrib, const traj_dataset<real> &dataset, int /*irrelevant*/
	)
	{
		const auto &mapping = dataset.mapping();
		const auto match = Impl::find_visual_attrib(mapping, visual_attrib);
		if (match.found)
		{
			const auto &ref = match.it->second;
			const auto &attrib = dataset.attribute(match.attrib_name);
			if (ref.transform.is_identity())
			{
				const auto &data = attrib.get_data<Vec3>();
				out->reserve(out->size() + data.num());
				std::transform(
					data.begin(), data.end(), std::back_inserter(*out),
					[] (const Vec3 &src) { return vec3_to_rgb(src); }
				);
			}
			else switch(ref.transform.get_src_type())
			{
				case AttribType::SCALAR:
				{
					const auto &data = attrib.get_data<real>();
					out->reserve(out->size() + data.num());
					if (mapping.uses_colormap())
					{
						const auto &color_scale = mapping.get_colormap().get_color_scale();
						std::transform(
							data.begin(), data.end(), std::back_inserter(*out),
							[&ref, &color_scale] (real src) {
								real tmp;
								ref.transform.exec(tmp, src);
								auto c = cgv::media::sample_sampled_color_scale((float)tmp, color_scale);
								return c;
							}
						);
					}
					else
						std::transform(
							data.begin(), data.end(), std::back_inserter(*out),
							[&ref] (real src) { Vec3 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(tmp); }
						);
					return;
				}
				case AttribType::VEC2:
				{
					const auto &data = attrib.get_data<Vec2>();
					out->reserve(out->size() + data.num());
					std::transform(
						data.begin(), data.end(), std::back_inserter(*out),
						[&ref] (const Vec2 &src) { Vec3 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(tmp); }
					);
					return;
				}
				case AttribType::VEC3:
				{
					const auto &data = attrib.get_data<Vec3>();
					out->reserve(out->size() + data.num());
					std::transform(
						data.begin(), data.end(), std::back_inserter(*out),
						[&ref] (const Vec3 &src) { Vec3 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(tmp); }
					);
					return;
				}
				case AttribType::VEC4:
				{
					const auto &data = attrib.get_data<Vec4>();
					out->reserve(out->size() + data.num());
					std::transform(
						data.begin(), data.end(), std::back_inserter(*out),
						[&ref] (const Vec4 &src) { Vec4 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(*(Vec3*)&tmp); }
					);
				}
			}
			return;
		}
		// default to dark-ish grey
		const Color c(1.f/3.f);
		const size_t num = dataset.positions().data.num();
		out->reserve(out->size() + num);
		for (size_t i=0; i<num; i++)
			out->emplace_back(c);
		// ^ ToDo: why does trying to do this simple fill operation with std::fill_n throw a runtime error
		//         about non-seekable vectors???
	}

	template <class T>
	void transform_attrib (
		T &out, VisualAttrib visual_attrib, const traj_dataset<real> &dataset, bool auto_tangents=false
	)
	{	// ToDo: implement component swizzling for visual attributes, so clients can exactly specify the format of
		//       their output draw arrays (like radius and its derivative inside vec4 positions and tangents, or
		//       everything inside a big mat4)
	}
};

template <class flt_type>
traj_manager<flt_type>::traj_manager() : pimpl(nullptr)
{
	pimpl = new Impl;
}

template <class flt_type>
traj_manager<flt_type>::~traj_manager()
{
	if (pimpl)
	{
		delete pimpl;
		pimpl = nullptr;
	}
}

template <class flt_type>
bool traj_manager<flt_type>::can_load (const std::string &path) const
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// handle path pointing to a directory
	if (cgv::utils::dir::exists(path))
	{
		// not supported yet!
		std::cout << "traj_loader: WARNING - loading a directory is not yet supported!" << std::endl << std::endl;
		return false;
	}

	// assume it's a file that can just be opened
	std::ifstream file(path);
	if (!file.is_open())
	{
		std::cout << "traj_loader: cannot open file:" << std::endl << '"'<<path<<'"' << std::endl << std::endl;
		return false;
	}

	// test if we find a suitable handler
	auto &handlers = trajectory_handler_registry<real>::handlers();
	for (auto &h : handlers)
		if (h->get_interface<traj_format_handler<flt_type> >()->can_handle(file))
			// yes we can...
			return true;
	// no we can't...
	return false;
}

template <class flt_type>
unsigned traj_manager<flt_type>::load (const std::string &path)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// handle path pointing to a directory
	if (cgv::utils::dir::exists(path))
	{
		// not supported yet!
		std::cout << "traj_loader: WARNING - loading a directory is not yet supported!" << std::endl << std::endl;
		return -1;
	}

	// assume it's a file that can just be opened
	std::ifstream file(path);
	if (!file.is_open())
	{
		std::cout << "traj_loader: cannot open file:" << std::endl << '"'<<path<<'"' << std::endl << std::endl;
		return -1;
	}

	// delegate actual loading to first suitable handler
	traj_dataset<real> new_dataset;
	auto &handlers = trajectory_handler_registry<real>::handlers();
	traj_format_handler<real> *handler = nullptr;
	for (auto &_h : handlers)
	{
		auto h = _h->get_interface<traj_format_handler<flt_type> >();
		if (h->can_handle(file))
		{
			new_dataset = h->read(file);
			if (new_dataset.has_data() && new_dataset.mapping().is_mapped(VisualAttrib::POSITION))
			{
				Impl::ensure_dataset_defaults(new_dataset);
				handler = h;
				break;
			}
			else
				std::cerr << "traj_loader: selected handler did not return usable data, trying next..."
				          << std::endl << std::endl;
		}
	}

	// post-process
	if (handler)
	{
		// add a dataset for the loaded trajectories
		new_dataset.data_source() = path;
		impl.datasets.emplace_back(new traj_dataset<real>(std::move(new_dataset)));
		impl.dirty = true; // we will need to rebuild the render data

		// done
		return (unsigned)impl.datasets.size()-1;
	}

	// indicate that we ended up not actually loading anything
	std::cerr << "traj_loader: no handler found for loading '"<<path<<"'"
	          << std::endl << std::endl;
	return -1;
}

template <class flt_type>
unsigned traj_manager<flt_type>::add_dataset (const traj_dataset<real> &dataset)
{
	auto &impl = *pimpl; // shortcut for saving one indirection
	impl.datasets.emplace_back(new traj_dataset<real>(dataset));
	Impl::ensure_dataset_defaults(*impl.datasets.back());
	impl.dirty = true; // we will need to rebuild the render data;
	return (unsigned)impl.datasets.size() - 1;
}

template <class flt_type>
unsigned traj_manager<flt_type>::add_dataset (traj_dataset<real> &&dataset)
{
	auto &impl = *pimpl; // shortcut for saving one indirection
	Impl::ensure_dataset_defaults(dataset);
	impl.datasets.emplace_back(new traj_dataset<real>(std::move(dataset)));
	impl.dirty = true; // we will need to rebuild the render data
	return (unsigned)impl.datasets.size() - 1;
}

template <class flt_type>
unsigned traj_manager<flt_type>::num_datasets (void) const
{
	return (unsigned)pimpl->datasets.size();
}

template <class flt_type>
const traj_dataset<flt_type>& traj_manager<flt_type>::dataset (unsigned index) const
{
	return *(pimpl->datasets[index].get());
}

template <class flt_type>
typename traj_manager<flt_type>::dataset_range traj_manager<flt_type>::datasets (void) const
{
	return dataset_range(*this);
}

template <class flt_type>
void traj_manager<flt_type>::clear (void)
{
	pimpl->datasets.clear();
	pimpl->dirty = true;
}

template <class flt_type>
const typename traj_manager<flt_type>::render_data& traj_manager<flt_type>::get_render_data (bool auto_tangents)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// check if the render data needs to be rebuild
	if (impl.dirty)
	{
		impl.rd.datasets.clear();
		impl.rd.positions.clear();
		impl.rd.tangents.clear();
		impl.rd.radii.clear();
		impl.rd.colors.clear();
		impl.rd.indices.clear();
		for (unsigned ds=0; ds<(unsigned)impl.datasets.size(); ds++)
		{
			// convencience shorthands
			auto &dataset = *impl.datasets[ds];
			const auto &positions = dataset.positions().attrib;
			auto &trajectories = dataset.trajectories(positions);

			// build line list indices
			const unsigned idx_base = (unsigned)impl.rd.positions.size();
			std::vector<range> traj_ranges; traj_ranges.reserve(trajectories.size());
			for (const auto &traj : trajectories)
			{
				if (traj.n < 1)
					continue;
				const unsigned idx_traj_offset = (unsigned)impl.rd.indices.size();
				impl.rd.indices.push_back(idx_base+traj.i0); unsigned idx=impl.rd.indices.back()+1;
				for (unsigned i=1; i<traj.n-1; i++)
				{
					impl.rd.indices.push_back(idx);
					impl.rd.indices.push_back(idx); // twice to gain line-list semantics
					idx++;
				}
				if (traj.n > 1)
					impl.rd.indices.push_back(idx); // final index only once
				else
					// degenerate single-sample segment
					impl.rd.indices.push_back(impl.rd.indices.back());
				traj_ranges.emplace_back(range{idx_traj_offset, (unsigned)impl.rd.indices.size()-idx_traj_offset});
			}
			impl.rd.datasets.emplace_back(render_data::dataset{
				/* full dataset range */ { idx_base, (unsigned)impl.rd.indices.size()-idx_base },
				/* individual trajectory ranges*/ std::move(traj_ranges)
			});
			auto &ds_info = impl.rd.datasets.back();

			// copy mapped attributes, applying the desired transformations (if any)
			impl.transform_attrib_old(&impl.rd.positions, VisualAttrib::POSITION, dataset);
			impl.transform_attrib_old(&impl.rd.radii, VisualAttrib::RADIUS, dataset);
			impl.transform_attrib_old(&impl.rd.tangents, VisualAttrib::TANGENT, dataset, auto_tangents ? ds : -1);
			impl.transform_attrib_old(&impl.rd.colors, VisualAttrib::COLOR, dataset);

			// ToDo: apply visual mapping to all involved attributes and distribute to user output draw arrays
			/*for (unsigned i = 0; i<(unsigned)dataset.positions().size(); i++)
			{
			}*/

			// Calculate per-trajectory median of node radii as well as dataset median node radius
			// ToDo: this should really also be weighted by segment length...
			// - (1) trajectory median radii
			std::vector<real> med_radii; med_radii.reserve(trajectories.size());
			for (auto &traj : ds_info.trajs)
			{
				std::vector<real> traj_radii; traj_radii.reserve(traj.n/2 + 1);
				for (unsigned i=0; i<traj.n; i+=2)
					traj_radii.emplace_back(impl.rd.radii[impl.rd.indices[traj.i0+i]]);
				traj_radii.emplace_back(impl.rd.radii[impl.rd.indices[traj.i0+traj.n-1]]);
				std::sort(traj_radii.begin(), traj_radii.end());
				if (traj_radii.size()%2 == 0)
				{
					const auto mid = traj_radii.size()/2;
					traj.med_radius = float(traj_radii[mid-1]+traj_radii[mid]) / 2.f;
				}
				else
					traj.med_radius = float(traj_radii[traj_radii.size()/2]);
				med_radii.emplace_back(traj.med_radius);
			}
			// - (2) dataset median radius
			std::sort(med_radii.begin(), med_radii.end());
			if (med_radii.size()%2 == 0)
			{
				const auto mid = med_radii.size()/2;
				ds_info.irange.med_radius = float(med_radii[mid-1]+med_radii[mid]) / 2.f;
			}
			else
				ds_info.irange.med_radius = float(med_radii[med_radii.size()/2]);
		}
		impl.dirty = false;
	}

	// done
	return impl.rd;
}

template <class flt_type>
const typename traj_manager<flt_type>::render_data& traj_manager<flt_type>::get_render_data (void) const
{
	// default return value representing dirty state situations
	static const render_data ood;

	// return actual render data if current, and empty dummy if dirty / out-of-date
	return pimpl->dirty ? ood : pimpl->rd;
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class traj_attribute<float>;
template class traj_attribute<double>;
template class attrib_transform<float>;
template class attrib_transform<double>;
template class visual_attribute_mapping<float>;
template class visual_attribute_mapping<double>;
template class traj_dataset<float>;
template class traj_dataset<double>;
template class traj_format_handler<float>;
template class traj_format_handler<double>;
template class traj_manager<float>;
template class traj_manager<double>;


////
// Object registration

// The trajectory format handler registry
cgv::base::object_registration<trajectory_handler_registry<float> > flt_trj_registry("trajectory handler registry (float)");
cgv::base::object_registration<trajectory_handler_registry<double> > dbl_trj_registry("trajectory handler registry (double)");
