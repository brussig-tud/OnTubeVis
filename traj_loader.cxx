
// C++ STL
#include <iostream>
#include <fstream>
#include <algorithm>
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

namespace type_str {
	static const std::string REAL       = "REAL";
	static const std::string VEC2       = "VEC2";
	static const std::string VEC3       = "VEC3";
	static const std::string VEC4       = "VEC4";
	static const std::string ERROR_TYPE = "ERROR_TYPE";
};

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

template <class flt_type>
traj_attribute<flt_type>::container_base::~container_base()
{}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (const traj_attribute &other) : data(nullptr)
{
	switch (other._type)
	{
		case AttribType::REAL:
			_type = AttribType::REAL;
			data = new container<real>(other.get_data<real>());
			return;
		case AttribType::VEC2:
			_type = AttribType::VEC2;
			data = new container<Vec2>(other.get_data<Vec2>());
			return;
		case AttribType::VEC3:
			_type = AttribType::VEC3;
			data = new container<Vec3>(other.get_data<Vec3>());
			return;
		case AttribType::VEC4:
			_type = AttribType::VEC4;
			data = new container<Vec4>(other.get_data<Vec4>());
			return;

		default:
			/* DoNothing() */;
	}
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (traj_attribute &&other)
	: _type(other._type), data(other.data)
{
	other.data = nullptr;
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (unsigned components) : data(nullptr)
{
	switch (components)
	{
		case 1:
			_type = AttribType::REAL;
			data = new container<real>();
			return;
		case 2:
			_type = AttribType::VEC2;
			data = new container<Vec2>();
			return;
		case 3:
			_type = AttribType::VEC3;
			data = new container<Vec3>();
			return;
		case 4:
			_type = AttribType::VEC4;
			data = new container<Vec4>();
			return;

		default:
			/* DoNothing() */;
	}
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<real> &&source)
	: _type(AttribType::REAL), data(nullptr)
{
	data = new container<real>(std::move(source));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec2> &&source)
	: _type(AttribType::VEC2), data(nullptr)
{
	data = new container<Vec2>(std::move(source));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec3> &&source)
	: _type(AttribType::VEC3), data(nullptr)
{
	data = new container<Vec3>(std::move(source));
}

template <class flt_type>
traj_attribute<flt_type>::traj_attribute (std::vector<Vec4> &&source)
	: _type(AttribType::VEC4), data(nullptr)
{
	data = new container<Vec4>(std::move(source));
}

template <class flt_type>
traj_attribute<flt_type>::~traj_attribute()
{
	if (data)
	{
		delete data;
		data = nullptr;
	}
}

template <class flt_type>
traj_attribute<flt_type>& traj_attribute<flt_type>::operator= (const traj_attribute &other)
{
	_type = other._type;
	data = other.data;
	return *this;
}

template <class flt_type>
traj_attribute<flt_type>& traj_attribute<flt_type>::operator= (traj_attribute &&other)
{
	this->~traj_attribute();
	_type = other._type;
	std::swap(data, other.data);
	return *this;
}

template <class flt_type>
const std::string& traj_attribute<flt_type>::type_string (void) const
{
	switch (_type)
	{
		case AttribType::REAL:
			return type_str::REAL;
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
	: tgt_type(AttribType::REAL), src_type(AttribType::VEC3)
{
	t_ptr = new vec3_to_real(transform_func);
}

template <class flt_type>
attrib_transform<flt_type>::attrib_transform(const real_to_real &transform_func)
	: tgt_type(AttribType::REAL), src_type(AttribType::REAL)
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
			case AttribType::REAL:
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
			case AttribType::REAL:
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
			case AttribType::REAL:
				delete (std::function<void(real&, const Vec2&)>*)t_ptr;
				goto _done;
		}
		case AttribType::REAL: switch (tgt_type) {
			case AttribType::VEC4:
				delete (std::function<void(Vec4&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::VEC3:
				delete (std::function<void(Vec3&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::VEC2:
				delete (std::function<void(Vec2&, const real&)>*)t_ptr;
				goto _done;
			case AttribType::REAL:
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
			case AttribType::REAL:
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
			case AttribType::REAL:
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
			case AttribType::REAL:
				t_ptr = new std::function<void(real&, const Vec2&)>(*(std::function<void(real&, const Vec2&)>*)other.t_ptr);
				return *this;
		}
		case AttribType::REAL: switch (tgt_type) {
			case AttribType::VEC4:
				t_ptr = new std::function<void(Vec4&, const real&)>(*(std::function<void(Vec4&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC3:
				t_ptr = new std::function<void(Vec3&, const real&)>(*(std::function<void(Vec3&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::VEC2:
				t_ptr = new std::function<void(Vec2&, const real&)>(*(std::function<void(Vec2&, const real&)>*)other.t_ptr);
				return *this;
			case AttribType::REAL:
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
			case AttribType::REAL:
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
			case AttribType::REAL:
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
			case AttribType::REAL:
			{
				auto &tgt = *(std::vector<real>*)target;
				auto &src = *(std::vector<Vec2>*)source;
				auto &t = *(std::function<void(real&, const Vec2&)>*)t_ptr;
				for (unsigned i=0; i<tgt.size(); i++)
					t(tgt[i], src[i]);
				return;
			}
		}
		case AttribType::REAL: switch (tgt_type)
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
			case AttribType::REAL:
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
	std::vector<Vec3> *positions;
	attribute_map<flt_type> attribs;
	std::vector<unsigned> indices;
	std::vector<traj_dataset::trajectory> trajs;
	visual_attribute_mapping<flt_type> attrmap;
	real avg_seg_len;

	/// helper methods
	Impl() : positions(nullptr), avg_seg_len(0) {}
	Impl(const std::string &name, const std::string &data_source)
		: name(name), data_source(data_source), positions(nullptr), avg_seg_len(0)
	{}
	Impl(const Impl *other)
		: name(other->name), data_source(other->data_source), positions(other->positions),
		  attribs(other->attribs), indices(other->indices), attrmap(other->attrmap), avg_seg_len(other->avg_seg_len)
	{}
	void operator= (const Impl *other)
	{
		name = other->name;
		data_source = other->data_source;
		positions = other->positions;
		attribs = other->attribs;
		indices = other->indices;
		attrmap = other->attrmap;
		avg_seg_len = other->avg_seg_len;
	}
	void clear (void)
	{
		name.clear();
		data_source.clear();
		positions = nullptr;
		attribs.clear();
		indices.clear();
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
	return pimpl->positions && !pimpl->indices.empty() && !pimpl->trajs.empty();
}

template <class flt_type>
std::string& traj_dataset<flt_type>::data_source (void)
{
	return pimpl->data_source;
}

template <class flt_type>
std::vector<typename traj_dataset<flt_type>::Vec3>& traj_dataset<flt_type>::positions (void)
{
	return *(pimpl->positions);
}

template <class flt_type>
typename attribute_map<flt_type>& traj_dataset<flt_type>::attributes (void)
{
	return pimpl->attribs;
}

template <class flt_type>
std::vector<unsigned>& traj_dataset<flt_type>::indices (void)
{
	return pimpl->indices;
}

template <class flt_type>
void traj_dataset<flt_type>::set_avg_segment_length (real length)
{
	pimpl->avg_seg_len = length;
}

template <class flt_type>
std::vector<typename traj_dataset<flt_type>::trajectory>& traj_dataset<flt_type>::trajectories (void)
{
	return pimpl->trajs;
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
const std::vector<typename traj_dataset<flt_type>::Vec3>& traj_dataset<flt_type>::positions (void) const
{
	return *(pimpl->positions);
}

template <class flt_type>
const attribute_map<flt_type>& traj_dataset<flt_type>::attributes (void) const
{
	return pimpl->attribs;
}

template <class flt_type>
const std::vector<unsigned>& traj_dataset<flt_type>::indices (void) const
{
	return pimpl->indices;
}

template <class flt_type>
flt_type traj_dataset<flt_type>::avg_segment_length (void) const
{
	return pimpl->avg_seg_len;
}

template <class flt_type>
const std::vector<typename traj_dataset<flt_type>::trajectory>& traj_dataset<flt_type>::trajectories (void) const
{
	return pimpl->trajs;
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
			impl.positions = &(it_data->second.get_data<Vec3>());
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
			impl.positions = &(it_data->second.get_data<Vec3>());
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
std::vector<typename traj_format_handler<flt_type>::Vec3>& traj_format_handler<flt_type>::positions (traj_dataset<real> &dataset)
{
	return dataset.positions();
}

template <class flt_type>
attribute_map<flt_type>& traj_format_handler<flt_type>::attributes (traj_dataset<real> &dataset)
{
	return dataset.attributes();
}

template <class flt_type>
std::vector<unsigned>& traj_format_handler<flt_type>::indices (traj_dataset<real> &dataset)
{
	return dataset.indices();
}

template <class flt_type>
void traj_format_handler<flt_type>::set_avg_segment_length (traj_dataset<real> &dataset, real length)
{
	return dataset.set_avg_segment_length(length);
}

template <class flt_type>
std::vector<typename traj_dataset<flt_type>::trajectory>& traj_format_handler<flt_type>::trajectories (traj_dataset<real> &dataset)
{
	return dataset.trajectories();
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
	std::vector<std::unique_ptr<traj_dataset<real> > > datasets;
	render_data rd;
	bool dirty = true;

	// helper methods
	Impl()
	{}
	~Impl()
	{}
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
		std::vector<T> *out, VisualAttrib visual_attrib, const traj_dataset<real> &dataset, bool auto_tangents=false
	)
	{
		const auto match = Impl::find_visual_attrib(dataset.mapping(), visual_attrib);
		if (match.found)
		{
			const auto &ref = match.it->second;
			const auto &attrib = dataset.attributes().find(match.attrib_name)->second;
			if (ref.transform.is_identity())
			{
				const auto &data = attrib.get_data<T>();
				out->reserve(out->size() + data.size());
				out->insert(out->end(), data.begin(), data.end());
			}
			else switch(ref.transform.get_src_type())
			{
				case AttribType::REAL:
				{
					const auto &data = attrib.get_data<real>();
					out->reserve(out->size() + data.size());
					for (unsigned i=0; i<data.size(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data[i]);
					}
					return;
				}
				case AttribType::VEC2:
				{
					const auto &data = attrib.get_data<Vec2>();
					out->reserve(out->size() + data.size());
					for (unsigned i=0; i<data.size(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data[i]);
					}
					return;
				}
				case AttribType::VEC3:
				{
					const auto &data = attrib.get_data<Vec3>();
					out->reserve(out->size() + data.size());
					for (unsigned i=0; i<data.size(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data[i]);
					}
					return;
				}
				case AttribType::VEC4:
				{
					const auto &data = attrib.get_data<Vec4>();
					out->reserve(out->size() + data.size());
					for (unsigned i=0; i<data.size(); i++)
					{
						out->emplace_back();
						ref.transform.exec(out->back(), data[i]);
					}
				}
			}
			return;
		}
		if (visual_attrib == VisualAttrib::RADIUS)
		{
			// default to 1/4th the average segment length
			const real r = dataset.avg_segment_length() / real(4);
			const size_t num = dataset.positions().size();
			out->reserve(out->size() + num);
			for (size_t i=0; i<num; i++)
				out->emplace_back(r);
			// ^ ToDo: why does trying to do this simple fill operation with std::fill_n throw a runtime error
			//         about non-seekable vectors???
			return;
		}
		if (visual_attrib == VisualAttrib::TANGENT)
		{
			if (auto_tangents)
			{
				// get all necessary pointers, references and metrics
				unsigned idx_offset = (unsigned)out->size();
				const auto *P = rd.positions.data() + idx_offset;
				const auto *R = rd.radii.data() + idx_offset;
				const auto &I = dataset.indices();
				const auto &trajs = dataset.trajectories();

				// generate tangents on per-trajectory basis (currently hard-coded as radius and segment length
				// adaptive variant of central differences)
				out->resize(idx_offset + dataset.positions().size());
				auto *o = ((std::vector<Vec4>*)out)->data() + idx_offset;
				for (auto &traj : trajs)
				{
					// skip single-sample trajectories (they will just retain a 0-length tangent)
					if (traj.n < 1) continue;

					// determine tangent of first sample in trajectory
					auto id_1st = I[traj.i0],
					     id_2nd = I[traj.i0+1];
					Vec3 tangent = P[id_2nd] - P[id_1st];
					o[id_1st] = vec4_from_vec3s(tangent, 0);

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
						o[id_mid] = vec4_from_vec3s(tangent, 0);
					}

					// determine tangent of last sample in trajectory
					id_1st = I[traj.i0 + traj.n-2];
					id_2nd = I[traj.i0 + traj.n-1];
					tangent = (P[id_2nd] - P[id_1st]);
					o[id_2nd] = vec4_from_vec3s(tangent, 0);
				}
				return;
			}
			// default to 0-length tangents (i.e. a poly-line)
			const size_t num = dataset.positions().size();
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
		std::vector<Color> *out, VisualAttrib visual_attrib, const traj_dataset<real> &dataset, bool auto_tangents
	)
	{
		const auto &mapping = dataset.mapping();
		const auto match = Impl::find_visual_attrib(mapping, visual_attrib);
		if (match.found)
		{
			const auto &ref = match.it->second;
			const auto &attrib = dataset.attributes().find(match.attrib_name)->second;
			if (ref.transform.is_identity())
			{
				const auto &data = attrib.get_data<Vec3>();
				out->reserve(out->size() + data.size());
				std::transform(
					data.begin(), data.end(), std::back_inserter(*out),
					[&ref] (const Vec3 &src) { return vec3_to_rgb(src); }
				);
			}
			else switch(ref.transform.get_src_type())
			{
				case AttribType::REAL:
				{
					const auto &data = attrib.get_data<real>();
					out->reserve(out->size() + data.size());
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
					out->reserve(out->size() + data.size());
					std::transform(
						data.begin(), data.end(), std::back_inserter(*out),
						[&ref] (const Vec2 &src) { Vec3 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(tmp); }
					);
					return;
				}
				case AttribType::VEC3:
				{
					const auto &data = attrib.get_data<Vec3>();
					out->reserve(out->size() + data.size());
					std::transform(
						data.begin(), data.end(), std::back_inserter(*out),
						[&ref] (const Vec3 &src) { Vec3 tmp; ref.transform.exec(tmp, src); return vec3_to_rgb(tmp); }
					);
					return;
				}
				case AttribType::VEC4:
				{
					const auto &data = attrib.get_data<Vec4>();
					out->reserve(out->size() + data.size());
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
		const size_t num = dataset.positions().size();
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
traj_dataset<flt_type>& traj_manager<flt_type>::dataset (unsigned index)
{
	return *(pimpl->datasets[index].get());
}

template <class flt_type>
void traj_manager<flt_type>::clear (void)
{
	pimpl->datasets.clear();
	pimpl->dirty = true;
}

template <class flt_type>
bool traj_manager<flt_type>::has_data (void) const
{
	return !(pimpl->datasets.empty());
}

template <class flt_type>
const typename traj_manager<flt_type>::render_data& traj_manager<flt_type>::get_render_data (bool auto_tangents)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// check if the render data needs to be rebuild
	if (impl.dirty)
	{
		impl.rd.positions.clear();
		impl.rd.tangents.clear();
		impl.rd.radii.clear();
		impl.rd.colors.clear();
		impl.rd.indices.clear();
		for (const auto &d_ptr : impl.datasets)
		{
			// convencience shorthand
			const auto &dataset = *d_ptr;

			// process indices
			unsigned idx_offset = (unsigned)impl.rd.positions.size();
			std::transform(
				dataset.indices().begin(), dataset.indices().end(), std::back_inserter(impl.rd.indices),
				[idx_offset] (unsigned index) -> unsigned { return index + idx_offset; }
			);

			// copy mapped attributes, applying the desired transformations (if any)
			impl.transform_attrib_old(&impl.rd.positions, VisualAttrib::POSITION, dataset);
			impl.transform_attrib_old(&impl.rd.radii, VisualAttrib::RADIUS, dataset);
			impl.transform_attrib_old(&impl.rd.tangents, VisualAttrib::TANGENT, dataset, auto_tangents);
			impl.transform_attrib_old(&impl.rd.colors, VisualAttrib::COLOR, dataset);
			goto skip;

			// apply visual mapping to all involved attributes and distribute to user output draw arrays
			unsigned num_samples = (unsigned)dataset.positions().size();
			for (unsigned i=0; i<num_samples; i++)
			{
			}
		skip:
			/* DoNothing() */;
		}
		impl.dirty = false;
	}

	// done
	return impl.rd;
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