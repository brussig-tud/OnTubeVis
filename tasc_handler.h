#pragma once

// C++ STL
#include <iostream>
#include <vector>

// CGV framework core
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/media/color.h>

// local includes
#include "traj_loader.h"


/// provides read capabilites for the TASC accident dataset.
template <class flt_type>
class tasc_handler : public traj_format_handler<flt_type>
{

public:

	/// real number type
	typedef typename traj_format_handler<flt_type>::real real;

	/// 2D vector type
	typedef typename traj_format_handler<flt_type>::Vec2 vec2;

	/// 3D vector type
	typedef typename traj_format_handler<flt_type>::Vec3 vec3;

	/// 4D vector type
	typedef typename traj_format_handler<flt_type>::Vec4 vec4;

	/// rgb color type
	typedef typename traj_format_handler<flt_type>::Color color;

	/// implementation forward
	struct Impl;

	/// reports the name "TASC JSON"
	const std::string& format_name (void) const;

	/// reports the known TASC file extensions the handler will claim regardless of content.
	const std::vector<std::string>& handled_extensions (void) const;

	/// test if the given data stream appears to be a supported TASC data file
	virtual bool can_handle (std::istream &contents) const;

	/// parse the given stream containing the file contents and report whether any data was loaded
	virtual traj_dataset<flt_type> read (std::istream &contents, DatasetOrigin source, const std::string &path);


protected:

	// proxy for the internal implementation class to be able to add attributes to the result dataset object
	template <class T>
	inline static typename traj_dataset<flt_type>::template attrib_info<T> add_attribute(traj_dataset<real> &ds, const std::string &name) {
		return traj_format_handler<real>::template add_attribute<T>(ds, name);
	}

	// proxy for the internal implementation class to be able to manipulate trajectory info in the result dataset object
	inline static std::vector<range>& trajectories (traj_dataset<real> &ds, const traj_attribute<real> &attrib) {
		return traj_format_handler<real>::trajectories(ds, attrib);
	}
};
