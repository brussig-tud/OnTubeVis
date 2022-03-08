#pragma once

// C++ STL
#include <iostream>

// CGV framework core
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/media/color.h>

// local includes
#include "traj_loader.h"


/// provides read capabilites for TGEN random trajectory dataset specification files
template <class flt_type>
struct sepia_handler : public traj_format_handler<flt_type>
{
	/// real number type
	typedef traj_format_handler::real real;

	/// 2D vector type
	typedef traj_format_handler::Vec2 Vec2;

	/// 3D vector type
	typedef traj_format_handler::Vec3 Vec3;

	/// 4D vector type
	typedef traj_format_handler::Vec4 Vec4;

	/// rgb color type
	typedef traj_format_handler::Color Color;


private:

	/// internal implementation
	struct Impl;


public:

	/// test if the given data stream appears to be a .tgen file
	virtual bool can_handle (std::istream &contents) const;

	/// parse the given stream containing the .tgen file contents and report whether any data was loaded
	virtual traj_dataset<real> read (std::istream &contents);
};
