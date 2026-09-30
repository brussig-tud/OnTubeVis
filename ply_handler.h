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


/// handler for loading trajectories from .ply files
template <class flt_type>
struct ply_handler : public traj_format_handler<flt_type>
{
	/// super class shorthand
	using super = traj_format_handler<flt_type>;

	/// real number type
	typedef typename traj_format_handler<flt_type>::real real;

	/// 2D vector type
	typedef typename traj_format_handler<flt_type>::Vec2 Vec2;

	/// 3D vector type
	typedef typename traj_format_handler<flt_type>::Vec3 Vec3;

	/// 4D vector type
	typedef typename traj_format_handler<flt_type>::Vec4 Vec4;

	/// rgb color type
	typedef typename traj_format_handler<flt_type>::Color Color;


private:

	/// internal implementation
	struct Impl;


public:

	/// reports the name "PLY"
	const std::string& format_name (void) const;

	/// reports a list with just "ply" in it - the handler will always claim files with this extension
	const std::vector<std::string>& handled_extensions (void) const;

	/// test if the given data stream appears to be a .ply file
	virtual bool can_handle (std::istream &contents) const;

	/// parse the given stream containing the .ply file contents and report whether any data was loaded
	virtual traj_dataset<flt_type> read (std::istream &contents, DatasetOrigin source, const std::string &path);
};
