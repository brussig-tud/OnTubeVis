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


/// provides read capabilites for Trello flight logs in .csv format
template <class flt_type>
class tellocsv_handler : public traj_format_handler<flt_type>
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


private:

	// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


public:

	
	/// reports the known file extensions the handler will claim regardless of content.
	const std::vector<std::string>& handled_extensions (void) const;

	/// test if the given data stream appears to be a Trello flight log file in a supported .csv format
	virtual bool can_handle (std::istream &contents) const;

	/// parse the given stream containing the file contents and report whether any data was loaded
	virtual traj_dataset<flt_type> read (std::istream &contents, DatasetOrigin source, const std::string &path);
};
