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


/// provides read and write capabilites for Hermite splines in .bezdat format
template <class flt_type>
class attrib_handle_manager
{

public:

	/// real number type
	typedef typename traj_format_handler<flt_type>::real real;

	/// 2D vector type
	typedef typename traj_format_handler<real>::Vec2 vec2;

	/// 3D vector type
	typedef typename traj_format_handler<real>::Vec3 vec3;

	/// 4D vector type
	typedef typename traj_format_handler<real>::Vec4 vec4;

	/// rgb color type
	typedef typename traj_format_handler<real>::Color color;


private:

	/// implementation forward
	struct Impl;

	/// implementation handle
	Impl *pimpl;


protected:

	/// cleanup logic
	void clear (void);


public:

	/// default constructor
	attrib_handle_manager();

	/// the destructor
	~attrib_handle_manager();

	/// assign a dataset to handle
	void set_dataset (const traj_dataset<real> &dataset);
};
