#pragma once

// C++ STL
#include <vector>

// CGV framework core
#include <cgv/math/fvec.h>

// Local includes
#include "traj_loader.h"


/// helper functions for generating arc length estimations and parametrizations for trajectories loaded
/// via the trajectory manager
namespace arclen
{
	/// Compute arclength approximation for all trajectory segments of all loaded datasets. The result
	/// is one 4-vector per segment, which encode the 4 control values of a Bezier curve approximating
	/// the segment arclength. The data is ordered in accordence with the index pairs in the render
	/// data of the trajectory manager, so the corresponding attribute indices can be found by doubling
	/// the segment index, and renderers can access the computed approximations using e.g. a storage
	/// buffer and the current primitive ID.
	template <class flt_type>
	std::vector<cgv::math::fvec<flt_type, 4>> compile_render_data (const traj_manager<flt_type> &mgr);
};
