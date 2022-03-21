#pragma once

// C++ STL
#include <vector>

// CGV framework core
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>
#include <cgv/render/render_types.h>

// Local includes
#include "traj_loader.h"


/// helper functions for generating arc length estimations and parametrizations for trajectories loaded
/// via the trajectory manager
namespace arclen
{
	/// encapsulates t_to_s and s_to_t maps of an arclength parametrization of a Hermite spline,
	/// represented by one 4-segment cubic Bezier splines (stored in a 4x4 matrix) per Hermite spline
	/// segment. The data should be ordered in accordance with the index pairs in the render data of the
	/// trajectory manager, so the corresponding attribute indices can be found by doubling the segment
	/// index, and renderers can access the computed approximations using e.g. a storage buffer and the
	/// current primitive ID.
	struct parametrization
	{
		/// Array of 4-segment cubic Bezier splines (the 4 sets of coefficients are stored in the
		/// columns of a 4x4 matrix) representing the t→s mapping for each Hermite spline segment.
		std::vector<cgv::render::render_types::mat4> t_to_s;

		/// Array of 4-segment cubic Bezier splines (the 4 sets of coefficients are stored in the
		/// columns of a 4x4 matrix) representing the s→t mapping for each Hermite spline segment.
		std::vector<cgv::render::render_types::mat4> s_to_t;
	};

	/// compute arclength approximation for all trajectory segments of all loaded datasets. The result
	/// is one 4x4 matrix per segment, which encodes a 4-segment cubic Bezier spline approximating
	/// the segment arclength. The data is ordered in accordance with the index pairs in the render
	/// data of the trajectory manager, so the corresponding attribute indices can be found by doubling
	/// the segment index, and renderers can access the computed approximations using e.g. a storage
	/// buffer and the current primitive ID.
	template <class flt_type>
	parametrization compute_parametrization (const traj_manager<flt_type> &mgr);

	/// create a GPU-side storage buffer holding the t→s arclength parametrization
	cgv::render::vertex_buffer upload_renderdata (
		cgv::render::context &ctx, const std::vector<cgv::render::render_types::mat4> &approximations
	);

	/// evaluate the given arclength approximation for some curve parameter t
	float eval (const cgv::render::render_types::mat4 &approx, float t);
};
