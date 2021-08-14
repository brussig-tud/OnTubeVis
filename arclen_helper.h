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
	/// Compute arclength approximation for all trajectory segments of all loaded datasets. The result
	/// is one 4-vector per segment, which encode the 4 control values of a Bezier curve approximating
	/// the segment arclength. The data is ordered in accordence with the index pairs in the render
	/// data of the trajectory manager, so the corresponding attribute indices can be found by doubling
	/// the segment index, and renderers can access the computed approximations using e.g. a storage
	/// buffer and the current primitive ID.
	template <class flt_type>
	std::vector<cgv::render::render_types::vec4> compile_renderdata (const traj_manager<flt_type> &mgr);

	/// Create a GPU-side storage buffer holding the per-segment arclength approximations contained in
	/// the provided array of vec4's.
	cgv::render::vertex_buffer upload_renderdata (
		cgv::render::context &ctx, const std::vector<cgv::render::render_types::vec4> &approximations
	);

	/// convencience function for compiling and uploading the arclength approximation render data in one
	/// go
	template <class flt_type>
	cgv::render::vertex_buffer compile_and_upload_renderdata (cgv::render::context &ctx,
	                                                          const traj_manager<flt_type> &mgr)
	{
		return std::move(upload_renderdata(ctx, compile_renderdata(mgr)));
	}
};