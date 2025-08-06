#pragma once

// C++ STL
#include <span>
#include <vector>

// CGV framework core
#include <cgv/render/context.h>
#include <cgv/render/vertex_buffer.h>

// Local includes
#include "traj_loader.h"


/// helper functions for generating arc length estimations and parametrizations for trajectories loaded
/// via the trajectory manager
namespace otv::arclen
{
	/// encapsulates t→s and s→t maps of an arclength parametrization of a Hermite spline, represented by
	/// 4-segment cubic Bezier splines (stored in a 4x4 matrix) for each Hermite spline segment and mapping
	/// direction. The data is ordered in accordance with the index pairs in the render data of the trajectory
	/// manager, so renderers can access the computed approximations using e.g. a storage buffer and the
	/// current primitive ID.
	struct parametrization
	{
		/// Array of 4-segment cubic Bezier splines (the 4 sets of coefficients are stored in the
		/// columns of a 4x4 matrix) representing the t→s mapping for each Hermite spline segment.
		std::vector<cgv::mat4> t_to_s;

		/// Array of 4-segment cubic Bezier splines (the 4 sets of coefficients are stored in the
		/// columns of a 4x4 matrix) representing the s→t mapping for each Hermite spline segment.
		/// ATTENTION: for reasons of efficiency, the mapping is local to each segment - an arc length 0
		/// will be mapped to t=0 and and the full segment arc length will be mapped to t=1
		std::vector<cgv::mat4> s_to_t;
	};

	/// compute the t-to-s parameterization for a single <b>linear</b> curve segment which is known to be @a dist
	/// units long, and apply the given offset to the parameterization. This is very fast compared to @ref
	/// compute_single_t_to_s().
	template <class flt_type>
	cgv::math::fmat<flt_type, 4, 4> single_linear_t_to_s (flt_type dist, flt_type offset);

	/// compute the t-to-s parameterization for a single curve segment, applying the specified offset to all values of s
	/// the parameterization yiels.
	template <class flt_type> cgv::math::fmat<flt_type, 4, 4> compute_single_t_to_s (
		const typename traj_manager<flt_type>::render_data::Vec3 &pos0,
		const typename traj_manager<flt_type>::render_data::Vec3 &tan0,
		const typename traj_manager<flt_type>::render_data::Vec3 &pos1,
		const typename traj_manager<flt_type>::render_data::Vec3 &tan1,
		const flt_type offset
	);

	/// compute arclength approximation for all trajectory segments of all loaded datasets.
	template <class flt_type> parametrization compute_parametrization (const traj_manager<flt_type> &mgr);

	/// Compute a linear arclength parametrization for all segments in the given dataset.
	/// Sufficiently accurate for densly sampled data, but much faster than the default computation.
	template <class flt_type>
	void linear_parametrization (
		typename traj_manager<flt_type>::render_data const&,
		typename traj_manager<flt_type>::render_data::dataset const&,
		std::span<cgv::mat4> t_to_s,
		std::span<cgv::mat4> s_to_t
	);

	/// create a GPU-side storage buffer holding the approximate t→s arclength mapping
	cgv::render::vertex_buffer upload_renderdata (
		cgv::render::context &ctx, const std::vector<cgv::mat4> &approximations
	);

	/// evaluate the given arclength approximation for some segment curve parameter t
	float eval (const cgv::mat4 &approx, float t);

	/// evaluate the given arclength parametrization for some local, normalized (i.e. between 0 and 1) arc
	/// length s, returning the corresponding segment curve parameter t.
	inline float map (const cgv::mat4 &param, float s) { return eval(param, s); }

	/// evaluate the given arclength parametrization for some local arc length s, returning the corresponding
	/// segment curve parameter t. Needs the segment length to internally scale s between 0 and 1.
	inline float map (const cgv::mat4 &param, float seg_length, float s) {
		return eval(param, s/seg_length);
	}

	/// evaluate the given arclength parametrization for some global arc length s, returning the corresponding
	/// segment curve parameter t. Needs the global arclength approximation for the segment to make sense of the
	/// value of s.
	inline float map (const cgv::mat4 &approx, const cgv::mat4 &param, float s) {
		const float s0 = approx[0];
		return map(param, approx[15]-s0, s-s0);
	}
};
