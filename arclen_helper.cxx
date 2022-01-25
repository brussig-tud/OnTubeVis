
// C++ STL
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>

// OpenMP
#include <omp.h>

// arclength library
#include "arclength/bezier.h"
#include "arclength/hermite.h"

// internal
#include "curveutils.h"

// implemented header
#include "arclen_helper.h"

////
// Local types and variables

// anonymous namespace begin
namespace {

// single 3rd degree curve segment
template <class flt_type>
struct curve_segment
{
	unsigned i0, i1;
	struct {
		Bezier<flt_type> t_to_s [4]; /// 4 cubic Bezier segments approximate a the arclength with respect to t
	} param;
	flt_type alen (flt_type t=1)
	{
		const flt_type _4t = 4*t,
		               _4tb4 = _4t / 4,
		               _4tb4x4 = _4tb4 * 4;
		const unsigned i = std::clamp((int)_4tb4x4, 0, 3);
		const flt_type inner_start = i*flt_type(0.25),
		               t_inner = (t-inner_start)*flt_type(4);
		const auto &approx = param.t_to_s[i];
		const auto result = approx.evaluate(t_inner).y;
		return result;
	}
};

// Anonymous namespace end
}


////
// Class implementation - arclen

// namespace open
namespace arclen {

template <class flt_type>
std::vector<cgv::render::render_types::mat4> compile_renderdata<flt_type> (const traj_manager<flt_type> &mgr)
{
	typedef flt_type real;
	typedef cgv::math::fvec<real, 4> rvec4;
	std::vector<cgv::render::render_types::mat4> result;

	// obtain render attributes and dataset topology
	const auto &rd = mgr.get_render_data();
	result.resize(rd.indices.size()/2);

	// approximate arclength for all datasets
	for (const auto &dataset : rd.datasets)
	{
		// approximate arclength for each trajectory in order
		//for (const auto &traj : trajs)
#pragma omp parallel for
		for (int traj_idx=0; traj_idx<dataset.trajs.size(); traj_idx++)
		{
			const auto &traj = dataset.trajs[traj_idx];
			const unsigned idx_offset = dataset.irange.i0 + traj.i0,
			               idx_n = idx_offset + traj.n;
			// compute individual segment global (w.r.t. current trajectory) arclength approximation 
			real length_sum = 0;
			for (unsigned i=idx_offset; i<idx_n; i+=2)
			{
				// global segment node indices
				curve_segment<real> seg{rd.indices[i], rd.indices[i+1]};
				// segment geometry
				const Bezier<real> b = Hermite<real>(
					rd.positions[seg.i0], rd.positions[seg.i1],
					vec3_from_vec4(rd.tangents[seg.i0]), vec3_from_vec4(rd.tangents[seg.i1])
				).to_bezier();

				// arc length
				// - fit beziers
				const auto alen_approx = b.arc_length_bezier_approximation(4);
				// - convert to calculation of trajectory-global arclength at segment
				for (unsigned j=0; j<4; j++) {
					const flt_type length_j = alen_approx.lengths[j+1] - alen_approx.lengths[j];
					const flt_type length_jsum = length_sum + alen_approx.lengths[j];
					seg.param.t_to_s[j].points[0].y = length_jsum;
					seg.param.t_to_s[j].points[1].y = length_jsum + alen_approx.y1[j]*length_j;
					// - offset by current global trajectory length
					seg.param.t_to_s[j].points[2].y = length_jsum + alen_approx.y2[j]*length_j;
					seg.param.t_to_s[j].points[3].y = length_sum  + alen_approx.lengths[j+1];
				}
				// - update global trajectory length
				length_sum += alen_approx.totalLength;
				// - testing
				/*const real _l[] = {seg.alen(0.f),
				                   seg.alen(0.125f),
				                   seg.alen(0.25f),
				                   seg.alen(0.375f),
				                   seg.alen(0.5f),
				                   seg.alen(0.625f),
				                   seg.alen(0.75f),
				                   seg.alen(0.875f),
				                   seg.alen(1.f)};*/

				// output control points
				// - construct temp array
				const float tmp[16] = {
					/* col1 */ (float)seg.param.t_to_s[0].points[0].y, (float)seg.param.t_to_s[0].points[1].y,
					           (float)seg.param.t_to_s[0].points[2].y, (float)seg.param.t_to_s[0].points[3].y,
					/* col2 */ (float)seg.param.t_to_s[1].points[0].y, (float)seg.param.t_to_s[1].points[1].y,
					           (float)seg.param.t_to_s[1].points[2].y, (float)seg.param.t_to_s[1].points[3].y,
					/* col3 */ (float)seg.param.t_to_s[2].points[0].y, (float)seg.param.t_to_s[2].points[1].y,
					           (float)seg.param.t_to_s[2].points[2].y, (float)seg.param.t_to_s[2].points[3].y,
					/* col4 */ (float)seg.param.t_to_s[3].points[0].y, (float)seg.param.t_to_s[3].points[1].y,
					           (float)seg.param.t_to_s[3].points[2].y, (float)seg.param.t_to_s[3].points[3].y
				};
				// - in-place construct matrix in list
				std::copy_n(tmp, 16, (float*)result[i/2]);
			}
		}
	}

	// done
	return result;
}

cgv::render::vertex_buffer upload_renderdata (
	cgv::render::context& ctx, const std::vector<cgv::render::render_types::mat4> &approximations
)
{
	// init new buffer object
	cgv::render::vertex_buffer new_sbo(cgv::render::VBT_STORAGE, cgv::render::VBU_STATIC_READ);
	if (!new_sbo.create(ctx, approximations))
		std::cerr << "[arclen::upload_renderdata] !!! unable to create Storage Buffer Object !!!" <<std::endl<<std::endl;

	// done
	return std::move(new_sbo);
}

float eval (const cgv::render::render_types::mat4 &approx, float t)
{
	// determine sub-segment and 
	const float t4 = t+t+t+t;
	const unsigned seg = std::max(std::min((signed)t4, 3), 0);
	const float t_inner = t4 - (float)seg;
	return evalBezier(approx.col(seg), t_inner);
}


//////
//
// Explicit template instantiations
//

// Only float and double variants are intended
template std::vector<cgv::render::render_types::mat4> compile_renderdata<float>(const traj_manager<float>&);
template std::vector<cgv::render::render_types::mat4> compile_renderdata<double>(const traj_manager<double>&);

// namespace close
};
