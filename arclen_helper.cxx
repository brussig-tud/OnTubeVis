
// C++ STL
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>

// OpenMP
#ifdef OMP_SUPPORT
	#include <omp.h>
#endif

// CGV Framework
#include <cgv/math/functions.h>

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
		Bezier<flt_type> t_to_s [4]; // 4 cubic Bezier segments approximating the arclength with respect to t
		Bezier<flt_type> s_to_t [4]; // 4 cubic Bezier segments approximating the inverse mapping of t_to_s
	} param;
};

// Anonymous namespace end
}


////
// Class implementation - arclen

// namespace open
namespace arclen {

template <class flt_type>
parametrization compute_parametrization (const traj_manager<flt_type> &mgr)
{
	typedef flt_type real;
	typedef typename traj_manager<flt_type>::render_data::Vec3 vec3;
	typedef typename traj_manager<flt_type>::render_data::Vec4 vec4;
	std::vector<cgv::render::render_types::mat4> t_to_s, s_to_t;

	// obtain render attributes and dataset topology
	const auto &rd = mgr.get_render_data();
	t_to_s.resize(rd.indices.size()/2);
	s_to_t.resize(rd.indices.size()/2);

	// approximate arclength for all datasets
	for (const auto &dataset : rd.datasets)
	{
		// approximate arclength for each trajectory in order
		#pragma omp parallel for
		for (int traj_idx=0; traj_idx<dataset.trajs.size(); traj_idx++)
		{
			const auto &traj = dataset.trajs[traj_idx];
			const unsigned idx_offset = dataset.irange.i0 + traj.i0,
			               idx_n = idx_offset + traj.n;
			// compute individual segment global (w.r.t. current trajectory) arclength approximation 
			real length_sum = 0;
			for (unsigned i=idx_offset, k=0; i<idx_n; i+=2, k++)
			{
				// global segment node indices
				curve_segment<real> seg{rd.indices[i], rd.indices[i+1]};
				// segment geometry
				const Bezier<real> b = Hermite<real>(
					rd.positions[seg.i0], rd.positions[seg.i1],
					vec3(rd.tangents[seg.i0]), vec3(rd.tangents[seg.i1])
				).to_bezier();

				// --- UNCOMMENT BELOW FOR GETTING COPY-PASTABLE ARRAY OF PRECISE ARCLENGTH SAMPLES IN YOUR DEBUGGER ---
				/*#define NUM_LG_SAMPLES 64
				real _t[NUM_LG_SAMPLES], _l[NUM_LG_SAMPLES];
				for (unsigned K=0; K< NUM_LG_SAMPLES; K++) {
					_t[K] = real(K)/ real(NUM_LG_SAMPLES-1);
					_l[K] = b.arc_length_legendre_gauss(_t[K], 8192);
				}*/

				// arc length
				// - fit beziers
				const auto alen_approx = b.arc_length_bezier_approximation(4);
				const auto inv_approx = b.parameterization_bezier_approximation(alen_approx);
				// - convert to trajectory-global arclength at segment
				for (unsigned j=0; j<4; j++) {
					// - important values
					const real length_j = alen_approx.lengths[j+1] - alen_approx.lengths[j],
					           length_jsum = length_sum + alen_approx.lengths[j],
					           dt_j = inv_approx.t[j+1] - inv_approx.t[j],
					           inv_approx_tj = inv_approx.t[j];
					// - offset arclength by current trajectory length
					seg.param.t_to_s[j].points[0].y = length_jsum;
					seg.param.t_to_s[j].points[1].y = length_jsum + alen_approx.y1[j]*length_j;
					seg.param.t_to_s[j].points[2].y = length_jsum + alen_approx.y2[j]*length_j;
					seg.param.t_to_s[j].points[3].y = length_sum  + alen_approx.lengths[j+1];
					// - offset t by current trajectory segment number
					seg.param.s_to_t[j].points[0].y = inv_approx_tj;
					seg.param.s_to_t[j].points[1].y = inv_approx_tj + inv_approx.y1[j]*dt_j;
					seg.param.s_to_t[j].points[2].y = inv_approx_tj + inv_approx.y2[j]*dt_j;
					seg.param.s_to_t[j].points[3].y = inv_approx.t[j+1];
				}
				seg.param.s_to_t[3].points[3].y = 1; // snap it to exactly 1
				// - update global trajectory length
				length_sum += alen_approx.totalLength;

				// output control points
				// - construct temp storage
				const float tmp[32] = {
					/* col1 */ (float)seg.param.t_to_s[0].points[0].y, (float)seg.param.t_to_s[0].points[1].y,
					           (float)seg.param.t_to_s[0].points[2].y, (float)seg.param.t_to_s[0].points[3].y,
					/* col2 */ (float)seg.param.t_to_s[1].points[0].y, (float)seg.param.t_to_s[1].points[1].y,
					           (float)seg.param.t_to_s[1].points[2].y, (float)seg.param.t_to_s[1].points[3].y,
					/* col3 */ (float)seg.param.t_to_s[2].points[0].y, (float)seg.param.t_to_s[2].points[1].y,
					           (float)seg.param.t_to_s[2].points[2].y, (float)seg.param.t_to_s[2].points[3].y,
					/* col4 */ (float)seg.param.t_to_s[3].points[0].y, (float)seg.param.t_to_s[3].points[1].y,
					           (float)seg.param.t_to_s[3].points[2].y, (float)seg.param.t_to_s[3].points[3].y,
					/* col1 */ (float)seg.param.s_to_t[0].points[0].y, (float)seg.param.s_to_t[0].points[1].y,
					           (float)seg.param.s_to_t[0].points[2].y, (float)seg.param.s_to_t[0].points[3].y,
					/* col2 */ (float)seg.param.s_to_t[1].points[0].y, (float)seg.param.s_to_t[1].points[1].y,
					           (float)seg.param.s_to_t[1].points[2].y, (float)seg.param.s_to_t[1].points[3].y,
					/* col3 */ (float)seg.param.s_to_t[2].points[0].y, (float)seg.param.s_to_t[2].points[1].y,
					           (float)seg.param.s_to_t[2].points[2].y, (float)seg.param.s_to_t[2].points[3].y,
					/* col4 */ (float)seg.param.s_to_t[3].points[0].y, (float)seg.param.s_to_t[3].points[1].y,
					           (float)seg.param.s_to_t[3].points[2].y, (float)seg.param.s_to_t[3].points[3].y
				};
				// - in-place construct matrices in list
				std::copy_n(tmp,    16, (float*)t_to_s[i/2]);
				std::copy_n(tmp+16, 16, (float*)s_to_t[i/2]);
				// - testing
				/*{ const auto &cur_t2s = t_to_s[i/2], &cur_s2t = s_to_t[i/2];
				  // t, approximated s, t from s-parametrization, error
				  const float t[5] = {0,                           1.f/3,                       .5f,                         2.f/3,                       1},
				              s[5] = {eval(cur_t2s, t[0]),         eval(cur_t2s, t[1]),         eval(cur_t2s, t[2]),         eval(cur_t2s, t[3]),         eval(cur_t2s, t[4])},
				              T[5] = {map(cur_t2s, cur_s2t, s[0]), map(cur_t2s, cur_s2t, s[1]), map(cur_t2s, cur_s2t, s[2]), map(cur_t2s, cur_s2t, s[3]), map(cur_t2s, cur_s2t, s[4])},
				              e[5] = {T[0]-t[0],                   T[1]-t[1],                   T[2]-t[2],                   T[3]-t[3],                   T[4]-t[4]};
				  std::cerr.flush(); }*/
			}
		}
	}

	// done
	return {std::move(t_to_s), std::move(s_to_t)};
}

cgv::render::vertex_buffer upload_renderdata (cgv::render::context& ctx, const std::vector<cgv::render::render_types::mat4> &approximations)
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
	// determine sub-segment
	const float t4 = t+t+t+t;
	const unsigned seg = cgv::math::clamp((signed)t4, 0, 3);
	const float t_inner = t4 - (float)seg;
	return evalBezier(approx.col(seg), t_inner);
}


//////
// Explicit template instantiations

// Only float and double variants are intended
template parametrization compute_parametrization<float>(const traj_manager<float>&);
template parametrization compute_parametrization<double>(const traj_manager<double>&);

// namespace close
};
