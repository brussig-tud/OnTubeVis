
// C++ STL
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <utility>

// arclength library
#include "arclength/bezier.h"
#include "arclength/hermite.h"

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
		Bezier<flt_type> t_to_s;
		Bezier<flt_type> s_to_t;
	} param;
};

// Anonymous namespace end
}


////
// Class implementation - arclen

// namespace open
namespace arclen {

template <class flt_type>
std::vector<cgv::math::fvec<flt_type, 4>> compile_render_data<flt_type> (const traj_manager<flt_type> &mgr)
{
	typedef flt_type real;
	typedef cgv::math::fvec<real, 4> Vec4;
	std::vector<Vec4> result;

	// obtain render attributes and dataset topology
	const auto &rd = mgr.get_render_data();
	result.reserve(rd.indices.size()/2 + 1/*safety margin*/);

	// approximate arclength
	for (unsigned ds=0; ds<rd.dataset_ranges.size(); ds++)
	{
		// obtain dataset topology
		const auto &dataset = mgr.dataset(ds);
		const auto &trajs = dataset.trajectories();
		const unsigned idx_offset_ds = rd.dataset_ranges[ds].i0;

		// approximate arclength for each trajectory in order
		for (const auto &traj : trajs)
		{
			const unsigned idx_offset = idx_offset_ds + traj.i0,
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
				// - fit bezier
				const auto alen_approx = b.arc_length_bezier_approximation();
				seg.param.t_to_s = alen_approx.get_curve();
				// - offset by current global trajectory length
				seg.param.t_to_s.points[2].y *= alen_approx.totalLength0;
				seg.param.t_to_s.points[3].y *= alen_approx.totalLength0;
				for (unsigned j=0; j<4; j++)
					seg.param.t_to_s.points[j].y += length_sum;
				/*const real l0 = seg.param.t_to_s.evaluate(0.f).y,
				           l1 = seg.param.t_to_s.evaluate(seg.param.t_to_s.points[1].x).y,
				           l2 = seg.param.t_to_s.evaluate(0.5f).y,
				           l3 = seg.param.t_to_s.evaluate(seg.param.t_to_s.points[2].x).y,
				           l4 = seg.param.t_to_s.evaluate(1.f).y;*/
				// - update global trajectory length
				length_sum += alen_approx.totalLength0;

				// output control points
				result.emplace_back(
					seg.param.t_to_s.points[0].y, seg.param.t_to_s.points[1].y,
					seg.param.t_to_s.points[2].y, seg.param.t_to_s.points[3].y
				);
			}
		}
	}

	// done
	return std::move(result);
}


//////
//
// Explicit template instantiations
//

// Only float and double variants are intended
template std::vector<cgv::math::fvec<float, 4>> compile_render_data<float>(const traj_manager<float> &mgr);
template std::vector<cgv::math::fvec<double, 4>> compile_render_data<double>(const traj_manager<double> &mgr);

// namespace close
};
