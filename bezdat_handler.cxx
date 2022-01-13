
// C++ STL
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <utility>
#include <limits>

// CGV framework core
#include <cgv/base/register.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>

// local includes
#include "regulargrid.h"

// implemented header
#include "bezdat_handler.h"


/////
// Some constant defines

/// multiple of the user-supplied distance unit for use as a tolerance to decide whether two node position values are similar or not
#define BEZDAT_SIM_TOLERANCE_POS (real(32)*std::numeric_limits<real>::epsilon())

/// multiple of the user-supplied distance unit for use as a tolerance to decide whether two node tangents are similar or not
#define BEZDAT_SIM_TOLERANCE_DPOS real(0.25)

/// multiple of the user-supplied distance unit for use as a tolerance to decide whether two node radius values are similar or not
#define BEZDAT_SIM_TOLERANCE_RAD std::numeric_limits<real>::infinity() //(real(128)*std::numeric_limits<real>::epsilon())

/// multiple of the user-supplied distance unit for use as a tolerance to decide whether two node radius derivatives are similar or not
#define BEZDAT_SIM_TOLERANCE_DRAD std::numeric_limits<real>::infinity() //real(0.125)

/// multiple of the user-supplied distance unit for use as a tolerance to decide whether two node color values are similar or not
#define BEZDAT_SIM_TOLERANCE_COL std::numeric_limits<real>::infinity() //(real(1024)*std::numeric_limits<real>::epsilon())

/// multiple of the user-supplied color distance unit for use as a tolerance to decide whether two node color derivatives are similar or not
#define BEZDAT_SIM_TOLERANCE_DCOL std::numeric_limits<real>::infinity() //real(0.125)

/// identifyier to use for position data
#define BEZDAT_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define BEZDAT_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define BEZDAT_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define BEZDAT_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color data
#define BEZDAT_DCOLOR_ATTRIB_NAME "dcolor"


////
// Local types and variables

// anonymous namespace begin
namespace {

/// one point in a bezdat dataset
template <class flt_type>
struct bezdat_point
{
	typedef flt_type real;
	typedef typename bezdat_handler<real>::Vec3 Vec3;
	Vec3 pos, color;
	real radius;
};

/// one segment in a bezdat dataset
struct bezdat_segment
{
	unsigned idx[4];
};

/// one Hermite position node directly converted from a bezdat dataset
template <class flt_type>
struct bezdat_node
{
	// Types
	typedef flt_type real;
	typedef typename bezdat_point<real>::Vec3 Vec3;

	// Data members
	Vec3 pos, dpos, col, dcol;
	real rad, drad;

	// Methods
	bezdat_node() {};
	bezdat_node(unsigned i_pnt, unsigned i_aux, const bezdat_segment &seg,
	            const std::vector<bezdat_point<real> > &points)
	{
		pos = points[seg.idx[i_pnt]].pos;
		col = points[seg.idx[i_pnt]].color;
		rad = points[seg.idx[i_pnt]].radius;
		if (i_aux & 0x1)
		{
			dpos = (points[seg.idx[i_aux]].pos-points[seg.idx[i_pnt]].pos) * real(3);
			dcol =   (points[seg.idx[i_aux]].color-points[seg.idx[i_pnt]].color) * real(3);
			drad = (points[seg.idx[i_aux]].radius-points[seg.idx[i_pnt]].radius) * real(3);
		}
		else
		{
			dpos = (points[seg.idx[i_pnt]].pos-points[seg.idx[i_aux]].pos) * real(3);
			dcol =   (points[seg.idx[i_pnt]].color-points[seg.idx[i_aux]].color) * real(3);
			drad = (points[seg.idx[i_pnt]].radius-points[seg.idx[i_aux]].radius) * real(3);
		}
	}

	bool is_similar (const bezdat_node<real> &other, real pUnit=1, real rUnit=1, real cUnit=1) const
	{
		// ToDo: actually parametrize value and angle tolerances
		const real
			diff_pos=(pos - other.pos).sqr_length(),
			thr_pos=BEZDAT_SIM_TOLERANCE_POS*BEZDAT_SIM_TOLERANCE_POS * pUnit*pUnit,
			diff_dpos=(dpos - other.dpos).sqr_length(),
			thr_dpos=BEZDAT_SIM_TOLERANCE_DPOS*BEZDAT_SIM_TOLERANCE_DPOS * pUnit*pUnit,
			diff_rad=(rad - other.rad),
			thr_rad=BEZDAT_SIM_TOLERANCE_RAD * rUnit,
			diff_drad=std::abs(drad - other.drad),
			thr_drad=BEZDAT_SIM_TOLERANCE_DRAD * rUnit,
			diff_col=(col - other.col).sqr_length(),
			thr_col=BEZDAT_SIM_TOLERANCE_COL*BEZDAT_SIM_TOLERANCE_COL * cUnit*cUnit,
			diff_dcol=(dcol - other.dcol).sqr_length(),
			thr_dcol=BEZDAT_SIM_TOLERANCE_DCOL*BEZDAT_SIM_TOLERANCE_DPOS * cUnit*cUnit;

		return   (diff_pos <= thr_pos) && (diff_rad <= thr_rad) && (diff_col <= thr_col)
		      && (diff_dpos<=thr_dpos) && (diff_drad<=thr_drad) && (diff_dcol<=thr_dcol);
	}
};

/// one Hermite segment
union hermite_segment
{
	struct {
		/// index of the first node of the segment
		unsigned n0;

		/// index of the second node of the segment
		unsigned n1;
	};

	/// indices of the first and second node of the segment
	unsigned n[2];
};

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
struct Impl { static const visual_attribute_mapping<flt_type> attrmap; };
template <class flt_type>
const visual_attribute_mapping<flt_type> Impl<flt_type>::attrmap({
	{VisualAttrib::POSITION, {BEZDAT_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {BEZDAT_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {BEZDAT_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {BEZDAT_COLOR_ATTRIB_NAME}}
});

template <class flt_type>
bool bezdat_handler<flt_type>::can_handle (std::istream &contents) const
{
	std::string str;
	stream_pos_guard g(contents);

	// check for tell-tale stream contents
	// - .bezdat header
	std::getline(contents, str);
	if (cgv::utils::to_lower(str).compare("bezdata 1.0") != 0)
	{
		//std::cout << "bezdat_handler: first line in stream must be \"BezDatA 1.0\", but found \"" << str << "\" instead!" << std::endl;
		return false;
	}
	// - check if rest of the file looks valid on first glance
	do { std::getline(contents, str); } while (!contents.eof() && str.empty());
	str = std::move(cgv::utils::to_upper(str.substr(0, 2)));
	return str.compare("PT")==0 || str.compare("BC")==0;
}

template <class flt_type>
traj_dataset<flt_type> bezdat_handler<flt_type>::read (std::istream &contents)
{
	// bezdat database
	std::vector<bezdat_point<real> > points;
	std::vector<bezdat_segment> bsegs;
	real avgNodeDist=0, avgRadiusDiff=0, avgColorDiff=0;

	// parse the stream until EOF
	std::string str;
	contents >> str;
	while (!contents.eof())
	{
		if (cgv::utils::to_upper(str).compare("PT") == 0)
		{
			// read the control point
			points.emplace_back(); bezdat_point<real> &new_point = points.back();
			contents >> new_point.pos;
			contents >> new_point.radius;
			contents >> new_point.color;
		}
		else if (cgv::utils::to_upper(str).compare("BC") == 0)
		{
			// read the segment
			bsegs.emplace_back(); bezdat_segment &new_segment = bsegs.back();
			contents >> new_segment.idx[0] >> new_segment.idx[1] >> new_segment.idx[2] >> new_segment.idx[3];
			// update estimate of average per-attribute node distances
			real curNodeDist = (  points[new_segment.idx[0]].pos
			                    - points[new_segment.idx[3]].pos).length(),
			     curRadiusDiff = std::abs(  points[new_segment.idx[3]].radius
			                              - points[new_segment.idx[0]].radius),
			     curColorDiff = (  points[new_segment.idx[0]].color
			                     - points[new_segment.idx[3]].color).length();
			const size_t num = bsegs.size();
			avgNodeDist =
				(real(num-1)*avgNodeDist + curNodeDist) / real(num);
			avgRadiusDiff =
				(real(num-1)*avgRadiusDiff + curRadiusDiff) / real(num);
			avgColorDiff =
				(real(num-1)*avgColorDiff + curColorDiff) / real(num);
		}
		contents >> str;
	}
	avgRadiusDiff = std::max(avgRadiusDiff, std::numeric_limits<real>::epsilon());
	avgColorDiff = std::max(avgColorDiff, std::numeric_limits<real>::epsilon());

	// build database of Hermite node data
	std::vector<bezdat_node<real> > nodes;
	std::list<std::vector<hermite_segment> > trajs;
	nodes.reserve(points.size());
	grid3D<real> nodes_db(
		avgNodeDist*real(2)*BEZDAT_SIM_TOLERANCE_POS,
		[&nodes] (Vec3 *pnt, size_t id) -> bool {
			if (id < nodes.size())
			{
				*pnt = nodes[id].pos;
				return true;
			}
			else
				return false;
		}
	);

	// merge similar nodes and determine individual trajectories
	std::vector<std::vector<bezdat_node<real> > > merge_log;
	merge_log.resize(points.size());
	std::vector<hermite_segment> *cur_traj_segments = nullptr;
	for (const auto &bseg : bsegs)
	{
		// prepere new hermite segment
		hermite_segment new_seg;

		// unique-ify this control point combination
		for (unsigned i=0; i<2; i++)
		{
			// determine which is outer and which is inner control point
			const unsigned i_pnt = i*2+i, i_aux = i*2+1-i;
			const bezdat_point<real> &pnt = points[bseg.idx[i_pnt]];

			// build list of nodes close to the one we're about to construct
			std::vector<size_t> neighbors;
			nodes_db.query(&neighbors, pnt.pos, true);

			// check if we can reuse a node
			bool not_found = true;
			bezdat_node<real> new_node(i_pnt, i_aux, bseg, points);
			for (size_t nb_id : neighbors)
			{
				if (new_node.is_similar(nodes[nb_id], avgNodeDist, avgRadiusDiff, avgColorDiff))
				{
					// node already exists, log the duplicate and reference it
					merge_log[nb_id].emplace_back(std::move(new_node));
					new_seg.n[i] = (unsigned)nb_id;
					not_found = false;
					break;
				}
			}
			if (not_found)
			{
				// this node is not yet in the database
				// - determine index of new node
				unsigned id = (unsigned)nodes.size();
				// - store the new node
				nodes.emplace_back(new_node);
				// - reference the new node
				nodes_db.insert(id);
				new_seg.n[i] = id;
				// - check if this looks like a new curve (indicated by discontinuity at
				//   first node of the segment)
				if (i==0)
				{
					trajs.emplace_back();
					cur_traj_segments = &(trajs.back());
				}
			}
		}

		// commit new hermite segment
		cur_traj_segments->emplace_back(new_seg);
	}

	// move merged nodes to position of cluster average
	for (unsigned i=0; i<nodes.size(); i++) if (!(merge_log[i].empty()))
	{
		auto &node = nodes[i];
		for (const auto &mn : merge_log[i])
		{
			node.pos += mn.pos;
			node.dpos += mn.dpos;
			node.rad += mn.rad;
			node.drad += mn.drad;
			node.col += mn.col;
			node.dcol += mn.dcol;
		}
		real num = real(merge_log[i].size()+1);
		node.pos /= num; node.dpos /= num;
		node.rad /= num; node.drad /= num;
		node.col /= num; node.dcol /= num;
	}

	// did we end up with any processed data at all?
	traj_dataset<real> ret;
	if (trajs.size() < 1)
		return std::move(ret);

	// commit attributes to common curve representation
	const size_t num = nodes.size();
	std::vector<Vec3> P; P.reserve(num);
	std::vector<Vec4> dP; dP.reserve(num);
	std::vector<real> R; R.reserve(num);
	std::vector<real> ts; ts.resize(num);
	std::vector<Vec3> C, dC; C.reserve(num); dC.reserve(num);
	for (auto &node : nodes)
	{
		P.emplace_back(node.pos);
		dP.emplace_back(vec4_from_vec3s(node.dpos, node.drad));
		R.emplace_back(node.rad);
		C.emplace_back(node.col/real(255));
		dC.emplace_back(node.dcol/real(255));
	}
	std::vector<unsigned> I; auto &ds_trajs = trajectories(ret);
	real avg_dist = 0;
	unsigned num_segs = 0;
	for (const auto &traj : trajs)
	{
		ds_trajs.emplace_back(range{
			/* 1st index */ (unsigned)I.size(),  /* num indices */ (unsigned)traj.size()*2
		});
		real t = 0;
		for (const auto& seg : traj)
		{
			ts[seg.n0] = t; ts[seg.n1] = ++t;
			I.push_back(seg.n0);
			I.push_back(seg.n1);
			avg_dist += (P[seg.n1] - P[seg.n0]).length();
			num_segs++;
		}
	}
	set_avg_segment_length(ret, avg_dist/real(num_segs));

	// commit to actual attribute storage
	auto &A = attributes(ret);
	A.emplace(BEZDAT_POSITION_ATTRIB_NAME, traj_attribute<real>{std::move(P), ts});
	A.emplace(BEZDAT_TANGENT_ATTRIB_NAME, traj_attribute<real>{std::move(dP), ts});
	A.emplace(BEZDAT_RADIUS_ATTRIB_NAME, traj_attribute<real>{std::move(R), ts});
	A.emplace(BEZDAT_COLOR_ATTRIB_NAME, traj_attribute<real>{std::move(C), ts});
	A.emplace(BEZDAT_DCOLOR_ATTRIB_NAME, traj_attribute<real>{std::move(dC), std::move(ts)});
	indices(ret) = std::move(I);
	ret.set_mapping(Impl<real>::attrmap);

	// print some stats
	std::cout << "bezdat_handler: loading completed! Stats:" << std::endl
	          << "  "<<points.size()<<" .bezdat control points" << std::endl
	          << "  "<<bsegs.size()<<" .bezdat curve segments" << std::endl
	          << " --- converted to: ---" << std::endl
	          << "  "<<nodes.size()<<" Hermite nodes" << std::endl
	          << "  "<<num_segs<<" Hermite segments" << std::endl
	          << "  "<<trajs.size()<<" distinct smooth "<<(trajs.size()>1?"intervals":"interval")
	          << std::endl << std::endl;

	// done
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct bezdat_handler<float>;
template struct bezdat_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<bezdat_handler<float> > flt_bezdat_reg("bezdat trajectory handler (float)");
cgv::base::object_registration<bezdat_handler<double> > dbl_bezdat_reg("bezdat trajectory handler (double)");
