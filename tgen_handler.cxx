
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

// implemented header
#include "tgen_handler.h"


/////
// Some constant defines

/// identifyier to use for position data
#define TGEN_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define TGEN_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define TGEN_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define TGEN_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color gradients
#define TGEN_DCOLOR_ATTRIB_NAME "dcolor"


////
// Local types and variables

// anonymous namespace begin
namespace {

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
struct tgen_handler<flt_type>::Impl {
	// fields
	static const visual_attribute_mapping<flt_type> attrmap;

	// helper methods
	static bool is_separator (const cgv::utils::token &token, const std::string &separators)
	{
		// only 1-char tokens can be separators
		if (token.end-token.begin != 1)
			return false;

		// check against each given separator
		for (const char sep : separators)
			if (sep == *token.begin)
				return true;
		return false;
	}
	static unsigned read_fields (
		const std::vector<cgv::utils::token> &tokens, const std::string &separators,
		std::vector<std::string> *out=nullptr
	)
	{
		// count each token that is not a separator as a field
		unsigned count = 0;
		std::vector<std::string> fields;
		for (const auto &token : tokens)
			if (!is_separator(token, separators))
			{
				count++;
				if (out)
					fields.emplace_back(std::string(token.begin, size_t(token.end - token.begin)));
			}

		// account for empty last field (indicated by last token being a separator)
		if (count > 0 && is_separator(tokens.back(), separators))
		{
			count++;
			if (out)
				fields.emplace_back();
		}

		// Commit field contents if requested
		if (out)
			*out = std::move(fields);

		// done
		return count;
	}
};
template <class flt_type>
const visual_attribute_mapping<flt_type> tgen_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {TGEN_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {TGEN_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {TGEN_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {TGEN_COLOR_ATTRIB_NAME}}
});

template <class flt_type>
bool tgen_handler<flt_type>::can_handle (std::istream &contents) const
{
	std::string str;
	stream_pos_guard g(contents);

	// check for tell-tale stream contents
	// - .tgen identifier
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> idfields;
	std::getline(contents, str);
	cgv::utils::split_to_tokens(str, tokens, " \t", false);
	if (   Impl::read_fields(tokens, " \t", &idfields) < 2
	    || idfields[0].compare("TGEN") != 0 || idfields[1][0] != 'v' || idfields[1].length() < 2)
	{
		//std::cout << tgen_handler: first line in stream must be \"TGEN v\"+version, but found \"" << str << "\" instead!" << std::endl;
		return false;
	}

	// get indicated version
	int v; size_t pos;
	try { v = std::stoi(idfields[1].substr(1, std::string::npos), &pos); }
	catch (...) { /* number parsing failed, probably malformed */ return false; }
	if (pos < 1)
		// number parsing failed, probably malformed
		return false;

	// we'll just say we can read it if we recognize the version number
	return v > 0 && v < 2;
}

template <class flt_type>
traj_dataset<flt_type> tgen_handler<flt_type>::read (std::istream &contents)
{
	// bezdat database
	/*std::vector<bezdat_point<real> > points;
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

	// ToDo: sort segments within each trajectory such that they form a series rather than a soup!!!
	struct trajblock {
		unsigned i0, i1;
		unsigned n0, n1;
	};
	for (auto &traj : trajs)
	{
		std::vector<trajblock> blocks; blocks.reserve(2);
		blocks.emplace_back(trajblock{0, 0, traj.front().n0, traj.front().n1});
		for (unsigned i=1; i<traj.size(); i++)
		{
			auto &block = blocks.back();
			const auto &seg = traj[i];
			if (seg.n0 == traj[block.i1].n1)
			{
				// segment belongs to current block
				block.i1 = i;
				block.n1 = seg.n1;
			}
			else
				// a new block begins
				blocks.emplace_back(trajblock{i, i, seg.n0, seg.n1});
		}
		std::cout.flush();
	}

	// commit attributes to common curve representation
	// - create attributes in dataset
	const unsigned num = (unsigned)nodes.size();
	auto &P = add_attribute<Vec3>(ret, BEZDAT_POSITION_ATTRIB_NAME);
	auto &dP = add_attribute<Vec4>(ret, BEZDAT_TANGENT_ATTRIB_NAME);
	auto &R = add_attribute<real>(ret, BEZDAT_RADIUS_ATTRIB_NAME);
	auto &C = add_attribute<Vec3>(ret, BEZDAT_COLOR_ATTRIB_NAME);
	auto &dC = add_attribute<Vec3>(ret, BEZDAT_DCOLOR_ATTRIB_NAME);
	P.data.reserve(num);
	dP.data.reserve(num);
	R.data.reserve(num);
	C.data.reserve(num);
	dC.data.reserve(num);
	// - commit data
	std::vector<range> ds_trajs;
	real avg_dist = 0;
	unsigned num_segs = 0;
	for (const auto &traj : trajs)
	{
		ds_trajs.emplace_back(range{
			/* 1st node *//* P.data.num(),  /* num nodes *//* (unsigned)traj.size() + 1
		});
		real t = 0;
		// commit first node
		const auto &seg = traj[0];
		P.data.append(nodes[seg.n0].pos, t);
		dP.data.append(Vec4(nodes[seg.n0].dpos, nodes[seg.n0].drad), t);
		R.data.append(nodes[seg.n0].rad, t);
		C.data.append(nodes[seg.n0].col/real(255), t);
		dC.data.append(nodes[seg.n0].dcol/real(255), ++t);
		for (const auto &seg : traj)
		{
			const auto &node = nodes[seg.n1];
			P.data.append(node.pos, t);
			dP.data.append(Vec4(node.dpos, node.drad), t);
			R.data.append(node.rad, t);
			C.data.append(node.col/real(255), t);
			dC.data.append(node.dcol/real(255), ++t);
			avg_dist += (nodes[seg.n1].pos - nodes[seg.n0].pos).length();
			num_segs++;
		}
	}
	set_avg_segment_length(ret, avg_dist/real(num_segs));

	// finalize
	ret.set_mapping(Impl<real>::attrmap);
	trajectories(ret, P.attrib) = ds_trajs;  // all attributes are sampled
	trajectories(ret, dP.attrib) = ds_trajs; // equally, so we can just
	trajectories(ret, R.attrib) = ds_trajs;  // duplicate the trajectory ranges
	trajectories(ret, C.attrib) = ds_trajs;  // across all attributes
	trajectories(ret, dC.attrib) = std::move(ds_trajs);

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
	return std::move(ret);*/
	return traj_dataset<real>();
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct tgen_handler<float>;
template struct tgen_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<tgen_handler<float> > flt_bezdat_reg("tgen trajectory handler (float)");
cgv::base::object_registration<tgen_handler<double> > dbl_bezdat_reg("tgen trajectory handler (double)");
