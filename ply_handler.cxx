
// C++ STL
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <utility>
#include <limits>
#include <ctime>

// CGV framework core
#include <cgv/base/register.h>
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/progression.h>

// local includes
#include "thick_line_data.h"
#include "ply.h"

// implemented header
#include "ply_handler.h"


/////
// Some constant defines

/// identifyier to use for position data
#define PLY_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define PLY_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define PLY_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define PLY_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color gradients
#define PLY_DCOLOR_ATTRIB_NAME "dcolor"


////
// Class implementation

template <class flt_type>
struct ply_handler<flt_type>::Impl {
	// fields
	static const visual_attribute_mapping<flt_type> attrmap;
};
template <class flt_type>
const visual_attribute_mapping<flt_type> ply_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {PLY_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {PLY_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {PLY_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {PLY_COLOR_ATTRIB_NAME}}
});

template <class flt_type>
const std::string& ply_handler<flt_type>::format_name (void) const {
	static const std::string fmt_name = "PLY";
	return fmt_name;
}

template <class flt_type>
const std::vector<std::string>& ply_handler<flt_type>::handled_extensions (void) const
{
	static const std::vector<std::string> exts = { "ply" };
	return exts;
}

template <class flt_type>
bool ply_handler<flt_type>::can_handle (std::istream &contents) const
{
	const stream_pos_guard g(contents);

	// check for tell-tale stream contents by sniffing the ply header
	char buffer[4096];
	contents.read(buffer, sizeof(buffer));
	const size_t byte_count = size_t(contents.gcount());
	contents.clear();

	// report whether the sniffed prefix looks like a valid PLY header
	return looks_like_ply_from_memory(buffer, byte_count);
}

template <class flt_type>
traj_dataset<flt_type> ply_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
){
	////
	// Step 1: load

	// read the PLY contents into a locally constructed thick line data representation, which
	// already provides the node data in Hermite form and interprets every 100 consecutive
	// vertices as one hair strand
	thick_line_data tld;
	if (!tld.read_ply(contents, .000244140625f, 1)) {
		std::cerr << "[ply_handler] ERROR: could not read PLY trajectory data!" << std::endl;
		return traj_dataset<real>();
	}
	if (tld.indices.empty()) {
		std::cerr << "[ply_handler] ERROR: PLY stream contains no trajectory data!" << std::endl;
		return traj_dataset<real>();
	}


	////
	// Step 2: transfer to canonical dataset

	// prepare dataset (we use the filename as dataset name since .ply does not support any kind of
	// meta fields which could contain a name for the dataset)
	traj_dataset<flt_type> ds(
		cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path)), path
	);
	// - create geometry attributes
	auto P = super::template add_attribute<Vec3>(ds, PLY_POSITION_ATTRIB_NAME);
	auto T = super::template add_attribute<Vec4>(ds, PLY_TANGENT_ATTRIB_NAME);
	auto R = super::template add_attribute<real>(ds, PLY_RADIUS_ATTRIB_NAME);
	auto C = super::template add_attribute<Vec3>(ds, PLY_COLOR_ATTRIB_NAME);

	// compile
	std::vector<range> ds_trajs;
	real avg_dist = 0;
	unsigned num_segs = 0;
	{
		// node access helpers - thick_line_data already stores the nodes in Hermite form with position+radius in
		// `spheres`, tangent+radius derivative in `tangents` and a node color
		const auto node_pos = [&tld] (unsigned n) -> Vec3 {
			const auto &s = tld.spheres[n];
			return Vec3(real(s[0]), real(s[1]), real(s[2]));
		};
		const auto append_node = [&] (unsigned n, real t) {
			const auto &s = tld.spheres[n];
			const auto &tn = tld.tangents[n];
			const auto &c = tld.colors[n];
			P.data.append(node_pos(n), t);
			T.data.append(Vec4(real(tn[0]), real(tn[1]), real(tn[2]), real(tn[3])), t);
			R.data.append(real(s[3]), t);
			C.data.append(Vec3(real(c.R()), real(c.G()), real(c.B())), t);
		};

		// optimize allocations
		const unsigned num_samples_bound = unsigned(tld.spheres.size()) + unsigned(tld.indices.size()/2);
		P.data.reserve(num_samples_bound);
		T.data.reserve(num_samples_bound);
		R.data.reserve(num_samples_bound);
		C.data.reserve(num_samples_bound);

		// transfer the bezier geometry into the hermite spline representation that traj_dataset expects:
		// thick_line_data commits the segments of each trajectory contiguously and only references globally unique node
		// indices, so a new trajectory begins exactly where a segment does not continue its predecessor. The closing
		// segment of a closed curve wraps back to the trajectory's first node; committing it like any other segment
		// duplicates that node and thereby preserves the closing Hermite segment within the linear sample chain.
		unsigned prev_end = 0;
		real t = 0;
		const unsigned num_segments = unsigned(tld.indices.size()/2);
		for (unsigned s=0; s<num_segments; s++)
		{
			// node indices of the current segment
			const unsigned a = tld.indices[2*s], b = tld.indices[2*s+1];

			// begin a new trajectory where the previous one ended
			if (s == 0 || a != prev_end)
			{
				if (s > 0)
					ds_trajs.back().n = unsigned(P.data.num()) - ds_trajs.back().i0;
				ds_trajs.emplace_back(range{unsigned(P.data.num()), 0});
				t = 0;
				append_node(a, t);
			}

			// commit the segment's end node and account for the segment length
			append_node(b, ++t);
			avg_dist += (node_pos(b) - node_pos(a)).length();
			num_segs++;
			prev_end = b;
		}
		// close the final trajectory
		if (!ds_trajs.empty())
			ds_trajs.back().n = unsigned(P.data.num()) - ds_trajs.back().i0;
	}

	// transfer trajectory ranges - all attributes are sampled at the Hermite nodes, so we can just duplicate the
	// trajectory ranges across all attributes
	const unsigned num_trajs = (unsigned)ds_trajs.size();
	traj_format_handler<flt_type>::trajectories(ds, P.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, T.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, R.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, C.attrib) = std::move(ds_trajs);

	// finalize
	ds.set_mapping(Impl::attrmap);
	super::set_avg_segment_length(ds, avg_dist/num_segs);

	// print some stats
	std::cout << "ply_handler: loading completed! Stats:" << std::endl
	          << "  " << tld.spheres.size() << " Hermite nodes" << std::endl
	          << "  " << num_segs << " Hermite segments" << std::endl
	          << "  " << num_trajs << " distinct " << (num_trajs>1?"trajectories":"trajectory")
	          << std::endl << std::endl;

	// done!
	return ds;
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct ply_handler<float>;
//template struct ply_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<ply_handler<float> > flt_ply_reg("ply trajectory handler (float)");
//cgv::base::object_registration<ply_handler<double> > dbl_ply_reg("ply trajectory handler (double)");
