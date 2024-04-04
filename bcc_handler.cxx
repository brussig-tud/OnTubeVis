
// C++ STL
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <limits>
#include <filesystem>

// CGV framework core
#include <cgv/base/register.h>
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv/os/line_break.h>
#include <cgv/utils/advanced_scan.h>

// 3rd party libs
#include <nlohmann/json.hpp>
#include <peridetic.h>
#include <WGS84toCartesian.hpp>

// implemented header
#include "bcc_handler.h"


// identifyier to use for position data
#define BCC_POSITION_ATTRIB_NAME "Position"

// identifyier to use for participant ID
#define BCC_CURVE_ID_ATTRIB_NAME "CurveID"

// identifyier to use for radius data
#define BCC_RADIUS_ATTRIB_NAME "_radius"

// identifyier to use for timestamp attribute
#define BCC_CURVE_PARAM_ATTRIB_NAME "t"


////
// Local helpers

// anonymous namespace begin
namespace {

// structure of the BCC file header
struct bcc_header {
    char sign[3];
    unsigned char byte_count;
    char curve_type[2];
    char dims;
    char up_dim;
    uint64_t num_curves;
    uint64_t num_ctrl_points;
    char info[40];
};

// anonymous namespace end
}


////
// Private implementation details

template <class flt_type>
struct bcc_handler<flt_type>::Impl
{};


////
// Class implementation

template <class flt_type>
const std::string& bcc_handler<flt_type>::format_name (void) const
{
	static const std::string fmt_name = "Binary Curve Collection";
	return fmt_name;
}

template <class flt_type>
const std::vector<std::string>& bcc_handler<flt_type>::handled_extensions (void) const
{
	static const std::vector<std::string> exts = {"bcc"};
	return exts;
}

template <class flt_type>
bool bcc_handler<flt_type>::can_handle (std::istream &contents) const
{
	const stream_pos_guard g(contents);
	bcc_header header;
	contents.read((char*)&header, sizeof(header));
	return false;
}

template <class flt_type>
traj_dataset<flt_type> bcc_handler<flt_type>::read (
	std::istream &, DatasetOrigin source, const std::string &path
){
	// prepare dataset container object and attribute storage
	traj_dataset<flt_type> ret;

	// discard the stream we're given, we need binary mode
	std::ifstream file(path, std::ifstream::binary);


	////
	// Load file contents

	// read in file header
	bcc_header header;
	file.read((char*)&header, sizeof(header));

	// basic checks
	if (   header.sign[0]!='B' || header.sign[1]!='C' || header.sign[2]!='C'
	    || header.byte_count != 0x44 /* we only support 4-byte floats and 4-byte ints */
	    || header.curve_type[0] !='C' /* we only support Catmul-Rom splines */
	    || header.dims != 3 /* we only support 3D curves */)
		// Not a BCC file
		return ret;

	// read in curve data
	std::vector<vec3> controlPoints(header.num_ctrl_points);
	std::vector<int> firstControlPoint(header.num_curves+1); // one more for easy curve length checks (see usage below)
	std::vector<char> isCurveLoop(header.num_curves);
	vec3 *cp = (vec3*)controlPoints.data();
	int prevCP = 0;
	for (uint64_t i=0; i<header.num_curves; i++ ) {
		int curve_num_pnts;
		file.read((char*)&curve_num_pnts, sizeof(int));
		isCurveLoop[i] = curve_num_pnts < 0;
		curve_num_pnts = std::abs(curve_num_pnts);
		file.read((char*)cp, header.dims*sizeof(float)* curve_num_pnts);
		cp += curve_num_pnts;
		firstControlPoint[i] = prevCP;
		prevCP += curve_num_pnts;
	}
	firstControlPoint.back() = prevCP;
	// sanity check
	assert(prevCP == header.num_ctrl_points);


	////
	// Convert to trajectory dataset

	// base attributes
	auto P = add_attribute<Vec3>(ret, BCC_POSITION_ATTRIB_NAME);
	auto T = add_attribute<real>(ret, BCC_CURVE_PARAM_ATTRIB_NAME);
	auto I = add_attribute<real>(ret, BCC_CURVE_ID_ATTRIB_NAME);
	auto R = add_attribute<real>(ret, BCC_RADIUS_ATTRIB_NAME);
	auto &Ptrajs = trajectories(ret, P.attrib);
	auto &Itrajs = trajectories(ret, I.attrib);

	// move over curve positions and t parameters
	P.data.values.reserve(header.num_ctrl_points);
	P.data.timestamps.reserve(P.data.values.capacity());
	T.data.values.reserve(P.data.values.capacity());
	T.data.timestamps.reserve(T.data.values.capacity());
	R.data.values.reserve(P.data.values.capacity());
	R.data.timestamps.reserve(R.data.values.capacity());
	I.data.values.reserve(header.num_curves*2);
	I.data.timestamps.reserve(I.data.values.capacity());
	double seg_len_accum = 0; unsigned num_segs = 0;
	for (unsigned i=0; i<header.num_curves; i++)
	{
		// convert control points, and generate "timestamps" (curve params at nodes) in the process
		const unsigned fCP = firstControlPoint[i], nCP = firstControlPoint[i+1],
		               numCP = nCP-fCP, n0 = (unsigned)P.data.values.size();
		// very first control point of a curve
		// - position synced attributes
		P.data.values.emplace_back(controlPoints[fCP]);
		P.data.timestamps.emplace_back(real(0));
		T.data.values.emplace_back(real(0));
		T.data.timestamps.emplace_back(real(0));
		// - curve id attribute
		I.data.values.emplace_back(real(i));
		I.data.timestamps.emplace_back(real(0));
		for (unsigned t=1; t<numCP; t++) {
			P.data.values.emplace_back(controlPoints[fCP+t]);
			const real f_flt = P.data.timestamps.emplace_back((real)t);
			T.data.values.emplace_back(f_flt);
			T.data.timestamps.emplace_back(f_flt);
			seg_len_accum += cgv::math::length(P.data.values.back() - P.data.values[P.data.values.size()-2]);
			num_segs++;
		}
		/* TODO: verify if this is really not needed for loop closure
		if (isCurveLoop[i])	{
			P.data.values.push_back(controlPoints[fCP]);
			const real f_flt = P.data.timestamps.emplace_back((real)numCP+1);
			T.data.values.emplace_back(f_flt);
			T.data.timestamps.emplace_back(f_flt);
			seg_len_accum += cgv::math::length(P.data.values.back() - P.data.values[P.data.values.size()-2]);
			num_segs++;
		}*/
		const unsigned nN = (unsigned)P.data.values.size() - n0;
		assert(nN == numCP);

		// second curve id "sample" for this trajectory
		I.data.values.emplace_back(real(i));
		I.data.timestamps.emplace_back(nN-1);

		// log trajectory info
		const range new_ptraj = {n0, nN}, new_itraj = {i*2, 2};
		Ptrajs.emplace_back(new_ptraj);
		Itrajs.emplace_back(new_itraj);
	}

	// Final check if we loaded something useful
	assert(num_segs == header.num_ctrl_points - header.num_curves);
	if (!num_segs)
		return traj_dataset<flt_type>(); // discard everything done up to now and just return an invalid dataset

	// invent radii
	set_avg_segment_length(ret, real(seg_len_accum/=double(num_segs)));
	for (const auto &_ : P.data.values)
		R.data.values.emplace_back(ret.avg_segment_length()*real(.75));
	R.data.timestamps = P.data.timestamps;

	// copy over trajectory info for position-synced attributes
	trajectories(ret, T.attrib) = Ptrajs;
	trajectories(ret, R.attrib) = Ptrajs;

	// The default visual attribute mapping for TASC data
	static const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {BCC_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {BCC_RADIUS_ATTRIB_NAME}}
	});
	ret.set_mapping(vamap);

	// Set dataset name (we just use the filename for now)
	super::name(ret) = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path));

	// done!
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class bcc_handler<float>;
template class bcc_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<bcc_handler<float> >  flt_bcc_reg("BCC trajectory handler (float)");
cgv::base::object_registration<bcc_handler<double> > dbl_bcc_reg("BCC trajectory handler (double)");
