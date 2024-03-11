
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
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv/os/line_break.h>
#include <cgv/utils/advanced_scan.h>

// 3rd party libs
#include <nlohmann/json.hpp>
#include <peridetic.h>
#include <WGS84toCartesian.hpp>

// implemented header
#include "tasc_handler.h"


// the desired minimum time between position samples (typically, in seconds)
#define TASC_MIN_TIME_BETWEEN_POS_SAMPLES 1

// identifyier to use for position data
#define TASC_POSITION_ATTRIB_NAME "Position"

// identifyier to use for radius data
#define TASC_SPEED_ATTRIB_NAME "Speed"

// identifyier to use for radius data
#define TASC_RADIUS_ATTRIB_NAME "_radius"

// identifyier to use for timestamp attribute
#define TASC_TIME_ATTRIB_NAME "Time"

// whether to use ECEF coordinates instead of Mercator cartesian + altitude
#define TASC_USE_ECEF_COORDINATES 0


template <class flt_type>
const std::string& tasc_handler<flt_type>::format_name (void) const
{
	static const std::string fmt_name = "TASC JSON";
	return fmt_name;
}

template <class flt_type>
const std::vector<std::string>& tasc_handler<flt_type>::handled_extensions (void) const
{
	static const std::vector<std::string> exts = {"json"};
	return exts;
}

template <class flt_type>
bool tasc_handler<flt_type>::can_handle (std::istream &contents) const
{
	const stream_pos_guard g(contents);
	nlohmann::json j;
	try
	{
		contents >> j;
	}
	catch (nlohmann::json::parse_error&)
	{
		return false;
	}
	if (j.is_object()) {
		const auto &parts = j["participants"];
		if (parts.is_array() && !parts.empty()) {
			for (const auto &p : parts) {
				const auto &trace = p["simulationResults"];
				if (trace.is_object()) {
					const auto &T = trace["t"]["values"], &X = trace["x"]["values"], &Y = trace["y"]["values"];
					if (T.is_array() && X.is_array() && Y.is_array())
						return true;
				}
			}
		}
	}
	return false;
}

template <class flt_type>
traj_dataset<flt_type> tasc_handler<flt_type>::read(
	std::istream &contents, DatasetOrigin source, const std::string &path
){
	// Parse the JSON stream
	nlohmann::json j;
	try {
		contents >> j;
	}
	catch (nlohmann::json::parse_error&) {
		return traj_dataset<flt_type>();
	}

	// prepare dataset container object and attribute storage
	traj_dataset<flt_type> ret;
	auto P = traj_format_handler<flt_type>::template add_attribute<vec3>(ret, TASC_POSITION_ATTRIB_NAME);
	auto S = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_SPEED_ATTRIB_NAME);
	auto T = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_TIME_ATTRIB_NAME);
	auto &Ptrajs = traj_format_handler<flt_type>::trajectories(ret, P.attrib);

	// Extract trajectory data for every participant
	// - prepare participant info storage
	std::vector<nlohmann::json> participant_infos;
	// - values
	if (j.is_object()) {
		const auto &parts = j["participants"];
		if (parts.is_array() && !parts.empty()) {
			for (const auto &p : parts) {
				auto &info = p["participantInfo"];
				const auto &trace = p["simulationResults"];
				if (info.is_object() && trace.is_object()) {
					// determine desired minimum sample distance
					const flt_type min_dist_sqr = std::pow(flt_type(info["length"]), flt_type(2));
					const auto &_T = trace["t"]["values"], &_X = trace["x"]["values"], &_Y = trace["y"]["values"],
					           &_S = trace["v"]["values"];
					if (_T.is_array() && _X.is_array() && _Y.is_array() && _S.is_array()) {
						range new_traj{P.attrib.num(), 0};
						for (unsigned i=0; i<_T.size() && i<_X.size() && i<_Y.size() && i<_S.size(); i++)
						{
							flt_type ts = _T[i];
							vec3 new_pos(_X[i], 0, _Y[i]);
							// decide whether to commit these samples
							if (i > 0) {
								const vec3 pdiff = new_pos - P.data.values.back();
								const flt_type pdiff_len_sqr = pdiff.sqr_length(),
								               tdiff = ts - P.data.timestamps.back();
								if (pdiff_len_sqr > min_dist_sqr || tdiff > flt_type(TASC_MIN_TIME_BETWEEN_POS_SAMPLES))
									/* include_this_sample() */;
								else
									continue;
							}
							P.data.timestamps.emplace_back(ts);
							P.data.values.emplace_back(new_pos);
							S.data.timestamps.emplace_back(ts);
							S.data.values.emplace_back(_S[i]);
							T.data.timestamps.emplace_back(ts);
							T.data.values.emplace_back(ts);
							new_traj.n++;
						}
						if (new_traj.n > 1) {
							Ptrajs.emplace_back(new_traj);
							participant_infos.emplace_back(std::move(info));
						}
					}
				}
			}
		}
	}
	// - copy over trajectory information for the other position-synchronous attributes
	assert(P.attrib.num()==S.attrib.num() && P.attrib.num()==T.attrib.num());
	traj_format_handler<flt_type>::trajectories(ret, S.attrib) = Ptrajs;
	traj_format_handler<flt_type>::trajectories(ret, T.attrib) = Ptrajs;

	////
	// Fully translate loaded data into traj_mgr dataset

	// for single precision float compatibility, we make everything relative to the first sample of the first trajectory position-wise...
	// TODO: position samples are in a local cartesian coordinate system relative to the crash location. We actually need to perform inverse
	//       transformation to obtain the lat/long bounds of the map tiles we need to fetch
/*#if defined(TASC_USE_ECEF_COORDINATES) && TASC_USE_ECEF_COORDINATES!=0
	const auto refpos = peri::xyzForLpa(peri::LPA{
		P.data.values[0].x(), P.data.values[0].y(), 0
	});
#else
	typedef std::array<double, 2> latlong;
	const latlong refpos = {P.data.values[0].x(), P.data.values[0].y()};
#endif

	// Transform lat/long coordinates to visualization world space and determine avg segment lengths
	// - scale factor for mercator-projected cartesian positions
	constexpr flt_type mscale = 1/(flt_type)TASC_MERCATOR_SCALEDOWN_FACTOR;
	// - the transformation loop
	for (auto &pos : P.data.values)
	{
		#if defined(TASC_USE_ECEF_COORDINATES) && TASC_USE_ECEF_COORDINATES!=0
			const auto xyzpos = peri::xyzForLpa(peri::LPA{pos.x(), pos.y(), 0}),
			           relpos = peri::LPA{xyzpos[0]-refpos[0], xyzpos[1]-refpos[1], xyzpos[2]-refpos[2]};
			pos.set((flt_type)relpos[0], (flt_type)relpos[1], (flt_type)relpos[2]);
		#else
			const auto mercator = wgs84::toCartesian(refpos, latlong{pos.x(), pos.y()});
			pos.set((flt_type)mercator[0]*mscale, 0, (flt_type)mercator[1]*mscale);
		#endif
	}*/
	// - the segment length stats collection loop
	double seg_dist_accum = 0;
	unsigned num_segs = P.attrib.num() - (unsigned)Ptrajs.size(); unsigned num_segs_test = 0;
	for (auto &traj : Ptrajs) for (unsigned i=traj.i0+1; i<traj.i0+traj.n; i++) {
		const flt_type seg_len = (P.data.values[i] - P.data.values[i-1]).length();
		seg_dist_accum += (double)seg_len;
		num_segs_test++;
	}
	assert(num_segs == num_segs_test);
	traj_format_handler<flt_type>::set_avg_segment_length(ret, flt_type(seg_dist_accum / num_segs));

	// At this point, we know if we loaded something useful
	if (num_segs < 1)
		return traj_dataset<flt_type>(); // discard everything done up to now and just return an invalid dataset

	// Use participant height as radius
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_RADIUS_ATTRIB_NAME);
	R.data.timestamps = P.data.timestamps;
	R.data.values.resize(P.attrib.num());
	for (unsigned t=0; t<Ptrajs.size(); t++) {
		const auto &traj = Ptrajs[t];
		const flt_type radius = participant_infos[t]["height"];
		for (unsigned i=traj.i0; i<traj.i0+traj.n; i++)
			R.data.values[i] = radius;
	}
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptrajs; // radius "samples" are again in sync with positions, so just copy traj info

	// The default visual attribute mapping for TASC data
	static const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {TASC_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {TASC_RADIUS_ATTRIB_NAME}}
	});
	ret.set_mapping(vamap);

	// Set dataset name (we just use the filename for now)
	traj_format_handler<flt_type>::name(ret) = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path));

	// done!
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct tasc_handler<float>;
template struct tasc_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<tasc_handler<float> >  flt_tasc_reg("TASC trajectory handler (float)");
cgv::base::object_registration<tasc_handler<double> > dbl_tasc_reg("TASC trajectory handler (double)");
