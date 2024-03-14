
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

// local includes
#include "csv_handler.h"
#include "csv_handler_detail.h"


// the desired minimum time between position samples (typically, in seconds)
#define TASC_MIN_TIME_BETWEEN_POS_SAMPLES 1

// identifyier to use for position data
#define TASC_POSITION_ATTRIB_NAME "Position"

// identifyier to use for speed data
#define TASC_SPEED_ATTRIB_NAME "Speed"

// identifyier to use for participant ID
#define TASC_PARTICIPANT_ID_ATTRIB_NAME "ParticipantID"

// identifyier to use for simulation run number
#define TASC_SIMULATION_RUN_ATTRIB_NAME "SimulationRun"

// identifyier to use for radius data
#define TASC_RADIUS_ATTRIB_NAME "_radius"

// identifyier to use for timestamp attribute
#define TASC_TIME_ATTRIB_NAME "Time"

// whether to use ECEF coordinates instead of Mercator cartesian + altitude
#define TASC_USE_ECEF_COORDINATES 0


////
// Private implementation details

template <class flt_type>
struct tasc_handler<flt_type>::Impl
{
	template <class T>
	using attrib_info = typename traj_dataset<flt_type>::template attrib_info<T>;

	static unsigned read_simulation_run (
		unsigned simulation_run_val, attrib_info<vec3> &P, attrib_info<flt_type> &S, attrib_info<flt_type> &T,
		attrib_info<flt_type> &R, attrib_info<flt_type> &participant_id, attrib_info<flt_type> *run_id_optional,
		std::vector<range> &Ptrajs, std::vector<range> &participant_id_trajs, flt_type &new_segs_avg_length,
		std::istream &contents
	)
	{
		// Parse the JSON stream
		nlohmann::json j;
		try {
			contents >> j;
		}
		catch (nlohmann::json::parse_error&) {
			return 0;
		}

		// Extract trajectory data for every participant
		// - prepare participant info storage
		std::vector<nlohmann::json> participant_infos;
		// - values
		unsigned num_segs = 0;
		if (j.is_object())
		{
			const auto &parts = j["participants"];
			if (parts.is_array() && !parts.empty())
			{
				double seg_dist_accum = 0;
				for (const auto &p : parts)
				{
					auto &info = p["participantInfo"];
					const auto &trace = p["simulationResults"];
					if (info.is_object() && trace.is_object())
					{
						// determine desired minimum sample distance
						const flt_type min_dist_sqr = std::pow(
							std::max(flt_type(info["length"]), flt_type(1)), flt_type(2)
						);
						const auto &_T = trace["t"]["values"], &_X = trace["x"]["values"], &_Y = trace["y"]["values"],
						           &_S = trace["v"]["values"];
						// use participant height as radius
						const flt_type radius = info["height"];
						if (_T.is_array() && _X.is_array() && _Y.is_array() && _S.is_array())
						{
							range new_traj{P.attrib.num(), 0}, new_meta{participant_id.attrib.num(), 2};
							// read trajectory
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
										// include this sample (start with accumulating the resulting segment length)
										seg_dist_accum += (double)cgv::math::length(new_pos - P.data.values.back());
									else
										continue;
								}
								P.data.timestamps.emplace_back(ts);
								P.data.values.emplace_back(new_pos);
								S.data.timestamps.emplace_back(ts);
								S.data.values.emplace_back(_S[i]);
								T.data.timestamps.emplace_back(ts);
								T.data.values.emplace_back(ts);
								R.data.timestamps.emplace_back(ts);
								R.data.values.emplace_back(radius);
								new_traj.n++;
							}
							if (new_traj.n > 1) {
								Ptrajs.emplace_back(new_traj);
								// log trajectory meta attributes
								const flt_type ts[2] = {
									P.data.timestamps[new_traj.i0],
									P.data.timestamps[new_traj.i0 + new_traj.n-1]
								};
								participant_id_trajs.emplace_back(new_meta);
								participant_id.data.timestamps.emplace_back(ts[0]);
								participant_id.data.values.emplace_back(p["id"]);
								participant_id.data.timestamps.emplace_back(ts[1]);
								participant_id.data.values.emplace_back(participant_id.data.values.back());
								if (run_id_optional) {
									run_id_optional->data.timestamps.emplace_back(ts[0]);
									run_id_optional->data.values.emplace_back(simulation_run_val);
									run_id_optional->data.timestamps.emplace_back(ts[1]);
									run_id_optional->data.values.emplace_back(simulation_run_val);
								}
								participant_infos.emplace_back(std::move(info));
							}
						}
					}
				}
				// record segment length stats
				num_segs = P.attrib.num() - (unsigned)Ptrajs.size();
				new_segs_avg_length = flt_type(seg_dist_accum / num_segs);
			}
		}
		// sanity check
		assert(P.attrib.num()==S.attrib.num() && P.attrib.num()==T.attrib.num());
		return num_segs;
	}

	inline static bool line_is_comment (const std::string &line) {
		return line.size() >= 1 && line[0] == '#';
	}
};


////
// Class implementation

template <class flt_type>
const std::string& tasc_handler<flt_type>::format_name (void) const
{
	static const std::string fmt_name = "TASC JSON";
	return fmt_name;
}

template <class flt_type>
const std::vector<std::string>& tasc_handler<flt_type>::handled_extensions (void) const
{
	static const std::vector<std::string> exts = {"json", "tasc"};
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
	// Check file type
	std::vector<std::string> columns;
	std::vector<cgv::utils::token> tokens;
	std::string line;
	const std::string ws = " \t";
	const bool ensemble = [&]() -> bool {
		const stream_pos_guard g(contents);
		return    csv_handler<flt_type>::Impl::read_next_nonempty_line(&line, &tokens, ws, contents, &columns) == 2
		       && columns[0].compare("TASC-OTV") == 0 && csv_handler<flt_type>::Impl::parse_field(columns[1]) > 0;
	}();

	// prepare dataset container object and attribute storage
	traj_dataset<flt_type> ret;
	// - synchronous attributes
	auto P = traj_format_handler<flt_type>::template add_attribute<vec3>(ret, TASC_POSITION_ATTRIB_NAME);
	auto S = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_SPEED_ATTRIB_NAME);
	auto T = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_TIME_ATTRIB_NAME);
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_RADIUS_ATTRIB_NAME);
	auto &Ptrajs = traj_format_handler<flt_type>::trajectories(ret, P.attrib);
	// - "meta" attributes
	auto participant_id = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_PARTICIPANT_ID_ATTRIB_NAME);
	auto &PIDtrajs = traj_format_handler<flt_type>::trajectories(ret, participant_id.attrib);

	unsigned num_segs = 0;
	if (ensemble)
	{
		////
		// We're loading a simulation ensemble

		// add simulation run "meta" attribute
		auto simulation_run = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_SIMULATION_RUN_ATTRIB_NAME);

		// parse file and log parameters
		while (!contents.eof())
		{
			const unsigned num_tokens = csv_handler<flt_type>::Impl::read_next_nonempty_line(&line, &tokens, ws, contents, &columns);
			if (Impl::line_is_comment(line))
				continue;
		}

		// copy participant id trajectory info to the added simulation run trajectory info as they share the same "meta" trajectory
		// structure
		traj_format_handler<flt_type>::trajectories(ret, simulation_run.attrib) = PIDtrajs;
	}
	else
	{
		////
		// We're loading an individual simulation run

		flt_type avg_seg_len;
		num_segs = Impl::read_simulation_run(0, P, S, T, R, participant_id, nullptr, Ptrajs, PIDtrajs, avg_seg_len, contents);
		traj_format_handler<flt_type>::set_avg_segment_length(ret, avg_seg_len);
	}

	// copy trajectory info to the other synchronous attributes
	traj_format_handler<flt_type>::trajectories(ret, S.attrib) = Ptrajs;
	traj_format_handler<flt_type>::trajectories(ret, T.attrib) = Ptrajs;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptrajs;

	// Final check if we loaded something useful
	if (!num_segs)
		return traj_dataset<flt_type>(); // discard everything done up to now and just return an invalid dataset

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
