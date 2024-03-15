
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

	enum class Mode {
		Single, Spaghetti, Stack
	} mode = Mode::Single;

	unsigned simulation_run_number = 0;
	flt_type h_offset = 0;
	attrib_info<vec3> P;
	attrib_info<flt_type> S, T, R, partID;
	traj_attribute<flt_type> runID;
	std::vector<range> &Ptrajs, &PIDtrajs;

	Impl(traj_dataset<flt_type> &ds)
		// init synchronous attributes
		: P(add_attribute<vec3>(ds, TASC_POSITION_ATTRIB_NAME)),
		  S(add_attribute<flt_type>(ds, TASC_SPEED_ATTRIB_NAME)),
		  T(add_attribute<flt_type>(ds, TASC_TIME_ATTRIB_NAME)),
		  R(add_attribute<flt_type>(ds, TASC_RADIUS_ATTRIB_NAME)),
		  Ptrajs(trajectories(ds, P.attrib)),
		// init "meta" attributes
		  partID(add_attribute<flt_type>(ds, TASC_PARTICIPANT_ID_ATTRIB_NAME)),
		  runID(1),
		  PIDtrajs(trajectories(ds, partID.attrib))
	{}

	unsigned read_simulation_run(flt_type &new_segs_avg_length, std::istream &contents)
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
							range new_traj{P.attrib.num(), 0}, new_meta{partID.attrib.num(), 2};
							// read trajectory
							for (unsigned i=0; i<_T.size() && i<_X.size() && i<_Y.size() && i<_S.size(); i++)
							{
								flt_type ts = _T[i];
								vec3 new_pos(_X[i], h_offset, _Y[i]);
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
								PIDtrajs.emplace_back(new_meta);
								partID.data.timestamps.emplace_back(ts[0]);
								partID.data.values.emplace_back(p["id"]);
								partID.data.timestamps.emplace_back(ts[1]);
								partID.data.values.emplace_back(partID.data.values.back());
								if (mode != Mode::Single) {
									auto &rID_data = runID.template get_data<flt_type>();
									rID_data.timestamps.emplace_back(ts[0]);
									rID_data.values.emplace_back(simulation_run_number);
									rID_data.timestamps.emplace_back(ts[1]);
									rID_data.values.emplace_back(simulation_run_number);
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
traj_dataset<flt_type> tasc_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
){
	// Check file type
	std::vector<std::string> fields;
	std::vector<cgv::utils::token> tokens;
	std::string line;
	const std::string ws = " \t\r";
	const bool ensemble = [&]() -> bool {
		const stream_pos_guard g(contents);
		return    csv_handler<flt_type>::Impl::read_next_nonempty_line(&line, &tokens, ws, contents, &fields) > 1
		       && fields[0].compare("TASC-OTV") == 0 && csv_handler<flt_type>::Impl::parse_field(fields[1]) > 0;
	}();

	// prepare dataset container object and attribute storage
	traj_dataset<flt_type> ret;

	// instantiate implementation
	Impl impl(ret);

	unsigned num_segs = 0;
	if (ensemble)
	{
		////
		// We're loading a simulation ensemble
		const std::filesystem::path orig = path,
		                            orig_path = orig.parent_path();
		impl.mode = Impl::Mode::Spaghetti;
		std::filesystem::path dir;

		// parse file and log parameters
		while (!contents.eof())
		{
			const unsigned num_tokens = csv_handler<flt_type>::Impl::read_next_nonempty_line(&line, &tokens, ws, contents, &fields);
			if (Impl::line_is_comment(line))
				continue;

			if (num_tokens > 1)
			{
				if (fields[0] == "mode" && fields[1] == "stacked")
					impl.mode = Impl::Mode::Stack;
				else if (fields[0] == "dir")
					dir = fields[1];
			}
		}

		// post-process ensemble path
		const std::filesystem::path ensemble_dir =
			dir.is_relative() ?	std::filesystem::absolute(orig_path/dir) : dir;

		// load individual simulation runs
		double avg_seg_len = 0;
		for (auto const &f : std::filesystem::directory_iterator{ensemble_dir})
		{
			if (   cgv::utils::to_lower(f.path().extension()) == ".json"
			    && std::filesystem::is_regular_file(f.path()))
			{
				std::ifstream runfile_contents(f.path());
				flt_type avg_newsegs_len;
				num_segs = impl.read_simulation_run(avg_newsegs_len, runfile_contents);
				traj_format_handler<flt_type>::set_avg_segment_length(ret, avg_newsegs_len);
				avg_seg_len += avg_newsegs_len;
				impl.simulation_run_number++;
				if (impl.mode == Impl::Mode::Stack)
					impl.h_offset += 5;
			}
		}
		// move simulation run "meta" attribute into the dataset
		traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TASC_SIMULATION_RUN_ATTRIB_NAME, std::move(impl.runID));
		traj_format_handler<flt_type>::trajectories(ret, impl.runID) = impl.PIDtrajs;

		// commit segment length stats
		traj_format_handler<flt_type>::set_avg_segment_length(ret, flt_type(avg_seg_len/impl.simulation_run_number));
	}
	else
	{
		////
		// We're loading an individual simulation run

		flt_type avg_seg_len;
		num_segs = impl.read_simulation_run(avg_seg_len, contents);
		traj_format_handler<flt_type>::set_avg_segment_length(ret, avg_seg_len);
	}

	// copy trajectory info to the other synchronous attributes
	traj_format_handler<flt_type>::trajectories(ret, impl.S.attrib) = impl.Ptrajs;
	traj_format_handler<flt_type>::trajectories(ret, impl.T.attrib) = impl.Ptrajs;
	traj_format_handler<flt_type>::trajectories(ret, impl.R.attrib) = impl.Ptrajs;

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
