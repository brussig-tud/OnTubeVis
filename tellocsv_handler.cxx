
// C++ STL
#include <vector>
#include <unordered_set>
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
#include <peridetic.h>
#include <WGS84toCartesian.hpp>

// local includes
#include "csv_handler_detail.h"

// implemented header
#include "tellocsv_handler.h"


// identifyier to use for position data
#define TRELLO_POSITION_ATTRIB_NAME "position"

// identifyier to use for radius data
#define TRELLOALTITUDE_ATTRIB_NAME "altitude"

// identifyier to use for radius data
#define TRELLO_GPSSPEED_ATTRIB_NAME "gps_speed"

// identifyier to use for radius data
#define TRELLO_RADIUS_ATTRIB_NAME "radius"

// identifyier to use for timestamp attribute
#define TRELLO_TIME_ATTRIB_NAME "time"

// Declare the proper csv_handler implementation type
#define DECLARE_CSV_IMPL typedef typename csv_handler<flt_type>::Impl CSVImpl


////
// Module-private globals

namespace {
	// A single named trello column
	struct TrelloColumn
	{
		const std::string name;
		int id = -1;
		TrelloColumn(const char *name) : name(name) {}
	};

	// A compound-value sourced from multiple Trello columns
	template <unsigned num_cols>
	struct TrelloCompound
	{
		TrelloColumn cols[num_cols];

		template<class... T>
		static TrelloCompound create(const T&... column_names) {
			return {column_names...};
		}

		std::pair<bool, TrelloColumn&> find (const std::string &name)
		{
			for (auto &col : cols)
				if (col.name.compare(name) == 0)
					return {true, col};
			TrelloColumn dummy("");
			return {false, dummy};
		}

		bool check_found (void) const
		{
			bool found = true;
			for (const auto &col : cols)
				found &= col.id >= 0;
			return found;
		}
	};

	// Fields known to contain the 3D position
	struct TrelloPos : public TrelloCompound<3>
	{
		TrelloColumn &x, &y, &z;
		TrelloPos()
			: TrelloCompound(TrelloCompound::create("MVO:posX[meters]", "MVO:posY[meters]", "MVO:posZ[meters]")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}
	};

	// Fields known to contain the 3D velocity vector
	struct TrelloVel : public TrelloCompound<3>
	{
		TrelloColumn &x, &y, &z;
		TrelloVel()
			: TrelloCompound(TrelloCompound::create("MVO:velX[meters/Sec]", "MVO:velY[meters/Sec]", "MVO:velZ[meters/Sec]")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}
	};
};


////
// Class implementation - tellocsv_handler

struct TrelloCSV
{
	// list of all columns in the table
	std::vector<std::string> columns;

	// well-known column for timestamp
	TrelloColumn time = TrelloColumn("Clock:offsetTime");

	// well-known columns for position
	TrelloPos pos;

	// well-known columns for velocity
	TrelloVel vel;

	unsigned remove_trailing_meta_fields (void)
	{
		for (unsigned i=1; i<4; i++)
		{
			const auto &col = columns[columns.size()-i];
			if (col.compare("ConvertDatV3") == 0) {
				columns.pop_back();
				columns.pop_back();
				i-=2;
			}
			else if (col.compare("Attribute|Value") == 0) {
				columns.pop_back();
				i--;
			}
		}
		return (unsigned)columns.size();
	}

	bool locate_known_fields (void)
	{
		for (unsigned i=0; i<(unsigned)columns.size(); i++)
		{
			// time
			if (time.name.compare(columns[i]) == 0)
				time.id = i;

			/* position */ {
				const auto it = pos.find(columns[i]);
				if (it.first)
					it.second.id = i;
			}
			/* velocity */ {
				const auto it = vel.find(columns[i]);
				if (it.first)
					it.second.id = i;
			}
		}
		bool success = time.id >= 0 && pos.check_found() && vel.check_found();
		return success;
	}
};

template <class flt_type>
bool tellocsv_handler<flt_type>::can_handle (std::istream &contents) const
{
	// init
	const stream_pos_guard g(contents);
	DECLARE_CSV_IMPL;
	CSVImpl csv_impl;
	TrelloCSV csv;

	// check for tell-tale stream contents
	std::string line;
	// - parse first row and check if there are enough columns
	const std::string &separators = ",";
	std::vector<cgv::utils::token> tokens;
	unsigned num_cols = CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &csv.columns);
	if (num_cols < 7)
		return false;
	// - check if all the fields we want are there
	CSVImpl::remove_enclosing_quotes(csv.columns);
	num_cols = csv.remove_trailing_meta_fields();
	if (!csv.locate_known_fields())
		return false;
	// - check if the first data line contains a readable timestamp and is of the same length as the header column
	std::vector<std::string> fields;
	if (CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields) < num_cols)
		return false;
	const real ts = CSVImpl::parse_field(fields[csv.time.id]);
	if (ts < std::numeric_limits<real>::infinity())
		return true;

	// the file is suspect!
	return false;
}

template <class flt_type>
traj_dataset<flt_type> tellocsv_handler<flt_type>::read(
	std::istream &contents, DatasetOrigin source, const std::string &path
)
{
	/*size_t nr_objects = 0;
	std::string line;
	std::map<std::string, size_t> type_counts;
	std::vector<obd_response_info> responses;
	std::map<std::string, std::vector<gps_info>> gps_info_map;
	std::map<std::string, std::vector<string_info>> string_series;
	std::map<std::string, std::vector<float_info<flt_type>>> float_series;
	std::map<std::string, std::vector<int_info>> int_series;
	std::map<std::string, std::vector<bool_info>> bool_series;
	do {
		// retrieve line containing single json object
		cgv::os::safe_getline(contents, line);
		if (line.empty())
			continue;
		// parse with nlohmann json
		std::stringstream ss(line);
		nlohmann::json j;
		ss >> j;
		// check for valid objects
		if (!j.is_object() || !j.contains("type"))
			continue;
		// extract data of known types
		if (j["type"] == "OBD_RESPONSE") {
			responses.push_back({
				j["source"].get<std::string>(),
				j["timestamp"].get<size_t>(),
				cgv::utils::parse_hex_bytes(j["data"]["bytes"].get<std::string>())
				});
			if (j["data"].contains("mode")) {
				for (auto i : j["data"].items()) {
					if (i.key() == "mode")
						continue;
					if (i.key() == "pid")
						continue;
					if (i.key() == "bytes")
						continue;
					if (i.key() == "supported_pids")
						continue;
					if (i.key() == "mode")
						continue;
					int pid = -1;
					if (j["data"].contains("pid"))
						pid = j["data"]["pid"].get<int>();
					if (i.value().is_string())
						string_series[i.key()].push_back({ pid, responses.back().timestamp, i.value().get<std::string>() });
					if (i.value().is_number_float())
					{
						std::string key = i.key();
						// special handling for throttle / brake
						if (cgv::utils::to_lower(key).compare("throttleposition")==0)
							if (pid==69)
								key = "brakePosition";
						float_series[key].push_back({ pid, responses.back().timestamp, i.value().get<flt_type>() });
					}
					else if (i.value().is_number_integer())
						int_series[i.key()].push_back({ pid, responses.back().timestamp, i.value().get<int>() });
					else if (i.value().is_boolean())
						bool_series[i.key()].push_back({ pid, responses.back().timestamp, i.value().get<bool>() });
					else
						if (type_counts.find(i.key()) == type_counts.end())
							type_counts[i.key()] = 1;
						else
							++type_counts[i.key()];
				}
			}
		}
		else if (j["type"] == "GPS") {
			gps_info_map[j["source"].get<std::string>()].push_back({
				j["source"].get<std::string>(),
				j["timestamp"].get<size_t>(),
				j["data"]["longitude"].get<double>(),
				j["data"]["latitude"].get<double>(),
				j["data"]["altitude"].get<double>(),
				j["data"]["gps_speed"].get<double>()
			});
		}
		// handle unknown types
		else {
			if (type_counts.find(j["type"]) == type_counts.end())
				type_counts[j["type"]] = 1;
			else
				++type_counts[j["type"]];
			++nr_objects;
		}
	} while (!contents.eof());
	std::cout << "nr unknown parsed objects: " << nr_objects << std::endl;
	for (auto tc : type_counts)
		std::cout << "  " << tc.first << ": " << tc.second << std::endl;*/

	////
	// transform loaded data into traj_mgr dataset

	// perpare dataset container object
	traj_dataset<flt_type> ret;
	static const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {TRELLO_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {TRELLO_RADIUS_ATTRIB_NAME}}
	});/*

	// for single precision float compatibility, we make everything relative to the first sample of the first trajectory position-wise...
#if defined(OBD_USE_ECEF_COORDINATES) && OBD_USE_ECEF_COORDINATES!=0
	const auto refpos = peri::xyzForLpa(peri::LPA{
		gps_info_map.cbegin()->second[0].latitude,
		gps_info_map.cbegin()->second[0].longitude,
		gps_info_map.cbegin()->second[0].altitude
	});
#else
	typedef std::array<double, 2> latlong;
	const latlong refpos = {gps_info_map.cbegin()->second[0].latitude, gps_info_map.cbegin()->second[0].longitude};
#endif
	// ...and relative to the earliest sample among all trajectories time-wise
	double reftime = std::numeric_limits<double>::infinity();
	for (const auto &e : gps_info_map)
		reftime = (double)std::min(e.second[0].timestamp, (size_t)reftime);
	auto convert_time = [&reftime] (size_t timestamp) -> flt_type {
		// ToDo: what do the timestamps mean? They're either in nanoseconds or there is centuries in between samples...
		return (flt_type)((double(timestamp) - reftime)/1000000000.0);
	};

	// create synchronous attributes
	auto P = traj_format_handler<flt_type>::template add_attribute<vec3>(ret, OBD_POSITION_ATTRIB_NAME);
	auto A = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_ALTITUDE_ATTRIB_NAME);
	auto V = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_GPSSPEED_ATTRIB_NAME);
	auto T = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_TIME_ATTRIB_NAME); // for now, commiting timestamps as their own attribute is the only way to have them selectable in the layers
	auto &Ptraj = traj_format_handler<flt_type>::trajectories(ret, P.attrib);

	// commit samples of each trajectory
	// - prepare scale factor for mercator-projected cartesian positions
	constexpr flt_type mscale = 1/(flt_type)OBD_MERCATOR_SCALEDOWN_FACTOR;
	// - various bookkeeping
	double seg_dist_accum = 0;
	unsigned num_segs=0, offset=0;
	// - the loop
	for (const auto &e : gps_info_map)
	{
		// convenience shorthand
		const auto &gps_infos = e.second;

		// only include trajectory if it has at least one segment
		if (gps_infos.size() < 2)
			continue;

		// commit samples
		unsigned num_samples_this_traj = 0;
		for (unsigned i=0; i<(unsigned)gps_infos.size(); i++)
		{
			// convenience shorthand
			const auto &gps_info = gps_infos[i];

			// convert from lattitude/longitude
			#if defined(OBD_USE_ECEF_COORDINATES) && OBD_USE_ECEF_COORDINATES!=0
				const auto xyzpos = peri::xyzForLpa(peri::LPA{gps_info.latitude, gps_info.longitude, gps_info.altitude}),
				           relpos = peri::LPA{xyzpos[0]-refpos[0], xyzpos[1]-refpos[1], xyzpos[2]-refpos[2]};
				vec3 pos((flt_type)relpos[0], (flt_type)relpos[1], (flt_type)relpos[2]);
			#else
				const auto mercator = wgs84::toCartesian(refpos, latlong{gps_info.latitude, gps_info.longitude});
				vec3 pos((flt_type)mercator[0]*mscale, (flt_type)gps_info.altitude*mscale, (flt_type)mercator[1]*mscale);
			#endif

			// keep track of segment length and throw out non-monotone samples
			flt_type time = convert_time(gps_info.timestamp);
			if (i > 0)
			{
				// ToDo: this very crude approach is far from optimal
				if (time <= P.data.timestamps.back())
					continue;

				const auto &prev = P.data.values.back();
				// eliminate duplicates - ToDo: why are there so many?
				if ((pos - prev).sqr_length() < (flt_type)std::numeric_limits<float>::epsilon()*2)
					continue;
				seg_dist_accum += (pos - prev).length();
				num_segs++;
			}

			// commit to storage
			P.data.append(pos, time);
			A.data.append((flt_type)gps_info.altitude, time);
			V.data.append((flt_type)gps_info.gps_speed, time);
			T.data.append(time, time);

			// book-keeping for building trajectory information after this loop
			num_samples_this_traj++;
		}

		// store trajectory info
		Ptraj.emplace_back(range{offset, num_samples_this_traj});
		offset += num_samples_this_traj;
	}

	// At this point, we know if we loaded something useful
	if (num_segs < 1)
		return traj_dataset<flt_type>(); // discard everything done up to now and just return an invalid dataset

	// non-position trajectory infos
	traj_format_handler<flt_type>::trajectories(ret, A.attrib) = Ptraj; // all attributes are sampled in sync with position, so we
	traj_format_handler<flt_type>::trajectories(ret, V.attrib) = Ptraj; // can duplicate the trajectory info from the positions to
	traj_format_handler<flt_type>::trajectories(ret, T.attrib) = Ptraj; // altitude, speed and time as well

	// invent radii
	traj_format_handler<flt_type>::set_avg_segment_length(ret, flt_type(seg_dist_accum / num_segs));
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_RADIUS_ATTRIB_NAME);
	R.data.values = std::vector<flt_type>(P.data.num(), ret.avg_segment_length()*real(0.125));
	R.data.timestamps = P.data.timestamps;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptraj; // invented radius "samples" are again in sync with positions, so just copy traj info

	// commit async attributes
	// ToDo: THIS ASSUMES THERE CAN ONLY EVER BE ONE TRAJECTORY IN AN OBD FILE!!! (which all evidence points to)
	for (const auto &e : float_series)
	{
		auto F = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &f : e.second)
			F.data.append(f.value, convert_time(f.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, F.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}
	for (const auto &e : int_series)
	{
		auto I = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &i : e.second)
			I.data.append((flt_type)i.value, convert_time(i.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, I.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}
	for (const auto &e : bool_series)
	{
		auto B = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &b : e.second)
			B.data.append((flt_type)b.value, convert_time(b.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, B.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}

	// visual attribute mapping
	ret.set_mapping(vamap);

	// set dataset name (we just use the filename for now)
	traj_format_handler<flt_type>::name(ret) = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path));

	// done!*/
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct tellocsv_handler<float>;
template struct tellocsv_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<tellocsv_handler<float> >  flt_trello_reg("Trello flight log handler (float)");
cgv::base::object_registration<tellocsv_handler<double> > dbl_trello_reg("Trello flight log handler (double)");
