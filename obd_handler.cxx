
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

// 3rd party libs
#include <json.hpp>
#include <peridetic.h>

// implemented header
#include "obd_handler.h"


/// identifyier to use for position data
#define OBD_POSITION_ATTRIB_NAME "position"

/// identifyier to use for radius data
#define OBD_ALTITUDE_ATTRIB_NAME "altitude"

/// identifyier to use for radius data
#define OBD_SPEED_ATTRIB_NAME "speed"

/// identifyier to use for radius data
#define OBD_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for timestamp attribute
#define OBD_TIME_ATTRIB_NAME "time"


template <class flt_type>
bool obd_handler<flt_type>::can_handle (std::istream &contents) const
{
	const stream_pos_guard g(contents); // do we need this??? [Benjamin] I guess not (at least not anymore), but would have to verify a few things to make sure it's safe to remove
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
		if (j["type"] == "META" && j.contains("data")) {
			auto d = j["data"];
			if (d.contains("ppcdf_version"))
				return true;
		}
	}
	return false;
}

struct obd_response_info
{
	std::string guid;
	size_t timestamp;
	std::vector<uint8_t> bytes;
};

struct gps_info_info
{
	std::string source;
	size_t timestamp;
	double longitude;
	double latitude;
	double altitude;
	double gps_speed;
};

uint8_t hex2int(char c)
{
	switch (c) {
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		return c - '0';
	case 'A':
	case 'B':
	case 'C':
	case 'D':
	case 'E':
	case 'F':
		return c - 'A';
	case 'a':
	case 'b':
	case 'c':
	case 'd':
	case 'e':
	case 'f':
		return c - 'a';
	}
	return 0;
}

std::vector<uint8_t> parse_bytes(const std::string& byte_str)
{
	std::vector<uint8_t> bytes;
	for (size_t i = 0; i < byte_str.size(); i += 2)
		bytes.push_back(16 * hex2int(byte_str[i]) + hex2int(byte_str[i + 1]));
	return bytes;
}

template <class flt_type>
traj_dataset<flt_type> obd_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
)
{
	size_t nr_objects = 0;
	std::string line;
	std::map<std::string, size_t> type_counts;
	std::vector<obd_response_info> responses;
	std::vector<gps_info_info> gps_infos;
	do { 
		// retrieve line containing single json object
		std::getline(contents, line);
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
		if (j["type"] == "OBD_RESPONSE")
			responses.push_back({ 
				j["source"].get<std::string>(), 
				j["timestamp"].get<size_t>(), 
				parse_bytes(j["data"]["bytes"].get<std::string>())
			});
		// handle unknown types
		else if (j["type"] == "GPS") {
			gps_infos.push_back({
				j["source"].get<std::string>(),
				j["timestamp"].get<size_t>(),
				j["data"]["longitude"].get<double>(),
				j["data"]["latitude"].get<double>(),
				j["data"]["altitude"].get<double>(),
				j["data"]["gps_speed"].get<double>()
			});
		}
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
		std::cout << "  " << tc.first << ": " << tc.second << std::endl;


	////
	// transform loaded data into traj_mgr dataset

	// perpare dataset container object
	traj_dataset<flt_type> ret;
	if (gps_infos.size() < 2)
		return ret;
	const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {OBD_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {OBD_RADIUS_ATTRIB_NAME}}
	});

	// for float compatibility, we make everything relative to the first sample in terms of position and time
	const auto refpos = peri::xyzForLpa(peri::LPA{gps_infos[0].latitude, gps_infos[0].longitude, gps_infos[0].altitude});
	const auto reftime = gps_infos[0].timestamp;

	// commit positions
	auto P = add_attribute<vec3>(ret, OBD_POSITION_ATTRIB_NAME);
	auto A = add_attribute<flt_type>(ret, OBD_ALTITUDE_ATTRIB_NAME);
	auto V = add_attribute<flt_type>(ret, OBD_SPEED_ATTRIB_NAME);
	auto T = add_attribute<flt_type>(ret, OBD_TIME_ATTRIB_NAME); // for now, commiting timestamps as their own attribute is the only way to have them selectable in the glyph layers
	auto &Ptraj = trajectories(ret, P.attrib);
	double seg_dist_accum = 0;
	for (unsigned i=0; i<(unsigned)gps_infos.size(); i++)
	{
		const auto &gps_info = gps_infos[i];
		const auto xyzpos = peri::xyzForLpa(peri::LPA{gps_info.latitude, gps_info.longitude, gps_info.altitude}),
		           relpos = peri::LPA{xyzpos[0]-refpos[0], xyzpos[1]-refpos[1], xyzpos[2]-refpos[2]};
		vec3 pos((flt_type)relpos[0], (flt_type)relpos[1], (flt_type)relpos[2]);
		if (i > 0)
		{
			const auto &prev = P.data.values.back();
			// eliminate duplicates - ToDo: why are there so many?
			if ((pos-prev).sqr_length() < std::numeric_limits<float>::epsilon()*2)
				continue;
			seg_dist_accum += (pos - P.data.values.back()).length();
		}
		// ToDo: what do the timestamps mean? Their either in nanoseconds or there is centuries in between samples...
		flt_type time = (flt_type)((gps_info.timestamp-reftime)/1000000000);
		P.data.append(pos, time);
		A.data.append((flt_type)gps_info.altitude, time);
		V.data.append((flt_type)gps_info.gps_speed, time);
		T.data.append(time, time);
	}
	Ptraj.emplace_back(range{0, P.data.num()});
	trajectories(ret, A.attrib) = Ptraj;	// all attributes are sampled in sync with position, so we
	trajectories(ret, V.attrib) = Ptraj;	// can duplicate the trajectory info from the positions to
	trajectories(ret, T.attrib) = Ptraj;	// altitude, speed and time as well

	// invent radii
	set_avg_segment_length(ret, flt_type(seg_dist_accum/(P.data.num()-1)));
	auto R = add_attribute<flt_type>(ret, OBD_RADIUS_ATTRIB_NAME);
	R.data.values = std::vector<flt_type>(P.data.num(), ret.avg_segment_length()*real(0.125));
	R.data.timestamps = P.data.timestamps;
	trajectories(ret, R.attrib) = Ptraj;
	// - visual attribute mapping
	ret.set_mapping(vamap);

	// done!
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct obd_handler<float>;
template struct obd_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<obd_handler<float> >  flt_obd_reg("obd trajectory handler (float)");
cgv::base::object_registration<obd_handler<double> > dbl_obd_reg("obd trajectory handler (double)");
