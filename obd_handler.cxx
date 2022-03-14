
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
#include <json.hpp>

// implemented header
#include "obd_handler.h"



template <class flt_type>
bool obd_handler<flt_type>::can_handle (std::istream &contents) const
{
	const stream_pos_guard g(contents); // do we need this???
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

	return traj_dataset<flt_type>();
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
