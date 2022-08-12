
// C++ STL
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <utility>
#include <limits>
#include <ctime>

// CGV framework core
#include <cgv/base/register.h>

// CGV framework utilities
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/file.h>

// 3rd party libs
#include <peridetic.h>
#include <WGS84toCartesian.hpp>

// implemented header
#include "sepia_handler.h"


/////
// Some constant defines

/// min supported version
#define SEPIA_CSV_MIN_VERSION 5

/// max supported version
#define SEPIA_CSV_MAX_VERSION 5

/// identifyier to use for position data
#define SEPIA_POSITION_ATTRIB_NAME "DR_Gps"

/// identifyier to use for radius data
#define SEPIA_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for timestamp attribute
#define SEPIA_TIME_ATTRIB_NAME "time"

// whether to use ECEF coordinates instead of Mercator cartesian + altitude
#define SEPIA_USE_ECEF_COORDINATES 0


////
// Local types and variables

// anonymous namespace begin
namespace {

// some no-op expression that can be optimized away
#define DO_NOTHING (0)

// enum specifying the type of SEPIA trajectory file
struct DatasetInfo
{
	typedef unsigned version_t;

	enum Type { SINGLE, COLLECTION, UNKNOWN } type;
	version_t version;
};
// convenience shorthands
typedef DatasetInfo DI;
typedef DatasetInfo::Type DT;

// struct representing some property
template <class T>
struct sepia_traj_prop
{
	typedef T prop_type;

	T val;
	bool encountered=false;
};

// double-like type for representing a timestamp
struct double_time
{
	// the actual time value
	double t;

	// convenience conversion
	inline operator double (void) { return t; }
};

// SePIA sample data types. Even though many of them share the same underlying integral type, they all need different parsing.
// To make the template magic work, they all get their distinct, compiler-discernable type.
// sample of an on/off state
template <class flt_type>
struct sample_on_off
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of an active/inactive state
template <class flt_type>
struct sample_active_inactive
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of an active/inactive regulation state
template <class flt_type>
struct sample_reg_notreg
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of a hit / not hit state of a control (e.g. the breaks)
template <class flt_type>
struct sample_hit_nothit
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of handbrake state
template <class flt_type>
struct sample_handbreak
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of seatbelt state
template <class flt_type>
struct sample_seatbelt
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of an open/closed state
template <class flt_type>
struct sample_open_closed
{
	bool val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of a left/center/right state (negative means left, positive right)
template <class flt_type>
struct sample_left_right
{
	int val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of gear selection indicator state (negative is reverse gears, crawling is not supported)
template <class flt_type>
struct sample_gearsel
{
	int val;
	inline operator flt_type (void) const { return (flt_type)val; }
};
// sample of scalar acceleration state. Needs its own special handling as acceleration can be stored in different units
template <class flt_type>
struct sample_accel
{
	flt_type val;
	inline operator flt_type (void) const { return val; }
};

// ordered timestamp -> sample map (mainly for closest-point queries)
template <class T>
using ordered_samples = std::map<double, T>;

// name -> ordered attribute samples dictionary
template <class T>
using ordered_samples_dict = std::unordered_map<std::string, ordered_samples<T>>;

// internal trajectory representation
template <class flt_type>
struct trajectory
{
	// types
	typedef flt_type real;
	typedef cgv::math::fvec<double, 2> gpsvec;
	typedef cgv::math::fvec<real, 3> vec3;

	// fields
	bool loaded;
	double start_time, toffset;
	ordered_samples<gpsvec> gps;
	ordered_samples_dict<flt_type> attribs_scalar;
	ordered_samples_dict<vec3> attribs_vec3;

	// methods
	trajectory() : loaded(false), start_time(0), toffset(0) {}
	trajectory(const trajectory &other)
		: loaded(other.loaded), start_time(other.start_time), toffset(other.toffset),
		  gps(other.gps), attribs_vec3(other.attribs_vec3), attribs_scalar(other.attribs_scalar)
	{}
	trajectory(trajectory &&other)
		: loaded(other.loaded), start_time(other.start_time), toffset(other.toffset),
		  gps(std::move(other.gps)), attribs_vec3(std::move(other.attribs_vec3)), attribs_scalar(std::move(other.attribs_scalar))
	{
		other.loaded = false;
		other.toffset = other.start_time = 0;
	}
	trajectory& operator= (const trajectory &other)
	{
		loaded = other.loaded;
		start_time = other.start_time;
		toffset = other.toffset;
		gps = other.gps;
		attribs_scalar = other.attribs_scalar;
		attribs_vec3 = other.attribs_vec3;
		return *this;
	}
	trajectory& operator= (trajectory &&other)
	{
		loaded = other.loaded; other.loaded = false;
		start_time = other.start_time; other.start_time = 0;
		toffset = other.toffset; other.toffset = 0;
		gps = std::move(other.gps);
		attribs_scalar = std::move(other.attribs_scalar);
		attribs_vec3 = std::move(other.attribs_vec3);
		return *this;
	}
};

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
struct sepia_handler<flt_type>::Impl {
	// types
	typedef flt_type real;
	typedef sepia_handler::Vec2 Vec2;
	typedef sepia_handler::Vec3 Vec3;
	typedef sepia_handler::Vec4 Vec4;

	// fields
	static const visual_attribute_mapping<real> attrmap;
	inline static const std::string collection_seps=" \t", single_seps="|";

	// helper methods
	// ToDo: factor out all generic parser utils into own module

	// infers type of file from the given header fields.
	inline static bool check_version (unsigned version)
	{
		return version >= SEPIA_CSV_MIN_VERSION && version <= SEPIA_CSV_MAX_VERSION;
	}
	static DatasetInfo check_type (const std::vector<std::string> &fields)
	{
		// return value for unknown files
		static const DatasetInfo unknown = {DT::UNKNOWN, DI::version_t(-1)};

		// check for collection
		if (   fields.size() == 3
			&& fields[0].compare("SEPIA") == 0 && fields[1].compare("TRAJECTORY") == 0 && fields[2].compare("COLLECTION") == 0)
			return DatasetInfo{DT::COLLECTION, 0};
		// check for individual trajectory
		if (fields.size() == 1)
		{
			std::vector<cgv::utils::token> tokens;
			std::vector<std::string> vfields;
			cgv::utils::split_to_tokens(fields[0], tokens, single_seps, false);
			Impl::read_fields(tokens, single_seps, &vfields);
			// check for known file structure
			if (vfields.size() == 2 && vfields[0].compare("Version") == 0)
			{
				unsigned version;
				if (parse_value(&version, vfields[1]))
					return DatasetInfo{DT::SINGLE, version};
			}
			// not an individual SEPIA trajectory after all
			return unknown;
		}
		// it's neither!
		return unknown;
	}
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
	static unsigned read_next_nonempty_line (
		std::string *line_out, std::vector<cgv::utils::token> *tokens, const std::string &separators,
		const std::string &whitespaces, std::istream &contents, std::vector<std::string>* fields_out=nullptr
	)
	{
		tokens->clear();
		unsigned num_tokens = 0;
		do {
			std::getline(contents, *line_out);
			if (!line_out->empty())
			{
				cgv::utils::split_to_tokens(*line_out, *tokens, separators, false, "", "", whitespaces);
				num_tokens = (unsigned)tokens->size();
			}
		}
		while (!contents.eof() && num_tokens < 1);
		return read_fields(*tokens, separators, fields_out);
	}
	inline static unsigned read_next_nonempty_line (
		std::string *line_out, std::vector<cgv::utils::token> *tokens, const std::string &separators,
		std::istream &contents, std::vector<std::string>* fields_out=nullptr
	)
	{
		return read_next_nonempty_line(line_out, tokens, separators, " \t", contents, fields_out);
	}
	inline static bool field_starts_comment (const std::string &field)
	{
		return field.size() >= 2 && field[0] == '/' && field[1] == '/';
	}

	// Timestamp utils
	enum class TimeFmt
	{
		UNKNOWN, SIMPLE_NUMBER, HMS, HMSms, YMW_HMS
	};
	inline static TimeFmt guess_timestamp_format (const std::string &field)
	{
		static const std::string seperators = ":._";
		std::vector<cgv::utils::token> tokens;
		cgv::utils::split_to_tokens(field, tokens, seperators, false);
		unsigned num_fields = 0;
		for (const auto &token : tokens)
		{
			if (is_separator(token, seperators))
				continue;
			num_fields++;
		}
		if (tokens.size() == 1)
			return TimeFmt::SIMPLE_NUMBER;
		else if (num_fields == 3 && tokens.size() > 4)
			return TimeFmt::HMS;
		else if (num_fields == 4 && tokens.size() > 6)
			return TimeFmt::HMSms;
		else if (   num_fields == 2 && tokens.size() > 2
		         && tokens[0].get_length() == 8 && tokens[2].get_length() == 6)
			return TimeFmt::YMW_HMS;
		return TimeFmt::UNKNOWN;
	}

	template <class T>
	inline static T string_to_value (const std::string &str, size_t *pos) { return (T)std::stoll(str, pos); }
	template <>
	inline static float string_to_value<float> (const std::string &str, size_t *pos) { return std::stof(str, pos); }
	template <>
	inline static double string_to_value<double> (const std::string &str, size_t *pos) { return std::stod(str, pos); }
	template <>
	inline static sample_on_off<real> string_to_value<sample_on_off<real>> (const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		if (str_lower.compare("an")==0 || str_lower.compare("on")==0)
			val = true;
		else if (str_lower.compare("aus")==0 || str_lower.compare("off")==0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'on' or 'off'");
		*pos = str.length();
		return {val};
	}
	template <>
	inline static sample_active_inactive<real> string_to_value<sample_active_inactive<real>> (const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		if (str_lower.compare("aktiv")==0 || str_lower.compare("active")==0)
			val = true;
		else if (str_lower.compare("nicht aktiv")==0 || str_lower.compare("inactive")==0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'active' or 'inactive'");
		*pos = str.length();
		return {val};
	}
	template <>
	inline static sample_reg_notreg<real> string_to_value<sample_reg_notreg<real>>(const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		if (str_lower.compare("regelt") == 0)
			val = true;
		else if (str_lower.compare("regelt nicht") == 0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'active' or 'inactive'");
		*pos = str.length();
		return { val };
	}
	template <>
	inline static sample_hit_nothit<real> string_to_value<sample_hit_nothit<real>>(const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		if (str_lower.compare("betaetigt")==0 || str_lower.compare("betätigt")==0)
			val = true;
		else if (str_lower.compare("nicht betaetigt")==0 || str_lower.compare("nicht betätigt")==0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'betaetigt' or 'nicht betaetigt'");
		*pos = str.length();
		return {val};
	}
	template <>
	inline static sample_handbreak<real> string_to_value<sample_handbreak<real>>(const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		// ToDo: find out what the flip the proper terms used by SePIA for an engaged handbrake are...
		if (str_lower.compare("gezogen")==0 || str_lower.compare("angezogen")==0 || str_lower.compare("betaetigt")==0 || str_lower.compare("betätigt")==0)
			val = true;
		else if (str_lower.compare("geloest") == 0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'active' or 'inactive'");
		*pos = str.length();
		return { val };
	}
	template <>
	inline static sample_seatbelt<real> string_to_value<sample_seatbelt<real>>(const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		// ToDo: find out what the flip the proper terms used by SePIA for an engaged handbrake are...
		if (str_lower.compare("gesteckt") == 0)
			val = true;
		/*else if (str_lower.compare("geloest") == 0)
			val = false;*/
		else
			throw std::invalid_argument("string contains no known representation for 'gesteckt' or 'nicht gesteckt'");
		*pos = str.length();
		return { val };
	}
	template <>
	inline static sample_open_closed<real> string_to_value<sample_open_closed<real>>(const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		bool val;
		// ToDo: find out what the flip the proper terms used by SePIA for an engaged handbrake are...
		if (str_lower.compare("offen")==0 || str_lower.compare("open")==0 || str_lower.compare("opened")==0)
			val = true;
		else if (str_lower.compare("geschlossen")==0 || str_lower.compare("closed")==0)
			val = false;
		else
			throw std::invalid_argument("string contains no known representation for 'offen' or 'geschlossen'");
		*pos = str.length();
		return { val };
	}
	template <>
	inline static sample_left_right<real> string_to_value<sample_left_right<real>> (const std::string &str, size_t *pos)
	{
		const std::string str_lower(cgv::utils::to_lower(str));
		int val;
		if (str_lower.compare("links") == 0 || str_lower.compare("left") == 0)
			val = -1;
		else if (str_lower.compare("rechts") == 0 || str_lower.compare("right") == 0)
			val = 1;
		// ToDo: find out what, if any, terms SePIA uses to represent dead-center
		else if (str_lower.compare("mitte") == 0 || str_lower.compare("neutral") == 0 || str_lower.compare("zentral") == 0 || str_lower.compare("center") == 0)
			val = 0;
		else
			throw std::invalid_argument("string contains no known representation for 'active' or 'inactive'");
		*pos = str.length();
		return {val};
	}
	template <>
	inline static sample_gearsel<real> string_to_value<sample_gearsel<real>>(const std::string &str, size_t *pos)
	{
		static const std::string seperators = " \t";
		std::vector<cgv::utils::token> tokens;
		std::vector<std::string> fields;
		cgv::utils::split_to_tokens(str, tokens, seperators, false);
		const auto num_fields = Impl::read_fields(tokens, seperators, &fields);
		if (num_fields < 1 || num_fields > 2)
			throw std::invalid_argument("invalid number of tokens");
		int val;
		size_t pos_local;
		if (num_fields == 1)
			if (fields[0][0] == '-')
			{
				val = 0;
				pos_local = 1;
			}
			else val = std::stoi(fields[0], &pos_local);
		else if (fields[0][0]=='m' || fields[0][0]=='a') // for now, we don't care whether gearbox operates in auto or manual
		{
			const auto pos_tmp = tokens[0].get_length() + tokens[1].get_length();
			val = std::stoi(fields[1], &pos_local);
			pos_local = pos_tmp + pos_local;
		}
		else
			throw std::invalid_argument("string contains no known gear selection descriptor");
		*pos = pos_local;
		return {val};
	}
	template <>
	inline static double_time string_to_value<double_time> (const std::string &str, size_t *pos)
	{
		switch (guess_timestamp_format(str))
		{
			case TimeFmt::SIMPLE_NUMBER:
			{
				double val;
				try { val = (double)std::stod(str, pos); }
				catch (const std::out_of_range&) { val = std::numeric_limits<double>::infinity(); }
				catch (...) { val = -0; }
				return { val };
			}
			case TimeFmt::HMS:
			{
				static const std::string seperators = ":._";
				std::vector<cgv::utils::token> tokens;
				cgv::utils::split_to_tokens(str, tokens, seperators, false);
				double val[3];
				size_t local_pos = 0;
				for (unsigned i=0; i<3; i++)
				{
					const auto &token = tokens[i*2];
					val[i] = string_to_value<double>(std::string(token.begin, size_t(token.end - token.begin)), &local_pos);
					*pos += local_pos + (i>0 ? tokens[i*2-1].get_length() : 0);
				}
				return { val[0]*60*60 + val[1]*60 + val[2] };
			}
			case TimeFmt::HMSms:
			{
				static const std::string seperators = ":._";
				std::vector<cgv::utils::token> tokens;
				cgv::utils::split_to_tokens(str, tokens, seperators, false);
				double val[4];
				size_t local_pos = 0;
				for (unsigned i=0; i<4; i++)
				{
					const auto &token = tokens[i*2];
					val[i] = string_to_value<double>(std::string(token.begin, size_t(token.end - token.begin)), &local_pos);
					*pos += local_pos + (i>0 ? tokens[i*2-1].get_length() : 0);
				}
				return { val[0]*60*60 + val[1]*60 + val[2] + val[3]/1000 };
			}
			case TimeFmt::YMW_HMS:
			{
				static const std::string seperators = ":._";
				std::vector<cgv::utils::token> tokens;
				cgv::utils::split_to_tokens(str, tokens, seperators, false);
				std::tm date;
				size_t local_pos = 0;
				date.tm_year = string_to_value<int>(std::string(tokens[0].begin,   4), &local_pos)-1900; *pos += local_pos;
				date.tm_mon  = string_to_value<int>(std::string(tokens[0].begin+4, 2), &local_pos)-1;    *pos += local_pos;
				date.tm_mday = string_to_value<int>(std::string(tokens[0].begin+6, 2), &local_pos);      *pos += local_pos;
				*pos += tokens[1].get_length();
				date.tm_hour = string_to_value<int>(std::string(tokens[2].begin,   2), &local_pos)-1;    *pos += local_pos;
				date.tm_min  = string_to_value<int>(std::string(tokens[2].begin+2, 2), &local_pos)-1;    *pos += local_pos;
				date.tm_sec  = string_to_value<int>(std::string(tokens[2].begin+4, 2), &local_pos)-1;    *pos += local_pos;
				date.tm_wday = 0;
				date.tm_yday = 0;
				date.tm_isdst = 0;
				double ymdhms = (double)mktime(&date);
				return { ymdhms };
			}
			default:
				throw std::invalid_argument("unrecognized timestamp format");
		}
	}

	template <class T>
	inline static bool parse_value (T *out, const std::string &field) noexcept
	{
		T v;
		size_t pos = 0;
		try { v = string_to_value<T>(field, &pos); }
		catch (...) { return false; }
		const bool success = pos > 0 && pos == field.size();
		if (success)
			*out = v;
		return success;
	}

	inline static bool collection_parse_switch (
		bool *out, const char *name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		if (cgv::utils::to_lower(fields[0].c_str()+1).compare(name) == 0)
		{
			if (fields.size() > 1)
				std::cerr << "[sepia_handler] WARNING: switch '"<<name<<"' has additional arguments specified, ignoring! Line: "<<line << std::endl;
			*out = true;
			return true; 
		}
		return false;
	}

	inline static bool traj_ignore_prop (const char* name, const std::vector<std::string> &fields) noexcept
	{
		return cgv::utils::to_lower(fields[0]).compare(cgv::utils::to_lower(name)) == 0;
	}
	template <class prop_type>
	inline static bool traj_parse_prop(
		prop_type *out, const char* name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		if (cgv::utils::to_lower(fields[0]).compare(cgv::utils::to_lower(name)) == 0)
		{
			if (fields.size() >= 2 && parse_value(&(out->val), fields[1]))
			{
				if (fields.size() > 2)
					std::cerr << "[sepia_handler] WARNING: property '"<<name<<"' has additional arguments specified, ignoring! Line: "<<line << std::endl;
				if (out->encountered)
					std::cerr << "[sepia_handler] WARNING: property '"<<name<<"' specified more than once! Using new value: "<<out->val << std::endl;
				out->encountered = true;
			}
			else
				std::cerr << "[sepia_handler] WARNING: '"<<name<<"' value could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true;
		}
		return false;
	}
	template <>
	inline static bool traj_parse_prop(
		typename sepia_traj_prop<typename trajectory<real>::gpsvec> *out, const char* name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		if (cgv::utils::to_lower(fields[0]).compare(cgv::utils::to_lower(name)) == 0)
		{
			trajectory<real>::gpsvec::value_type c0, c1;
			if (fields.size() >= 4 && parse_value(&c0, fields[2]) && parse_value(&c1, fields[3]))
			{
				out->val.set(c0, c1);
				if (fields.size() > 4)
					std::cerr << "[sepia_handler] WARNING: property '"<<name<<"' has additional arguments specified, ignoring! Line: "<<line << std::endl;
				if (out->encountered)
					std::cerr << "[sepia_handler] WARNING: property '"<<name<<"' specified more than once! Using new value: "<<out->val << std::endl;
				out->encountered = true;
			}
			else
				std::cerr << "[sepia_handler] WARNING: '"<<name<<"' value could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true;
		}
		return false;
	}

	inline static bool traj_ignore_sample (const char* name, const std::vector<std::string> &fields) noexcept
	{
		return cgv::utils::to_lower(fields[1]).compare(cgv::utils::to_lower(name)) == 0;
	}
	template <class sample_type>
	inline static unsigned traj_parse_sample (
		double *ts, sample_type *out, const char* name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		if (cgv::utils::to_lower(fields[1]).compare(cgv::utils::to_lower(name)) == 0)
		{
			if (fields.size() > 2)
			{
				if (fields.size() > 4)
					std::cerr << "[sepia_handler] WARNING: too many columns in '"<<name<<"' sample! Line: "<<line << std::endl;
				double ts_local;
				if (Impl::parse_value(&ts_local, fields[0]) && Impl::parse_value(out, fields[2]))
				{
					*ts = ts_local; // commit timestamp
					return 1;
				}
			}
			std::cerr << "[sepia_handler] WARNING: '"<<name<<"' value could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return non-zero even in case we couldn't parse it
			return 2;
		}
		return 0;
	}
	template <>
	inline static unsigned traj_parse_sample<sample_accel<real>> (
		double *ts, sample_accel<real> *out, const char* name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		if (cgv::utils::to_lower(fields[1]).compare(cgv::utils::to_lower(name)) == 0)
		{
			if (fields.size() > 2)
			{
				if (fields.size() > 4)
					std::cerr << "[sepia_handler] WARNING: too many columns in '"<<name<<"' sample! Line: "<<line << std::endl;
				double ts_local;
				real val;
				if (Impl::parse_value(&ts_local, fields[0]) && Impl::parse_value(&val, fields[2]))
				{
					if (fields.size() > 3)
					{
						std::string unit = cgv::utils::to_lower(fields[3]);
						if (unit.compare("g") == 0)
							val *= real(9.8067);
						else
							std::cerr << "[sepia_handler] WARNING: unknown unit specified for '"<<name<<"', committing value as-is. Line: "<<line << std::endl;
					}
					else
						std::cerr << "[sepia_handler] WARNING: no unit specified for '"<<name<<"', committing value as-is. Line: "<<line << std::endl;
					// commit values
					*ts = ts_local; 
					out->val = val;
					return 1;
				}
			}
			std::cerr << "[sepia_handler] WARNING: '"<<name<<"' value could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return non-zero even in case we couldn't parse it
			return 2;
		}
		return 0;
	}
	template <class T, class sample_type>
	inline static unsigned traj_parse_sample_and_commit (
		ordered_samples_dict<T> *attrib_store, sample_type &reg, const char *name, const std::vector<std::string> &fields, const std::string &line
	) noexcept
	{
		double ts_local;
		unsigned parse_result = traj_parse_sample(&ts_local, &reg, name, fields, line);
		if (parse_result == 1)
		{
			// parsing successful, commmit to storage
			auto &attrib = (*attrib_store)[name];
			auto it = attrib.find(ts_local);
			if (it == attrib.end())
				attrib.emplace(ts_local, (T)reg);
			else
			{
				std::cerr << "[sepia_handler] WARNING: duplicate sample timestamp in attribute '"<<name<<"', overwriting previous value" << std::endl;
				it->second = (T)reg;
			}
			return 1;
		}
		// just pass on parsing result
		return parse_result;
	}

	static trajectory<real> load_trajectory (unsigned format_version, std::istream &contents)
	{
		// parsing workspace
		std::string line;
		std::vector<cgv::utils::token> tokens;
		std::vector<std::string> fields;

		// properties
		sepia_traj_prop<double_time> time0, timeN;
		sepia_traj_prop<trajectory<real>::gpsvec> gps0, gpsN;

		// trajectory properties
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::single_seps, "", contents, &fields))
		{
			// parse handled properties
			if      (Impl::traj_parse_prop(&time0, "Timestamp Start", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_prop(&timeN, "Timestamp Ende", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_prop(&gps0, "GPS Start", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_prop(&gpsN, "GPS Ende", fields, line)) DO_NOTHING;

			// skip ignored properties
			else if (Impl::traj_ignore_prop("Triggernummer", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Trigger-Timestamp", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("DD-Sensor", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Konfiguration", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Triggergrund", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS Trigger", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS alt", fields)) DO_NOTHING;

			// check if header is finished
			else if (cgv::utils::to_lower(line).compare("zeit|signal|wert|einheit") == 0)
				break;

			/// unknown declaration, print warning
			else
				std::cerr << "[sepia_handler] WARNING: unknown property: '"<<fields[0]<<"', ignoring" << std::endl;
		}
		if (contents.eof())
		{
			std::cerr << "[sepia_handler] ERROR: reached end-of-file before any actual data!" << std::endl;
			return trajectory<real>();
		}

		// samples
		// - typed "registers"
		double ts, dbl;
		real scl;
		int ival;
		sample_on_off<real> oo;
		sample_active_inactive<real> ai;
		sample_reg_notreg<real> rn;
		sample_hit_nothit<real> hn;
		sample_handbreak<real> hb;
		sample_seatbelt<real> sb;
		sample_open_closed<real> oc;
		sample_left_right<real> lr;
		sample_gearsel<real> gs;
		sample_accel<real> a;
		// - temporary attribute storage
		trajectory<real> traj;
		std::unordered_set<std::string> unknowns;
		// - set relevant properties from header
		if (time0.encountered)
			traj.start_time = time0.val;
		if (gps0.encountered)
			traj.gps[0] = gps0.val;
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::single_seps, "", contents, &fields))
		{
			// parse handled sample types
			// - boolean states
			if      (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Motor_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Blinker_L_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Blinker_R_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Blinker_Warnblinker_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Licht_Abblend_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Licht_Fern_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Licht_Hupe_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Licht_Nebelschluss_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Licht_Stand_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Gurt_WL_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Nebelschluss_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, rn, "ABS_E", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, rn, "ABS_E_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "ABS_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "GW_ABS_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "GW_ABS_WL_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, ai, "ESP_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, ai, "ESP_S_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, rn, "ESP_E", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, rn, "ESP_E_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "ESP_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "GW_ESP_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, ai, "Zusatz_Bremse_Licht_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, hn, "Zusatz_Kickdown", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, hn, "Bremse_Pedal_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, hn, "Bremse_Pedal_S_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, hn, "Bremse_Pedal_S_2", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, hb, "Bremse_Hand", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Airbag_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Airbag_WL_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, sb, "Gurt_VL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Gurt_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Batterie_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_KuehlMtemp_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Oeldruck_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_GW_Werkstatt_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "GW_ESP_KL_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "GW_ESP_KL_2", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_Motor_Kuehlung_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_Anlassschalter", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_Heckscheibe_Heizung", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_Klimaanlage", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oo, "Zusatz_Lueftung_Innenraum_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oc, "Zusatz_Tuer_links", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, oc, "Zusatz_Tuer_rechts", fields, line)) DO_NOTHING;
			// - discrete state
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, ival, "Gang_Wahl", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, ival, "Gang_Getriebe", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, gs, "Zusatz_Gang_Anzeige_KI", fields, line)) DO_NOTHING;
			// - scalar values
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Motor_n", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Motor_n_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Zusatz_Motor_Temperatur", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Rad_n_VL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Rad_n_VR", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Rad_n_HL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Rad_n_HR", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "FZ_v", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "FZ_v_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, a, "FZ_a_y", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, a, "FZ_a_y_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, a, "FZ_a_y_2", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "FZ_Gierrate", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "DR_Gps_v", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "DR_Gierrate", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Zusatz_Bremse_Moment", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Batterie_U", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Temperatur_Aussen", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Temperatur_Aussen_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "KM", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "KM_1", fields, line)) DO_NOTHING;
			// - percentages
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Gaspedal_W", fields, line) == 1) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Zusatz_Tankinhalt", fields, line) == 1) DO_NOTHING;
			else if (Impl::traj_parse_sample_and_commit(&traj.attribs_scalar, scl, "Zusatz_Licht_Instrumente", fields, line) == 1) DO_NOTHING;
			// - scalars that need additional attributes for deciding the sign
			else if (Impl::traj_parse_sample(&ts, &scl, "Lenkrad_W_B", fields, line) == 1) {
				auto &steering_pos = traj.attribs_scalar["Lenkrad_W"];
				if (steering_pos.find(ts) == steering_pos.end())
					steering_pos[ts] = scl;
				else
					steering_pos[ts] *= scl;
			}
			else if (Impl::traj_parse_sample(&ts, &lr, "Lenkrad_W_R", fields, line) == 1) {
				auto &steering_pos = traj.attribs_scalar["Lenkrad_W"];
				if (steering_pos.find(ts) == steering_pos.end())
					steering_pos[ts] = lr;
				else
					steering_pos[ts] *= lr;
			}
			else if (Impl::traj_parse_sample(&ts, &scl, "Lenkrad_W_v_B", fields, line) == 1) {
				auto &steering_vel = traj.attribs_scalar["Lenkrad_W_v"];
				if (steering_vel.find(ts) == steering_vel.end())
					steering_vel[ts] = scl;
				else
					steering_vel[ts] *= scl;
			}
			else if (Impl::traj_parse_sample(&ts, &lr, "Lenkrad_W_v_R", fields, line) == 1) {
				auto &steering_vel = traj.attribs_scalar["Lenkrad_W_v"];
				if (steering_vel.find(ts) == steering_vel.end())
					steering_vel[ts] = lr;
				else
					steering_vel[ts] *= lr;
			}
			// - vectors
			else if (Impl::traj_parse_sample(&ts, &a, "DR_a_x", fields, line) == 1) traj.attribs_vec3["DR_a"][ts].x() = a.val;
			else if (Impl::traj_parse_sample(&ts, &a, "DR_a_y", fields, line) == 1) traj.attribs_vec3["DR_a"][ts].y() = a.val;
			else if (Impl::traj_parse_sample(&ts, &a, "DR_a_z", fields, line) == 1) traj.attribs_vec3["DR_a"][ts].z() = a.val;
			else if (Impl::traj_parse_sample(&ts, &dbl, "DR_Gps_Latitude", fields, line) == 1) traj.gps[ts].x() = dbl;
			else if (Impl::traj_parse_sample(&ts, &dbl, "DR_Gps_Longitude", fields, line) == 1) traj.gps[ts].y() = dbl;

			// skip ignored properties
			else if (Impl::traj_ignore_sample("Rad_n_VL_1", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("Bremse_DOT_S", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("DR_Gps_Date", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("DR_Gps_Time", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("DR_Trigger", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("Zusatz_Fahr_S", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("Zusatz_Parksperre", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_sample("Zusatz_Zuendung_Ganganzeige", fields)) DO_NOTHING;

			// unknown sample type, print warning
			else
			{
				// some logic to make sure we print this warning only once
				if (unknowns.find(fields[1]) == unknowns.end())
				{
					std::cerr << "[sepia_handler] WARNING: unknown sample type: '"<<fields[1]<<"', ignoring" << std::endl;
					unknowns.emplace(fields[1]);
				}
			}

			// for setting end-of-loop breakpoints
			std::cerr.flush();
		}

		// post-cleanup
		if (traj.gps.size() > 2)
		{
			// quick-and-dirty hack to get rid of duplicate first GPS sample in some datasets
			auto it_first = traj.gps.cbegin(), it_second = it_first; it_second++;
			if ((it_second->second - it_first->second).sqr_length() < std::numeric_limits<float>::epsilon()*2)
				traj.gps.erase(it_second);
		}

		// finally, we introduce the timestamps of the least-frequently sampled scalar attribute as its own attribute
		auto it_lf = traj.attribs_scalar.cbegin();
		unsigned num_samples = std::numeric_limits<unsigned>::max();
		for (auto it=traj.attribs_scalar.cbegin(); it!=traj.attribs_scalar.cend(); it++)
			if (it->second.size() < num_samples)
			{
				num_samples = (unsigned)it->second.size();
				it_lf = it;
			}
		if (it_lf != traj.attribs_scalar.cend())
		{
			auto &tsattrib = traj.attribs_scalar[SEPIA_TIME_ATTRIB_NAME];
			for (const auto &sample : it_lf->second)
				tsattrib.emplace(sample.first, (real)sample.first);
		}

		// done!
		traj.loaded = true;
		return std::move(traj);
	}
};
template <class flt_type>
const visual_attribute_mapping<flt_type> sepia_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {SEPIA_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {SEPIA_RADIUS_ATTRIB_NAME}}
});

template <class flt_type>
bool sepia_handler<flt_type>::can_handle (std::istream &contents) const
{
	std::string str;
	const stream_pos_guard g(contents);

	// check for tell-tale stream contents
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> idfields;
	std::getline(contents, str);
	cgv::utils::split_to_tokens(str, tokens, Impl::collection_seps, false);
	Impl::read_fields(tokens, Impl::collection_seps, &idfields);
	const auto dsinfo = Impl::check_type(idfields);

	// we'll just say we can read it if we could recognize its internal type
	return dsinfo.type == DT::SINGLE || dsinfo.type == DT::COLLECTION;
}

template <class flt_type>
traj_dataset<flt_type> sepia_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string& path
)
{
	// return value
	traj_dataset<real> ret;

	// parsing workspace
	std::string line;
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> fields;

	// find out dataset type
	std::getline(contents, line);
	cgv::utils::split_to_tokens(line, tokens, Impl::collection_seps, false);
	Impl::read_fields(tokens, Impl::collection_seps, &fields);
	const auto dsinfo = Impl::check_type(fields);

	// loaded trajectories working database
	std::vector<trajectory<real>> trajs;

	// load single trajectory or collection
	bool consistent_timestamps=false, retain_inconsistent_attribs=false;
	if (dsinfo.type == DT::SINGLE)
	{
		// version check
		if (!Impl::check_version(dsinfo.version))
			std::cerr << "[sepia_handler] WARNING: trajectory format version "<<dsinfo.version<<" is unsupported!" << std::endl;

		// delegate to individual trajectory loading code
		trajs.emplace_back(Impl::load_trajectory(dsinfo.version, contents));
		if (!trajs.back().loaded)
			return std::move(ret);
	}
	else
	{
		// Parse trajectory collection description
		// - spec database
		std::vector<std::string> traj_files;
		// - parse line by line
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::collection_seps, contents, &fields))
		{
			// handle options
			if (fields[0][0] == '!')
			{
				// sanity check
				if (fields[0].size() < 2)
				{
					std::cerr << "[sepia_handler] WARNING: option specifier '!' encountered without option name, ignoring" << std::endl;
					continue;
				}

				// parse it
				if      (Impl::collection_parse_switch(&consistent_timestamps, "consistent_timestamps", fields, line)) DO_NOTHING;
				else if (Impl::collection_parse_switch(&retain_inconsistent_attribs, "retain_inconsistent_attribs", fields, line))
					std::cerr << "[sepia_handler] WARNING: retaining of inconsistent attribute declarations across trajectories not yet implemented!" << std::endl;

				// unknown declaration, print warning
				else
					std::cerr << "[sepia_handler] WARNING: unknown option: '"<<fields[0].c_str()+1<<"', ignoring" << std::endl;
			}
			// handle trajectory file entry
			else
			{
				bool trajfile_not_found = true;
				if (cgv::utils::file::exists(line))
				{
					trajfile_not_found = false;
					traj_files.emplace_back(line);
				}
				else {
					std::string possible_fn = cgv::utils::file::get_path(path)+"/"+line;
					if (cgv::utils::file::exists(possible_fn))
					{
						trajfile_not_found = false;
						traj_files.emplace_back(possible_fn);
					}
				}
				if (trajfile_not_found)
					std::cerr << "[sepia_handler] WARNING: trajectory file '"<<line<<"'could not be found, ignoring" << std::endl;
			}
		}
		// - load individual trajectories
		if (traj_files.empty())
		{
			std::cerr << "[sepia_handler] ERROR: trajectory collection does not specify any existent files!" << std::endl;
			return std::move(ret);
		}

		// load all trajectory files in the collection
		for (const auto file : traj_files)
		{
			std::ifstream contents(file);
			if (contents.is_open())
			{
				line.clear(); tokens.clear(); fields.clear();
				std::getline(contents, line);
				cgv::utils::split_to_tokens(line, tokens, Impl::collection_seps, false);
				Impl::read_fields(tokens, Impl::collection_seps, &fields);
				const auto dsinfo = Impl::check_type(fields);
				if (dsinfo.type == DT::SINGLE)
				{
					if (!Impl::check_version(dsinfo.version))
						std::cerr << "[sepia_handler] WARNING: trajectory file '"<<file<<"' uses unsupported format version "<<dsinfo.version << std::endl;
					auto traj_new = Impl::load_trajectory(dsinfo.version, contents);
					if (traj_new.loaded)
						trajs.emplace_back(std::move(traj_new));
					else
						std::cerr << "[sepia_handler] WARNING: trajectory file '"<<file<<"' could not be loaded properly" << std::endl;
				}
				else
					std::cerr << "[sepia_handler] WARNING: file '"<<file<<"' does not appear to contain a SEPIA individual trajectory definition, ignoring" << std::endl;
			}
			else
				std::cerr << "[sepia_handler] WARNING: cannot open trajectory file '"<<file<<'\'' << std::endl;
		}
	}

	// unify trajectories
	// - for single precision float compatibility, we make every position relative to the first sample of the first trajectory
	constexpr real altitude = 0;
#if defined(SEPIA_USE_ECEF_COORDINATES) && SEPIA_USE_ECEF_COORDINATES!=0
	const auto refpos = peri::xyzForLpa(peri::LPA{
		trajs[0].gps.cbegin()->second.x(), trajs[0].gps.cbegin()->second.y(), altitude
	});
#else
	typedef std::array<double, 2> latlong;
	const latlong refpos = {trajs[0].gps.cbegin()->second.x(), trajs[0].gps.cbegin()->second.y()};
#endif
	// - prepare timestamp unification
	if (consistent_timestamps)
	{
		double tstart_min =  std::numeric_limits<double>::infinity();
		for (const auto &traj : trajs)
			tstart_min = std::min(traj.start_time, tstart_min);
		for (auto &traj : trajs)
			traj.toffset = traj.start_time - tstart_min;
	}
	// - unifying of inconsistent attributes
	/*if (retain_inconsistent_attribs)
		// ToDo: implement!
	else*/
	{
		std::unordered_set<std::string>
			all_scalar_attribs, all_vec3_attribs, consistent_scalar_attribs, consistent_vec3_attribs;

		// check attribute consistency
		for (const auto &traj : trajs)
		{
			for (const auto &scl : traj.attribs_scalar)
				all_scalar_attribs.emplace(scl.first);
			for (const auto &v3 : traj.attribs_vec3)
				all_vec3_attribs.emplace(v3.first);
		}
		for (const auto &aname : all_scalar_attribs)
		{
			bool consistent = true;
			for (const auto &traj : trajs)
				if (traj.attribs_scalar.find(aname) == traj.attribs_scalar.end())
				{
					consistent = false;
					break;
				}
			if (consistent)
				consistent_scalar_attribs.emplace(aname);
			else
				std::cerr << "[sepia_handler] throwing out inconsistent attribute '"<<aname<<"'!" << std::endl;
		}
		for (const auto &aname : all_vec3_attribs)
		{
			bool consistent = true;
			for (const auto &traj : trajs)
				if (traj.attribs_vec3.find(aname) == traj.attribs_vec3.end())
				{
					consistent = false;
					break;
				}
			if (consistent)
				consistent_vec3_attribs.emplace(aname);
			else
				std::cerr << "[sepia_handler] throwing out inconsistent attribute '"<<aname<<"'!" << std::endl;
		}

		// unify into common attribute trajectory structure
		// - positions
		auto P = add_attribute<Vec3>(ret, SEPIA_POSITION_ATTRIB_NAME);
		auto &Ptraj = trajectories(ret, P.attrib);
		real seg_dist_accum = 0;
		unsigned num_segs = 0;
		for (const auto &traj : trajs)
		{
			const unsigned offset = P.data.num();
			for (auto sample=traj.gps.cbegin(); sample!=traj.gps.cend(); sample++)
			{
				const auto &gpspos = sample->second;
				#if defined(SEPIA_USE_ECEF_COORDINATES) && SEPIA_USE_ECEF_COORDINATES!=0
					const auto xyzpos = peri::xyzForLpa(peri::LPA{gpspos.x(), gpspos.y(), altitude}),
					           relpos = peri::LPA{xyzpos[0]-refpos[0], xyzpos[1]-refpos[1], xyzpos[2]-refpos[2]};
					Vec3 pos((flt_type)relpos[0], (flt_type)relpos[1], (flt_type)relpos[2]);
				#else
					const auto mercator = wgs84::toCartesian(refpos, latlong{gpspos.x(), gpspos.y()});
					Vec3 pos((flt_type)mercator[0], altitude, (flt_type)mercator[1]);
				#endif
				if (sample != traj.gps.cbegin())
				{
					seg_dist_accum += (pos - P.data.values.back()).length();
					num_segs++;
				}
				P.data.append(pos, (real)(sample->first + traj.toffset));
			}
			Ptraj.emplace_back(range{offset, (unsigned)traj.gps.size()});
		}
		set_avg_segment_length(ret, seg_dist_accum / num_segs);
		// - scalar attributes
		for (const auto &aname : consistent_scalar_attribs)
		{
			auto A = add_attribute<real>(ret, aname);
			auto &Atraj = trajectories(ret, A.attrib);
			if (aname.compare(SEPIA_TIME_ATTRIB_NAME)) for (auto &traj : trajs)
			{
				const unsigned offset = A.data.num();
				const auto &attrib_src = traj.attribs_scalar[aname];
				for (const auto &sample : attrib_src)
					A.data.append(sample.second, (real)(sample.first+traj.toffset));
				Atraj.emplace_back(range{offset, (unsigned)attrib_src.size()});
			}
			else for (auto &traj : trajs)
			{
				const unsigned offset = A.data.num();
				const auto &attrib_src = traj.attribs_scalar[aname];
				for (const auto &sample : attrib_src)
					A.data.append((real)(double(sample.second)+traj.toffset), (real)(sample.first+traj.toffset));
				Atraj.emplace_back(range{offset, (unsigned)attrib_src.size()});
			}
		}
		// - vec3 attributes
		for (const auto &aname : consistent_vec3_attribs)
		{
			auto A = add_attribute<Vec3>(ret, aname);
			auto &Atraj = trajectories(ret, A.attrib);
			for (auto &traj : trajs)
			{
				const unsigned offset = A.data.num();
				const auto &attrib_src = traj.attribs_vec3[aname];
				for (const auto &sample : attrib_src)
					A.data.append(sample.second, (real)(sample.first+traj.toffset));
				Atraj.emplace_back(range{offset, (unsigned)attrib_src.size()});
			}
		}
		// - invent radii
		auto R = add_attribute<real>(ret, SEPIA_RADIUS_ATTRIB_NAME);
		R.data.values = std::vector<real>(P.data.num(), ret.avg_segment_length()*real(0.125));
		R.data.timestamps = P.data.timestamps;
		trajectories(ret, R.attrib) = trajectories(ret, P.attrib);
		// - visual attribute mapping
		ret.set_mapping(Impl::attrmap);
	}

	// done!
	return std::move(ret);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct sepia_handler<float>;
template struct sepia_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<sepia_handler<float> > flt_sepia_reg("sepia trajectory handler (float)");
cgv::base::object_registration<sepia_handler<double> > dbl_sepia_reg("sepia trajectory handler (double)");
