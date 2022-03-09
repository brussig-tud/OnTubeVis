
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

// implemented header
#include "sepia_handler.h"


/////
// Some constant defines

/// min supported version
#define SEPIA_CSV_MIN_VERSION 5

/// max supported version
#define SEPIA_CSV_MAX_VERSION 5

/// identifyier to use for position data
#define SEPIA_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define SEPIA_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define SEPIA_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define SEPIA_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color gradients
#define SEPIA_DCOLOR_ATTRIB_NAME "dcolor"


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

// sample of an on/off state
struct sample_on_off
{
	bool val;
};
// sample of an active/inactive state
struct sample_active_inactive
{
	bool val;
};
// sample of an active/inactive regulation state
struct sample_reg_notreg
{
	bool val;
};
// sample of a hit / not hit state of a control (e.g. the breaks)
struct sample_hit_nothit
{
	bool val;
};
// sample of handbrake state
struct sample_handbreak
{
	bool val;
};
// sample of a left/center/right state (negative means left, positive right)
struct sample_left_right
{
	int val;
};
// sample of gear selection indicator state (negative is reverse gears, crawling is not supported)
struct sample_gearsel
{
	int val;
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
	inline static sample_on_off string_to_value<sample_on_off> (const std::string &str, size_t *pos)
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
	inline static sample_active_inactive string_to_value<sample_active_inactive> (const std::string &str, size_t *pos)
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
	inline static sample_reg_notreg string_to_value<sample_reg_notreg>(const std::string &str, size_t *pos)
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
	inline static sample_hit_nothit string_to_value<sample_hit_nothit>(const std::string &str, size_t *pos)
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
	inline static sample_handbreak string_to_value<sample_handbreak>(const std::string &str, size_t *pos)
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
	inline static sample_left_right string_to_value<sample_left_right> (const std::string &str, size_t *pos)
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
	inline static sample_gearsel string_to_value<sample_gearsel>(const std::string &str, size_t *pos)
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
			if (   (fields.size()==2 || (fields.size() > 2 && Impl::field_starts_comment(fields[2])))
			    && Impl::parse_value(&(out->val), fields[1]))
			{
				if (out->encountered)
					std::cerr << "[sepia_handler] WARNING: '"<<name<<"' specified more than once! Using new value: "<<out->val << std::endl;
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
		return cgv::utils::to_lower(fields[0]).compare(cgv::utils::to_lower(name)) == 0;
	}
	template <class sample_type>
	inline static bool traj_parse_sample (
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
					return true;
				}
			}
			std::cerr << "[sepia_handler] WARNING: '"<<name<<"' value could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true;
		}
		return false;
	}

	static bool load_trajectory (traj_dataset<real> *ds_out, unsigned format_version, std::istream &contents)
	{
		// parsing workspace
		std::string line;
		std::vector<cgv::utils::token> tokens;
		std::vector<std::string> fields;

		// properties
		sepia_traj_prop<double_time> time0, timeN;

		// trajectory properties
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::single_seps, "", contents, &fields))
		{
			// parse handled properties
			if      (Impl::traj_parse_prop(&time0, "Timestamp Start", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_prop(&timeN, "Timestamp Ende", fields, line)) DO_NOTHING;

			// skip ignored properties
			else if (Impl::traj_ignore_prop("Triggernummer", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Trigger-Timestamp", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("DD-Sensor", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Konfiguration", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("Triggergrund", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS Start", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS Ende", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS Trigger", fields)) DO_NOTHING;
			else if (Impl::traj_ignore_prop("GPS alt", fields)) DO_NOTHING;

			// check if header is finished
			else if (cgv::utils::to_lower(line).compare("zeit|signal|wert|einheit") == 0)
				break;

			/// unknown declaration, print warning
			else
				std::cerr << "[sepia_handler] WARNING: unknown property: '"<<fields[0]<<"', ignoring" << std::endl;
		}

		// samples
		double ts, dbl;
		int ival;
		real flt, pct;
		sample_on_off oo;
		sample_active_inactive ai;
		sample_reg_notreg rn;
		sample_hit_nothit hn;
		sample_left_right lr;
		sample_handbreak hb;
		sample_gearsel gs;
		std::unordered_set<std::string> unknowns;
		std::map<double, cgv::math::fvec<double, 2>> GPS_pos;
		std::map<double, Vec3> DR_a;
		std::map<double, real> steering_pos;
		std::map<double, real> steering_vel;
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::single_seps, "", contents, &fields))
		{
			// parse handled sample types
			real v;
			// - boolean states
			if      (Impl::traj_parse_sample(&ts, &oo, "Motor_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Blinker_L_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Blinker_R_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Blinker_Warnblinker_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Licht_Abblend_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Licht_Fern_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Licht_Hupe_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Licht_Nebelschluss_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Licht_Stand_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Zusatz_GW_Gurt_WL_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "Zusatz_GW_Nebelschluss_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &rn, "ABS_E", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &rn, "ABS_E_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "ABS_WL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &ai, "ESP_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &ai, "ESP_S_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &rn, "ESP_E", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &rn, "ESP_E_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &oo, "ESP_KL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &ai, "Zusatz_Bremse_Licht_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &hn, "Zusatz_Kickdown", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &hn, "Bremse_Pedal_S", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &hn, "Bremse_Pedal_S_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &hn, "Bremse_Pedal_S_2", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &hb, "Bremse_Hand", fields, line)) DO_NOTHING;
			// - discrete state
			else if (Impl::traj_parse_sample(&ts, &ival, "Gang_Wahl", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &ival, "Gang_Getriebe", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &gs, "Zusatz_Gang_Anzeige_KI", fields, line)) DO_NOTHING;
			// - scalar values
			else if (Impl::traj_parse_sample(&ts, &flt, "Motor_n", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Rad_n_VL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Rad_n_VR", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Rad_n_HL", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Rad_n_HR", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "FZ_v", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "FZ_a_y", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "FZ_a_y_1", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "FZ_Gierrate", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "DR_Gps_v", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "DR_Gierrate", fields, line)) DO_NOTHING; 
			else if (Impl::traj_parse_sample(&ts, &flt, "Zusatz_Bremse_Moment", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Batterie_U", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "Temperatur_Aussen", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &flt, "KM", fields, line)) DO_NOTHING;
			// - scalars that need additional attributes for deciding the sign
			else if (Impl::traj_parse_sample(&ts, &v, "Lenkrad_W_B", fields, line)) {
				if (steering_pos.find(ts) == steering_pos.end())
					steering_pos[ts] = v;
				else
					steering_pos[ts] *= v;
			}
			else if (Impl::traj_parse_sample(&ts, &lr, "Lenkrad_W_R", fields, line)) {
				if (steering_pos.find(ts) == steering_pos.end())
					steering_pos[ts] = (real)lr.val;
				else
					steering_pos[ts] *= (real)lr.val;
			}
			else if (Impl::traj_parse_sample(&ts, &v, "Lenkrad_W_v_B", fields, line)) {
				if (steering_vel.find(ts) == steering_vel.end())
					steering_vel[ts] = v;
				else
					steering_vel[ts] *= v;
			}
			else if (Impl::traj_parse_sample(&ts, &lr, "Lenkrad_W_v_R", fields, line)) {
				if (steering_vel.find(ts) == steering_vel.end())
					steering_vel[ts] = (real)lr.val;
				else
					steering_vel[ts] *= (real)lr.val;
			}
			// - percentages
			else if (Impl::traj_parse_sample(&ts, &pct, "Gaspedal_W", fields, line)) DO_NOTHING;
			else if (Impl::traj_parse_sample(&ts, &pct, "Zusatz_Licht_Instrumente", fields, line)) DO_NOTHING;
			// - vectors
			else if (Impl::traj_parse_sample(&ts, &flt, "DR_a_x", fields, line)) DR_a[ts].x() = flt;
			else if (Impl::traj_parse_sample(&ts, &flt, "DR_a_y", fields, line)) DR_a[ts].y() = flt;
			else if (Impl::traj_parse_sample(&ts, &flt, "DR_a_z", fields, line)) DR_a[ts].z() = flt;
			else if (Impl::traj_parse_sample(&ts, &dbl, "DR_Gps_Latitude", fields, line)) GPS_pos[ts].x() = dbl;
			else if (Impl::traj_parse_sample(&ts, &dbl, "DR_Gps_Longitude", fields, line)) GPS_pos[ts].y() = dbl;

			// skip ignored properties
			else if (Impl::traj_ignore_sample("Blinker_R_S", fields)) DO_NOTHING;

			/// unknown sample type, print warning
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

		// done!
		return false;
	}
};
template <class flt_type>
const visual_attribute_mapping<flt_type> sepia_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {SEPIA_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {SEPIA_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {SEPIA_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {SEPIA_COLOR_ATTRIB_NAME}}
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

	// decide how to proceed
	if (dsinfo.type == DT::SINGLE)
	{
		if (!Impl::check_version(dsinfo.version))
			std::cerr << "[sepia_handler] WARNING: trajectory format version "<<dsinfo.version<<" is unsupported!" << std::endl;
		// delegate to individual trajectory loading code
		Impl::load_trajectory(&ret, dsinfo.version, contents);
	}
	else
	{
		// Parse trajectory collection description
		// - spec database
		bool consistent_timestamps = false;
		std::vector<std::string> traj_files;
		// - parse line by line
		while (   !contents.eof()
		       && Impl::read_next_nonempty_line(&line, &tokens, Impl::collection_seps, contents, &fields))
		{
			// handle options
			if (fields[0][0] == '!')
			{
				if (fields[0].size() < 2)
				{
					std::cerr << "[sepia_handler] WARNING: option specifier '!' encountered without option name, ignoring" << std::endl;
					continue;
				}

				// parse it
				if (Impl::collection_parse_switch(&consistent_timestamps, "consistent_timestamps", fields, line)) DO_NOTHING;

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
			std::cerr << "[sepia_handler] ERROR: trajectory collection does not specify any existent files!" << std::endl;
		else
		{
			std::vector<attribute_map<real>> attribs;
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
						traj_dataset<real> new_ds;
						if (!Impl::check_version(dsinfo.version))
							std::cerr << "[sepia_handler] WARNING: trajectory file '"<<file<<"' uses unsupported format version "<<dsinfo.version << std::endl;
						if (Impl::load_trajectory(&new_ds, dsinfo.version, contents))
							attribs.emplace_back(std::move(attributes(new_ds)));
						else
							std::cerr << "[sepia_handler] WARNING: trajectory file '"<<file<<"' could not be loaded properly" << std::endl;
					}
					else
						std::cerr << "[sepia_handler] WARNING: file '"<<file<<"' does not appear to contain a SEPIA individual trajectory definition" << std::endl;
				}
				else
					std::cerr << "[sepia_handler] WARNING: cannot open trajectory file '"<<file<<'\'' << std::endl;
			}
		}
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
