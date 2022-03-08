
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

// CGV framework utilities
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/file.h>

// implemented header
#include "sepia_handler.h"


/////
// Some constant defines

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

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
struct sepia_handler<flt_type>::Impl {
	// fields
	static const visual_attribute_mapping<flt_type> attrmap;
	inline static const std::string collection_seps=" \t", single_seps="|";

	// helper methods

	// infers type of file from the given header fields.
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
				if (parse_int(&version, vfields[1]))
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
		std::istream &contents, std::vector<std::string> *fields_out=nullptr
	)
	{
		tokens->clear();
		unsigned num_tokens = 0;
		do {
			std::getline(contents, *line_out);
			if (!line_out->empty())
			{
				cgv::utils::split_to_tokens(*line_out, *tokens, separators, false);
				num_tokens = (unsigned)tokens->size();
			}
		}
		while (!contents.eof() && num_tokens < 1);
		return read_fields(*tokens, separators, fields_out);
	}
	inline static bool field_starts_comment (const std::string &field)
	{
		return field.size() >= 2 && field[0] == '/' && field[1] == '/';
	}
	template <class int_type>
	inline static bool parse_int (int_type *out, const std::string &field)
	{
		int_type v;
		size_t pos = 0;
		try { v = (int_type)std::stoll(field, &pos); }
		catch (...) { return false; }
		const bool success = pos > 0 && pos == field.size();
		if (success)
			*out = v;
		return success;
	}
	template <class int_type>
	inline static bool parse_int_prop (
		int_type *out, bool *visited, const char *name, const std::vector<std::string> &fields, const std::string &line
	)
	{
		if (cgv::utils::to_lower(fields[0]).compare(name) == 0)
		{
			if (   (fields.size()==2 || (fields.size() > 2 && Impl::field_starts_comment(fields[2])))
			    && Impl::parse_int(out, fields[1]))
			{
				if (*visited)
					std::cerr << "[sepia_handler] WARNING: '"<<name<<"' specified more than once! Using new value: "<<*out << std::endl;
				*visited = true;
			}
			else
				std::cerr << "[sepia_handler] WARNING: '"<<name<<"' specification could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true; 
		}
		return false;
	}
	inline static bool parse_switch (
		bool *out, const char *name, const std::vector<std::string> &fields, const std::string &line
	)
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
	// - .sepia identifier
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> idfields;
	std::getline(contents, str);
	cgv::utils::split_to_tokens(str, tokens, Impl::collection_seps, false);
	Impl::read_fields(tokens, Impl::collection_seps, &idfields);
	const auto dsinfo = Impl::check_type(idfields);
	if (dsinfo.type == DT::UNKNOWN)
		return false;

	// if not a collection, check supported version
	if (dsinfo.type == DT::SINGLE && dsinfo.version != 5)
		return false;

	// we'll just say we can read it
	return true;
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
		// delegate to individual trajectory loading code
		DO_NOTHING;
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

				// parse option
				if (Impl::parse_switch(&consistent_timestamps, "consistent_timestamps", fields, line)) DO_NOTHING;

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
			std::cerr << "[sepia_handler] ERROR: trajectory collection does not specify any loadable files!" << std::endl;
		else
			for (const auto file : traj_files)
				DO_NOTHING;
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
