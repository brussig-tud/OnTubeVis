
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
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>

// local includes
#include "demo.h"

// implemented header
#include "tgen_handler.h"


/////
// Some constant defines

/// identifyier to use for position data
#define TGEN_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define TGEN_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define TGEN_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define TGEN_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color gradients
#define TGEN_DCOLOR_ATTRIB_NAME "dcolor"


////
// Local types and variables

// anonymous namespace begin
namespace {

// some no-op expression that can be optimized away
#define DO_NOTHING (0)

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
struct tgen_handler<flt_type>::Impl {
	// fields
	static const visual_attribute_mapping<flt_type> attrmap;

	// helper methods
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
		std::string *line_out, std::vector<cgv::utils::token> *tokens,
		const std::string &separators, const std::string &quotations, std::istream &contents,
		std::vector<std::string> *fields_out=nullptr
	)
	{
		tokens->clear();
		unsigned num_tokens = 0;
		do {
			std::getline(contents, *line_out);
			if (!line_out->empty())
			{
				cgv::utils::split_to_tokens(*line_out, *tokens, separators, false, quotations, quotations);
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
					std::cerr << "[tgen_handler] WARNING: '"<<name<<"' specified more than once! Using new value: "<<*out << std::endl;
				*visited = true;
			}
			else
				std::cerr << "[tgen_handler] WARNING: '"<<name<<"' specification could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true; 
		}
		return false;
	}
};
template <class flt_type>
const visual_attribute_mapping<flt_type> tgen_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {TGEN_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {TGEN_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {TGEN_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {TGEN_COLOR_ATTRIB_NAME}}
});

template <class flt_type>
bool tgen_handler<flt_type>::can_handle (std::istream &contents) const
{
	std::string str;
	const stream_pos_guard g(contents);

	// check for tell-tale stream contents
	// - .tgen identifier
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> idfields;
	std::getline(contents, str);
	cgv::utils::split_to_tokens(str, tokens, " \t", false);
	if (   Impl::read_fields(tokens, " \t", &idfields) != 2
	    || idfields[0].compare("TGEN") != 0 || idfields[1][0] != 'v' || idfields[1].length() < 2)
	{
		//std::cout << "[tgen_handler] first line in stream must be \"TGEN v\"+version, but found \"" << str << "\" instead!" << std::endl;
		return false;
	}

	// get indicated version
	int v; size_t pos;
	try { v = std::stoi(idfields[1].substr(1, std::string::npos), &pos); }
	catch (...) {
		// number parsing failed, probably malformed
		return false;
	}
	if (pos < 1)
		// number parsing failed, probably malformed
		return false;

	// we'll just say we can read it if we recognize the version number
	return v > 0 && v < 2;
}

template <class flt_type>
traj_dataset<flt_type> tgen_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
)
{
	// statics
	const static std::string whitespaces(" \t"), quotations("'\"");

	// misc state
	std::mt19937 generator((std::mt19937::result_type)std::time(0));
	std::uniform_int_distribution<unsigned> rnd(0, -1);

	// specification database
	unsigned seed;
	unsigned num;
	unsigned segs;
	bool seed_specified=false, num_specified=false, segs_specified=false;

	// parsing workspace
	std::string line;
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> fields;

	// parsing loop
	// - skip TGEN identifier
	std::getline(contents, line);
	// - parse line by line
	while (   !contents.eof()
	       && Impl::read_next_nonempty_line(&line, &tokens, whitespaces, quotations, contents, &fields))
	{
		// skip comment-only lines
		if (Impl::field_starts_comment(fields[0]))
			continue;

		// parse declarations
		if      (Impl::parse_int_prop(&seed, &seed_specified, "seed",  fields, line)) DO_NOTHING;
		else if (Impl::parse_int_prop(&num,  &num_specified,  "num",   fields, line)) DO_NOTHING;
		else if (Impl::parse_int_prop(&segs, &segs_specified, "segs",  fields, line)) DO_NOTHING;

		// unknown declaration, print warning
		else
			std::cerr << "[tgen_handler] WARNING: unknown declaration: '"<<fields[0]<<"', ignoring" << std::endl;
	}

	// post-process parsed input
	if (!num_specified && !segs_specified)
	{
		std::cerr << "[tgen_handler] ERROR: not all necessary properties specified!" << std::endl;
		return traj_dataset<real>();
	}
	if (!seed_specified) seed = rnd(generator);

	// generate dataset
	std::vector<demo::trajectory> trajs;
	for (unsigned i=0; i<num; i++)
		trajs.emplace_back(demo::gen_trajectory(segs, seed+i));

	// done!
	return demo::compile_dataset(trajs);
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct tgen_handler<float>;
//template struct tgen_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<tgen_handler<float> > flt_tgen_reg("tgen trajectory handler (float)");
//cgv::base::object_registration<tgen_handler<double> > dbl_tgen_reg("tgen trajectory handler (double)");
