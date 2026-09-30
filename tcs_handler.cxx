
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
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/progression.h>

// local includes
#include "thick_line_data.h"

// implemented header
#include "tcs_handler.h"


/////
// Some constant defines

/// identifyier to use for position data
#define TCS_POSITION_ATTRIB_NAME "position"

/// identifyier to use for tangent data
#define TCS_TANGENT_ATTRIB_NAME "tangent"

/// identifyier to use for radius data
#define TCS_RADIUS_ATTRIB_NAME "radius"

/// identifyier to use for color data
#define TCS_COLOR_ATTRIB_NAME "color"

/// identifyier to use for color gradients
#define TCS_DCOLOR_ATTRIB_NAME "dcolor"


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
struct tcs_handler<flt_type>::Impl {
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
					std::cerr << "[tcs_handler] WARNING: '"<<name<<"' specified more than once! Using new value: "<<*out << std::endl;
				*visited = true;
			}
			else
				std::cerr << "[tcs_handler] WARNING: '"<<name<<"' specification could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true; 
		}
		return false;
	}
	inline static bool parse_string_prop (
		std::string *out, bool *visited, const char *name, const std::vector<std::string> &fields, const std::string &line
	){
		if (cgv::utils::to_lower(fields[0]).compare(name) == 0)
		{
			if (fields.size()==2 || (fields.size() > 2 && Impl::field_starts_comment(fields[2])))
			{
				*out = fields[1];
				if (*visited)
					std::cerr << "[tcs_handler] WARNING: '"<<name<<"' specified more than once! Using new value: "<<*out << std::endl;
				*visited = true;
			}
			else
				std::cerr << "[tcs_handler] WARNING: '"<<name<<"' specification could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true;
		}
		return false;
	}
	inline static bool parse_scene_prop (
		thick_line_data::scene_type *out, bool *visited, const char *name, const std::vector<std::string> &fields, const std::string &line
	){
		bool processed = false;
		std::string value;
		if (parse_string_prop(&value, &processed, name, fields, line))
		{
			if (cgv::utils::to_lower(value) == "simple") {
				*out = thick_line_data::scene_type::simple;
				*visited = processed;
			}
			else if (cgv::utils::to_lower(value) == "random") {
				*out = thick_line_data::scene_type::random;
				*visited = processed;
			}
			else if (cgv::utils::to_lower(value) == "procedural") {
				*out = thick_line_data::scene_type::procedural;
				*visited = processed;
			}
			else
				std::cerr << "[tcs_handler] WARNING: '"<<name<<"' specification could not be parsed! Line: "<<line << std::endl;

			// we were responsible for this field, so return 'true' even in case we couldn't parse it
			return true;
		}
		return false;
	}
};
template <class flt_type>
const visual_attribute_mapping<flt_type> tcs_handler<flt_type>::Impl::attrmap({
	{VisualAttrib::POSITION, {TCS_POSITION_ATTRIB_NAME}}, {VisualAttrib::TANGENT, {TCS_TANGENT_ATTRIB_NAME}},
	{VisualAttrib::RADIUS, {TCS_RADIUS_ATTRIB_NAME}}, {VisualAttrib::COLOR, {TCS_COLOR_ATTRIB_NAME}}
});

template <class flt_type>
const std::string& tcs_handler<flt_type>::format_name (void) const {
	static const std::string fmt_name = "TCS";
	return fmt_name;
}

template <class flt_type>
bool tcs_handler<flt_type>::can_handle (std::istream &contents) const
{
	std::string str;
	const stream_pos_guard g(contents);

	// check for tell-tale stream contents
	// - .tcs identifier
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> idfields;
	std::getline(contents, str);
	cgv::utils::split_to_tokens(str, tokens, " \t", false);
	if (   Impl::read_fields(tokens, " \t", &idfields) != 2
	    || idfields[0].compare("TCS") != 0 || idfields[1][0] != 'v' || idfields[1].length() < 2)
	{
		//std::cout << "[tcs_handler] first line in stream must be \"TCS v\"+version, but found \"" << str << "\" instead!" << std::endl;
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
traj_dataset<flt_type> tcs_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
){
	////
	// Step 1: load

	// statics
	const static std::string whitespaces(" \t"), quotations("'\"");

	// specification database
	thick_line_data::scene_type scene;
	bool scene_specified=false;

	// parsing workspace
	std::string line;
	std::vector<cgv::utils::token> tokens;
	std::vector<std::string> fields;

	// parsing loop
	// - skip TCS identifier
	std::getline(contents, line);
	// - parse line by line
	while (   !contents.eof()
	       && Impl::read_next_nonempty_line(&line, &tokens, whitespaces, quotations, contents, &fields))
	{
		// skip comment-only lines
		if (Impl::field_starts_comment(fields[0]))
			continue;

		// parse declarations
		if      (Impl::parse_scene_prop(&scene, &scene_specified, "scene",  fields, line)) DO_NOTHING;

		// unknown declaration, print warning
		else
			std::cerr << "[tcs_handler] WARNING: unknown declaration: '"<<fields[0]<<"', ignoring" << std::endl;
	}

	// post-process parsed input
	if (!scene_specified) {
		std::cerr << "[tcs_handler] ERROR: not all necessary properties specified!" << std::endl;
		return traj_dataset<real>();
	}

	// generate dataset
	thick_line_data tld;
	tld.select_scene(scene);
	if (tld.indices.empty()) {
		std::cerr << "[tcs_handler] ERROR: scene contains no trajectory data!" << std::endl;
		return traj_dataset<real>();
	}


	////
	// Step 2: transfer to canonical dataset

	// prepare dataset (we use the filename as dataset name since .tcs does not support any kind of
	// meta fields which could contain a name for the dataset)
	traj_dataset<flt_type> ds(
		cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path)), path
	);
	// - create geometry attributes
	auto P = super::template add_attribute<Vec3>(ds, TCS_POSITION_ATTRIB_NAME);
	auto T = super::template add_attribute<Vec4>(ds, TCS_TANGENT_ATTRIB_NAME);
	auto R = super::template add_attribute<real>(ds, TCS_RADIUS_ATTRIB_NAME);
	auto C = super::template add_attribute<Vec3>(ds, TCS_COLOR_ATTRIB_NAME);

	// compile
	std::vector<range> ds_trajs;
	real avg_dist = 0;
	unsigned num_segs = 0;
	{
		// node access helpers - thick_line_data already stores the nodes in Hermite form with position+radius in
		// `spheres`, tangent+radius derivative in `tangents` and a node color
		const auto node_pos = [&tld] (unsigned n) -> Vec3 {
			const auto &s = tld.spheres[n];
			return Vec3(real(s[0]), real(s[1]), real(s[2]));
		};
		const auto append_node = [&] (unsigned n, real t) {
			const auto &s = tld.spheres[n];
			const auto &tn = tld.tangents[n];
			const auto &c = tld.colors[n];
			P.data.append(node_pos(n), t);
			T.data.append(Vec4(real(tn[0]), real(tn[1]), real(tn[2]), real(tn[3])), t);
			R.data.append(real(s[3]), t);
			C.data.append(Vec3(real(c.R()), real(c.G()), real(c.B())), t);
		};

		// optimize allocations
		const unsigned num_samples_bound = unsigned(tld.spheres.size()) + unsigned(tld.indices.size()/2);
		P.data.reserve(num_samples_bound);
		T.data.reserve(num_samples_bound);
		R.data.reserve(num_samples_bound);
		C.data.reserve(num_samples_bound);

		// transfer the bezier geometry into the hermite spline representation that traj_dataset expects:
		// thick_line_data commits the segments of each trajectory contiguously and only references globally unique node
		// indices, so a new trajectory begins exactly where a segment does not continue its predecessor. The closing
		// segment of a closed curve wraps back to the trajectory's first node; committing it like any other segment
		// duplicates that node and thereby preserves the closing Hermite segment within the linear sample chain.
		unsigned prev_end = 0;
		real t = 0;
		const unsigned num_segments = unsigned(tld.indices.size()/2);
		for (unsigned s=0; s<num_segments; s++)
		{
			// node indices of the current segment
			const unsigned a = tld.indices[2*s], b = tld.indices[2*s+1];

			// begin a new trajectory where the previous one ended
			if (s == 0 || a != prev_end)
			{
				if (s > 0)
					ds_trajs.back().n = unsigned(P.data.num()) - ds_trajs.back().i0;
				ds_trajs.emplace_back(range{unsigned(P.data.num()), 0});
				t = 0;
				append_node(a, t);
			}

			// commit the segment's end node and account for the segment length
			append_node(b, ++t);
			avg_dist += (node_pos(b) - node_pos(a)).length();
			num_segs++;
			prev_end = b;
		}
		// close the final trajectory
		if (!ds_trajs.empty())
			ds_trajs.back().n = unsigned(P.data.num()) - ds_trajs.back().i0;
	}

	// transfer trajectory ranges - all attributes are sampled at the Hermite nodes, so we can just duplicate the
	// trajectory ranges across all attributes
	const unsigned num_trajs = (unsigned)ds_trajs.size();
	traj_format_handler<flt_type>::trajectories(ds, P.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, T.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, R.attrib) = ds_trajs;
	traj_format_handler<flt_type>::trajectories(ds, C.attrib) = std::move(ds_trajs);

	// finalize
	ds.set_mapping(Impl::attrmap);
	super::set_avg_segment_length(ds, avg_dist/num_segs);

	// print some stats
	std::cout << "tcs_handler: loading completed! Stats:" << std::endl
	          << "  " << tld.spheres.size() << " Hermite nodes" << std::endl
	          << "  " << num_segs << " Hermite segments" << std::endl
	          << "  " << num_trajs << " distinct " << (num_trajs>1?"trajectories":"trajectory")
	          << std::endl << std::endl;

	// done!
	return ds;
}


////
// Explicit template instantiations

// Only float and double variants are intended
template struct tcs_handler<float>;
//template struct tcs_handler<double>;


////
// Object registration

// Register both float and double handlers
cgv::base::object_registration<tcs_handler<float> > flt_tcs_reg("tcs trajectory handler (float)");
//cgv::base::object_registration<tcs_handler<double> > dbl_tcs_reg("tcs trajectory handler (double)");
