
// C++ STL
#include <cmath>
#include <array>
#include <vector>
#include <map>
#include <set>
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
#include <cgv/utils/advanced_scan.h>

// 3rd party libs
#include <WGS84toCartesian.hpp>

// local includes
#include "regulargrid.h"

// implemented header
#include "csv_handler.h"


////
// Class implementation - csv_descriptor

struct csv_descriptor::Impl
{
	// fields
	csv_descriptor::csv_properties props;
	std::vector<attribute> attribs;
	std::string name, separators;
};

csv_descriptor::csv_descriptor() : pimpl(nullptr)
{
	pimpl = new Impl;
}

csv_descriptor::csv_descriptor(const csv_descriptor &other) : csv_descriptor()
{
	pimpl->name = other.pimpl->name;
	pimpl->props = other.pimpl->props;
	pimpl->attribs = other.pimpl->attribs;
	pimpl->separators = other.pimpl->separators;
}

csv_descriptor::csv_descriptor(csv_descriptor &&other) : pimpl(other.pimpl)
{
	other.pimpl = nullptr;
}

csv_descriptor::csv_descriptor(
	const std::string &name, const std::string &separators, const std::vector<attribute> &attributes
)
	: csv_descriptor()
{
	pimpl->name = name;
	pimpl->separators = separators;
	auto &attribs = pimpl->attribs = attributes;
	pimpl->props = infer_properties(attribs);
}

csv_descriptor::csv_descriptor(
	const std::string &name, std::string &&separators, const std::vector<attribute> &attributes
)
	: csv_descriptor()
{
	pimpl->name = name;
	pimpl->separators = std::move(separators);
	auto &attribs = pimpl->attribs = attributes;
	pimpl->props = infer_properties(attribs);
}

csv_descriptor::csv_descriptor(
	const std::string &name, const std::string &separators, std::vector<attribute> &&attributes
)
	: csv_descriptor()
{
	pimpl->name = name;
	pimpl->separators = separators;
	auto &attribs = pimpl->attribs = std::move(attributes);
	pimpl->props = infer_properties(attribs);
}

csv_descriptor::csv_descriptor(
	const std::string &name, std::string &&separators, std::vector<attribute> &&attributes
)
	: csv_descriptor()
{
	pimpl->name = name;
	pimpl->separators = std::move(separators);
	auto &attribs = pimpl->attribs = std::move(attributes);
	pimpl->props = infer_properties(attribs);
}

csv_descriptor::~csv_descriptor()
{
	if (pimpl)
		delete pimpl;
}

csv_descriptor& csv_descriptor::operator= (const csv_descriptor &other)
{
	pimpl->props = other.pimpl->props;
	pimpl->attribs = other.pimpl->attribs;
	pimpl->separators = other.pimpl->separators;
	return *this;
}

csv_descriptor& csv_descriptor::operator= (csv_descriptor &&other)
{
	this->~csv_descriptor();
	pimpl = other.pimpl;
	other.pimpl = nullptr;
	return *this;
}

const std::string& csv_descriptor::name (void) const
{
	return pimpl->name;
}

const std::string& csv_descriptor::separators (void) const
{
	return pimpl->separators;
}

const std::vector<csv_descriptor::attribute>& csv_descriptor::attributes (void) const
{
	return pimpl->attribs;
}

const csv_descriptor::csv_properties& csv_descriptor::properties (void) const
{
	return pimpl->props;
}

csv_descriptor::csv_properties csv_descriptor::infer_properties (const std::vector<attribute> &attributes)
{
	csv_properties ret;
	ret.header = false;
	ret.multi_traj = false;
	unsigned num_cols = 0; signed max_col_id = 0;
	for (unsigned i=0; i<attributes.size(); i++)
	{
		const auto &attrib = attributes[i];
		num_cols += (unsigned)attrib.columns.size();

		if (attrib.semantics == CSV::POS)
			ret.pos_id = i;
		else if (attrib.semantics == CSV::TRAJ_ID)
		{
			ret.multi_traj = true;
			ret.traj_id = i;
		}

		for (const auto &col : attrib.columns)
		{
			max_col_id = std::max(col.number, max_col_id);
			if (col.number < 0)
				ret.header = true;
		}
	}
	ret.max_col_id = std::max(num_cols-1, (unsigned)max_col_id);
	return ret;
}


////
// Class implementation - csv_handler

template <class flt_type>
struct csv_handler<flt_type>::Impl
{
	// types
	typedef flt_type real;
	typedef typename csv_handler::Vec3 Vec3;
	typedef typename csv_handler::Vec4 Vec4;
	typedef typename csv_handler::Color Color;
	struct declared_attrib
	{
		const csv_descriptor::attribute &desc;
		std::vector<unsigned> field_ids;
		std::map<unsigned, traj_attribute<real>> trajs;
		traj_attribute<real> ds_attrib;

		declared_attrib(const csv_descriptor::attribute &adesc)
			: desc(adesc), ds_attrib((unsigned)adesc.columns.size())
		{
			field_ids.reserve(adesc.columns.size());
		}
	};
	struct undeclared_attrib
	{
		std::string name;
		unsigned field_id;
		std::map<unsigned, traj_attribute<real>> trajs;
		traj_attribute<real> ds_attrib;

		undeclared_attrib() : ds_attrib(1) {}
	};

	// fields
	csv_descriptor csv_desc;
	bool desc_valid=false, has_data=false;
	real avg_dist = 0;
	visual_attribute_mapping<real> vmap_hints;

	// helper methods
	void common_init (void)
	{
		desc_valid = csv_handler<flt_type>::is_csv_descriptor_valid(csv_desc);
		if (!desc_valid)
			std::cerr << "csv_handler: WARNING - invalid description of .csv data structure supplied!"
			          << std::endl << std::endl;
	}
	static traj_attribute<real>& ensure_traj (
		std::map<unsigned, traj_attribute<real>> &trajs, unsigned traj_id, unsigned num_components
	)
	{
		auto it = trajs.find(traj_id);
		if (it == trajs.end())
			return trajs.emplace(traj_id, num_components).first->second;
		return it->second;
	}
	static void concat_attribute_containers (
		traj_attribute<real> &dest, const traj_attribute<real> &src
	)
	{
		switch (dest.type())
		{
			case AttribType::SCALAR:
			{
				auto &dst_data = dest.template get_data<real>();
				const auto &src_data = src.template get_data<real>();
				std::copy(src_data.values.begin(), src_data.values.end(), std::back_inserter(dst_data.values));
				std::copy(src_data.timestamps.begin(), src_data.timestamps.end(), std::back_inserter(dst_data.timestamps));
				break;
			}
			case AttribType::VEC2:
			{
				auto &dst_data = dest.template get_data<Vec2>();
				const auto &src_data = src.template get_data<Vec2>();
				std::copy(src_data.values.begin(), src_data.values.end(), std::back_inserter(dst_data.values));
				std::copy(src_data.timestamps.begin(), src_data.timestamps.end(), std::back_inserter(dst_data.timestamps));
				break;
			}
			case AttribType::VEC3:
			{
				auto &dst_data = dest.template get_data<Vec3>();
				const auto &src_data = src.template get_data<Vec3>();
				std::copy(src_data.values.begin(), src_data.values.end(), std::back_inserter(dst_data.values));
				std::copy(src_data.timestamps.begin(), src_data.timestamps.end(), std::back_inserter(dst_data.timestamps));
				break;
			}
			case AttribType::VEC4:
			{
				auto &dst_data = dest.template get_data<Vec4>();
				const auto &src_data = src.template get_data<Vec4>();
				std::copy(src_data.values.begin(), src_data.values.end(), std::back_inserter(dst_data.values));
				std::copy(src_data.timestamps.begin(), src_data.timestamps.end(), std::back_inserter(dst_data.timestamps));
				break;
			}

			default:
				static_assert(true, "[csv_handler] !!!INCOMPATIBLE DATATYPES FOR CONCATENATION!!!");
		}
	}
	bool check_header (const std::vector<std::string> &header_fields)
	{
		for (const auto &attrib : csv_desc.attributes())
			for (const auto &col : attrib.columns)
			{
				bool not_found=true;
				if (col.case_sensitive)
				{
					for (const auto &field : header_fields)
						if (field.compare(col.name) == 0)
						{
							not_found = false;
							break;
						}
				}
				else for (const auto &field : header_fields)
					if (    cgv::utils::to_lower(field).compare(cgv::utils::to_lower(col.name))
					     == 0)
					{
						not_found = false;
						break;
					}
				if (not_found)
					return false;
			}
		return true;
	}
	bool special_fields_not_readable (const std::vector<std::string> &line_fields)
	{
		for (const auto &attrib : csv_desc.attributes())
			if (attrib.semantics != CSV::ARBITRARY)
				for (const auto &col : attrib.columns)
				{
					double test = 0;
					size_t pos = 0;
					try { test = std::stod(line_fields[col.number], &pos); }
					catch (const std::out_of_range&) { /* DoNothing() */; }
					catch (...) { return true; }
					if (!pos)
						return true;
				}
		return false;
	}
	static bool is_header (const std::vector<std::string> &line_fields)
	{
		// in addition to being the very first line in a .csv file that actually contains something, we require the
		// headerfields to contain absolutely no number-convertible whole strings whatsoever
		for (const auto &field : line_fields)
		{
			double test = 0; // overflow due to too-large values should not affect this test, therefor we use double always
			size_t pos = 0;
			try { test = std::stod(field, &pos); }
			catch (...) { /* DoNothing() */; }
			if (pos > 0)
				return false;
		}
		return true;
	}
	static bool is_separator (const cgv::utils::token &token, const std::string &separators)
	{
		// only 1-char tokens can be separators
		if (token.end - token.begin != 1)
			return false;

		// check against each given separator
		for (const char sep : separators)
			if (sep == *token.begin)
				return true;
		return false;
	}
	static void remove_enclosing_quotes (std::vector<std::string> &fields)
	{
		for (auto& field : fields)
		{
			cgv::utils::trim(field);
			size_t pos = field.find_first_of("\"");
			if (pos == 0)
				field = field.substr(1);
			int len = (int)field.length();
			if (len > 0)
			{
				pos = field.rfind("\"");
				if (pos + 1 == field.length())
					field = field.substr(0, len - 1);
			}
		}
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
		const std::string &separators, std::istream &contents, std::vector<std::string> *fields_out=nullptr
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
	inline static real parse_field (const std::string &field)
	{
		real val;
		try { val = (real)std::stod(field); }
		catch (const std::out_of_range&) { val = std::numeric_limits<real>::infinity(); }
		catch (...) { val = 0; }
		return val;
	}
	template <unsigned components>
	static cgv::math::fvec<real, components> parse_fields (
		const std::vector<std::string> &fields, const std::vector<unsigned> field_ids
	)
	{
		cgv::math::fvec<real, components> ret;
		// ToDo: investigate compile-time for-loop (explicit unrolling) techniques
		for (unsigned i=0; i<components; i++)
			ret[i] = parse_field(fields[field_ids[i]]);
		return std::move(ret);
	}

	// timestamp stuff
	enum class TimeFmt
	{
		UNKNOWN, SIMPLE_NUMBER, HMS, HMS_MS
	};
	inline static TimeFmt guess_timestamp_format (const std::string &field)
	{
		static const std::string seperators = ":.";
		std::vector<cgv::utils::token> tokens;
		cgv::utils::split_to_tokens(field, tokens, seperators, false);
		unsigned num_fields = 0;
		for (const auto &token : tokens)
		{
			if (is_separator(token, seperators))
				continue;
			num_fields++;
		}
		if (num_fields == 3 && tokens.size() > 4)
			return TimeFmt::HMS;
		else if (num_fields == 4 && tokens.size() > 6)
			return TimeFmt::HMS_MS;
		return TimeFmt::SIMPLE_NUMBER;
	}
	inline static double parse_timestamp_field (TimeFmt fmt, const std::string &field)
	{
		switch (fmt)
		{
			case TimeFmt::SIMPLE_NUMBER:
			{
				double val;
				try { val = std::stod(field); }
				catch (const std::out_of_range&) { val = std::numeric_limits<double>::infinity(); }
				catch (...) { val = -std::numeric_limits<double>::infinity(); }
				return val;
			}
			case TimeFmt::HMS:
			{
				static const std::string seperators = ":.";
				std::vector<cgv::utils::token> tokens;
				cgv::utils::split_to_tokens(field, tokens, seperators, false);
				double val[3];
				for (unsigned i=0; i<3; i++)
				{
					const auto &token = tokens[i*2];
					try { val[i] = std::stod(std::string(token.begin, size_t(token.end - token.begin))); }
					catch (const std::out_of_range&) { val[i] = std::numeric_limits<double>::infinity(); }
					catch (...) { val[i] = -std::numeric_limits<double>::infinity(); }
				}
				return val[0]*60*60 + val[1]*60 + val[2];
			}
			case TimeFmt::HMS_MS:
			{
				static const std::string seperators = ":.";
				std::vector<cgv::utils::token> tokens;
				cgv::utils::split_to_tokens(field, tokens, seperators, false);
				double val[4];
				for (unsigned i=0; i<4; i++)
				{
					const auto &token = tokens[i*2];
					try { val[i] = std::stod(std::string(token.begin, size_t(token.end - token.begin))); }
					catch (const std::out_of_range&) { val[i] = std::numeric_limits<double>::infinity(); }
					catch (...) {
					val[i] = -std::numeric_limits<double>::infinity();
					}
				}
				return val[0]*60*60 + val[1]*60 + val[2] + val[3]/1000;
			}
			default:
				/* DoNothing() */;
		}
		return 0;
	}
};

template <class flt_type>
csv_handler<flt_type>::csv_handler() : pimpl(nullptr)
{
	pimpl = new Impl;
}

template <class flt_type>
csv_handler<flt_type>::csv_handler(
	const csv_descriptor &csv_desc, const visual_attribute_mapping<real> &vmap_hints
)
	: csv_handler()
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// commit name and descriptor
	impl.csv_desc = csv_desc;
	impl.vmap_hints = vmap_hints;
	impl.common_init();
}

template <class flt_type>
csv_handler<flt_type>::csv_handler(
	csv_descriptor &&csv_desc, const visual_attribute_mapping<real> &vmap_hints
)
	: csv_handler()
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// commit name and descriptor
	impl.csv_desc = std::move(csv_desc);
	impl.vmap_hints = vmap_hints;
	impl.common_init();
}

template <class flt_type>
csv_handler<flt_type>::~csv_handler()
{
	if (pimpl)
		delete pimpl;
}

template <class flt_type>
const std::vector<std::string>& csv_handler<flt_type>::handled_extensions (void) const
{
	// for now, we don't claim any file extensions
	// ToDo: add option to csv_descriptor to specify file extensions, which will then be reported to callers here
	return traj_format_handler<flt_type>::handled_extensions();
}

template <class flt_type>
void csv_handler<flt_type>::cleanup (void)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// reset our private fields
	impl.has_data = false;
	impl.avg_dist = 0;
}

template <class flt_type>
bool csv_handler<flt_type>::can_handle (std::istream &contents) const
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// init
	std::string line;
	const stream_pos_guard g(contents);

	// check for tell-tale stream contents
	// - parse first row and check if there are enough columns
	const std::string &separators = impl.csv_desc.separators();
	const auto &props = impl.csv_desc.properties();
	std::vector<std::string> columns;
	std::vector<cgv::utils::token> tokens;
	if (  Impl::read_next_nonempty_line(&line, &tokens, separators, contents, &columns)
	    < props.max_col_id)
		return false;

	Impl::remove_enclosing_quotes(columns);

	// - inspect contents more closely
	if (props.header)
	{
		if (impl.check_header(columns))
			// header matches up, so at this point we don't do any further tests and just assume it'll work
			return true;
		return false;
	}
	else
	{
		// also actually process up to two lines in the stream to see if we can actually interpret the text-encoded data
		if (impl.is_header(columns))
		{
			// even if we don't need a header - in case the file does have one, we will require it to match up
			if (!impl.check_header(columns))
				return false;
			// header checked out ok, read in first actual data row
			Impl::read_next_nonempty_line(&line, &tokens, separators, contents, &columns);
		}
		if (impl.special_fields_not_readable(columns))
			return false;
	}

	// apparently a valid .csv that we can interpret
	return true;
}

template <class flt_type>
traj_dataset<flt_type> csv_handler<flt_type>::read (
	std::istream &contents, DatasetOrigin source, const std::string &path
)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// file parsing database
	const std::string &separators = impl.csv_desc.separators();
	std::string line;
	std::vector<std::string> fields;
	std::vector<cgv::utils::token> tokens;

	// read in first row and do initial pre-processing
	Impl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields);
	std::set<unsigned> undeclared_cols;
	for (unsigned i=0; i<fields.size(); i++)
		undeclared_cols.emplace_hint(undeclared_cols.end(), i);
	bool header_present;

	// route .csv file columns to declared attributes
	const auto &props = impl.csv_desc.properties();
	const auto &csv_attribs = impl.csv_desc.attributes();
	std::vector<typename Impl::declared_attrib> declared_attribs;
	declared_attribs.reserve(csv_attribs.size());
	int timestamp_id = -1;
	if (props.header)
	{
		// we know that we have a header because our attribute declaration requires one
		header_present = true;
		Impl::remove_enclosing_quotes(fields);

		// find actual .csv columns belonging to each declared attribute
		for (const auto &csv_attrib : csv_attribs)
		{
			if (csv_attrib.semantics == CSV::TIMESTAMP)
				timestamp_id = (int)declared_attribs.size();
			declared_attribs.emplace_back(csv_attrib);
			auto &attrib = declared_attribs.back();
			// for each colum declaration, search the corresponding field in the actual .csv header row
			for (const auto &col : csv_attribs[props.pos_id].columns)
				for (unsigned i=0; i<(unsigned)fields.size(); i++)
				{
					if (   (col.case_sensitive && fields[i].compare(col.name) == 0)
					    || (   !(col.case_sensitive)
					        && cgv::utils::to_lower(fields[i]).compare(cgv::utils::to_lower(col.name)) == 0))
					{
						attrib.field_ids.emplace_back(i);
						undeclared_cols.erase(i);
					}
				}
		}
	}
	else
	{
		// store whether or not the file has a header
		header_present = Impl::is_header(fields);
		if(header_present)
			Impl::remove_enclosing_quotes(fields);

		// just commit the user-declared column numbers
		for (const auto &csv_attrib : csv_attribs)
		{
			if (csv_attrib.semantics == CSV::TIMESTAMP)
				timestamp_id = (int)declared_attribs.size();
			declared_attribs.emplace_back(csv_attrib);
			auto &attrib = declared_attribs.back();
			for (const auto &col : csv_attrib.columns)
			{
				attrib.field_ids.emplace_back(col.number);
				undeclared_cols.erase(col.number);
			}
		}
	}
	// route remaining .csv file columns to undeclared attributes
	std::vector<typename Impl::undeclared_attrib> undeclared_attribs;
	undeclared_attribs.reserve(undeclared_cols.size());
	for (unsigned i : undeclared_cols)
	{
		undeclared_attribs.emplace_back();
		auto &attrib = undeclared_attribs.back();
		if (header_present)
			attrib.name = fields[i];
		else
			attrib.name = "attr_"+std::to_string(i);
		attrib.field_id = i;
	}
	undeclared_cols.clear(); // <-- we don't need these from here on

	// ensure we're at the first actual data row
	if (header_present)
		Impl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields);

	// prepare timestamp parsing
	auto timestamp_format = Impl::TimeFmt::UNKNOWN;
	if (timestamp_id > -1)
		timestamp_format = Impl::guess_timestamp_format(fields[declared_attribs[timestamp_id].field_ids.front()]);

	// parse the stream until EOF
	bool nothing_loaded = true;
	real dist_accum = 0;
	unsigned running_traj_id = 0;
	double first_timestamp = 0, prev_timestamp = 0.;
	bool first = true;
	while (!contents.eof())
	{
		// read in timestamps if present
		double t;
		if(timestamp_id > -1) {
			auto &ts_csv = declared_attribs[timestamp_id];
			t = Impl::parse_timestamp_field(
				timestamp_format, fields[ts_csv.field_ids.front()]
			);
			if (first && !props.multi_traj)
				first_timestamp = t;
		}

		// determine trajectory id of this row
		unsigned traj_id;
		if (props.multi_traj)
		{
			const int traj_id_field = declared_attribs[props.traj_id].field_ids.front();
			traj_id = std::atoi(fields[traj_id_field].c_str());
		}
		else if(timestamp_id > -1)
		{
			if(!first) {
				if(prev_timestamp > t)
					++running_traj_id;
			}
			first = false;
			prev_timestamp = t;
			traj_id = running_traj_id;
		}
		else
			traj_id = 0;

		// make sure position traj exists for various kinds of forward queries
		auto &P = Impl::ensure_traj(declared_attribs[props.pos_id].trajs, traj_id, 3).template get_data<Vec3>().values;

		// commit timestamp as actual data point if present
		real t_mod;
		if(timestamp_id > -1) {
			auto &ts_csv = declared_attribs[timestamp_id];
			auto &ts_attrib = Impl::ensure_traj(ts_csv.trajs, traj_id, 1);
			t_mod = (real)(t - first_timestamp);
			ts_attrib.template get_data<real>().append(t_mod, t_mod);
		} else {
			t_mod = (real)(t = (real)P.size());
		}
	
		// read in all declared attributes
		for (auto &attrib : declared_attribs)
		{
			if (attrib.desc.semantics == CSV::TIMESTAMP)
				// timestamps are handled above
				continue;

			switch (attrib.field_ids.size())
			{
				case 1:
				{
					auto &a = Impl::ensure_traj(attrib.trajs, traj_id, 1);
					a.template get_data<real>().append(
						Impl::parse_field(fields[attrib.field_ids.front()]), (real)t_mod
					);
					continue;
				}

				case 2:
				{
					auto &a = Impl::ensure_traj(attrib.trajs, traj_id, 2);
					a.template get_data<Vec2>().append(
						std::move(Impl::template parse_fields<2>(fields, attrib.field_ids)), (real)t_mod
					);
					continue;
				}

				case 3:
				{
					auto &a = Impl::ensure_traj(attrib.trajs, traj_id, 3);
					a.template get_data<Vec3>().append(
						std::move(Impl::template parse_fields<3>(fields, attrib.field_ids)), (real)t_mod
					);
					nothing_loaded = false; // this is guaranteed to capture the position attribute, since positions are always Vec3
					continue;
				}

				case 4:
				{
					auto &a = Impl::ensure_traj(attrib.trajs, traj_id, 4);
					a.template get_data<Vec4>().append(
						std::move(Impl::template parse_fields<4>(fields, attrib.field_ids)), (real)t_mod
					);
					continue;
				}

				default:
					/* DoNothing() */;
			}
		}

		// read in all undeclared attributes
		for (auto &attrib : undeclared_attribs)
		{
			auto &a = Impl::ensure_traj(attrib.trajs, traj_id, 1);
			a.template get_data<real>().append(Impl::parse_field(fields[attrib.field_id]), (real)t_mod);
		}

		// update segment length counter
		if (P.size() > 1)
			dist_accum += (P.back() - P[P.size()-2]).length();

		// read in next data row
		Impl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields);
	}

	// did we succeed at loading anything?
	traj_dataset<real> ret;
	if (nothing_loaded)
		return std::move(ret);

	// transform from intermediate storage to final data layout and transfer into output dataset
	for (auto &attr : declared_attribs)
	{
		std::vector<range> ds_trajs;
		// special treatment for first trajectory
		auto it = attr.trajs.begin();
		attr.ds_attrib = std::move(it->second);
		ds_trajs.emplace_back(range{ 0, attr.ds_attrib.num() });
		it++; for (; it!=attr.trajs.end(); it++)
		{
			const auto &traj_attrib = it->second;
			// generate range
			ds_trajs.emplace_back(range{ attr.ds_attrib.num(), traj_attrib.num() });
			// copy data points
			Impl::concat_attribute_containers(attr.ds_attrib, traj_attrib);
		}
		// commit to dataset
		traj_format_handler<flt_type>::trajectories(ret, attr.ds_attrib) = std::move(ds_trajs);
		traj_format_handler<flt_type>::attributes(ret).emplace(attr.desc.name, std::move(attr.ds_attrib));
	}
	for (auto &attr : undeclared_attribs)
	{
		std::vector<range> ds_trajs;
		// special treatment for first trajectory
		auto it = attr.trajs.begin();
		attr.ds_attrib = std::move(it->second);
		ds_trajs.emplace_back(range{ 0, attr.ds_attrib.num() });
		// copy datapoints from remaining trajectories
		it++; for (; it!=attr.trajs.end(); it++)
		{
			const auto &traj_attrib = it->second;
			// generate range
			ds_trajs.emplace_back(range{ attr.ds_attrib.num(), traj_attrib.num() });
			// copy data points
			Impl::concat_attribute_containers(attr.ds_attrib, traj_attrib);
		}
		// commit to dataset
		traj_format_handler<flt_type>::trajectories(ret, attr.ds_attrib) = std::move(ds_trajs);
		traj_format_handler<flt_type>::attributes(ret).emplace(attr.name, std::move(attr.ds_attrib));
	}
	// prepare invented radii
	auto R = traj_format_handler<flt_type>::template add_attribute<real>(ret, "radius");

	// set visual mapping
	visual_attribute_mapping<real> vamap(impl.vmap_hints);
	if (!vamap.is_mapped(VisualAttrib::POSITION))
		// ToDo: perform more fine-grained / smart test to retain any hinted at transformation if
		// pre-mapped name and position attribute name from the csv table description match up
		vamap.map_attribute(VisualAttrib::POSITION, csv_attribs[props.pos_id].name);
	ret.set_mapping(std::move(vamap));

	// determine remaining stats
	const auto &P = traj_format_handler<flt_type>::positions(ret);
	const unsigned
		num_samples = P.data.num(),
		num_segs = std::max<int>(num_samples-(unsigned)traj_format_handler<flt_type>::trajectories(ret, P.attrib).size(), 1);
	traj_format_handler<flt_type>::set_avg_segment_length(ret, real(dist_accum / double(num_segs)));

	// invent radii now that all stats are known
	R.data.values = std::vector<real>(num_samples, ret.avg_segment_length()*real(0.25));
	R.data.timestamps = P.data.timestamps;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = traj_format_handler<flt_type>::trajectories(ret, P.attrib);

	// set dataset name (we use the filename since our .csv model does not support other kinds of fields outside the actual data
	// which could contain the name)
	traj_format_handler<flt_type>::name(ret) = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path));

	// print stats
	const unsigned num_trajs = (unsigned)declared_attribs[props.pos_id].trajs.size();
	std::cout << "csv_handler: loading completed! Stats:" << std::endl
	          << "  " << num_samples<<" samples" << std::endl
	          << "  " << num_segs<<" segments" << std::endl
	          << "  " << num_trajs<<(num_trajs>1?" trajectories":" trajectory") << std::endl
	          << std::endl;

	// done
	return std::move(ret);
}

template <class flt_type>
bool csv_handler<flt_type>::is_csv_descriptor_valid (const csv_descriptor &csv_desc)
{
	bool pos_found=false, traj_id_found=false, timestamp_found=false;
	for (const auto &attrib : csv_desc.attributes())
	{
		if (attrib.semantics == CSV::POS)
		{
			pos_found = true;
			if (attrib.columns.empty() || attrib.columns.size() > 3)
				return false;
		}
		else if (attrib.semantics == CSV::TRAJ_ID)
		{
			traj_id_found = true;
			if (attrib.columns.empty() || attrib.columns.size() > 1)
				return false;
		}
		else if (attrib.semantics == CSV::TIMESTAMP)
		{
			timestamp_found = true;
			if (attrib.columns.empty() || attrib.columns.size() > 1)
				return false;
		}
		else if (   (attrib.semantics == CSV::POS && pos_found)
		         || (attrib.semantics == CSV::TRAJ_ID && traj_id_found)
		         || (attrib.semantics == CSV::TIMESTAMP && timestamp_found))
			return false;
	}
	return pos_found;
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class csv_handler<float>;
template class csv_handler<double>;


////
// Object registration

// Register example handler for the IML multi-user study .csv files
static const csv_descriptor csv_imluser_desc("IML user trajectory", ",", {
	{ "timestamp", {"timestamp", false, 0}, CSV::TIMESTAMP },
	{ "id",        {"id", false, 1}, CSV::TRAJ_ID },
	{ "position",  {{"pos_1", false, 2}, {"pos_2", false, 3}, {"pos_3", false, 4}}, CSV::POS }}
),
csv_imldevice_desc("IML device trajectory", ",", {
	{ "timestamp", {"timestamp", false, 0}, CSV::TIMESTAMP },
	{ "userid",    {"userid", false, 1}, CSV::TRAJ_ID },
	{ "position",  {{"spacePos_1", false, 4}, {"spacePos_2", false, 5}, {"spacePos_3", false, 6}}, CSV::POS }}
);

cgv::base::object_registration_2<
	csv_handler<float>, csv_descriptor, visual_attribute_mapping<float>
> csv_imluser_reg(
	csv_imluser_desc,
	visual_attribute_mapping<float>({
		{VisualAttrib::POSITION, {
			// we scale up the dataset to get more sensible numbers (mitigates floating point rounding errors etc.)
			"position", attrib_transform<float>::vec3_to_vec3(
				[](csv_handler<float>::Vec3 &out, const csv_handler<float>::Vec3 &in) {
					out = 128.f * in;
				}
			)
		 }},
		{VisualAttrib::RADIUS, {
			// we scale up the dataset to get more sensible numbers (mitigates floating point rounding errors etc.)
			"radius", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) {
					out = 128 * in;
				}
			)
		 }},
		{VisualAttrib::COLOR, {
			// ids are either 1 or 2, so by substracting 1 we can use them as input to a color scale
			"id", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) { out = in - 1; }
			)
		 }}},
		// make user trajectories a green-ish color, with the first user being a bit lighter than the second
		colormap({{ 178.f/255.f, 223.f/255.f, 138.f/255.f }, { 51.f/255.f, 160.f/255.f, 44.f/255.f }})
	),
	"csv handler (float) - "+csv_imluser_desc.name()
),
csv_imldevice_reg(
	csv_imldevice_desc,
	visual_attribute_mapping<float>({
		{VisualAttrib::POSITION, {
			// we flip the direction of the x and z axes, as the tracking system for the devices in the IML
			// study used a different coordinate system, also scale up to get more sensible numbers (mitigates
			// floating point rounding errors etc.)
			"position", attrib_transform<float>::vec3_to_vec3(
				[](csv_handler<float>::Vec3 &out, const csv_handler<float>::Vec3 &in) {
					out.x() = -128 * in.x();
					out.y() =  128 * in.y();
					out.z() = -128 * in.z();
				}
			)
		 }},
		{VisualAttrib::RADIUS, {
			// we scale up the dataset to get more sensible numbers (mitigates floating point rounding errors etc.)
			"radius", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) {
					out = 128 * in;
				}
			)
		 }},
		{VisualAttrib::COLOR, {
			// user ids are either 1 or 2, so by substracting 1 we can use them as input to a color scale
			"userid", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) { out = in - 1; }
			)
		 }}},
		// make device trajectories a blue-ish color, with the first user being a bit lighter than the second
		colormap({{ 166.f/255.f, 206.f/255.f, 227.f/255.f }, { 31.f/255.f, 120.f/255.f, 180.f/255.f }})
	),
	"csv handler (float) - "+csv_imldevice_desc.name()
);

// Register handler for streamline .csv files exported from paraview
static const csv_descriptor csv_paraview_streamline_desc("Paraview Streamline", ",", {
	{ "timestamp", {"IntegrationTime", false, 4}, CSV::TIMESTAMP },
	{ "position",  {{"Points:0", false, 13}, {"Points:1", false, 14}, {"Points:2", false, 15}}, CSV::POS },
	{ "velocity",  {{"U:0", false, 0}, {"U:1", false, 1}, {"U:2", false, 2}} }}
);
//static const csv_descriptor csv_paraview_streamline_desc("Paraview Streamline", ",", {
//	{ "timestamp", {"\"IntegrationTime\"", false, 4}, CSV::TIMESTAMP },
//	{ "position",  {{"\"Points:0\"", false, 13}, {"\"Points:1\"", false, 14}, {"\"Points:2\"", false, 15}}, CSV::POS },
//	{ "velocity",  {{"\"U:0\"", false, 0}, {"\"U:1\"", false, 1}, {"\"U:2\"", false, 2}} } }
//);

cgv::base::object_registration_2<
	csv_handler<float>, csv_descriptor, visual_attribute_mapping<float>
> csv_paraview_streamline_reg(
	csv_paraview_streamline_desc,
	visual_attribute_mapping<float>({
		{VisualAttrib::RADIUS, {
			// increase radius a bit to get thicker tubes with more visible area
			"radius", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) {
					out = 1.25f*in;
				}
			)
		 }}}
	),
	"csv handler (float) - "+csv_paraview_streamline_desc.name()
);

// Register handler for package delivery drone trajectory .csv files
static const csv_descriptor csv_pkg_drone_streamline_desc("Delivery Drone Trajectory", ",", {
	{ "timestamp", {"time", false, 1}, CSV::TIMESTAMP },
	{ "id",        {"id", false, 0}, CSV::TRAJ_ID },
	{ "position",  {{"position_x", false, 6}, {"position_y", false, 7}, {"position_z", false, 8}}, CSV::POS },
	{ "velocity",  {{"velocity_x", false, 13}, {"velocity_y", false, 14}, {"velocity_z", false, 15}} },
	{ "linear_acceleration",  {{"linear_acceleration_x", false, 19}, {"linear_acceleration_y", false, 20}, {"linear_acceleration_z", false, 21}} },
	// for reading raw files with GPS coordinates
	/*{ "timestamp", {"time", false, 0}, CSV::TIMESTAMP },
	{ "position",  {{"position_x", false, 5}, {"position_y", false, 6}, {"position_z", false, 7}}, CSV::POS },
	{ "velocity",  {{"velocity_x", false, 12}, {"velocity_y", false, 13}, {"velocity_z", false, 14}} },
	{ "linear_acceleration",  {{"linear_acceleration_x", false, 18}, {"linear_acceleration_y", false, 19}, {"linear_acceleration_z", false, 20}} },*/
	}
);

cgv::base::object_registration_2<
	csv_handler<float>, csv_descriptor, visual_attribute_mapping<float>
> csv_pkg_drone_streamline_reg(
	csv_pkg_drone_streamline_desc,
	visual_attribute_mapping<float>({
		{VisualAttrib::RADIUS, {
			// increase radius a bit to get thicker tubes with more visible area
			"radius", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) {
					out = 4.0f*in;
				}
			)
		}}}
	),
	"csv handler (float) - " + csv_pkg_drone_streamline_desc.name()
);

// Register handler for RTLola trace export .csv files
static const csv_descriptor csv_rtlola_desc("RTLola Trace", ",", {
	{ "time", {"time", false, 0}, CSV::TIMESTAMP },
	{ "position",  {{"lat", false, 1}, {"lon", false, 2}, {"altitude", false, 3}}, CSV::POS } }
);

cgv::base::object_registration_2<
	csv_handler<float>, csv_descriptor, visual_attribute_mapping<float>
> csv_rtlola_reg(
	csv_rtlola_desc,
	visual_attribute_mapping<float>({
		{VisualAttrib::POSITION, {
			// transform lat/long/alt to cartesian coordinates using mercator projection
			"position", attrib_transform<float>::vec3_to_vec3(
				[](csv_handler<float>::Vec3 &out, const csv_handler<float>::Vec3 &in) {
					typedef std::array<double, 2> latlong;
					const static latlong refpos = {in.x(), in.y()};
					const auto mercator = wgs84::toCartesian(refpos, latlong{in.x(), in.y()});
					out.set((float)mercator[0]*1.f, in.z()*1.f, (float)mercator[1]*1.f);
				}
			)
		 }},
		{VisualAttrib::RADIUS, {
			// increase radius a bit to get thicker tubes with more visible area
			"radius", attrib_transform<float>::real_to_real(
				[](float &out, const float &in) {
					out = in;
				}
			)
		}}}
	),
	"csv handler (float) - " + csv_rtlola_desc.name()
);
