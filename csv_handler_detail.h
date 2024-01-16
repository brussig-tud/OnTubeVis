#pragma once

// parent module
#include "csv_handler.h"


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
		bool prevsep = false;
		for (const auto &token : tokens)
		{
			if (is_separator(token, separators))
			{
				if (prevsep) {
					count++;
					fields.emplace_back();
				}
				prevsep = true;
			}
			else
			{
				prevsep = false;
				count++;
				if (out)
					fields.emplace_back(std::string(token.begin, size_t(token.end - token.begin)));
			}
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
				cgv::utils::split_to_tokens(*line_out, *tokens, separators, false, "", "", "");
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
		catch (...) { val = std::numeric_limits<real>::quiet_NaN(); }
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
