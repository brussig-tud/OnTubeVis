
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
#define TELLO_POSITION_ATTRIB_NAME "position"

// identifyier to use for radius data
#define TELLO_RADIUS_ATTRIB_NAME "radius"

// identifyier to use for radius data
#define TELLO_VELOCITY_ATTRIB_NAME "altitude"

// identifyier to use for timestamp attribute
#define TELLO_TIME_ATTRIB_NAME "time"

// Declare the proper csv_handler implementation type
#define DECLARE_CSV_IMPL_TYPE typedef typename csv_handler<real>::Impl CSVImpl


////
// Module-private globals

namespace {
	// A single named trello column
	struct TelloColumn
	{
		const std::string name;
		int id = -1;
		TelloColumn(const char *name) : name(name) {}
	};

	// A compound-value sourced from multiple Trello columns
	template <unsigned N>
	struct TelloCompound
	{
		inline static constexpr unsigned num_cols = N;

		TelloColumn cols[num_cols];

		template<class... T>
		static TelloCompound create(const T&... column_names) {
			return {column_names...};
		}

		std::pair<bool, TelloColumn&> find (const std::string &name)
		{
			for (auto &col : cols)
				if (col.name.compare(name) == 0)
					return {true, col};
			TelloColumn dummy("");
			return {false, dummy};
		}

		bool check_found (void) const
		{
			bool found = true;
			for (const auto &col : cols)
				found &= col.id >= 0;
			return found;
		}

		bool is_part (unsigned col_id) const
		{
			for (const auto &col : cols)
				if (col.id == col_id)
					return true;
			return false;
		}

		template <class flt_type>
		void store_component (std::array<flt_type, num_cols> &databuf, unsigned col_id, flt_type val) {
			for (unsigned c=0; c<num_cols; c++) {
				if (cols[c].id == col_id) {
					databuf[c] = val;
					return;
				}
			}
		}

		template <class flt_type>
		std::array<flt_type, num_cols> create_databuf (void) {
			std::array<flt_type, num_cols> ret;
			std::fill_n(ret.data(), num_cols, std::numeric_limits<flt_type>::quiet_NaN());
			return ret;
		}

		template <class flt_type>
		static bool validate_databuf (const std::array<flt_type, num_cols> &databuf) {
			for (const auto &v : databuf)
				if (!(v < std::numeric_limits<flt_type>::infinity()))
					return false;
			return true;
		}
	};

	// Fields known to contain the 3D position
	struct TelloPos : public TelloCompound<3>
	{
		TelloColumn &x, &y, &z;
		TelloPos()
			: //TelloCompound(TelloCompound::create("MVO:posX[meters]", "MVO:posY[meters]", "MVO:posZ[meters]")),
			  TelloCompound(TelloCompound::create("GPS:Long", "GPS:Lat", "GPS:heightMSL")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}

		// override the defaul validation until it's decided what to do about missing z-values
		/*template <class flt_type>
		bool validate_databuf (std::array<flt_type, 3> &databuf) {
			const bool valid = databuf[0] < std::numeric_limits<flt_type>::infinity() && databuf[1] < std::numeric_limits<flt_type>::infinity();
			if (valid && !(databuf[2] < std::numeric_limits<flt_type>::infinity()))
				databuf[2] = 0;
			return valid;
		}*/

		inline static bool latlong (void) { return true; }
	};

	// Fields known to contain the 3D velocity vector
	struct TelloVel : public TelloCompound<3>
	{
		TelloColumn &x, &y, &z;
		TelloVel()
			: TelloCompound(TelloCompound::create("IMU_ATTI(0):velE", "IMU_ATTI(0):velN", "IMU_ATTI(0):velD")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}

		// override the defaul validation until it's decided what to do about missing z-values
		template <class flt_type>
		bool validate_databuf (std::array<flt_type, 3> &databuf) {
			const bool valid = databuf[0] < std::numeric_limits<flt_type>::infinity() && databuf[1] < std::numeric_limits<flt_type>::infinity();
			if (valid && !(databuf[2] < std::numeric_limits<flt_type>::infinity()))
				databuf[2] = 0;
			return valid;
		}
	};
};


////
// Class implementation - tellocsv_handler

struct TrelloCSV
{
	// list of all columns in the table
	std::vector<std::string> columns;

	// well-known column for timestamp
	TelloColumn time = TelloColumn("Clock:offsetTime");

	// well-known columns for position
	TelloPos pos;

	// well-known columns for velocity
	//TelloVel vel;

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
			/* velocity *//* {
				const auto it = vel.find(columns[i]);
				if (it.first)
					it.second.id = i;
			}*/
		}
		bool success = time.id >= 0 && pos.check_found()/* && vel.check_found()*/;
		return success;
	}
};

template <class flt_type>
bool tellocsv_handler<flt_type>::can_handle (std::istream &contents) const
{
	// init
	const stream_pos_guard g(contents);
	DECLARE_CSV_IMPL_TYPE;
	CSVImpl csv_impl;
	TrelloCSV tello;

	// check for tell-tale stream contents
	std::string line;
	// - parse first row and check if there are enough columns
	const std::string &separators = ",";
	std::vector<cgv::utils::token> tokens;
	unsigned num_cols = CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &tello.columns);
	if (num_cols < 7)
		return false;
	// - check if all the fields we want are there
	CSVImpl::remove_enclosing_quotes(tello.columns);
	num_cols = tello.remove_trailing_meta_fields();
	if (!tello.locate_known_fields())
		return false;
	// - check if the first data line contains a readable timestamp and is of the same length as the header column
	std::vector<std::string> fields;
	if (CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields) < num_cols)
		return false;
	const real ts = CSVImpl::parse_field(fields[tello.time.id]);
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
	////
	// Prelude

	// init
	DECLARE_CSV_IMPL_TYPE;
	CSVImpl csv_impl;
	TrelloCSV tello;

	// read in columns
	const std::string &separators = ",";
	std::string line;
	std::vector<cgv::utils::token> tokens;
	unsigned num_cols = CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &tello.columns);
	// - locate special fields we know about
	CSVImpl::remove_enclosing_quotes(tello.columns);
	num_cols = tello.remove_trailing_meta_fields();
	tello.locate_known_fields();

	// prepare attribute arrays
	// - quick-lookup table
	std::vector<traj_attribute<real>*> attrib_from_col;
	std::vector<range*> atraj_from_col;
	attrib_from_col.reserve(num_cols);
	atraj_from_col.reserve(num_cols);
	// - dataset container object
	traj_dataset<real> ret;
	// - known attributes
	auto P = traj_format_handler<real>::template add_attribute<vec3>(ret, TELLO_POSITION_ATTRIB_NAME);
	//auto V = traj_format_handler<real>::template add_attribute<vec3>(ret, TELLO_VELOCITY_ATTRIB_NAME);
	auto T = traj_format_handler<real>::template add_attribute<real>(ret, TELLO_TIME_ATTRIB_NAME); // for now, commiting timestamps as their own attribute is the only way to have them selectable in the layers
	auto &Ptraj = traj_format_handler<real>::trajectories(ret, P.attrib);
	//auto &Vtraj = traj_format_handler<real>::trajectories(ret, V.attrib);
	auto &Ttraj = traj_format_handler<real>::trajectories(ret, T.attrib);
	// - other attributes
	for (unsigned i=0; i<num_cols; i++)
	{
		if (i==tello.time.id || tello.pos.is_part(i)/* || tello.vel.is_part(i)*/) {
			attrib_from_col.emplace_back(nullptr);
			atraj_from_col.emplace_back(nullptr);
		}
		else {
			auto &attrib = traj_format_handler<real>::template add_attribute<real>(ret, tello.columns[i]).attrib;
			attrib_from_col.emplace_back(&attrib);
			atraj_from_col.emplace_back(&traj_format_handler<real>::trajectories(ret, attrib).emplace_back(range{0, 0}));
		}
	}

	////
	// Load actual data

	// parse the stream until EOF
	real dist_accum = 0;
	double first_timestamp = 0, prev_timestamp = 0.;
	typedef std::array<double, 2> latlong;
	latlong refpos;
	while (!contents.eof())
	{
		// read current line of data
		std::vector<std::string> fields;
		CSVImpl::read_next_nonempty_line(&line, &tokens, separators, contents, &fields);
		if (fields.size() < tello.time.id+1)
			// this line cannot possibly contain readable data
			continue;

		// parse timestamp field
		const real ts = CSVImpl::parse_field(fields[tello.time.id]);
		if (!(ts < std::numeric_limits<real>::infinity()))
			// this line doesn't have a valid timestamp, so we can't process any of its data
			continue;
		T.data.timestamps.emplace_back(T.data.values.emplace_back(ts));

		// prepare temporary compound field databuffers for deferred decision on whether a full compound datapoint can be added from this line or not
		auto databuf_pos = tello.pos.create_databuf<real>()/*, databuf_vel = tello.vel.create_databuf<real>()*/;

		// parse each field and if it contains a value, add it to the corresponding attribute array
		for (unsigned i=0; i<(unsigned)fields.size(); i++)
		{
			if (i==tello.time.id)
				/* DoNothing() */;
			else if (tello.pos.is_part(i)) {
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
					tello.pos.store_component(databuf_pos, i, val);
			}/*
			else if (tello.vel.is_part(i)) {
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
					tello.vel.store_component(databuf_vel, i, val);
			}*/
			else
			{
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
				{
					auto &data = attrib_from_col[i]->template get_data<real>();
					data.timestamps.emplace_back(ts);
					data.values.emplace_back(val);
					atraj_from_col[i]->n++;
				}
			}
		}

		// commit compound fields if they could be read completely
		if (tello.pos.validate_databuf(databuf_pos)) {
			const bool is_first = P.data.values.empty();
			const vec3 P_last = is_first ? vec3() : P.data.values.back();
			if (is_first)
				refpos = {databuf_pos[0], databuf_pos[1]};
			const auto mercator = wgs84::toCartesian(refpos, latlong{databuf_pos[0], databuf_pos[1]});
			const vec3 unproj_pos((real)mercator[0], databuf_pos[2], (real)mercator[1]);
			vec3 diff;
			if (!is_first) {
				diff = unproj_pos - P.data.values.back();
				if (ts - P.data.timestamps.back() < real(1))
					goto _skip_pos_sample;
				/*if (diff.sqr_length() < real(1))
					goto _skip_pos_sample;*/
			}
			P.data.timestamps.emplace_back(ts);
			P.data.values.emplace_back(unproj_pos);
			dist_accum += is_first ? real(0) : diff.length();
		_skip_pos_sample:
			/* end_of_block */;
		}
		/*if (tello.vel.validate_databuf(databuf_vel)) {
			V.data.timestamps.emplace_back(ts);
			V.data.values.emplace_back(databuf_vel);
		}*/
	}

	// did we load anything usable?
	if (P.data.values.empty())
		// no position samples were loaded, dataset will be useless
		return traj_dataset<real>();


	////
	// Finalize

	// Build trajectory information (we only ever have one trajectory so it's easy and can be done wholesale here)
	Ptraj.emplace_back(range{0, P.attrib.num()});
	//Vtraj.emplace_back(range{0, V.attrib.num()});
	Ttraj.emplace_back(range{0, T.attrib.num()});

	// Remove empty attributes
	for (unsigned i=0; i<num_cols; i++) {
		const auto attrib = attrib_from_col[i];
		if (attrib != nullptr && attrib->num() < 1)
			traj_format_handler<real>::remove_attribute(ret, *attrib);
	}

	// invent radii
	const unsigned num_segs = (unsigned)(P.data.values.size()-1);
	traj_format_handler<flt_type>::set_avg_segment_length(ret, dist_accum/real(num_segs));
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TELLO_RADIUS_ATTRIB_NAME);
	R.data.values = std::vector<flt_type>(P.data.num(), ret.avg_segment_length()*real(0.125));
	R.data.timestamps = P.data.timestamps;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptraj; // invented radius "samples" are in sync with positions, so just copy traj info

	// setup visual mapping
	static const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {TELLO_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {TELLO_RADIUS_ATTRIB_NAME}}
	});
	ret.set_mapping(std::move(vamap));

	// done!
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
