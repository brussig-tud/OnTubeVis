
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
	template <unsigned num_cols>
	struct TelloCompound
	{
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
				if (cols[c].id == col_id)
					databuf[c] = val;
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
			: TelloCompound(TelloCompound::create("MVO:posX[meters]", "MVO:posY[meters]", "MVO:posZ[meters]")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}
	};

	// Fields known to contain the 3D velocity vector
	struct TelloVel : public TelloCompound<3>
	{
		TelloColumn &x, &y, &z;
		TelloVel()
			: TelloCompound(TelloCompound::create("MVO:velX[meters/Sec]", "MVO:velY[meters/Sec]", "MVO:velZ[meters/Sec]")),
			  x(cols[0]), y(cols[1]), z(cols[2])
		{}
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
	TelloVel vel;

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
			/* velocity */ {
				const auto it = vel.find(columns[i]);
				if (it.first)
					it.second.id = i;
			}
		}
		bool success = time.id >= 0 && pos.check_found() && vel.check_found();
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
	attrib_from_col.reserve(num_cols);
	// - dataset container object
	traj_dataset<real> ret;
	// - known attributes
	auto P = traj_format_handler<real>::template add_attribute<vec3>(ret, TELLO_POSITION_ATTRIB_NAME);
	auto V = traj_format_handler<real>::template add_attribute<vec3>(ret, TELLO_VELOCITY_ATTRIB_NAME);
	auto T = traj_format_handler<real>::template add_attribute<real>(ret, TELLO_TIME_ATTRIB_NAME); // for now, commiting timestamps as their own attribute is the only way to have them selectable in the layers
	auto &Ptraj = traj_format_handler<real>::trajectories(ret, P.attrib);
	// - other attributes
	for (unsigned i=0; i<num_cols; i++)
	{
		if (i==tello.time.id || tello.pos.is_part(i) || tello.vel.is_part(i)) {
			attrib_from_col.emplace_back(nullptr);
		}
		else {
			auto &attrib = traj_format_handler<real>::template add_attribute<real>(ret, tello.columns[i]).attrib;
			attrib_from_col.emplace_back(&attrib);
		}
	}

	////
	// Load actual data

	// parse the stream until EOF
	real dist_accum = 0;
	double first_timestamp = 0, prev_timestamp = 0.;
	bool first = true;
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

		// prepare temporary compound field databuffers for deferred decision on whether a full compound datapoint can be added from this line or not
		auto databuf_pos = tello.pos.create_databuf<real>(), databuf_vel = tello.vel.create_databuf<real>();

		// parse each field and if it contains a value, add it to the corresponding attribute array
		for (unsigned i=0; i<(unsigned)fields.size(); i++)
		{
			if (i==tello.time.id)
				/* DoNothing() */;
			else if (tello.pos.is_part(i)) {
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
					tello.pos.store_component(databuf_pos, i, val);
			}
			else if (tello.vel.is_part(i)) {
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
					tello.vel.store_component(databuf_vel, i, val);
			}
			else
			{
				const real val = CSVImpl::parse_field(fields[i]);
				if (val < std::numeric_limits<real>::infinity())
				{
					auto &attrib = *attrib_from_col[i];
					auto &data = attrib.template get_data<real>();
					data.timestamps.emplace_back(ts);
					data.values.emplace_back(val);
				}
			}
		}

		// commit compound fields if they could be read completely
		if (tello.pos.validate_databuf(databuf_pos)) {
			P.data.timestamps.emplace_back(ts);
			P.data.values.emplace_back(databuf_pos);
		}
		if (tello.vel.validate_databuf(databuf_vel)) {
			V.data.timestamps.emplace_back(ts);
			V.data.values.emplace_back(databuf_vel);
		}
	}

	// did we load anything usable?
	if (P.data.values.empty())
		// no position samples were loaded, dataset will be useless
		return traj_dataset<real>();


	////
	// Finalize

	// invent radii
	const unsigned num_segs = (unsigned)(P.data.values.size()-1);
	traj_format_handler<flt_type>::set_avg_segment_length(ret, dist_accum / real(num_segs));
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, TELLO_RADIUS_ATTRIB_NAME);
	R.data.values = std::vector<flt_type>(P.data.num(), ret.avg_segment_length()*real(0.125));
	R.data.timestamps = P.data.timestamps;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptraj; // invented radius "samples" are in sync with positions, so just copy traj info

	// setup visual mapping
	static const visual_attribute_mapping<real> vamap({
		{VisualAttrib::POSITION, {TELLO_POSITION_ATTRIB_NAME}}, {VisualAttrib::RADIUS, {TELLO_RADIUS_ATTRIB_NAME}}
	});
	ret.set_mapping(std::move(vamap));/*

	// for single precision float compatibility, we make everything relative to the first sample of the first trajectory position-wise...
#if defined(OBD_USE_ECEF_COORDINATES) && OBD_USE_ECEF_COORDINATES!=0
	const auto refpos = peri::xyzForLpa(peri::LPA{
		gps_info_map.cbegin()->second[0].latitude,
		gps_info_map.cbegin()->second[0].longitude,
		gps_info_map.cbegin()->second[0].altitude
	});
#else
	typedef std::array<double, 2> latlong;
	const latlong refpos = {gps_info_map.cbegin()->second[0].latitude, gps_info_map.cbegin()->second[0].longitude};
#endif
	// ...and relative to the earliest sample among all trajectories time-wise
	double reftime = std::numeric_limits<double>::infinity();
	for (const auto &e : gps_info_map)
		reftime = (double)std::min(e.second[0].timestamp, (size_t)reftime);
	auto convert_time = [&reftime] (size_t timestamp) -> flt_type {
		// ToDo: what do the timestamps mean? They're either in nanoseconds or there is centuries in between samples...
		return (flt_type)((double(timestamp) - reftime)/1000000000.0);
	};

	// create synchronous attributes
	auto P = traj_format_handler<flt_type>::template add_attribute<vec3>(ret, OBD_POSITION_ATTRIB_NAME);
	auto A = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_ALTITUDE_ATTRIB_NAME);
	auto V = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_GPSSPEED_ATTRIB_NAME);
	auto T = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_TIME_ATTRIB_NAME); // for now, commiting timestamps as their own attribute is the only way to have them selectable in the layers
	auto &Ptraj = traj_format_handler<flt_type>::trajectories(ret, P.attrib);

	// commit samples of each trajectory
	// - prepare scale factor for mercator-projected cartesian positions
	constexpr flt_type mscale = 1/(flt_type)OBD_MERCATOR_SCALEDOWN_FACTOR;
	// - various bookkeeping
	double seg_dist_accum = 0;
	unsigned num_segs=0, offset=0;
	// - the loop
	for (const auto &e : gps_info_map)
	{
		// convenience shorthand
		const auto &gps_infos = e.second;

		// only include trajectory if it has at least one segment
		if (gps_infos.size() < 2)
			continue;

		// commit samples
		unsigned num_samples_this_traj = 0;
		for (unsigned i=0; i<(unsigned)gps_infos.size(); i++)
		{
			// convenience shorthand
			const auto &gps_info = gps_infos[i];

			// convert from lattitude/longitude
			#if defined(OBD_USE_ECEF_COORDINATES) && OBD_USE_ECEF_COORDINATES!=0
				const auto xyzpos = peri::xyzForLpa(peri::LPA{gps_info.latitude, gps_info.longitude, gps_info.altitude}),
				           relpos = peri::LPA{xyzpos[0]-refpos[0], xyzpos[1]-refpos[1], xyzpos[2]-refpos[2]};
				vec3 pos((flt_type)relpos[0], (flt_type)relpos[1], (flt_type)relpos[2]);
			#else
				const auto mercator = wgs84::toCartesian(refpos, latlong{gps_info.latitude, gps_info.longitude});
				vec3 pos((flt_type)mercator[0]*mscale, (flt_type)gps_info.altitude*mscale, (flt_type)mercator[1]*mscale);
			#endif

			// keep track of segment length and throw out non-monotone samples
			flt_type time = convert_time(gps_info.timestamp);
			if (i > 0)
			{
				// ToDo: this very crude approach is far from optimal
				if (time <= P.data.timestamps.back())
					continue;

				const auto &prev = P.data.values.back();
				// eliminate duplicates - ToDo: why are there so many?
				if ((pos - prev).sqr_length() < (flt_type)std::numeric_limits<float>::epsilon()*2)
					continue;
				seg_dist_accum += (pos - prev).length();
				num_segs++;
			}

			// commit to storage
			P.data.append(pos, time);
			A.data.append((flt_type)gps_info.altitude, time);
			V.data.append((flt_type)gps_info.gps_speed, time);
			T.data.append(time, time);

			// book-keeping for building trajectory information after this loop
			num_samples_this_traj++;
		}

		// store trajectory info
		Ptraj.emplace_back(range{offset, num_samples_this_traj});
		offset += num_samples_this_traj;
	}

	// At this point, we know if we loaded something useful
	if (num_segs < 1)
		return traj_dataset<flt_type>(); // discard everything done up to now and just return an invalid dataset

	// non-position trajectory infos
	traj_format_handler<flt_type>::trajectories(ret, A.attrib) = Ptraj; // all attributes are sampled in sync with position, so we
	traj_format_handler<flt_type>::trajectories(ret, V.attrib) = Ptraj; // can duplicate the trajectory info from the positions to
	traj_format_handler<flt_type>::trajectories(ret, T.attrib) = Ptraj; // altitude, speed and time as well

	// invent radii
	traj_format_handler<flt_type>::set_avg_segment_length(ret, flt_type(seg_dist_accum / num_segs));
	auto R = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, OBD_RADIUS_ATTRIB_NAME);
	R.data.values = std::vector<flt_type>(P.data.num(), ret.avg_segment_length()*real(0.125));
	R.data.timestamps = P.data.timestamps;
	traj_format_handler<flt_type>::trajectories(ret, R.attrib) = Ptraj; // invented radius "samples" are again in sync with positions, so just copy traj info

	// commit async attributes
	// ToDo: THIS ASSUMES THERE CAN ONLY EVER BE ONE TRAJECTORY IN AN OBD FILE!!! (which all evidence points to)
	for (const auto &e : float_series)
	{
		auto F = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &f : e.second)
			F.data.append(f.value, convert_time(f.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, F.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}
	for (const auto &e : int_series)
	{
		auto I = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &i : e.second)
			I.data.append((flt_type)i.value, convert_time(i.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, I.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}
	for (const auto &e : bool_series)
	{
		auto B = traj_format_handler<flt_type>::template add_attribute<flt_type>(ret, e.first);
		for (const auto &b : e.second)
			B.data.append((flt_type)b.value, convert_time(b.timestamp));
		traj_format_handler<flt_type>::trajectories(ret, B.attrib).emplace_back(range{0, (unsigned)e.second.size()});
	}

	// visual attribute mapping
	ret.set_mapping(vamap);

	// set dataset name (we just use the filename for now)
	traj_format_handler<flt_type>::name(ret) = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(path));

	// done!*/
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
