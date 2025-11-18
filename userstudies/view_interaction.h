#pragma once

// C++ STL
#include <chrono>
#include <map>

// CGV framework core
#include <cgv/gui/event_handler.h>
#include <cgv/gui/help_message.h>

// CGV framework plugins
#include <plugins/crg_stereo_view/stereo_view_interactor.h>


namespace userstudies {

struct view_interaction_timeline;
struct view_interaction_accumulator
{
	using time_point = std::chrono::system_clock::time_point;

	view_interaction_accumulator() : start_time(std::chrono::system_clock::now()) {}
	view_interaction_accumulator(const view_interaction_accumulator&) = default;
	view_interaction_accumulator(view_interaction_accumulator&&) = default;

	template <class T>
	struct datapoint {
		/// how move change has accumulated including this point
		T accum;

		/// how much change is added at this point (this is a backwards difference!)
		T delta;

		/// create an all-zero datapoint
		inline static datapoint zeroed (void) {
			return {T(0), T(0)};
		};

		/// format a comma-separated pair of field names of this datapoint
		inline static std::string field_names (const std::string &prefix) {
			return prefix+"accum,"+prefix+"delta";
		}

		inline std::string format (void) const {
			return std::to_string(accum)+","+std::to_string(delta);
		}
	};

	time_point start_time;
	std::map<float, datapoint<double>> orbit, pan, roll, zoom, focus_move;
	std::map<float, datapoint<unsigned>> focus_move_count;

	inline void clear (void) {
		orbit.clear();
		pan.clear();
		roll.clear();
		zoom.clear();
		focus_move.clear();
		focus_move_count.clear();
	}
	inline void reset (void) {
		clear();
		start_time = std::chrono::system_clock::now();
	}

	template <class T>
	void log (std::map<float,datapoint<T>> &timeline, const time_point &time, const T delta) const
	{
		using datapoint = datapoint<T>;
		const auto delta_abs = (T)std::abs((double)delta);
		const auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time - start_time).count();
		const auto time_s = float(time_ms) / 1000;
		if (timeline.empty())
			timeline.emplace(time_s, datapoint{delta_abs, delta_abs});
		else {
			auto it = timeline.lower_bound(time_s);
			if (it != timeline.end())
			{
				if (it->first == time_s) {
					it->second.accum += delta_abs;
					it->second.delta += delta_abs;
					return;
				}
				std::string msg =
					"timeline already contains a later datapoint - old:"+std::to_string(it->first)+" > new:" +
					std::to_string(time_s);
				throw std::runtime_error(std::move(msg));
			}
			it = std::prev(timeline.end());
			timeline.emplace_hint(
				timeline.end(), time_s, datapoint{it->second.accum + delta_abs, delta_abs}
			);
		}
	}

	inline double log_interaction (const view_interaction &interaction)
	{
		constexpr double _1_over_pi = 0.31830988618379067153776752674502872499;
		switch (interaction.kind)
		{
			case view_interaction::Kind::Orbit: {
				const double degrees = interaction.amount*_1_over_pi * 180.;
				log(orbit, interaction.time, degrees);
				return degrees;
			}
			case view_interaction::Kind::Pan: {
				const double percent = interaction.amount*100.;
				log(pan, interaction.time, percent);
				return percent;
			}
			case view_interaction::Kind::Roll: {
				const double degrees = interaction.amount*_1_over_pi * 180.;
				log(roll, interaction.time, degrees);
				return degrees;
			}
			case view_interaction::Kind::Zoom: {
				const double percent = interaction.amount*100.;
				log(zoom, interaction.time, percent);
				return percent;
			}
			case view_interaction::Kind::FocusChange:
				log(focus_move, interaction.time, interaction.amount);
				log(focus_move_count, interaction.time, (unsigned)1);
				return interaction.amount;

			default:
				// We don't log anything else
				return 0;
		}
	}

	view_interaction_timeline create_timeline (void) const;
};

struct view_interaction_timeline {
	view_interaction_timeline() = default;
	view_interaction_timeline(const view_interaction_timeline&) = default;
	view_interaction_timeline(view_interaction_timeline&&) = default;

	template<class T>
	using datapoint = view_interaction_accumulator::datapoint<T>;
	using datapoint_count = view_interaction_accumulator::datapoint<unsigned>;
	struct record {
		inline static record zeroes (void) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(),
				datapoint::zeroed(), datapoint_count::zeroed()
			};
		}
		inline static record from_orbit (const datapoint<double> &orbit_data) {
			using datapoint = datapoint<double>;
			return {
				orbit_data, datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(),
				datapoint::zeroed(), datapoint_count::zeroed()
			};
		}
		inline static record from_pan (const datapoint<double> &pan_data) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), pan_data, datapoint::zeroed(), datapoint::zeroed(),
				datapoint::zeroed(), datapoint_count::zeroed()
			};
		}
		inline static record from_roll (const datapoint<double> &roll_data) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), datapoint::zeroed(), roll_data, datapoint::zeroed(),
				datapoint::zeroed(), datapoint_count::zeroed()
			};
		}
		inline static record from_zoom (const datapoint<double> &zoom_data) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(), zoom_data,
				datapoint::zeroed(), datapoint_count::zeroed()
			};
		}
		inline static record from_focus_move (const datapoint<double> &focus_move_data) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(),
				datapoint::zeroed(), focus_move_data, datapoint_count::zeroed()
			};
		}
		inline static record from_focus_move_count (const datapoint<unsigned> &focus_move_count_data) {
			using datapoint = datapoint<double>;
			return {
				datapoint::zeroed(), datapoint::zeroed(), datapoint::zeroed(),
				datapoint::zeroed(), datapoint::zeroed(), focus_move_count_data
			};
		}
		datapoint<double> orbit, pan, roll, zoom, focus_move;
		datapoint<unsigned> focus_move_count;
	};
	std::map<float, record> records;

	void write_csv (const std::string &filename) const;
};

}
