
// Implemented header
#include "view_interaction.h"

// C++ STL
#include <filesystem>


namespace userstudies {

view_interaction_timeline view_interaction_accumulator::create_timeline(void) const
{
	using record = view_interaction_timeline::record;
	view_interaction_timeline timeline;
	// orbit interaction
	for (const auto &[time, datapoint] : orbit) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_orbit(datapoint));
		else
			it->second.orbit = datapoint;
	}
	// pan interaction
	for (const auto &[time, datapoint] : pan) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_pan(datapoint));
		else
			it->second.pan = datapoint;
	}
	// roll interaction
	for (const auto &[time, datapoint] : roll) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_roll(datapoint));
		else
			it->second.roll = datapoint;
	}
	// zoom interaction
	for (const auto &[time, datapoint] : zoom) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_zoom(datapoint));
		else
			it->second.zoom = datapoint;
	}
	// focus_move interaction
	// - amount
	for (const auto &[time, datapoint] : focus_move) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_focus_move(datapoint));
		else
			it->second.focus_move = datapoint;
	}
	// - count
	for (const auto &[time, datapoint] : focus_move_count) {
		auto it = timeline.records.find(time);
		if (it == timeline.records.end())
			timeline.records.emplace(time, record::from_focus_move_count(datapoint));
		else
			it->second.focus_move_count = datapoint;
	}

	// tap out early if nothing was recorded
	if (timeline.records.empty())
		return std::move(timeline);

	// fill in accumulation gaps
	auto it = timeline.records.begin();
	// - first record
	it->second.orbit.accum = it->second.orbit.delta;
	it->second.pan.accum = it->second.pan.delta;
	it->second.roll.accum = it->second.roll.delta;
	it->second.zoom.accum = it->second.zoom.delta;
	it->second.focus_move.accum = it->second.focus_move.delta;
	it->second.focus_move_count.accum = it->second.focus_move_count.delta;
	record *prev = &it->second;
	++it;
	// - all remaining records
	for (; it!=timeline.records.end(); ++it) {
		it->second.orbit.accum = prev->orbit.accum + it->second.orbit.delta;
		it->second.pan.accum = prev->pan.accum + it->second.pan.delta;
		it->second.roll.accum = prev->roll.accum + it->second.roll.delta;
		it->second.zoom.accum = prev->zoom.accum + it->second.zoom.delta;
		it->second.focus_move.accum = prev->focus_move.accum + it->second.focus_move.delta;
		it->second.focus_move_count.accum = prev->focus_move_count.accum + it->second.focus_move_count.delta;
		prev = &it->second;
	}

	// sanity check
	constexpr double err_tol = 0.015625;
	const auto last_record = std::prev(timeline.records.end());
	if (!orbit.empty()) {
		const auto last = std::prev(orbit.end());
		const auto diff = last->second.accum-last_record->second.orbit.accum;
		if (std::abs(diff) > err_tol)
			std::cerr << "WARNING: accumulation of orbit actions does not match between individual and merged timelines!\n"
			          << "         diff = "<<diff << std::endl;
	}
	if (!pan.empty()) {
		const auto last = std::prev(pan.end());
		const auto diff = last->second.accum-last_record->second.pan.accum;
		if (std::abs(diff) > err_tol)
			std::cerr << "WARNING: accumulation of panning actions does not match between individual and merged timelines!\n"
			          << "         diff = "<<diff << std::endl;
	}
	if (!roll.empty()) {
		const auto last = std::prev(roll.end());
		const auto diff = last->second.accum-last_record->second.roll.accum;
		if (std::abs(diff) > err_tol)
			std::cerr << "WARNING: accumulation of roll actions does not match between individual and merged timelines!\n"
			          << "         diff = "<<diff << std::endl;
	}
	if (!zoom.empty()) {
		const auto last = std::prev(zoom.end());
		const auto diff = last->second.accum-last_record->second.zoom.accum;
		if (std::abs(diff) > err_tol)
			std::cerr << "WARNING: accumulation of zoom actions does not match between individual and merged timelines!\n"
			          << "         diff = "<<diff << std::endl;
	}
	if (!focus_move.empty()) {
		/* amounts */ {
			const auto last = std::prev(focus_move.end());
			const auto diff = last->second.accum-last_record->second.focus_move.accum;
			if (std::abs(diff) > err_tol)
				std::cerr << "WARNING: accumulation of focus move actions does not match between individual and merged timelines!\n"
				          << "         diff = "<<diff << std::endl;
		}
		/* counts */ {
			const auto last = std::prev(focus_move_count.end());
			const signed diff = last->second.accum-last_record->second.focus_move_count.accum;
			if (diff != 0)
				std::cerr << "WARNING: focus move action counts do not match between individual and merged timelines!\n"
				          << "         diff = "<<diff << std::endl;
		}
	}

	// done!
	return std::move(timeline);
}


}
