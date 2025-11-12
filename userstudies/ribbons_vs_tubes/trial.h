#pragma once

// C++ STL
#include <chrono>
#include <optional>

// CGV Framework
#include <cgv/gui/file_helper.h>

// local includes
#include <userstudies/trial.h>
#include <userstudies/view_interaction.h>


namespace userstudies {
namespace RvT {

struct task_state {
	view_interaction_accumulator view_interactions;

	static task_state create (void) {
		return task_state{};
	}
};
struct trial : public userstudies::trial
{
	screenshot* screenshot_ptr = nullptr;
	std::string trial_id;
	signed cur_task = -1;
	std::vector<unsigned> task_run_counts;

	std::optional<task_state> state;

	bool on_view_interaction (const view_interaction &interaction) override
	{
		if (state.has_value())
		{
			auto &vi = state.value().view_interactions;
			switch (interaction.kind)
			{
				case view_interaction::Kind::Orbit:
					std::clog << "orbited "<<vi.log_interaction(interaction)<<"°" << std::endl;
					return true;
				case view_interaction::Kind::Pan:
					std::clog << "panned "<<vi.log_interaction(interaction)<<"%" << std::endl;
					return true;
				case view_interaction::Kind::Roll:
					std::clog << "rolled "<<vi.log_interaction(interaction)<<"°" << std::endl;
					return true;
				case view_interaction::Kind::Zoom:
					std::clog << "zoomed "<<vi.log_interaction(interaction)<<'%' << std::endl;
					return true;
				case view_interaction::Kind::FocusChange:
					std::clog << "focus changed "<<vi.log_interaction(interaction)<<" units" << std::endl;
					return true;
				case view_interaction::Kind::FocusChangeFromZoom:
					std::clog << "focus changed "<<interaction.amount<<" units (via zoom action)" << std::endl;
			}
		}
		return false;
	}

	bool setup (screenshot *tasks, std::string trial_id) override
	{
		if (tasks->get_shot_count() < 1 || trial_id.empty())
			return false;

		task_run_counts.clear();
		task_run_counts.reserve(tasks->get_shot_count());
		for (unsigned i=0; i<tasks->get_shot_count(); ++i)
			task_run_counts.emplace_back(0);

		screenshot_ptr = tasks;
		this->trial_id = trial_id;
		cur_task = -1;
		return true;
	}

	bool advance_to_next_task (void) override
	{
		const signed next_task = cur_task+1;
		const unsigned count = screenshot_ptr->get_shot_count();
		if (next_task < (signed)count) {
			cur_task = next_task;
			std::clog << "advancing to task "<<cur_task+1<<'/'<<count << std::endl;
			screenshot_ptr->set_active_shot_by_index(cur_task);
			return true;
		}
		std::clog << "no more tasks! (already at "<<cur_task+1<<'/'<<count<<')' << std::endl;
		return false;
	}

	void begin_cur_task (void) override {
		if (state.has_value()) {
			std::string msg("begin_cur_task(): task already started!");
			std::cerr << "INTERNAL ERROR: "<<msg << std::endl;
			throw std::runtime_error(std::move(msg));
		}
		state.emplace(task_state::create());
	}
	void end_cur_task (void) override
	{
		if (!state.has_value()) {
			std::string msg("end_cur_task(): no task started!");
			std::cerr << "INTERNAL ERROR: "<<msg << std::endl;
			throw std::runtime_error(std::move(msg));
		}

		++task_run_counts[cur_task];
		const std::string data_filename =
			  "trial_"+trial_id+"_task_"+std::to_string(cur_task)+".csv";

		const auto dur = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now() - state.value().view_interactions.start_time
		).count() / 1000.f;
		std::clog << "task finished! user took "<<dur<<"s" << std::endl;

		auto interaction_timeline = state.value().view_interactions.create_timeline();
		std::clog << "recorded interactions at "<<interaction_timeline.records.size()<<" unique times" << std::endl;

		interaction_timeline.records.emplace_hint(
			interaction_timeline.records.end(), dur, view_interaction_timeline::record::zeroes()
		);
		state.reset();
	}

	signed get_cur_task (void) override {
		return cur_task;
	}

	bool is_cur_task_ongoing (void) override {
		return state.has_value();
	}

	unsigned cur_task_run_count (void) override {
		return cur_task < 0 ? 0 : task_run_counts[cur_task];
	}

private:
	void cleanup (void) override {
		screenshot_ptr = nullptr;
		trial_id.clear();
		task_run_counts.clear();
		cur_task = -1;
	};
};

}}
