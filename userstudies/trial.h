#pragma once

// local includes
#include <plugins/screenshot/screenshot.h>
#include <userstudies/view_interaction.h>

namespace userstudies {

struct trial
{
	cgv::gui::file_helper definition_file;

	virtual ~trial() = default;

	virtual bool on_view_interaction (const view_interaction&) { return false; }

	virtual bool setup (screenshot *tasks, std::string trial_id) = 0;
	virtual bool advance_to_next_task (void) = 0;
	virtual void begin_cur_task (void) = 0;
	virtual void end_cur_task (void) = 0;
	virtual signed get_cur_task (void) = 0;
	virtual bool is_cur_task_ongoing (void) = 0;
	virtual unsigned cur_task_run_count (void) = 0;
	virtual void stop (void) {
		cleanup();
		trial::definition_file.set_file_name("");
	}

private:
	virtual void cleanup (void) = 0;
};

}
