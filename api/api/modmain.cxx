
//////
//
// Includes
//

// C++ STL
#include <iostream>
#include <memory>
#include <thread>
#include <future>

// Public interface
#include <OnTubeVis/OnTubeVis.h>

// Private interface
#include <helper/module_handling.h>
#include <state/on_tube_vis.h>
#include <state/streaming.h>



//////
//
// Class implementations
//

////
// Streaming commands used by this unit

// terminate the OnTubeVis service
struct terminate_command : public command
{
	virtual const std::string& describe (void) final {
		const static std::string desc = "terminate";
		return desc;
	}

	virtual bool handle (void) final {
		on_tube_vis::running = false;
		return notify_result(true);
	}
};



//////
//
// Globals
//

// Anonymous namespace to avoid symbol conflicts between compilation units
namespace {
	std::future<int> otv_retval;
	std::thread otv_thread;
}



//////
//
// Functions
//

// The internal runner for the OnTubeVis main loop
void otv_runner (std::promise<int> &&p, int argc, const char *const *argv)
{
	// Outsmart the compiler and/or linker and make sure _our_ main function is called
	const auto my_main = get_on_tube_vis_main();
	p.set_value(my_main(argc, argv));
}

OTV_API bool otv__startup (const int argc, const char *const *argv)
{
	// If OnTubeVis was already started, don't do anything
	if (otv_thread.joinable())
		return false;

	// Adapt command line arguments
	const auto argc_adapted = argc+1;
	auto argv_adapted = std::make_unique<const char*[]>(argc_adapted);
	argv_adapted[0] = otv__get_module_filepath();
	for (unsigned i=0; i<argc; i++)
		argv_adapted[i+1] = argv[i];

	// Hand off to the fake CGV Framework main loop
	std::promise<int> p;
	otv_retval = p.get_future();
	otv_thread = std::thread(otv_runner, std::move(p), argc_adapted, argv_adapted.get());

	// Report success if we managed to spawn the runner thread
	return otv_thread.joinable();
}

OTV_API bool otv__wait_for_startup (void)
{
	// Sanity check (we should have at least spawned the runner thread for this call to make sense)
	if (!otv_thread.joinable())
		return false;

	/* Wait for the startup to finish */ {
		std::unique_lock l(on_tube_vis::init_mtx);
		while (on_tube_vis::init_pending)
			on_tube_vis::init_cv.wait(l);
	}
	return on_tube_vis::running;
}

OTV_API OTV_TerminateResult otv__terminate (void)
{
	auto te = std::make_shared<terminate_command>();
	std::clog << "otv__terminate: requesting termination." << std::endl;
	command_stream::push(te);
	const bool result = te->fetch_result();
	std::clog << "otv__terminate: implementation returned '"<<result<<"' for request to terminate." << std::endl;
	if (result) {
		otv_thread.join();
		return {.exit_code=otv_retval.get(), .terminated=true};
	}
	return {.terminated=false};
}

OTV_API const char* otv__get_module_filepath (void) {
	return on_tube_vis_module_filepath().c_str();
}

OTV_API void otv__unique_module_tag (void)
{}
