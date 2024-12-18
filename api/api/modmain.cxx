
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
#include <state/core.h>
#include <state/streaming.h>

// Local includes
#include <on_tube_vis.h>



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
		otv_instance->quit();
		return notify_result(true);;
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
void otv_runner (std::promise<int> &&p, std::vector<char*> &&args)
{
	// Report status
	std::clog << "OnTubeVis service is starting." << std::endl;

	// Outsmart the compiler and/or linker and make sure _our_ main function is called
	const auto cgv_main = get_on_tube_vis_main();

	// Hand off control flow
	p.set_value(cgv_main((int)args.size(), args.data()));

	// Service has exited
	std::clog << "OnTubeVis service is terminating." << std::endl;
}

OTV_API bool otv__startup (const int argc, const char *const *argv)
{
	// If OnTubeVis was already started, don't do anything
	if (otv_thread.joinable())
		return false;

	// Log startup
	std::clog << "Starting OnTubeVis service..." << std::endl;

	// Adapt command line arguments
	std::vector<char*> args_adapted; args_adapted.reserve(argc+1);
	args_adapted.emplace_back(const_cast<char*>(otv__get_module_filepath()));
	args_adapted.emplace_back(const_cast<char*>("option:service"));
	std::clog << "- number of forwarded arguments: "<<argc << std::endl;
	for (unsigned i=0; i<argc; i++) {
		std::clog << "  "<<i+1<<": "<<argv[i] << std::endl;
		args_adapted.emplace_back(const_cast<char*>(argv[i]));
	}

	// Inform about effective command line
	std::clog << "- effective OnTubeVis command line arguments:" << std::endl;
	for (int i=0; i<args_adapted.size(); i++)
		std::clog << "  "<<i<<": "<<args_adapted[i] << std::endl;
	std::clog << std::endl;

	// Spawn the thread the CGV Framework main loop will run in
	std::promise<int> p;
	otv_retval = p.get_future();
	otv_thread = std::thread(otv_runner, std::move(p), std::move(args_adapted));

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
		while (!otv_instance)
			on_tube_vis::init_cv.wait(l);
	}
	return true;
}

OTV_API OTV_TerminateResult otv__terminate (void)
{
	auto te = std::make_shared<terminate_command>();
	std::clog << "otv__terminate: requesting termination." << std::endl;
	command_stream::push(te);
	const bool result = te->fetch_result();
	std::clog << "otv__terminate: implementation returned '"<<result<<"' for request to terminate." << std::endl;
	if (result) {
		std::unique_lock l(on_tube_vis::init_mtx);
		while (otv_instance != nullptr)
			on_tube_vis::init_cv.wait(l);
		l.unlock();
		otv_thread.join();
		return {otv_retval.get(), true};
	}
	return {-1, false};
}

OTV_API const char* otv__get_module_filepath (void) {
	return on_tube_vis_module_filepath().c_str();
}

OTV_API void otv__unique_module_tag (void) {}
