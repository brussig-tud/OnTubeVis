
//////
//
// Includes
//

// C++ STL
#include <iostream>
#include <thread>

// Private interface
#include <state/on_tube_vis.h>
#include <state/streaming.h>
#include <util/cppstream.h>



//////
//
// Functions
//

// The actual CGV Framework entry point as it would be provided by the static cgv_viewer_lib.
int api_main (int argc, char** argv)
{
	// Report status
	std::clog << "OnTubeVis headless service started." << std::endl;

	// Main event loop
	/* - init and notify once it's done */ {
		std::lock_guard g(on_tube_vis::init_mtx);
		on_tube_vis::init_pending = false;
		on_tube_vis::running = true; // normally we would check initialization success before setting this
		on_tube_vis::init_cv.notify_all();
	}
	// - enter the main loop
	while (on_tube_vis::running)
	{
		// Process a command (and block waiting if there is none)
		auto cmd = command_stream::fetch();
		if (!cmd->handle())
			std::cerr << "OnTubeVis: command "<<hex(cmd.get())<<" ("<<cmd->describe()<<") failed execution!"
			          << std::endl;

		// Make sure we don't hog a CPU core
		std::this_thread::yield();
	}

	// Shut down
	std::clog << "OnTubeVis headless service is terminating." << std::endl;
	return 0;
}
