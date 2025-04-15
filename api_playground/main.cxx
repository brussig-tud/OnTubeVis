/**
 * @file
 * @brief
 *		A C++-based playground application for testing the API in various ways (forcing edge cases etc.). Does not serve
 *		any fixed purpose and may change drastically and erratically at any time.
 */


//////
//
// Includes
//

// C++ STL
#include <iostream>
#define _M_PI 3.14159265358979323846fM_PI
#define M_PI _M_PI

// Platform SDKs
#if defined(OTVAPI_PLAYGROUND_USE_DLOPEN) && OTVAPI_PLAYGROUND_USE_DLOPEN!=0
	#error Runtime loading of an OnTubeVis API implementation is not yet implemented in this example!
	#ifdef _WIN32
		#define NOMINMAX
		#include <windows.h>
	#else
		#include <dlfcn.h>
	#endif
#endif

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// Testcases
#include "testcase/debug.h"
#include "testcase/nominal.h"

// Local includes
#include "../util.h"



//////
//
// Functions
//

/// @brief Encapsulates the OnTubeVis API shutdown logic.
///
/// This encapsulates the common cleanup logic needed for both successful program shutdown and termination in case of
/// various errors.
int shutdown_otv (void)
{
	const OTV_TerminateResult status = otv__terminate();
	if (status.terminated) {
		// OnTubeVis did shut down
		std::clog << "OnTubeVis service exited with code "<<status.exit_code << std::endl;
		return status.exit_code;
	}
	// Something went wrong...
	std::clog << "OnTubeVis service did not honor shutdown request!" << std::endl;
	std::clog << "Performing unclean shutdown." << std::endl;
	return -1;
}

/// The program entry point.
int main (int argc, char** argv)
{
	// Forward control to the service
	// - init the OnTubeVis implementation
	otv__startup(argc-1, (const char *const*)argv+1);
	// - while the init is ongoing, we could do other stuff.
	const bool otv_initialized = otv__wait_for_startup();
	if (!otv_initialized) {
		std::clog << "OnTubeVis service failed to initialize!" << std::endl;
		return -1;
	}


	////
	// Execute a test case

	// Collect test configuration
	auto config = testcase::nominal_setup/*debug_setup*/();

	// Set up visualization
	VisSetup setup(otv__create_VisSetup(config.name.c_str()));
	config.apply(setup);

	// Start a visualization session with the obtained setup
	const bool session_started = otv__start_vis_session(setup.handle);
	if (!session_started) {
		std::clog << "Unable to start visualization session!" << std::endl;
		const int exit_code = shutdown_otv();
		return exit_code ? -1 : exit_code;
	}

	// Run the test case
	testcase::nominal_run/*debug_run*/(config);


	////
	// Shutdown

	// Wait for user to terminate the test client
	printf("\nStreaming done. Press ENTER to shut down.");
	getchar();

	// Request service to stop and quit
	return shutdown_otv();
}
