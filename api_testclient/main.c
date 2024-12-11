/**
 * @file
 * @brief
 *		A minimal C11 example of how to use the OnTubeVis API for setting up a visualization and streaming data to it.
 */


//////
//
// Includes
//

// C standard library
#include <stdio.h>

// Platform SDKs
#if defined(OTVAPI_CLIENT_USE_DLOPEN) && OTVAPI_CLIENT_USE_DLOPEN!=0
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



//////
//
// Functions
//

// The program entry point.
int main (int argc, char** argv)
{
	// Forward control to the service
	// - init the OnTubeVis implementation
	otv__startup(argc-1, (const char *const*)argv+1);
	// - while the init is ongoing, we could do other stuff.
	const bool otv_initialized = otv__wait_for_startup();
	if (!otv_initialized) {
		printf("OnTubeVis service failed to initialize!\n");
		return -1;
	}

	////
	// --- [BEGIN] A super simplified streaming example -----------------------------------------

	////
	// Preparation: Create and activate a visualization setup

	// Create it. The provided name will show up in the OnTubeVis GUI as the name of the visualized dataset
	OTV_VisSetupHandle test_setup = otv__create_VisSetup("test");
	uint32_t traj_ids[3];

	/* Add a surface color layer */ {
		OTV_LayerConfig new_layer = {
			.type=SurfaceColor, .outline=0,
			.static_params=otv__construct_SurfaceColorInfo(
				/* color_map: */Rainbow, /*interpolation_mode: */Cubic
			)
		};
		otv__add_layer(test_setup, &new_layer);
	}

	// Add two trajectories
	traj_ids[0] = otv__add_trajectory(test_setup, /* radius: */.5f);
	traj_ids[1] = otv__add_trajectory(test_setup, /* radius: */.5f);

	/* Add a sign blob layer. It will be added to all existing trajectories also. */ {
		OTV_LayerConfig new_layer = {
			.type=SignBlob, .outline=.0625f,
			.static_params=otv__construct_SignBlobInfo(
				/* color: */otv__Rgb(1/3.f, 2/3.f, 1), /* color_map: */UndefinedColormap,
				/* radius: */1, /* value: (not used here, see 'static_flags')*/0,
				/* static_flags: */SBI_STATIC_COLOR
			)
		};
		otv__add_layer(test_setup, &new_layer);
	}

	// Now we add another trajectory. It will inherit the already set-up layers.
	traj_ids[2] = otv__add_trajectory(test_setup, /* radius: */.5f);


	// Start a visualization session with the above setup
	const bool session_started = otv__start_vis_session(test_setup);
	otv__free_VisSetup(test_setup); // now not needed anymore
	if (!session_started) {
		printf("Unable to start visualization session!\n");
		return -1;
	}


	////
	// Stream stuff into OnTubeVis

	// Keep track of the most recent glyph's extent to check if there is no overlap when we want to stream another one
	float last_border;

	/* Stream a test glyph (trajectory segment not yet there) */ {
		OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */0.5f, /* color: (configured to be static)*/0, /* value: */.5f
		);
		const OTV_Vec2 extents = otv__instantiate_Glyph(traj_ids[1], 1, &sign_blob);
		printf("Streaming new glyph - extents relative to anchor are [%f..%f]\n", extents.x, extents.y);
		last_border = sign_blob.s + extents.y;
		otv__stream_glyph(traj_ids[1], /* layer: */1, &sign_blob);
	}

	/* Stream a test trajectory */ {
		// 1st node
		OTV_HermiteNode n = {
			.time=0, .position={0,0,0}, .tangent={0,0,0}
		};
		otv__stream_spline_node(traj_ids[1], &n, NULL/* first sample doesn't have an arclength */);

		// Stream a second test glyph (first trajectory segment still missing its end node)
		OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */2.f, /* color: (configured to be static)*/0, /* value: */.125f
		);
		const OTV_Vec2 extents = otv__instantiate_Glyph(traj_ids[1], 1, &sign_blob);
		printf("Streaming new glyph - free space to previous one is %f\n", sign_blob.s+extents.x - last_border);
		last_border = sign_blob.s+extents.y;
		otv__stream_glyph(traj_ids[1], /* layer: */1, &sign_blob);

		// 2nd node
		n.time += 10;
		n.position.x += 4;
		const OTV_SegmentArclen alen = {{
			{0, 1/3.f,  2/3.f, 1}, // this basically describes a linear re-parametrization that essentially
			{1, 4/3.f,  5/3.f, 2}, // just scales the curve parameter t=0..1 to s=0..4. This is only valid
			{2, 7/3.f,  8/3.f, 3}, // because we know our two nodes are exactly 4 units apart and are
			{3,10/3.f, 11/3.f, 4}  // interpolated linearly (since the tangents are 0 at both ends).
		}}; // Normally this parameterization would need to be fitted using distance samples taken along the path!
		otv__stream_spline_node(traj_ids[1], &n, &alen);
	}

	/* Stream a third test glyph (falls onto the now complete first segment) */ {
		OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */3.75f, /* color: (configured to be static)*/0, /* value: */-.75f
		);
		const OTV_Vec2 extents = otv__instantiate_Glyph(traj_ids[1], 1, &sign_blob);
		printf("Streaming new glyph - free space to previous one is %f\n", sign_blob.s+extents.x - last_border);
		last_border = sign_blob.s+extents.y;
		otv__stream_glyph(traj_ids[1], /* layer: */1, &sign_blob);
	}


	////
	// Shut down

	// Request service to stop and quit
	const OTV_TerminateResult status = otv__terminate();
	if (status.terminated) {
		// OnTubeVis did shut down
		printf("OnTubeVis service exited with code %i.\n", status.exit_code);
		return status.exit_code;
	}
	// Something went wrong...
	printf("OnTubeVis service did not honor shutdown request!\n");
	printf("Performing unclean shutdown.\n");
	return -1;
}
