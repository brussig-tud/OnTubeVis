
#ifndef __TESTCASE_NOMINAL_H__
#define __TESTCASE_NOMINAL_H__


//////
//
// Includes
//

// C++ STL
#include <array>
#include <type_traits>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include "common.h"
#include "stream/stream.h"



//////
//
// Module namespace(s)
//

namespace testcase {



//////
//
// Functions
//

/// Create visualization setup for the test case '@a nominal'.
auto nominal_setup (void)
{
	// Create the configuration
	return OTVConfiguration(
		"nominal", 3, 0.5f,
		SurfaceColorLayer(Rainbow, Linear),
		SignBlobLayer(.03125f, 1, otv__Rgb(1/3.f, 2/3.f, 1)),
		RectangleLayer(.03125f, otv__Rgb(6/7.f, 1/3.f, 0.03125f))
	);
}
typedef std::invoke_result_t<decltype(nominal_setup)> NominalConfig;

/// Entry point for the test case '@a nominal'.
void nominal_run (const NominalConfig &config)
{
	// Keep track of the most recent glyph's extent per layer to check if there is no overlap when streaming new ones
	auto last_border = make_array<NominalConfig::num_layers>(-std::numeric_limits<float>::infinity());

	auto traj = stream::Nodes::compile({
		stream::Nodes::Event{.0f, cgv::vec3(0,0,0), cgv::vec3(4,0,0)},
		stream::Nodes::Event{4.f, cgv::vec3(4,0,0), cgv::vec3(4,0,0)}
	});
	const auto &segment = traj.segment(0);

	const float s0 = segment.s_from_time(0);
	const float s1 = segment.s_from_time(1);
	const float s2 = segment.s_from_time(2);
	const float s3 = segment.s_from_time(3);
	const float s4 = segment.s_from_time(4);
	std::clog << "alen: "<<s0<<", "<<s1<<", "<<s2<<", "<<s3<<", "<<s4 << std::endl;

	/* Stream test glyphs (trajectory segment not yet there) */ {
		// First sign blob on layer 1
		OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */.0f, /* color: (configured to be static)*/0, /* value: */1.f
		);
		OTV_Vec2 extents = otv__instantiate_Glyph(config.traj_ids[1], 1, &sign_blob);
		printf("Streaming new sign blob - extents relative to anchor are [%f..%f]\n", extents.x, extents.y);
		printf(
			"                        - free space to previous one is %f\n",
			sign_blob.s+extents.x - last_border[1]
		);
		last_border[1] = sign_blob.s+extents.y;
		last_border[1] = sign_blob.s + extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */1, &sign_blob);

		// First surface color sample on layer 0
		OTV_GlyphData surface_color = otv__construct_SurfaceColorData(/* s: */0, /* color: */0);
		extents = otv__instantiate_Glyph(config.traj_ids[1], 0, &surface_color);
		printf("Streaming new surface color sample - extents relative to anchor are [%f..%f]\n",
		       extents.x, extents.y);
		last_border[0] = surface_color.s+extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */0, &surface_color);
	}

	/* Stream a test trajectory */ {
		// 1st node
		OTV_HermiteNode n = { // our test segment will be an x-line 4 units long...
			.time=0, .position={0, 0, 0}, .tangent={4, 0, 0}
		}; // ...so the x-derivative must also be 4 if we want a uniform parametrization
		otv__stream_spline_node(config.traj_ids[1], &n, NULL/* first sample doesn't have an arclength */);
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// Stream a second sign blob glyph (first trajectory segment still missing its end node)
		OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */1.f, /* color: (configured to be static)*/0, /* value: */1.f
		);
		OTV_Vec2 extents = otv__instantiate_Glyph(config.traj_ids[1], 1, &sign_blob);
		printf(
			"Streaming new sign blob - free space to previous one is %f\n", sign_blob.s+extents.x - last_border[1]
		);
		last_border[1] = sign_blob.s+extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */1, &sign_blob);
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// 2nd node
		const OTV_HermiteNode n0 = n; // we need to keep a copy of the first node for computing arc length
		n.time += 10;
		n.position.x += 4;
		// - compute arc length - we use the numerical approximation provided by the API here, which is relatively slow
		//   (albeit fairly accurate). Consider using information available to you in your data to infer arc length
		//   yourself.
		const OTV_SegmentArclen alen = otv__compute_arclen(&n0, &n, 0);
		// - dispatch the node
		otv__stream_spline_node(config.traj_ids[1], &n, &alen);
		std::this_thread::sleep_for(std::chrono::seconds(1));

		// Stream a second surface color sample on layer 0
		OTV_GlyphData surface_color = otv__construct_SurfaceColorData(/* s: */2, /* color: */1);
		extents = otv__instantiate_Glyph(config.traj_ids[1], 0, &surface_color);
		printf(
			"Streaming new surface color sample - free space to previous one is %f\n",
			surface_color.s+extents.x - last_border[0]
		);
		last_border[0] = surface_color.s+extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */0, &surface_color);
		std::this_thread::sleep_for(std::chrono::milliseconds(333));
	}

	/* Stream a third surface color sample on layer 0 */ {
		OTV_GlyphData surface_color = otv__construct_SurfaceColorData(/* s: */3.95, /* color: */0.5);
		const OTV_Vec2 extents = otv__instantiate_Glyph(config.traj_ids[1], 0, &surface_color);
		printf(
			"Streaming new surface color sample - free space to previous one is %f\n",
			surface_color.s+extents.x - last_border[0]
		);
		last_border[0] = surface_color.s+extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */0, &surface_color);
		std::this_thread::sleep_for(std::chrono::milliseconds(333));
	}

	/* Stream two rectangle glyphs at once */ {
		const OTV_GlyphData rectangles[2] = {
			otv__construct_RectangleData(
				/* s: */2, /* color: (configured to be static)*/0,
				/* width: (configured to be static)*/1, /* height: (configured to be static)*/1
			),
			otv__construct_RectangleData(
				/* s: */3, /* color: (configured to be static)*/0,
				/* width: (configured to be static)*/1, /* height: (configured to be static)*/.5f
			)
		};
		const OTV_Vec2 extents0 = otv__instantiate_Glyph(config.traj_ids[1], 2, &rectangles[0]);
		printf("Streaming new rectangle - extents relative to anchor are [%f..%f]\n", extents0.x, extents0.y);
		printf(
			"                        - free space to previous one is %f\n",
			rectangles[0].s+extents0.x - last_border[2]
		);
		last_border[2] = rectangles[0].s+extents0.y;
		const OTV_Vec2 extents1 = otv__instantiate_Glyph(config.traj_ids[1], 2, &rectangles[1]);
		printf(
			"Streaming new rectangle - free space to previous one is %f\n",
			rectangles[1].s+extents1.x - last_border[2]
		);
		last_border[2] = rectangles[1].s+extents1.y;
		otv__stream_glyphs(config.traj_ids[1], /* layer: */2, rectangles, 2);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	/* Stream a third sign blob glyph (falls onto the now complete first segment) */ {
		const OTV_GlyphData sign_blob = otv__construct_SignBlobData(
			/* s: */2.f, /* color: (configured to be static)*/0, /* value: */1.f
		);
		const OTV_Vec2 extents = otv__instantiate_Glyph(config.traj_ids[1], 1, &sign_blob);
		printf(
			"Streaming new sign blob  - free space to previous one is %f\n",
			sign_blob.s+extents.x - last_border[1]
		);
		last_border[1] = sign_blob.s+extents.y;
		otv__stream_glyph(config.traj_ids[1], /* layer: */1, &sign_blob);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}



//////
//
// Module namespace(s) close
//

// ::testcase
};


#endif // ifndef __TESTCASE_NOMINAL_H__
