
#ifndef __TESTCASE_TRIANGLES_H__
#define __TESTCASE_TRIANGLES_H__


//////
//
// Includes
//

// C++ STL
#include <array>
#include <type_traits>

// CGV Framework
#include <cgv/math/ftransform.h>

// OnTubeVis API
#include <OnTubeVis/OnTubeVis.h>

// Local includes
#include "common.h"
#include "stream/dataset.h"
#include "stream/eventseq.h"



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
auto triangles_setup (void)
{
	// Create the configuration
	return OTVConfiguration(
		"triangles", 3, 0.5f, 3, OTV_ExtrapolProgression::Instant,
		SurfaceColorLayer(Rainbow, Linear),
		CircleLayer(.03125f, Acton),
		IsoscelesTriangleLayer(
			.03125f, otv__Rgb(1,0,1), layer::WidthProperty{0.75},
			layer::HeightProperty{1.25f}/*, layer::OrientationProperty{157.5}*/
		)
	);
}
typedef std::invoke_result_t<decltype(triangles_setup)> TriangleConfig;

/// Entry point for the test case '@a nominal'.
void triangles_run (const TriangleConfig &config)
{
	// Create event stream
	stream::Dataset data = stream::Dataset::construct(config);

	// Base trajectory (will be identical except for translation)
	auto traj1 = stream::Nodes::compile(config, {
		stream::Nodes::Event{.0f, cgv::vec3(0,0,0), cgv::vec3(2,0,0)},
		stream::Nodes::Event{1.f, cgv::vec3(2,0.25f,0.125f), cgv::vec3(2,0,0)},
		stream::Nodes::Event{2.f, cgv::vec3(4,.125f,-.125f), cgv::vec3(2,0,0)}/*,
		stream::Nodes::Event{4.f, cgv::vec3(6,-.125f,0), cgv::vec3(2,0,0)}*/
	});
	stream::NodesStreamer traj1_stream(traj1, 0);

	// Surface color layer
	stream::Glyphs t1l0;
	/* 1st segment */ {
		const auto &seg = traj1.segment(0);
		float time = 0;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 0)
			);
		time = 1/3.f;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .75f)
		);
		time = 4/3.f;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .25f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj1.segment(1);
		float time = 2.5f;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .9f)
			);
		time = 3.5f;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .125f)
		);
	}
	/* beyond traj */ {
		const auto &seg = traj1.segment(1).extrapol(0);
		float time = 4.5f;
		t1l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 1)
		);
	}
	stream::GlyphsStreamer t1l0_stream(t1l0, 1, 0);

	// Isosceles Triangle layer
	stream::Glyphs t1l1;
	/* 1st segment */ {
		const auto &seg = traj1.segment(0);
		float time = .125f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), 0, .5f)
			);
		time = 0.95f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), .25f, .75f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj1.segment(1);
		float time = 4/3.f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), .5f, 0.9f)
			);
		time = 1.99f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), .75f, 2/3.f)
		);
	}
	/* 3rd segment */ {
		const auto &seg = traj1.segment(/*2*/1).extrapol(0);
		float time = 2.75f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), 1, 1/3.f)
			);
	}
	/* beyond traj */ {
		const auto &seg = traj1.segment(/*2*/1).extrapol(/*0*/1);
		float time = 3 + 1/3.f;
		t1l1.add(
			time,
			otv__construct_CircleData(seg.s_from_time(time), .5f, .5f)
		);
	}
	stream::GlyphsStreamer t1l1_stream(t1l1, 1, 2);

	// Isosceles Triangle layer
	stream::Glyphs t1l2;
	/* 1st segment */ {
		const auto &seg = traj1.segment(0);
		float time = .125f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), 0, .125f, .5f, -45)
			);
		time = 0.95f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), .25f, .5f, .25f, -22.5)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj1.segment(1);
		float time = 4/3.f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), .5f, 0.75f, .75f, 0)
			);
		time = 1.99f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), .75f, 1.f, .875f, 22.5)
		);
	}
	/* 3rd segment */ {
		const auto &seg = traj1.segment(/*2*/1).extrapol(0);
		float time = 2.75f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), 1, 2/3.f, 1, 45)
			);
	}
	/* beyond traj */ {
		const auto &seg = traj1.segment(/*2*/1).extrapol(/*0*/1);
		float time = 3 + 1/3.f;
		t1l2.add(
			time,
			otv__construct_IsoscelesTriangleData(seg.s_from_time(time), 1, 1/3.f, 1.75, 67.5)
		);
	}
	stream::GlyphsStreamer t1l2_stream(t1l2, 1, 2);

	// Build event stream
	data.set_node_stream(1, std::move(traj1));
	data.set_glyph_stream(1, 0, std::move(t1l0));
	data.set_glyph_stream(1, 1, std::move(t1l1));
	data.set_glyph_stream(1, 2, std::move(t1l2));
	const auto event_stream = stream::EventSequence::compile(data);

	// Play back in OnTubeVis
	event_stream.play_back();
}



//////
//
// Module namespace(s) close
//

// ::testcase
};


#endif // ifndef __TESTCASE_TRIANGLES_H__
