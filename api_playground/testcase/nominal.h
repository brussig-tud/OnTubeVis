
#ifndef __TESTCASE_NOMINAL_H__
#define __TESTCASE_NOMINAL_H__


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
auto nominal_setup (void)
{
	// Create the configuration
	return OTVConfiguration(
		"nominal", 3, 3, 0.5f,
		SurfaceColorLayer(Rainbow, Linear),
		SignBlobLayer(.03125f, 1, otv__Rgb(1/3.f, 2/3.f, 1)),
		RectangleLayer(.03125f, otv__Rgb(6/7.f, 1/3.f, 0.03125f))
	);
}
typedef std::invoke_result_t<decltype(nominal_setup)> NominalConfig;

/// Entry point for the test case '@a nominal'.
void nominal_run (const NominalConfig &config)
{
	// Create event stream
	stream::Dataset data = stream::Dataset::construct(config);

	// Base trajectory (will be identical except for translation)
	auto traj0 = stream::Nodes::compile(config, {
		stream::Nodes::Event{.0f, cgv::vec3(0,0,0), cgv::vec3(4,0,0)},
		stream::Nodes::Event{4.f, cgv::vec3(4,0.5f,0.125f), cgv::vec3(4,0,0)},
		stream::Nodes::Event{8.f, cgv::vec3(8,-0.125,-0.25f), cgv::vec3(4,0,0)},
		stream::Nodes::Event{12.f, cgv::vec3(12,.125,0.25f), cgv::vec3(4,0,0)},
	});

	// 2nd trajectory (time-shifted and translated)
	const float traj1_dt = .25f;
	auto traj1 = traj0.transformed(translate4(cgv::vec3{1.f, .0f, -1.5f}), traj1_dt);

	// 3nd trajectory (time-shifted and translated)
	const float traj2_dt = -.25f;
	auto traj2 = traj0.transformed(translate4(cgv::vec3{-1.25f, .0f, -3.25f}), traj2_dt);

	// Glyphs on layer 0
	stream::Glyphs t0l0;
	/* 1st segment */ {
		const auto &seg = traj0.segment(0);
		float time = 0;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 0)
			);
		time = 0.5f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .25f)
		);
		time = 1.f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .5f)
		);
		time = 3.5f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .9f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj0.segment(1);
		float time = 4.f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .75)
			);
		time = 4.75f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 1.f)
		);
		time = 5.9f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .5f)
		);
	}
	/* 3rd segment */ {
		const auto &seg = traj0.segment(2);
		float time = 10.f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 1/3.f)
		);
	}
	/* beyond last segment */ {
		const auto &seg = traj0.segment(2).extrapol(0);
		float time = 13.125f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .75)
		);
	}

	// Glyphs on layer 1
	stream::Glyphs t1l1;
	/* 1st segment */ {
		const auto &seg = traj1.segment(0);
		float time = traj1_dt;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, 0)
			);
		time = traj1_dt+1.5f;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, -.5f)
		);
		time = traj1_dt+2.75f;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, -1.f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj1.segment(1);
		float time = traj1_dt+4.f;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, .25f)
			);
		time = traj1_dt+5.f;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, .75f)
		);
	}
	/* 3rd segment */ {
		const auto &seg = traj1.segment(2);
		float time = traj1_dt+9.125f;
		t1l1.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 1/3.f)
		);
	}
	/* beyond last segment */ {
		const auto &seg = traj1.segment(2).extrapol(0);
		float time = traj1_dt+12+1/3.f;
		t1l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, .125f)
		);
	}

	// Build event stream
	data.set_node_stream(1, std::move(traj1));
	data.set_node_stream(2, traj2);
	data.set_glyph_stream(0, 0, t0l0);
	data.set_glyph_stream(0, 1, traj0.adapt_glyphs(t1l1, -traj1_dt));
	data.set_glyph_stream(1, 1, t1l1);
	data.set_glyph_stream(2, 0, traj2.adapt_glyphs(t0l0, traj2_dt));
	data.set_glyph_stream(2, 1, traj2.adapt_glyphs(t1l1, -traj1_dt + traj2_dt));
	data.set_node_stream(0, std::move(traj0));
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


#endif // ifndef __TESTCASE_NOMINAL_H__
