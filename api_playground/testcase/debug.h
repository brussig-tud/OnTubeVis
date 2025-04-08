
#ifndef __TESTCASE_DEBUG_H__
#define __TESTCASE_DEBUG_H__


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
auto debug_setup (void)
{
	// Create the configuration
	return OTVConfiguration(
		"debug", 3, 3, 0.5f,
		SurfaceColorLayer(Rainbow, Linear),
		SignBlobLayer(.03125f, 1, otv__Rgb(1/3.f, 2/3.f, 1)),
		RectangleLayer(.03125f, otv__Rgb(6/7.f, 1/3.f, 0.03125f))
	);
}
typedef std::invoke_result_t<decltype(debug_setup)> NominalConfig;

/// Entry point for the test case '@a nominal'.
void debug_run (const NominalConfig &config)
{
	// Create event stream
	stream::Dataset data = stream::Dataset::construct(config);

	// Base trajectory (will be identical except for translation)
	auto traj0 = stream::Nodes::compile(config, {
		stream::Nodes::Event{.0f, cgv::vec3(0,0,0), cgv::vec3(2,0,0)},
		stream::Nodes::Event{2.f, cgv::vec3(2,0.25f,0/*.125f*/), cgv::vec3(2,0,0)},
		stream::Nodes::Event{4.f, cgv::vec3(4,.125f,-.125f), cgv::vec3(2,0,0)}
	});
	stream::NodesStreamer traj0_stream(traj0, 0);

	auto traj1 = traj0.transformed(translate4(cgv::vec3{1.f, .0f, -1.5f}));
	stream::NodesStreamer traj1_stream(traj1, 1);

	auto traj2 = traj0.transformed(translate4(cgv::vec3{-1.f, .0f, -1.5f}));
	stream::NodesStreamer traj2_stream(traj2, 2);

	// Surface color layer
	stream::Glyphs t0l0;
	/* 1st segment */ {
		const auto &seg = traj0.segment(0);
		float time = 0;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 0)
			);
		time = 1/3.f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .75f)
		);
		time = 4/3.f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .25f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj0.segment(1);
		float time = 2.5f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .9f)
			);
		time = 3.5f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), .125f)
		);
	}
	/* beyond traj */ {
		const auto &seg = traj0.segment(1).extrapol(0);
		float time = 4.5f;
		t0l0.add(
			time,
			otv__construct_SurfaceColorData(seg.s_from_time(time), 1)
		);
	}
	stream::GlyphsStreamer t0l0_stream(t0l0, 0, 0);
	stream::GlyphsStreamer t1l0_stream(t0l0, 1, 0);
	stream::GlyphsStreamer t2l0_stream(t0l0, 2, 0);

	// Sign Blob layer
	stream::Glyphs t0l1;
	/* 1st segment */ {
		const auto &seg = traj0.segment(0);
		float time = .25f;
		t0l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, 0)
			);
		time = 1.5f;
		t0l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, .75f)
		);
	}
	/* 2nd segment */ {
		const auto &seg = traj0.segment(1);
		float time = 2.75f;
		t0l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, -.75f)
			);
		time = 3.5f;
		t0l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, -.125f)
		);
	}
	/* beyond traj */ {
		const auto &seg = traj0.segment(1).extrapol(0);
		float time = 4.5f;
		t0l1.add(
			time,
			otv__construct_SignBlobData(seg.s_from_time(time), 0, 1/3.f)
		);
	}
	stream::GlyphsStreamer t0l1_stream(t0l1, 0, 1);
	stream::GlyphsStreamer t1l1_stream(t0l1, 1, 1);
	stream::GlyphsStreamer t2l1_stream(t0l1, 2, 1);

	/*traj0_stream.stream_up_to_time(1);
	t0l0_stream.stream_up_to_time(0);

	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(1);
	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(2);

	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(3);
	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(4);

	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(5);


	//::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(1);
	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(2);
	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(3);
	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj0_stream.stream_up_to_time(3);

	//std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj0_stream.stream_up_to_time(5);*/

	/*traj0_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(1);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(2);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(3);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l0_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t0l1_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj0_stream.stream_up_to_time(7);*/


	traj2_stream.stream_up_to_time(1);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(0);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(5);


	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(2);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj2_stream.stream_up_to_time(3);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj2_stream.stream_up_to_time(5);

	/*std::cerr << "[ENTER]" << std::endl; getchar();
	traj2_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(1);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(2);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(3);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l1_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t2l0_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj2_stream.stream_up_to_time(7);*/

	// Build event stream
	//data.set_node_stream(1, std::move(traj1));
	/*data.set_glyph_stream(0, 0, std::move(t0l0));
	data.set_glyph_stream(0, 1, traj0.adapt_glyphs(t1l1, -traj1_dt));
	data.set_glyph_stream(1, 1, std::move(t1l1));
	data.set_node_stream(0, std::move(traj0));
	const auto event_stream = stream::EventSequence::compile(data);

	// Play back in OnTubeVis
	event_stream.play_back();*/
}



//////
//
// Module namespace(s) close
//

// ::testcase
};


#endif // ifndef __TESTCASE_DEBUG_H__
