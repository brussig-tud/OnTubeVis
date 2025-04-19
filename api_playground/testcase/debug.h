
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
		"debug", 3, 0.5f, 3, OTV_ExtrapolProgression::Instant,
		LinePlotLayer(
			.03125f, Linear, otv__Rgb(1,0,1), otv__Rgb(0,1,1)
		)
	);
}
typedef std::invoke_result_t<decltype(debug_setup)> DebugConfig;

/// Entry point for the test case '@a nominal'.
void debug_run (const DebugConfig &config)
{
	// Create event stream
	stream::Dataset data = stream::Dataset::construct(config);

	// Base trajectory (will be identical except for translation)
	auto traj1 = stream::Nodes::compile(config, {
		stream::Nodes::Event{.0f, cgv::vec3(0,0,0), cgv::vec3(2,0,0)},
		stream::Nodes::Event{8.f, cgv::vec3(2,0.25f,0.125f), cgv::vec3(2,0,0)},
		stream::Nodes::Event{16.f, cgv::vec3(4,.125f,-.125f), cgv::vec3(2,0,0)},
		stream::Nodes::Event{24.f, cgv::vec3(6,-.125f,0), cgv::vec3(2,0,0)}
	});
	stream::NodesStreamer traj1_stream(traj1, 0);

	// Line plot layer
	stream::Glyphs t1l0;
	/* 1st segment */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(0);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	/* 2nd segment */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(1);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	/* 3rd segment */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(2);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	/* beyond traj, 1st extrapol */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(2).extrapol(0);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	/* beyond traj, 2nd extrapol */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(2).extrapol(1);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	/* beyond traj, 3nd extrapol */ {
		constexpr unsigned num_samples = 8;
		const auto seg = traj1.segment(2).extrapol(2);
		for (unsigned sample=0; sample<num_samples; ++sample) {
			const float t = .25f/float(num_samples) + sample/float(num_samples), s = seg.s(t);
			const float x = t*4*std::numbers::pi_v<float>;
			const std::array v{ .5f+.5f*std::cos(x), .5f+.5f*std::sin(x) };
			t1l0.add(
				seg.time_from_t(t),
				otv__construct_LinePlotData(s, 2, v.data())
			);
		}
	}
	stream::GlyphsStreamer t1l0_stream(t1l0, 1, 0);

	/*traj1_stream.stream_up_to_time(1);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(0);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l0_stream.stream_up_to_time(5);


	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l1_stream.stream_up_to_time(1);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l1_stream.stream_up_to_time(2);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l1_stream.stream_up_to_time(3);
	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l1_stream.stream_up_to_time(4);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj1_stream.stream_up_to_time(2);

	std::cerr << "[ENTER]" << std::endl; getchar();
	t1l1_stream.stream_up_to_time(5);

	std::cerr << "[ENTER]" << std::endl; getchar();
	traj1_stream.stream_up_to_time(3);*/

	// Build event stream
	data.set_node_stream(1, std::move(traj1));
	data.set_glyph_stream(1, 0, std::move(t1l0));
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


#endif // ifndef __TESTCASE_DEBUG_H__
