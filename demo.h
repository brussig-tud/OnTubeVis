#pragma once

// C++ STL
#include <cmath>
#include <chrono>
#include <random>

// CGV framework core
#include <cgv/render/render_types.h>

// Local includes
#include <traj_loader.h>


/// represents logical trajectory data
struct demo : public traj_format_handler<float>
{
	inline static const std::string ATTRIB_POSITION = "position";
	inline static const std::string ATTRIB_TANGENT = "tangent";
	inline static const std::string ATTRIB_RADIUS = "radius";
	inline static const std::string ATTRIB_COLOR = "color";

	// a single demo trajectory
	struct trajectory
	{
		/// positions
		std::vector<Vec3> positions;

		/// tangents
		std::vector<Vec4> tangents;

		/// radii
		std::vector<float> radii;

		/// colors
		std::vector<Vec3> colors;

		/// some scalar attribute
		std::vector<float> attrib_scalar;

		/// some 2-vector attribute
		std::vector<Vec2> attrib_vec2;

		/// some 3-vector attribute
		std::vector<Vec3> attrib_vec3;

		/// some 4-vector attribute
		std::vector<Vec3> attrib_vec4;
	};

	/// generate a trajectory and some attributes
	static trajectory gen_trajectory (unsigned num_samples, unsigned seed=0)
	{
		constexpr float pi = 3.14159265358979323846f;

		// prepare
		if (!seed)
			seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
		std::mt19937 generator(seed);
		std::uniform_real_distribution<float> uni_0to1(0, 1), cube10(-10, 10);
		std::normal_distribution<float> norm_m1to1(0, 0.3333f);
		trajectory traj;

		// first segment
		// - uniformly sample the unit sphere for initial direction
		float theta = 2*pi * uni_0to1(generator),
		      phi = std::acos(1.f - 2*uni_0to1(generator));
		Vec3 dir(std::sin(phi) * std::cos(theta),
		         std::sin(phi) * std::sin(theta),
		         std::cos(phi));
		// - choose initial segment properties
		float radius=0.25, length = 10*radius;
		// - compose
		traj.positions.emplace_back(cube10(generator), cube10(generator), cube10(generator));
		traj.tangents.emplace_back(Vec4(dir, 0.f));
		traj.radii.emplace_back(radius);
		traj.colors.emplace_back(0.75f, 0.8f, 0.9f);
		for (unsigned i=1; i<num_samples; i++)
		{
			// - update position
			traj.positions.emplace_back(traj.positions.back() + dir*length);
			// - generate new properties
			const Vec3 dirdelta(norm_m1to1(generator), norm_m1to1(generator), norm_m1to1(generator)),
			           newdir(cgv::math::normalize(dir + dirdelta));
			/* update_radius() */
			const float tanlen = length*0.75f;
			// - compose
			traj.tangents.emplace_back(Vec4(tanlen*cgv::math::normalize((dir+newdir)*0.5), 0.f));
			traj.radii.emplace_back(radius);
			traj.colors.emplace_back(0.75f, 0.8f, 0.9f);

			// iterate
			dir = newdir;
		}

		// Done!
		return std::move(traj);
	}

	static typename traj_dataset<float> compile_dataset (const std::vector<trajectory> &trajectories)
	{
		// our static visual attribute mapping
		static const visual_attribute_mapping<float> attrmap({
			{VisualAttrib::POSITION, {ATTRIB_POSITION}}, {VisualAttrib::TANGENT, {ATTRIB_TANGENT}},
			{VisualAttrib::RADIUS, {ATTRIB_RADIUS}}, {VisualAttrib::COLOR, {ATTRIB_COLOR}}
		});

		// prepare data containers
		typename traj_dataset<float> ds;
		auto &P = add_attribute<Vec3>(ds, ATTRIB_POSITION);
		auto &T = add_attribute<Vec4>(ds, ATTRIB_TANGENT);
		auto &R = add_attribute<float>(ds, ATTRIB_RADIUS);
		auto &C = add_attribute<Vec3>(ds, ATTRIB_COLOR);
		auto &I = indices(ds);
		auto &ds_trajs = traj_format_handler<float>::trajectories(ds);
		ds.set_mapping(attrmap);

		// compile
		unsigned idx=0, idx_base=0, num_segs=0;
		real avg_dist = 0;
		for (const auto &traj : trajectories)
		{
			// copy over visual attributes
			std::copy(traj.positions.begin(), traj.positions.end(), std::back_inserter(P));
			std::copy(traj.tangents.begin(), traj.tangents.end(), std::back_inserter(T));
			std::copy(traj.radii.begin(), traj.radii.end(), std::back_inserter(R));
			std::copy(traj.colors.begin(), traj.colors.end(), std::back_inserter(C));

			// build indices and determine avg segment length
			I.push_back(idx); unsigned idx_cur = idx+1;
			for (unsigned i=1; i<(unsigned)traj.positions.size()-1; i++)
			{
				I.push_back(idx_cur);
				I.push_back(idx_cur); // twice to gain line-list semantics
				avg_dist += (P[idx_cur] - P[idx_cur-1]).length();
				num_segs++; idx_cur++;
			}
			if (traj.positions.size() > 1)
			{
				I.push_back(idx_cur); // final index only once
				avg_dist += (P[idx_cur] - P[idx_cur-1]).length();  // account for final
			}
			else
				// degenerate single-sample segment
				I.push_back(idx);
			num_segs++;

			// store trajectory indexing info
			ds_trajs.emplace_back(range{idx_base, (unsigned)I.size()-idx_base});
			idx = idx_cur + 1;
			idx_base = (unsigned)I.size();
		}

		set_avg_segment_length(ds, avg_dist/num_segs);
		return std::move(ds);
	}
};


// anonymous namespace for separation of global variable instances per compilation unit
namespace {
};
