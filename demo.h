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
	/// maths constant π
	inline static const float pi = 3.14159265358979323846f;
	/// maths constant ℯ
	inline static const float e = 2.71828182845904523536f;

	inline static const std::string ATTRIB_POSITION = "position";
	inline static const std::string ATTRIB_TANGENT = "tangent";
	inline static const std::string ATTRIB_RADIUS = "radius";
	inline static const std::string ATTRIB_COLOR = "color";

	inline static const auto uni_0or1 = std::uniform_int_distribution<int>(0, 1);
	inline static const auto uni_0to1 = std::uniform_real_distribution<float>(0, 1);

	/// a single demo trajectory
	struct trajectory
	{
		/// a time-stamped attribute value
		template <class T>
		struct attrib_value
		{
			/// default constructor
			inline attrib_value() {}
			/// convenience constructor
			inline attrib_value (float t, const T& value) : t(t), value(value) {}

			/// timestamp
			float t;
			/// attribute value at this timestamp
			T value;
		};

		/// positions
		std::vector<Vec3> positions;

		/// tangents
		std::vector<Vec4> tangents;

		/// radii
		std::vector<float> radii;

		/// colors
		std::vector<Vec3> colors;

		/// some scalar attribute
		std::vector<attrib_value<float>> attrib_scalar;

		/// some 2-vector attribute
		std::vector<attrib_value<Vec2>> attrib_vec2;

		/// some 3-vector attribute
		std::vector<attrib_value<Vec3>> attrib_vec3;

		/// some 4-vector attribute
		std::vector<attrib_value<Vec4>> attrib_vec4;
	};

	/// returns the "north" pole of the unit hypersphere
	template <class T>
	static const T& pole(void)
	{
		static struct pole_struct {
			T point;
			pole_struct() {
				point[0] = 1;
				for (unsigned i=1; i<T::dims; i++)
					point[i] = 0;
			}
		} _pole;
		return _pole.point;
	}
	template <>
	static const float& pole<float> (void)
	{
		static const float _1 = 1;
		return _1;
	}

	/// 1D non-periodic noise-like function.
	inline static float noise (float x, float phase=0, float bias=0, float scale=1, float freq=1)
	{
		const float q = x*freq - phase;
		// arbitrary and hand-tuned coefficients, irrational factors ℯ and π for non-periodicity
		return bias + scale*0.175f*(-3.2f*std::sin(-1.3f*q) - 1.2f*std::sin(-1.7f*e*q) + 1.9f*std::sin(0.7f*pi*q));
	}

	/// generic length template
	template <class T>
	inline static float len (const T &v) { return v.length(); }
	template <>
	inline static float len<float> (const float &v) { return std::abs(v); }

	/// generic normalization template
	template <class T>
	inline static T normalized(const T &v) { return v/len(v); }

	/// generate a random point on the unit hypersphere
	template <class T>
	static T gen_unit_point (std::mt19937 &generator)
	{
		// special handling of scalar case for efficiency (statically resolved at compile-time)
		if (T::dims == 1)
			return { gen_unit_point<float>(generator) };
		
		// generate random parameters for use with n-spherical coordinates
		constexpr unsigned last_coord = T::dims-1, num_phis = T::dims-1;
		constexpr   signed last_phi = num_phis-1;
		float phi[num_phis];
		for (signed i=0; i<last_phi; i++)
			phi[i] = std::acos(1.f - 2*uni_0to1(generator));
		phi[last_phi] = 2*pi * uni_0to1(generator);

		// calculate point on hypersphere from parameters
		T result;
		// - initialize first and last coordiante
		result[0] = std::cos(phi[0]);
		result[last_coord] = std::sin(phi[last_phi]);
		// - build up second to (n-1)th coordinates
		for (unsigned i=1; i<last_coord; i++)
		{
			// applies to x_0 to x_{n-1}
			result[i] = std::cos(phi[i]);
			for (unsigned j=0; j<i; j++)
				result[i] *= std::sin(phi[j]);
			// update x_n
			result[last_coord] *= std::sin(phi[i-1]);
		}

		// done!
		return std::move(result);
	}
	template <>
	static float gen_unit_point<float> (std::mt19937& generator)
	{
		return 2*uni_0or1(generator) - 1.f;
	}

	/// generates a random velocity vector with the given mean magnitude and sigma standard deviation
	template <class T>
	static T gen_velocity (float mean, float sigma, std::mt19937 &generator)
	{
		std::normal_distribution<float> norm(mean, sigma);
		const T p = gen_unit_point<T>(generator);
		const float mag = norm(generator);
		return mag * p;
	}

	/// generates a random direction offset given a sigma standard deviation of angle (in radians)
	template <class T>
	static T gen_dir_offset(float sigma, std::mt19937 &generator)
	{
		// special handling of scalar case for efficiency (statically resolved at compile-time)
		if (T::dims == 1)
			return { gen_dir_offset<float>(sigma, generator) };
		
		// generate random parameters for use with n-spherical coordinates
		std::normal_distribution<float> norm(0, sigma);
		constexpr unsigned last_coord = T::dims-1, num_phis = T::dims-1;
		constexpr   signed last_phi = num_phis-1;
		float phi[num_phis];
		for (signed i=0; i<last_phi; i++)
			phi[i] = norm(generator);
		phi[last_phi] = 2*pi * uni_0to1(generator);

		// calculate point on hypersphere from parameters
		T result;
		// - initialize first and last coordiante
		result[0] = std::cos(phi[0]);
		result[last_coord] = std::sin(phi[last_phi]);
		// - build up second to (n-1)th coordinates
		for (unsigned i=1; i<last_coord; i++)
		{
			// applies to x_0 to x_{n-1}
			result[i] = std::cos(phi[i]);
			for (unsigned j=0; j<i; j++)
				result[i] *= std::sin(phi[j]);
			// update x_n
			result[last_coord] *= std::sin(phi[i-1]);
		}

		// done!
		return std::move(result);
	}
	template <>
	static float gen_dir_offset<float> (float sigma, std::mt19937& generator)
	{
		// We interpret it as the projection of a 2D direction change onto the original line of movement
		std::normal_distribution<float> norm(0, sigma);
		return std::cos(norm(generator));
	}

	/// randomly perturbates a velocity vector given standard deviations from the original velocity direction and magnitude
	template <class T>
	static T gen_vel_perturbation(const T& vel_orig, float sigma_dir, float sigma_mag, std::mt19937 &generator)
	{
		std::normal_distribution<float> norm(0, sigma_mag);
		const float dmag_new = norm(generator), mag_new = len(vel_orig)+dmag_new;
		const T dvec = gen_dir_offset<T>(sigma_dir, generator) - pole<T>();
		return mag_new * normalized(normalized(vel_orig)+dvec);
	}

	/// generate a generic attribute with slightly varying sampling rate
	template <class T>
	static std::vector<trajectory::attrib_value<T>> gen_attribute (
		float mean, float sigma, float t0, float tn, float dt_mean, float dt_var, std::mt19937 &generator
	)
	{
		// using a different phase for the non-periodic sine-based function has the effect of "seeding" it - by
		// chosing it from a sizable portion of the float value range we get something quite random-looking
		const float phase = (uni_0to1(generator) - 0.5f) * 1048576*pi;

		// first sample
		std::vector<trajectory::attrib_value<T>> result;
		float t = t0;
		result.emplace_back(t, gen_velocity<T>(mean, sigma, generator));
		const float sigma_mag = len(result.back().value)/3.f;

		// remaining samples
		for (unsigned s=0; t<=tn; s++)
		{
			const float dt_raw = noise((float)s, phase, dt_mean, dt_var, 0.5f);
			const float dt = std::max(
				dt_raw,
				std::max(	// <-- make sure we actually advance t everytime
					std::numeric_limits<float>::epsilon(),
					std::numeric_limits<float>::epsilon() * t
				)
			);
			t += dt;
			result.emplace_back(t, gen_vel_perturbation<T>(result.back().value, pi/6, sigma_mag, generator));
		}

		// done!
		return std::move(result);
	}

	/// generate a trajectory and some attributes
	static trajectory gen_trajectory (unsigned num_samples, unsigned seed=0)
	{
		// prepare
		if (!seed)
			seed = (unsigned)std::chrono::system_clock::now().time_since_epoch().count();
		std::mt19937 generator(seed);
		std::uniform_real_distribution<float> cube10(-10, 10);
		std::normal_distribution<float> norm_sigma1by3(0, 0.3333f);
		trajectory traj;

		// geometry
		// - uniformly sample the unit sphere for initial direction
		Vec3 dir(gen_unit_point<Vec3>(generator));
		// - choose initial segment properties
		float radius=0.25f, length = 10*radius;
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
			const Vec3 dirdelta(norm_sigma1by3(generator), norm_sigma1by3(generator), norm_sigma1by3(generator)),
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

		// generic, independently sampled attributes
		const float tn = (float)num_samples + 1;
		traj.attrib_scalar = gen_attribute<float>(0, 0.25f, 0, tn, 1/12.f, 1/48.f, generator);
		traj.attrib_vec2 = gen_attribute<Vec2>(1, 0.33333f, 0, tn, 1/4.f, 1/16.f, generator);
		traj.attrib_vec3 = gen_attribute<Vec3>(1, 0.33333f, 0, tn, 1/4.f, 1/16.f, generator);
		traj.attrib_vec4 = gen_attribute<Vec4>(1, 0.33333f, 0, tn, 1/4.f, 1/16.f, generator);

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

		// done!
		set_avg_segment_length(ds, avg_dist/num_segs);
		return std::move(ds);
	}
};


// anonymous namespace for separation of global variable instances per compilation unit
namespace {
};
