#pragma once

// C++ STL
#include <cmath>
#include <chrono>
#include <random>
#include <numeric>

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

		/// some scalar attributes
		std::vector<attrib_value<float>> attrib_scalar;
		std::vector<attrib_value<float>> attrib_scalar1;
		std::vector<attrib_value<float>> attrib_scalar2;
		std::vector<attrib_value<float>> attrib_scalar3;

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

	/// 1D non-periodic noise-like function build from 3 sines
	inline static float sine_noise (float x, float phase=0, float bias=0, float scale=1, float freq=1)
	{
		const float q = x*freq - phase;
		// arbitrary and hand-tuned coefficients, irrational factors ℯ and π for non-periodicity
		return bias + scale*0.175f*(-3.2f*std::sin(-1.3f*q) - 1.2f*std::sin(-1.7f*e*q) + 1.9f*std::sin(0.7f*pi*q));
	}

	/// simple parametrized noise function
	struct noise_function
	{
		float phase = 0, bias = 0, scale = 1, freq = 1;
		float operator() (float x) const {
			return sine_noise(x, phase, bias, scale, freq);
		}
	};

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
	static float gen_unit_point<float> (std::mt19937&)
	{
		return pole<float>();
	}

	/// generates a random velocity vector with the given mean magnitude and sigma standard deviation
	template <class T>
	static T gen_velocity (const noise_function &noise, std::mt19937 &generator)
	{
		const T p = gen_unit_point<T>(generator);
		const float mag = noise(0);
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
	static float gen_dir_offset<float> (float, std::mt19937&)
	{
		// We ignore direction completely - just return the pole
		return  pole<float>();
	}

	/// randomly perturbates a velocity vector given standard deviations from the original velocity direction and magnitude
	template <class T>
	static T gen_vel_perturbation(float t, const T& vel_orig, float sigma_dir, const noise_function &noise, std::mt19937 &generator)
	{
		const T dvec = gen_dir_offset<T>(sigma_dir, generator) - pole<T>();
		const float mag_new = noise(t);
		return mag_new * normalized(normalized(vel_orig)+dvec);
	}
	/// special handling of scalar case for efficiency reasons
	template <>
	static float gen_vel_perturbation<float>(float t, const float&, float, const noise_function &noise, std::mt19937&)
	{
		return noise(t);
	}

	/// generate a generic attribute with slightly varying sampling rate
	template <class T>
	static std::vector<trajectory::attrib_value<T>> gen_attribute (
		float mean, float sigma, float t0, float tn, float dt_mean, float dt_var, std::mt19937 &generator
	)
	{
		// determine noise function parameters
		noise_function noise;
		// - using a different phase for the non-periodic sine-based function has the effect of "seeding" it - by
		//   chosing it from a sizable portion of the float value range we get something quite random-looking
		noise.phase = (uni_0to1(generator) - 0.5f) * 131072*pi;
		// - feature size of the noise function should be about double the mean sampling distance (according to Nyquist
		//   and Shannon...), we make it 4 times for good measure and to mitigate local undersampling.
		//   ToDo: make this user-overridable for intentionally aliased or oversampled signals
		noise.freq = .25f / dt_mean;
		// - mean translates 1:1 to bias (the y-offset of the zero-line)
		noise.bias = mean;
		// - expected scale corresponds to roughly 2 times sigma
		noise.scale = sigma+sigma;

		// first sample
		std::vector<trajectory::attrib_value<T>> result;
		float t = t0;
		result.emplace_back(t, gen_velocity<T>(noise, generator));

		// remaining samples
		for (unsigned s=0; t<=tn; s++)
		{
			const float dt_raw = sine_noise((float)s, noise.phase, dt_mean, dt_var, 0.5f);
			const float dt = std::max(
				dt_raw,
				std::max(	// <-- make sure we actually advance t everytime
					std::numeric_limits<float>::epsilon(),
					std::numeric_limits<float>::epsilon() * t
				)
			);
			t += dt;
			result.emplace_back(t, gen_vel_perturbation<T>(t, result.back().value, pi/6, noise, generator));
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
		//traj.attrib_scalar = gen_attribute<float>(0, 0.25f, 0, tn, 1/12.f, 1/48.f, generator);
		traj.attrib_scalar = gen_attribute<float>(0, 0.25f, 0, tn, 1/12.f, 0.f, generator);
		traj.attrib_scalar1 = gen_attribute<float>(0, 0.25f, 0, tn, 2/12.f, 0.f, generator);
		traj.attrib_scalar2 = gen_attribute<float>(0, 0.25f, 0, tn, 3/12.f, 0.f, generator);
		traj.attrib_scalar3 = gen_attribute<float>(0, 0.25f, 0, tn, 6/12.f, 0.f, generator);
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

		// prepare dataset
		typename traj_dataset<float> ds("Fuzzball", "DEMO");
		std::vector<range> ds_trajs, ds_trajs_scalar, ds_trajs_scalar1, ds_trajs_scalar2, ds_trajs_scalar3, ds_trajs_vec2, ds_trajs_vec3, ds_trajs_vec4;
		// - create geometry attributes
		auto &P = add_attribute<Vec3>(ds, ATTRIB_POSITION);
		auto &T = add_attribute<Vec4>(ds, ATTRIB_TANGENT);
		auto &R = add_attribute<real>(ds, ATTRIB_RADIUS);
		auto &C = add_attribute<Vec3>(ds, ATTRIB_COLOR);
		// - create free attributes
		auto &attrib_scalar = add_attribute<real>(ds, "scalar");
		auto &attrib_scalar1 = add_attribute<real>(ds, "scalar1");
		auto &attrib_scalar2 = add_attribute<real>(ds, "scalar2");
		auto &attrib_scalar3 = add_attribute<real>(ds, "scalar3");
		auto &attrib_vec2 = add_attribute<Vec2>(ds, "vec2");
		auto &attrib_vec3 = add_attribute<Vec3>(ds, "vec3");
		auto &attrib_vec4 = add_attribute<Vec4>(ds, "vec4");

		// compile
		unsigned idx=0, idx_base=0, num_segs=0;
		real avg_dist = 0;
		for (const auto &traj : trajectories)
		{
			// optimize allocations
			unsigned new_capacity = P.data.num() + (unsigned)traj.positions.size();
			P.data.reserve(new_capacity);
			T.data.reserve(new_capacity);
			R.data.reserve(new_capacity);
			C.data.reserve(new_capacity);

			// set trajectory indexing info
			ds_trajs.emplace_back(range{ P.data.num(), (unsigned)traj.positions.size() });
			ds_trajs_scalar.emplace_back(range{ attrib_scalar.data.num(), (unsigned)traj.attrib_scalar.size() });
			ds_trajs_scalar1.emplace_back(range{ attrib_scalar1.data.num(), (unsigned)traj.attrib_scalar1.size() });
			ds_trajs_scalar2.emplace_back(range{ attrib_scalar2.data.num(), (unsigned)traj.attrib_scalar2.size() });
			ds_trajs_scalar3.emplace_back(range{ attrib_scalar3.data.num(), (unsigned)traj.attrib_scalar3.size() });
			ds_trajs_vec2.emplace_back(range{ attrib_vec2.data.num(), (unsigned)traj.attrib_vec2.size() });
			ds_trajs_vec3.emplace_back(range{ attrib_vec3.data.num(), (unsigned)traj.attrib_vec3.size() });
			ds_trajs_vec4.emplace_back(range{ attrib_vec4.data.num(), (unsigned)traj.attrib_vec4.size() });

			// copy over position attribute, generate timestamps and determine avg segment length
			P.data.append(traj.positions.front(), 0);
			for (unsigned i=1; i<(unsigned)traj.positions.size(); i++)
			{
				avg_dist += (traj.positions[i] - traj.positions[i-1]).length();
				P.data.append(traj.positions[i], (float)i);
				num_segs++;
			}
			// copy over remaining geometry attributes
			std::copy(traj.tangents.begin(), traj.tangents.end(), std::back_inserter(T.data.values));
			std::copy(traj.radii.begin(), traj.radii.end(), std::back_inserter(R.data.values));
			std::copy(traj.colors.begin(), traj.colors.end(), std::back_inserter(C.data.values));

			// commit free attributes
			for (const auto &attrib : traj.attrib_scalar)
				attrib_scalar.data.append(attrib.value, attrib.t);
			for(const auto &attrib : traj.attrib_scalar1)
				attrib_scalar1.data.append(attrib.value, attrib.t);
			for(const auto &attrib : traj.attrib_scalar2)
				attrib_scalar2.data.append(attrib.value, attrib.t);
			for(const auto &attrib : traj.attrib_scalar3)
				attrib_scalar3.data.append(attrib.value, attrib.t);
			for (const auto &attrib : traj.attrib_vec2)
				attrib_vec2.data.append(attrib.value, attrib.t);
			for (const auto &attrib : traj.attrib_vec3)
				attrib_vec3.data.append(attrib.value, attrib.t);
			for (const auto &attrib : traj.attrib_vec4)
				attrib_vec4.data.append(attrib.value, attrib.t);
		}

		// transfer trajectory ranges
		// - geometry attributes
		traj_format_handler<float>::trajectories(ds, P.attrib) = ds_trajs;
		traj_format_handler<float>::trajectories(ds, T.attrib) = ds_trajs;
		traj_format_handler<float>::trajectories(ds, R.attrib) = ds_trajs;
		traj_format_handler<float>::trajectories(ds, C.attrib) = std::move(ds_trajs);
		// - free attributes
		traj_format_handler<float>::trajectories(ds, attrib_scalar.attrib) = std::move(ds_trajs_scalar);
		traj_format_handler<float>::trajectories(ds, attrib_scalar1.attrib) = std::move(ds_trajs_scalar1);
		traj_format_handler<float>::trajectories(ds, attrib_scalar2.attrib) = std::move(ds_trajs_scalar2);
		traj_format_handler<float>::trajectories(ds, attrib_scalar3.attrib) = std::move(ds_trajs_scalar3);
		traj_format_handler<float>::trajectories(ds, attrib_vec2.attrib) = std::move(ds_trajs_vec2);
		traj_format_handler<float>::trajectories(ds, attrib_vec3.attrib) = std::move(ds_trajs_vec3);
		traj_format_handler<float>::trajectories(ds, attrib_vec4.attrib) = std::move(ds_trajs_vec4);

		// finalize
		ds.set_mapping(attrmap);
		set_avg_segment_length(ds, avg_dist/num_segs);

		// done!
		return std::move(ds);
	}
};


// anonymous namespace for separation of global variable instances per compilation unit
namespace {
};
