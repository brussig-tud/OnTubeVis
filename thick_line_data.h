#pragma once

#include <random>
#include <istream>
#include <string>
#include <vector>
#include <cgv/math/fvec.h>
#include <cgv/math/fmat.h>
#include <cgv/media/color.h>

struct thick_line_data
{
	const float kPi = (float)M_PI;
	float magic_factor = 0.75f;
	std::vector<uint32_t>  indices;
	std::vector<cgv::vec4> spheres;
	std::vector<cgv::vec4> tangents;
	std::vector<cgv::rgb>  colors;
	std::vector<cgv::mat4> bezier_tubes;
	void clear();
	int add_node(const cgv::vec3& center, const cgv::vec3& tangent, float radius, float radius_derivative, const cgv::rgb& color);
	void add_segment(int start_node_idx, int end_node_idx);
	void compute_bezier_tubes();
	cgv::mat4x3 get_split_tube(size_t i, int part_idx) const;
	cgv::rgb get_tube_color(float t, size_t i, int part_idx = -1) const;
	void init_simple();
	void generate(int nr_tubes = 10, int nr_segments = 100);
	int append_open_curve(const std::vector<cgv::vec3>& P,
		const std::vector<float>& radii,
		const std::vector<cgv::rgb>& color);
	int generate_torus_knot(
		int p, int q, float ring_radius, float tube_offset, float z_scale,
		const cgv::vec3& center, int key_count, float tube_radius,
		int bead_freq, float bead_amp, float bead_phase,
		const cgv::rgb& color_a, const cgv::rgb& color_b);
	void generate_woven_bundle(
		int outer_count, int inner_count, float outer_radius, float inner_radius,
		float turns, const cgv::vec3& base, float length, int key_count,
		float strand_radius,
		const cgv::rgb& outer_color_a, const cgv::rgb& outer_color_b,
		const cgv::rgb& inner_color_a, const cgv::rgb& inner_color_b);
	int generate_lorenz_trail(
		uint64_t seed, float target_size, int keys, float base_radius,
		const cgv::vec3& center, const cgv::rgb& head_color, const cgv::rgb& tail_color);
	int generate_tendril(
		std::mt19937_64& rng, const cgv::vec3& base_pos, cgv::vec3 dir,
		int keys, float step, float curl, float base_radius, float bead_phase,
		const cgv::rgb& root_color, const cgv::rgb& tip_color);
	void generate_trajectories(uint64_t seed);
	thick_line_data();
	enum class scene_type {
		simple, random, procedural
	};
	void select_scene(scene_type scene);
	/// read from file with format detected from extension. Currently supported: bcc, ply
	bool read(const std::string& file_path, float default_radius = 0.01f, float tangent_scale = 1.0f);
	/// read from input stream over the contents of a valid ply file
	bool read_ply (std::istream &contents, float default_radius = 0.01f, float tangent_scale = 1.0f);
};
