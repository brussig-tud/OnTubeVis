#pragma once

#include "arclen_helper.h"
#include "traj_loader.h"
#include "glyph_layer_manager.h"
#include "render/common.h"

struct layer_compile_result {
	std::vector<irange> ranges;
	std::vector<float> timestamps;
	glyph_storage glyphs;

	layer_compile_result(size_t mapped_attribute_count) : glyphs(mapped_attribute_count) {}

	bool empty() const {
		return ranges.empty() || glyphs.empty() || timestamps.empty();
	}
};

struct attribute_trajectory_pair {
	const traj_attribute<float>* attribute_handle = nullptr;
	const std::vector<range>* trajectories = nullptr;
};

class layer_compile_state {
public:
	struct segment_result {
		segment_time<float> segtime = { 0.0f, -1.0f };
		float t_local = 0.0f;
		float s = 0.0f;

		bool is_valid() const {
			return segtime.t0 <= segtime.t1;
		}
	};

	layer_compile_state(
		attribute_trajectory_pair position_attribute,
		const std::vector<cgv::mat4>& arc_lengths,
		const glyph_shape* shape,
		size_t mapped_attribute_count
	);

	bool has_trajectory() const {
		return trajectory_index < position_attribute.trajectories->size();
	}

	range get_trajectory() const {
		return current_range;
	}

	void skip_to_trajectory_end() {
		local_segment_index = local_segment_count;
	}

	bool has_segment() const {
		return local_segment_index < local_segment_count;
	}

	void advance_to_next_trajectory();

	segment_result find_next_fitting_segment_at_time(float t);

	void place_glyph(
		const glyph_layer_manager::configuration::layer_configuration& layer_config,
		const std::vector<float>& attrib_values,
		float s,
		float t,
		float length_scale,
		bool show_hidden
	);

	size_t get_trajectory_index() const {
		return trajectory_index;
	}

	size_t get_global_segment_index() const {
		return global_segment_index;
	}

	size_t get_local_segment_count() const {
		return local_segment_count;
	}

	size_t get_local_segment_index() const {
		return local_segment_index;
	}

	layer_compile_result get_result() const {
		return result;
	}

private:
	// clamp value v to in range and remap to out range
	float clamp_remap(float v, const cgv::vec2 in, const cgv::vec2 out);

	void setup_trajectory();

	void increment_segment_index();

	segment_time<float> advance_to_segment_containing_time(float t);

	bool can_place_glyph_at_time(segment_time<float> segtime, float t) const;

	std::tuple<float, float> get_segment_local_time_and_arc_length(segment_time<float> segtime, float t) const;

	const attribute_trajectory_pair position_attribute;
	const std::vector<cgv::mat4>& arc_lengths;
	const glyph_shape* shape = nullptr;
	size_t trajectory_index = 0;

	range current_range;
	size_t global_attribute_offset = 0;
	size_t global_segment_offset = 0;
	size_t global_segment_index = 0;
	size_t local_segment_count = 0;
	size_t local_segment_index = 0;
	float prev_glyph_size = 0.0f;
	float last_committed_s = 0.0f;

	// Scratch buffer to hold mapped glyph parameters that are needed to compute its size
	std::vector<float> glyph_params;

	layer_compile_result result;
};

class attribute_interpolation_state {
public:
	attribute_interpolation_state(std::vector<attribute_trajectory_pair> attributes);

	bool initialize_indices(size_t trajectory_index, float t);

	void increment_to_time(float t);

	void interpolate_values(float t);

	bool reached_end() const;

	const std::vector<float>& get_attribute_values() {
		return values;
	}

private:
	struct attribute_entry {
		const traj_attribute<float>* handle = nullptr;
		const std::vector<range>* trajectories = nullptr;
		cgv::uvec2 piece_indices = { 0, 1 };
		unsigned index0 = 0;
		unsigned index1 = 1;
		unsigned indices_end = 0;
	};
	std::vector<attribute_entry> attributes;
	std::vector<float> values;
};

class glyph_compiler {
public:
	std::vector<layer_compile_result> compile_glyph_attributes(
		const traj_dataset<float>& data_set,
		const arclen::parametrization& parametrization,
		const glyph_layer_manager::configuration& layers_config
	) const;

	bool include_hidden_glyphs = false;
	float length_scale = 1.0f;
	float sample_step_threshold = 0.005f;
	float attribute_timestamp_epsilon = 0.001f;

private:
	// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
	layer_compile_result compile_glyphs_front_at_samples(
		attribute_trajectory_pair position_attribute,
		const std::vector<cgv::mat4>& arc_lengths,
		const glyph_layer_manager::configuration::layer_configuration& layer_config,
		const glyph_shape* shape,
		const std::vector<attribute_trajectory_pair>& mapped_attributes
	) const;

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_uniform_time(
		attribute_trajectory_pair position_attribute,
		const std::vector<cgv::mat4>& arc_lengths,
		const glyph_layer_manager::configuration::layer_configuration& layer_config,
		const glyph_shape* shape,
		const std::vector<attribute_trajectory_pair>& mapped_attributes
	) const;

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_equidistant(
		attribute_trajectory_pair position_attribute,
		const arclen::parametrization& parametrization,
		const glyph_layer_manager::configuration::layer_configuration& layer_config,
		const glyph_shape* shape,
		const std::vector<attribute_trajectory_pair>& mapped_attributes
	) const;

	layer_compile_result compile_glyph_layer(
		const traj_dataset<float>& data_set,
		const arclen::parametrization& parametrization,
		const std::vector<std::string>& attribute_names,
		attribute_trajectory_pair position_attribute,
		const glyph_layer_manager::configuration::layer_configuration& layer_config
	) const;
};
