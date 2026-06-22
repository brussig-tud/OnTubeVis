#include "compiler.h"

#include <cgv/math/compare_float.h>
#include <cgv/math/functions.h>


layer_compile_state::layer_compile_state(
	attribute_trajectory_pair position_attribute,
	const std::vector<cgv::mat4>& arc_lengths,
	const glyph_shape* shape,
	size_t mapped_attribute_count) :
	position_attribute(position_attribute),
	arc_lengths(arc_lengths),
	shape(shape),
	result(mapped_attribute_count) {

	// Ensure enough space to hold all glyph parameters. A persistent buffer is used to avoid allocating a new one for every individual glyph.
	glyph_params.resize(shape->num_size_attribs(), 0.0f);

	// Resize the result to hold one range per segment.
	size_t total_segment_count = position_attribute.attribute_handle->num() - position_attribute.trajectories->size();
	result.ranges.resize(total_segment_count);

	// Setup first trajectory.
	setup_trajectory();
}

void layer_compile_state::advance_to_next_trajectory() {
	// Update global from local state variables.
	trajectory_index++;
	global_attribute_offset = result.glyphs.get_glyph_count();
	global_segment_offset += local_segment_count;
	global_segment_index = global_segment_offset;

	// Reset local state variables.
	current_range = {};
	local_segment_count = 0;
	local_segment_index = 0;
	prev_glyph_size = 0.0f;
	last_committed_s = 0.0f;
	prev_glyph_size = 0.0f;
	last_committed_s = 0.0f;

	setup_trajectory();
}

layer_compile_state::segment_result layer_compile_state::find_next_fitting_segment_at_time(float t) {
	segment_time segtime = advance_to_segment_containing_time(t);

	if(can_place_glyph_at_time(segtime, t)) {
		segment_result result;
		result.segtime = segtime;
		std::tie(result.t_local, result.s) = get_segment_local_time_and_arc_length(result.segtime, t);
		return result;
	}

	return {};
}

void layer_compile_state::place_glyph(
	const glyph_layer_manager::configuration::layer_configuration& layer_config,
	const std::vector<float>& attribute_values,
	float s,
	float length_scale,
	bool show_hidden
) {
	// Setup glyph parameters.
	size_t parameter_index = 0;
	for(const auto& mapping_triple : layer_config.glyph_mapping_parameters) {
		if(mapping_triple.type == 0) {
			// Constant attribute
			glyph_params[parameter_index] = mapping_triple.v->output_range.y();
		} else {
			// Use windowing and remapping to get the value of the glyph parameter
			glyph_params[parameter_index] = clamp_remap(attribute_values[mapping_triple.idx], mapping_triple.v->input_range, mapping_triple.v->output_range);
		}
		++parameter_index;
	}


	float new_glyph_size = shape->get_size(glyph_params);
	new_glyph_size /= length_scale;

	// infer potential glyph extents
	const float min_dist = result.glyphs.empty() ?
		new_glyph_size :
		std::max(new_glyph_size, prev_glyph_size);

	bool show = result.glyphs.get_glyph_count() == global_attribute_offset || s >= last_committed_s + min_dist || min_dist < 0.0f;

	if(show || show_hidden) {
		auto& curr_range = result.ranges[global_segment_index];
		if(curr_range.n < 1) {
			// first free attribute that falls into this segment
			curr_range.i0 = static_cast<int>(result.glyphs.get_glyph_count());
			curr_range.n = 1;

			// handle overlap to the previous segments
			if(local_segment_index > 0) {
				size_t curr_global_segment_index = global_segment_index;

				float min_s = s - 0.5f * new_glyph_size;
				if(min_s >= 0.0f) {
					while(curr_global_segment_index > global_segment_offset && arc_lengths[curr_global_segment_index - 1][15] > min_s) {
						// if there have been no glyphs committed to the previous segment until now, also update its start index
						auto& prev_range = result.ranges[curr_global_segment_index - 1];
						if(prev_range.n == 0)
							prev_range.i0 = curr_range.i0;
						prev_range.n++;
						curr_global_segment_index--;
					}
				}

			}
		} else {
			// one more free attribute that falls into this segment
			curr_range.n++;
			// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
			if(global_segment_index > 0 && new_glyph_size < 0.0f) {
				// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
				result.ranges[global_segment_index - 1].n++;
			}
		}

		// Store the new glyph at the position of s (arc length). Add an integer mask containing debug info. Currently only the glyph visibility is included with 1 indicating a hidden glyph.
		int debug_info = show ? 0 : 1;
		result.glyphs.append(s, *reinterpret_cast<float*>(&debug_info), attribute_values);
	}

	//store the size when this glyph is actually placed
	if(show) {
		prev_glyph_size = new_glyph_size;
		last_committed_s = s;
	}
}

// clamp value v to in range and remap to out range
float layer_compile_state::clamp_remap(float v, const cgv::vec2 in, const cgv::vec2 out) {
	if(cgv::math::is_zero(in.x() - in.y()))
		return out.x();
	v = cgv::math::clamp(v, in.x(), in.y());
	return cgv::math::map(v, in.x(), in.y(), out.x(), out.y());
}

void layer_compile_state::setup_trajectory() {
	if(has_trajectory()) {
		current_range = position_attribute.trajectories->at(trajectory_index);
		local_segment_count = current_range.n - 1;
	}
}

void layer_compile_state::increment_segment_index() {
	local_segment_index++;
	global_segment_index++;
}

segment_time<float> layer_compile_state::advance_to_segment_containing_time(float t) {
	auto segtime = segment_time_get(*position_attribute.attribute_handle, current_range, static_cast<unsigned>(local_segment_index));
	while(t >= segtime.t1 && local_segment_index < local_segment_count - 1) {
		increment_segment_index();
		segtime = segment_time_get(*position_attribute.attribute_handle, current_range, static_cast<unsigned>(local_segment_index));

		// handle overlap from previous segment
		if(result.ranges[global_segment_index - 1].n > 0) {
			// using half size of previous glyph
			bool overlaps = arc_lengths[global_segment_index][0] < result.glyphs.get_last_glyph_position() + 0.5f * prev_glyph_size;
			// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
			bool has_infinite_size = prev_glyph_size < 0.0f;
			if(overlaps || has_infinite_size) {
				result.ranges[global_segment_index].i0 = static_cast<int>(result.glyphs.get_glyph_count()) - 1;
				result.ranges[global_segment_index].n = 1;
			}
		}
	}
	return segtime;
}

bool layer_compile_state::can_place_glyph_at_time(segment_time<float> segtime, float t) const {
	return (local_segment_index == local_segment_count - 1 || t >= segtime.t0) && t <= segtime.t1;
}

std::tuple<float, float> layer_compile_state::get_segment_local_time_and_arc_length(segment_time<float> segtime, float t) const {
	float t_local = segtime.t01(t);
	float s = arclen::eval(arc_lengths[global_segment_index], t_local);
	return { t_local, s };
}
	
attribute_interpolation_state::attribute_interpolation_state(std::vector<attribute_trajectory_pair> attributes) {
	for(const auto& attribute : attributes)
		this->attributes.push_back({ attribute.attribute_handle, attribute.trajectories });
	values.resize(attributes.size(), 0.0f);
}

bool attribute_interpolation_state::initialize_indices(size_t trajectory_index, float t) {
	// initialize all attribute index pairs to point to the first two attributes
	// and gather the total count of attributes for this trajectory
	for(attribute_entry& attribute : attributes) {
		const auto& attribute_trajectory = attribute.trajectories->at(trajectory_index);

		if(attribute_trajectory.n < 2) // single-sample trajectory, assignment doesn't make sense here
			return false;

		attribute.index0 = attribute_trajectory.i0;
		attribute.index1 = attribute_trajectory.i0 + 1;

		// test if the first sample time point is before the first attribute sample for each attribute
		auto a = attribute.handle->signed_magnitude_at(attribute.piece_indices.x());
		if(t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
			attribute.index1 = attribute.index0;

		attribute.indices_end = attribute_trajectory.i0 + attribute_trajectory.n;
	}

	return true;
}

void attribute_interpolation_state::increment_to_time(float t) {
	for(attribute_entry& attribute : attributes) {
		unsigned last_index = attribute.indices_end - 1;
		// increment indices until the sample time point lies between the first and second attribute sample
		while(t > attribute.handle->signed_magnitude_at(attribute.index1).t && attribute.index0 < last_index) {
			attribute.index0 = attribute.index1;
			attribute.index1 = std::min(attribute.index0 + 1, last_index);
		}
	}
}

void attribute_interpolation_state::interpolate_values(float t) {
	// interpolate each mapped attribute value
	for(size_t i = 0; i < attributes.size(); ++i) {
		const attribute_entry& attribute = attributes[i];

		auto a0 = attribute.handle->signed_magnitude_at(attribute.index0);
		auto a1 = attribute.handle->signed_magnitude_at(attribute.index1);

		float length = a1.t - a0.t;

		float t_local = 0.0f;
		if(std::abs(length) > std::numeric_limits<float>::epsilon())
			t_local = (t - a0.t) / length;

		values[i] = cgv::math::lerp(a0.val, a1.val, t_local);
	}
}

bool attribute_interpolation_state::reached_end() const {
	// check whether the indices of all attributes have reached the end
	// TODO: This currently already fires when just one index reached the end. Shouldn't we wait for all?
	for(const attribute_entry& attribute : attributes) {
		if(attribute.index0 >= attribute.indices_end - 1)
			return true;
	}
	return false;
}

std::vector<layer_compile_result> glyph_compiler::compile_glyph_attributes(
	const traj_dataset<float>& data_set,
	const arclen::parametrization& parametrization,
	const glyph_layer_manager::configuration& layers_config
) const {
	std::vector<layer_compile_result> res;
	if(layers_config.layer_configs.empty())
		return res;

	attribute_trajectory_pair position_attribute;
	position_attribute.attribute_handle = &data_set.positions().attrib;
	position_attribute.trajectories = &data_set.trajectories(*position_attribute.attribute_handle);

	// Build seperate range and attribs buffers for each glyph layer.
	// Could be parallelized per layer but this only gives a very moderate speedup (and only if more than one layer is used).
	// Parallelization of the trajectory loop in each layer might be a better bet to improve performance but is non-trivial to implement.
//#pragma omp parallel for
//		for(int layer_idx = 0; layer_idx < layer_count; ++layer_idx) {...}

	for(const auto& layer_config : layers_config.layer_configs) {
		// ToDo: FixMe: Glyphs with no mapped attributes are currently not supported. This prevents placement of glyphs with constant parameters.
		// Trying this would currently result in a runtime error.
		if(!layer_config.mapped_attributes.empty())
			res.push_back(compile_glyph_layer(data_set, parametrization, data_set.get_attribute_names(), position_attribute, layer_config));
	}

	return res;
}

// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
layer_compile_result glyph_compiler::compile_glyphs_front_at_samples(
	attribute_trajectory_pair position_attribute,
	const std::vector<cgv::mat4>& arc_lengths,
	const glyph_layer_manager::configuration::layer_configuration& layer_config,
	const glyph_shape* shape,
	const std::vector<attribute_trajectory_pair>& mapped_attributes
) const {
	const size_t attribute_count = mapped_attributes.size();

	// stores an index for each attribute
	std::vector<unsigned> attribute_indices(attribute_count, 0);
	// stores one past the last valid index for each attribute
	std::vector<unsigned> max_attribute_indices(attribute_count);
	// stores if an attribute has a sample at the current location
	std::vector<bool> attribute_has_sample(attribute_count);
	// stores evaluated datapoints for later reuse
	std::vector<traj_attribute<float>::datapoint_mag> data_points(attribute_count);
	// stores the attribute values
	std::vector<float> attribute_values(attribute_count);
	
	layer_compile_state state(position_attribute, arc_lengths, shape, attribute_count);

	// - compile data
	while(state.has_trajectory()) {
		const range position_trajectory = state.get_trajectory();
			
		// reset the sample availability status
		std::fill(attribute_has_sample.begin(), attribute_has_sample.end(), false);

		// stores the minimum t over all current attribute sample points in each iteration
		float min_t = std::numeric_limits<float>::max();
		// stores the index of the attribute with the minimum t
		unsigned min_a_idx = 0;

		for(size_t i = 0; i < attribute_count; ++i) {
			const auto& attribute_trajectory = mapped_attributes[i].trajectories->at(state.get_trajectory_index());
			attribute_indices[i] = attribute_trajectory.i0;
			max_attribute_indices[i] = attribute_trajectory.i0 + attribute_trajectory.n;

			if(attribute_trajectory.n < 2) // single-sample trajectory, assignment doesn't make sense here
				state.skip_to_trajectory_end();
		}

		while(state.has_segment()) {
			min_t = std::numeric_limits<float>::max();

			for(size_t i = 0; i < attribute_count; ++i) {
				auto a = mapped_attributes[i].attribute_handle->signed_magnitude_at(attribute_indices[i]);
				data_points[i] = a;
				if(a.t < min_t) {
					min_a_idx = static_cast<unsigned>(i);
					min_t = a.t;
				}
			}

			auto segment = state.find_next_fitting_segment_at_time(min_t);
			if(segment.is_valid()) {
				for(size_t i = 0; i < attribute_count; ++i) {
					unsigned attrib_idx = attribute_indices[i];

					const auto& a_curr = data_points[i];
					float val = a_curr.val;

					bool found_sample = std::abs(min_t - a_curr.t) < attribute_timestamp_epsilon;
					attribute_has_sample[i] = found_sample;

					if(!found_sample && attrib_idx > 0) {
						// get interpolated value
						auto a_prev = mapped_attributes[i].attribute_handle->signed_magnitude_at(attribute_indices[i]);
						float t = cgv::math::normalize(min_t, a_prev.t, a_curr.t);
						val = cgv::math::lerp(a_prev.val, val, t);
					}

					attribute_values[i] = val;
				}

				state.place_glyph(layer_config, attribute_values, segment.s, length_scale, include_hidden_glyphs);
			} else {
				// If the attrib does not fall into the current segment something is out of order.
				// We just increment the index of the attribute with the minimal timestamp.
				attribute_has_sample[min_a_idx] = true;
			}

			// increment indices and check whether the indices of all attributes have reached the end
			for(size_t i = 0; i < attribute_count; ++i) {
				const unsigned max_attribute_index = max_attribute_indices[i];
				if(attribute_has_sample[i])
					attribute_indices[i] = std::min(max_attribute_index, ++attribute_indices[i]);
				if(attribute_indices[i] >= max_attribute_index)
					state.skip_to_trajectory_end();
			}
		}

		state.advance_to_next_trajectory();
	}

	return state.get_result();
}

// generate a glyph at uniformly spaced time steps by interpolating attributes
layer_compile_result glyph_compiler::compile_glyphs_front_uniform_time(
	attribute_trajectory_pair position_attribute,
	const std::vector<cgv::mat4>& arc_lengths,
	const glyph_layer_manager::configuration::layer_configuration& layer_config,
	const glyph_shape* shape,
	const std::vector<attribute_trajectory_pair>& mapped_attributes
) const {
	const size_t attribute_count = mapped_attributes.size();

	layer_compile_state state(position_attribute, arc_lengths, shape, attribute_count);
	attribute_interpolation_state interpolation(mapped_attributes);

	if(layer_config.sampling_step < sample_step_threshold) {
		std::cout << "sample step too low" << std::endl;
		state.skip_to_trajectory_end();
	}

	// - compile data
	while(state.has_trajectory()) {
		const range position_trajectory = state.get_trajectory();

		// stores the current t at which we want to sample the attributes
		float sample_t = 0.0f;
			
		if(!interpolation.initialize_indices(state.get_trajectory_index(), sample_t))
			state.skip_to_trajectory_end();

		while(state.has_segment()) {
			interpolation.increment_to_time(sample_t);

			auto segment = state.find_next_fitting_segment_at_time(sample_t);
			if(segment.is_valid()) {
				interpolation.interpolate_values(sample_t);
				state.place_glyph(layer_config, interpolation.get_attribute_values(), segment.s, length_scale, include_hidden_glyphs);
			}

			if(interpolation.reached_end())
				state.skip_to_trajectory_end();

			// increment the sample time point
			sample_t += layer_config.sampling_step;
		}

		state.advance_to_next_trajectory();
	}

	return state.get_result();
}

// generate a glyph at uniformly spaced time steps by interpolating attributes
layer_compile_result glyph_compiler::compile_glyphs_front_equidistant(
	attribute_trajectory_pair position_attribute,
	const arclen::parametrization& parametrization,
	const glyph_layer_manager::configuration::layer_configuration& layer_config,
	const glyph_shape* shape,
	const std::vector<attribute_trajectory_pair>& mapped_attributes
) const {
	const size_t attribute_count = mapped_attributes.size();

	layer_compile_state state(position_attribute, parametrization.t_to_s, shape, attribute_count);
	attribute_interpolation_state interpolation(mapped_attributes);
	
	if(layer_config.sampling_step < sample_step_threshold) {
		std::cout << "sample step too low" << std::endl;
		return state.get_result();
	}

	// - compile data
	while(state.has_trajectory()) {
		const range position_trajectory = state.get_trajectory();

		// stores the current t at which we want to sample the attributes
		float sample_t = 0.0f, sample_s = 0.0f;
			
		if(!interpolation.initialize_indices(state.get_trajectory_index(), sample_t))
			state.skip_to_trajectory_end();

		while(state.has_segment()) {
			interpolation.increment_to_time(sample_t);

			auto segment = state.find_next_fitting_segment_at_time(sample_t);
			if(segment.is_valid()) {
				interpolation.interpolate_values(sample_t);
				state.place_glyph(layer_config, interpolation.get_attribute_values(), segment.s, length_scale, include_hidden_glyphs);
			}

			if(interpolation.reached_end())
				state.skip_to_trajectory_end();

			// increment the sample point
			sample_s += layer_config.sampling_step;

			// update current sample_t
			size_t next_local_segment_index = state.get_local_segment_index(), next_global_segment_index = state.get_global_segment_index();
			// - find segment the next point is in
			while(next_local_segment_index < state.get_local_segment_count() && sample_s > parametrization.t_to_s[next_global_segment_index][15]) {
				next_local_segment_index++;
				next_global_segment_index++;
			}
			// - terminate immediately if next sample point is beyond trajectory bound
			if(next_local_segment_index >= state.get_local_segment_count())
				break;
			// - query arc lenght parametrization for segment t and offset to get actual global timestamp
			const float sample_t_local = arclen::map(parametrization.t_to_s[next_global_segment_index], parametrization.s_to_t[next_global_segment_index], sample_s);
			segment.segtime = segment_time_get(*position_attribute.attribute_handle, position_trajectory, static_cast<unsigned>(next_local_segment_index));
			sample_t = cgv::math::lerp(segment.segtime.t0, segment.segtime.t1, sample_t_local);
		}

		state.advance_to_next_trajectory();
	}

	return state.get_result();
}

layer_compile_result glyph_compiler::compile_glyph_layer(
	const traj_dataset<float>& data_set,
	const arclen::parametrization& parametrization,
	const std::vector<std::string>& attribute_names,
	attribute_trajectory_pair position_attribute,
	const glyph_layer_manager::configuration::layer_configuration& layer_config
) const {
	std::vector<attribute_trajectory_pair> mapped_attributes;

	for(size_t i = 0; i < layer_config.mapped_attributes.size(); ++i) {
		int attribute_index = layer_config.mapped_attributes[i];
		if(attribute_index < 0 || static_cast<size_t>(attribute_index) >= attribute_names.size()) {
			std::cout << "Error: glyph_compiler::compile_glyph_layer - attribute index out of range" << std::endl;
			continue;
		}

		attribute_trajectory_pair mapped_attribute;

		// from the docs: returns an explicitly invalid attribute interface that acts "empty" on all relevant queries
		// if no attribute of the given name exists in the dataset 
		mapped_attribute.attribute_handle = &data_set.attribute(attribute_names[attribute_index]);
			
		// from the docs: returns an explicitly invalid range that indicates zero samples in the trajectory if the dataset has
		// no trajectory information for the attribute
		mapped_attribute.trajectories = &data_set.trajectories(*mapped_attribute.attribute_handle);

		mapped_attributes.push_back(mapped_attribute);
	}

	switch(layer_config.sampling_strategy) {
	case AttributeSamplingStrategy::kUniformTime:
		return compile_glyphs_front_uniform_time(position_attribute, parametrization.t_to_s, layer_config, layer_config.shape_ptr, mapped_attributes);
	case AttributeSamplingStrategy::kEquidistant:
		return compile_glyphs_front_equidistant(position_attribute, parametrization, layer_config, layer_config.shape_ptr, mapped_attributes);
	case AttributeSamplingStrategy::kOriginalSamples:
		return compile_glyphs_front_at_samples(position_attribute, parametrization.t_to_s, layer_config, layer_config.shape_ptr, mapped_attributes);
	default:
		return layer_compile_result(0);
	}
}
