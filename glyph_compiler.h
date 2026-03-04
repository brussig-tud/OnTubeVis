#pragma once

// CGV framework core
#include <cgv/math/compare_float.h>
#include <cgv/math/functions.h>

// local includes
#include "arclen_helper.h"
#include "traj_loader.h"
#include "glyph_layer_manager.h"



class glyph_compiler {
public:
	// helper struct for range entries with start index i0 and count n
	struct irange { int i0, n; };

	// helper struct for glyph attributes
	struct glyph_attributes {
		size_t count = 0;
		std::vector<float> data;

		bool empty() const { return size() == 0; }

		size_t size() const { return data.size(); }

		size_t glyph_count() const {
			return size() / (2 + count);
		}

		void add(const float& x) {
			data.push_back(x);
		}

		float& operator [](int idx) {
			return data[idx];
		}

		float operator [](int idx) const {
			return data[idx];
		}

		float last_glyph_s() const {
			if(size() > 0)
				return data[size() - 1 - 1 - count];
			else
				return 0.0f;
		}
	};

	struct layer_compile_result {
		bool empty = true;
		std::vector<irange> ranges;
		glyph_attributes attribs;
	};

	struct result {
		bool success = false;
		std::vector<layer_compile_result> layers;
	};

	bool include_hidden_glyphs = false;
	float length_scale = 1.0f;

	result compile_glyph_attributes(const traj_dataset<float>& data_set, const arclen::parametrization& parametrization, const glyph_layer_manager::configuration& layers_config) {
		result res;
		if(layers_config.layer_configs.empty())
			return res;

		const auto& position_attribute = data_set.positions().attrib;
		const auto& position_trajectories = data_set.trajectories(position_attribute);

		auto attrib_names = data_set.get_attribute_names();

		size_t layer_count = layers_config.layer_configs.size();
		res.layers.resize(layer_count);

		// Build seperate range and attribs buffers for each glyph layer.
		// Could be parallelized per layer but this only gives a very moderate speedup (and only if more than one layer is used).
		// Parallelization of the trajectory loop in each layer might be a better bet to improve performance but is non-trivial to implement.
//#pragma omp parallel for
//		for(int layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
		for(size_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
			const auto& layer_config = layers_config.layer_configs[layer_idx];
			if(!layer_config.mapped_attributes.empty())
				res.layers[layer_idx] = compile_glyph_layer(layer_idx, data_set, parametrization, attrib_names, layer_config, position_attribute, position_trajectories);
		}

		res.success = true;
		return res;
	}

private:
	struct layer_compile_info {
		const glyph_shape* current_shape;
		std::vector<const traj_attribute<float>*> mapped_attribs;
		std::vector<const std::vector<range>*> attribs_trajs;
		size_t attrib_count = 0;

		std::vector<irange> ranges;
		glyph_attributes attribs;

		layer_compile_info(const glyph_shape* shape_ptr) : current_shape(shape_ptr) {}

		void update_attribute_count() {
			attrib_count = mapped_attribs.size();
			attribs.count = attrib_count;
		}
	};

	// clamp value v to in range and remap to out range
	float clamp_remap(float v, const cgv::vec2 in, const cgv::vec2 out) {
		if(cgv::math::is_zero(in.x() - in.y()))
			return out.x();
		v = cgv::math::clamp(v, in.x(), in.y());
		return cgv::math::map(v, in.x(), in.y(), out.x(), out.y());
	}

	void fill_glyph_params(const std::vector<glyph_layer_manager::configuration::glyph_mapping_triple>& mapping_parameters, const std::vector<float>& attrib_values, std::vector<float>& out_params) {
		for(size_t i = 0; i < mapping_parameters.size(); ++i) {
			const auto& triple = mapping_parameters[i];
			if(triple.type == 0) {
				// constant attribute
				out_params[i] = triple.v->output_range.y();
			} else {
				// use windowing and remapping to get the value of the glyph parameter
				out_params[i] = clamp_remap(attrib_values[triple.idx], triple.v->input_range, triple.v->output_range);
			}
		}
	}

	// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
	void compile_glyphs_front_at_samples(const traj_attribute<float>& P, const std::vector<range>& tube_trajs, const std::vector<cgv::mat4>& arc_length, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		// convenience shorthands
		const size_t attrib_count = lci.attrib_count;
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		auto& ranges = lci.ranges;
		auto& attribs = lci.attribs;
		const auto& alen = arc_length;

		// create an index for each attribute
		std::vector<unsigned> attrib_indices(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<traj_attribute<float>::datapoint_mag> data_points(attrib_count);
		std::vector<float> attrib_values(attrib_count);
		std::vector<bool> has_sample(attrib_count);
		std::vector<float> glyph_params(lci.current_shape->num_size_attribs());

		// - compile data
		unsigned traj_offset = 0;
		for(unsigned trj = 0; trj < (unsigned)tube_trajs.size(); trj++) {
			const auto &tube_traj = tube_trajs[trj];
			//const auto *alen = render.arclen_data.data();
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// reset the sample availability status
			std::fill(has_sample.begin(), has_sample.end(), false);

			// stores the minimum t over all current attribute sample points in each iteration
			float min_t;
			// stores the index of the attribute with the minimum t
			unsigned min_a_idx = 0;

			bool run = true;
			for(size_t i = 0; i < attrib_indices.size(); ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				attrib_indices[i] = traj_range.i0;
				if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
					run &= false;
			}
			run &= seg < num_segments;



			// following variable only needed for debugging
			unsigned glyph_idx = 0;



			while(run) {
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				min_t = std::numeric_limits<float>::max();

				for(size_t i = 0; i < attrib_count; ++i) {
					auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i]);
					data_points[i] = a;
					if(a.t < min_t) {
						min_a_idx = (unsigned)i;
						min_t = a.t;
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(P, tube_traj, seg);
				while(min_t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time_get(P, tube_traj, ++seg);

					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
							ranges[global_seg].n = 1;
						} else {
							if(alen[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
								ranges[global_seg].n = 1;
							}
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				//if((min_t >= segtime.t0 && min_t < segtime.t1)
				//	|| (seg == num_segments - 1 && min_t <= segtime.t1)) {
				if((seg == num_segments - 1 || min_t >= segtime.t0) && min_t <= segtime.t1) {
					// compute segment-relative t and arclength
					const float t_seg = (min_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(alen[global_seg], t_seg);

					for(size_t i = 0; i < attrib_count; ++i) {
						unsigned attrib_idx = attrib_indices[i];

						const auto& a_curr = data_points[i];
						float val = a_curr.val;

						// TODO: make epsilon adjustable
						bool found_sample = abs(min_t - a_curr.t) < 0.001f;
						has_sample[i] = found_sample;

						if(!found_sample && attrib_idx > 0) {
							// get interpolated value
							auto a_prev = mapped_attribs[i]->signed_magnitude_at(attrib_idx - 1);
							float t = (min_t - a_prev.t) / (a_curr.t - a_prev.t);
							val = cgv::math::lerp(a_prev.val, val, t);
						}

						attrib_values[i] = val;
					}

					// setup parameters of potential glyph
					fill_glyph_params(layer_config.glyph_mapping_parameters, attrib_values, glyph_params);
					
					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_committed_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = static_cast<int>(attribs.glyph_count());
							cur_range.n = 1;

							// handle overlap to previous segment (this only works for a single previous segment)
							/*if(seg > 0 && alen[global_seg - 1][15] > s - 0.5f*new_glyph_size) {
								// if there have been no glyphs committed to the previous segment until now, also update its start index
								if(ranges[global_seg - 1].n == 0)
									ranges[global_seg - 1].i0 = cur_range.i0;
								ranges[global_seg - 1].n++;
							}*/

							// handle overlap to the previous segments
							if(seg > 0) {
								int prev_seg = static_cast<int>(global_seg - 1);
								int min_global_seg = static_cast<int>(traj_offset);
								float min_s = s - 0.5f*new_glyph_size;
								if(min_s >= 0.0f) {
									while(prev_seg >= min_global_seg && alen[prev_seg][15] > min_s) {
										// if there have been no glyphs committed to the previous segment until now, also update its start index
										auto& prev_range = ranges[prev_seg];
										if(prev_range.n == 0)
											prev_range.i0 = cur_range.i0;
										prev_range.n++;
										prev_seg--;
									}
								}
							}
						} else {
							// one more free attribute that falls into this segment
							cur_range.n++;
							// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
							if(global_seg > 0 && new_glyph_size < 0.0) {
								// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
								ranges[global_seg - 1].n++;
							}
						}
						// store the new glyph
						attribs.add(s);
						int debug_info = include_glyph ? 0 : 1;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_committed_s = s;
					}

				} else {
					// If the attrib does not fall into the current segment something is out of order.
					// We just increment the attribute index with the minimal timestamp.
					has_sample[min_a_idx] = true;
				}

				// increment indices and check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attrib_count; ++i) {
					const auto &traj_range = attribs_trajs[i]->at(trj);
					const unsigned max_attrib_index = traj_range.i0 + traj_range.n;
					// only increment indices of attributes that have a sample at the current location (min_a.t)
					if(has_sample[i])
						attrib_indices[i] = std::min(max_attrib_index, ++attrib_indices[i]);
					if(attrib_indices[i] >= max_attrib_index)
						run &= false;
				}

				run &= seg < num_segments;
				++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}

		// fill the attribute buffer with one glyph entry if it is empty (will cause crash otherwise)
		if(attribs.empty()) {
			attribs.add(0.0f);
			attribs.add(0.0f);
			for(size_t i = 0; i < attrib_count; ++i)
				attribs.add(0.0f);
		}
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	void compile_glyphs_front_uniform_time(const traj_attribute<float>& P, const std::vector<range>& tube_trajs, const std::vector<cgv::mat4>& arc_length, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		// convenience shorthands
		const size_t attrib_count = lci.attrib_count;
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		auto& ranges = lci.ranges;
		auto& attribs = lci.attribs;
		const auto& alen = arc_length;

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attrib_indices(attrib_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attrib_index_counts(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<float> attrib_values(attrib_count);
		std::vector<float> glyph_params(lci.current_shape->num_size_attribs());

		// - compile data
		unsigned traj_offset = 0;
		for(unsigned trj = 0; trj < (unsigned)tube_trajs.size(); trj++) {
			const auto &tube_traj = tube_trajs[trj];
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0.0f;
			const float sample_step = layer_config.sampling_step;

			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = cgv::uvec2(idx, idx + 1);
				attrib_index_counts[i] = traj_range.i0 + traj_range.n;

				if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
					run &= false;
			}
			run &= seg < num_segments;

			// TODO: make this adapt to data set?
			if(sample_step < 0.005) {
				std::cout << "sample step too low" << std::endl;
				run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attrib_count; ++i) {
				auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attrib_indices[i].y() = attrib_indices[i].x();
			}



			// following variable only needed for debugging
			unsigned glyph_idx = 0;



			while(run) {
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				// iterate over each attribute
				for(size_t i = 0; i < attrib_count; ++i) {
					const auto& mapped_attrib = mapped_attribs[i];
					cgv::uvec2& indices = attrib_indices[i];
					const unsigned count = attrib_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(P, tube_traj, seg);
				while(sample_t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time_get(P, tube_traj, ++seg);

					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
							ranges[global_seg].n = 1;
						} else {
							if(alen[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
								ranges[global_seg].n = 1;
							}
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				if((seg == num_segments - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arclength
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(alen[global_seg], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attrib_count; ++i) {
						const cgv::uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					// setup parameters of potential glyph
					fill_glyph_params(layer_config.glyph_mapping_parameters, attrib_values, glyph_params);

					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_committed_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto& cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = static_cast<int>(attribs.glyph_count());
							cur_range.n = 1;

							// handle overlap to the previous segments
							if(seg > 0) {
								int prev_seg = static_cast<int>(global_seg - 1);
								int min_global_seg = static_cast<int>(traj_offset);
								float min_s = s - 0.5f * new_glyph_size;
								if(min_s >= 0.0f) {
									while(prev_seg >= min_global_seg && alen[prev_seg][15] > min_s) {
										// if there have been no glyphs committed to the previous segment until now, also update its start index
										auto& prev_range = ranges[prev_seg];
										if(prev_range.n == 0)
											prev_range.i0 = cur_range.i0;
										prev_range.n++;
										prev_seg--;
									}
								}
							}
						} else {
							// one more free attribute that falls into this segment
							cur_range.n++;
							// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
							if(global_seg > 0 && new_glyph_size < 0.0) {
								// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
								ranges[global_seg - 1].n++;
							}
						}
						// store the new glyph
						attribs.add(s);
						int debug_info = include_glyph ? 0 : 1;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_committed_s = s;
					}
				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attrib_count; ++i) {
					if(attrib_indices[i].x() >= attrib_index_counts[i] - 1)
						run &= false;
				}

				run &= seg < num_segments;
				//++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample time point
				sample_t += sample_step;
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	void compile_glyphs_front_equidistant(const traj_attribute<float> &P, const std::vector<range> &tube_trajs, const arclen::parametrization &param, const glyph_layer_manager::configuration::layer_configuration &layer_config, layer_compile_info &lci) {
		// convenience shorthands
		const size_t attrib_count = lci.attrib_count;
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		auto& ranges = lci.ranges;
		auto& attribs = lci.attribs;

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attrib_indices(attrib_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attrib_index_counts(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<float> attrib_values(attrib_count);
		std::vector<float> glyph_params(lci.current_shape->num_size_attribs());

		// - compile data
		unsigned traj_offset = 0;
		for(unsigned trj = 0; trj < (unsigned)tube_trajs.size(); trj++) {
			const auto &tube_traj = tube_trajs[trj];
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0, sample_s = 0;
			const float sample_step = layer_config.sampling_step;

			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = cgv::uvec2(idx, idx + 1);
				attrib_index_counts[i] = traj_range.i0 + traj_range.n;

				if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
					run &= false;
			}
			run &= seg < num_segments;

			// TODO: make this adapt to data set?
			if(sample_step < 0.005) {
				std::cout << "sample step too low" << std::endl;
				run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attrib_count; ++i) {
				auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attrib_indices[i].y() = attrib_indices[i].x();
			}



			// following variable only needed for debugging
			unsigned glyph_idx = 0;



			while(run) {
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				// iterate over each attribute
				for(size_t i = 0; i < attrib_count; ++i) {
					const auto& mapped_attrib = mapped_attribs[i];
					cgv::uvec2& indices = attrib_indices[i];
					const unsigned count = attrib_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(P, tube_traj, seg);
				while(sample_t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time_get(P, tube_traj, ++seg);

					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
							ranges[global_seg].n = 1;
						} else {
							if(param.t_to_s[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
								ranges[global_seg].n = 1;
							}
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				if((seg == num_segments - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arclength
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(param.t_to_s[global_seg], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attrib_count; ++i) {
						const cgv::uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					// setup parameters of potential glyph
					fill_glyph_params(layer_config.glyph_mapping_parameters, attrib_values, glyph_params);
					
					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_committed_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = static_cast<int>(attribs.glyph_count());
							cur_range.n = 1;

							// handle overlap to the previous segments
							if(seg > 0) {
								int prev_seg = static_cast<int>(global_seg - 1);
								int min_global_seg = static_cast<int>(traj_offset);
								float min_s = s - 0.5f*new_glyph_size;
								if(min_s >= 0.0f) {
									while(prev_seg >= min_global_seg && param.t_to_s[prev_seg][15] > min_s) {
										// if there have been no glyphs committed to the previous segment until now, also update its start index
										auto& prev_range = ranges[prev_seg];
										if(prev_range.n == 0)
											prev_range.i0 = cur_range.i0;
										prev_range.n++;
										prev_seg--;
									}
								}
							}
						} else {
							// one more free attribute that falls into this segment
							cur_range.n++;
							// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
							if(global_seg > 0 && new_glyph_size < 0.0) {
								// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
								ranges[global_seg - 1].n++;
							}
						}
						// store the new glyph
						attribs.add(s);
						int debug_info = include_glyph ? 0 : 1;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_committed_s = s;
					}

				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attrib_count; ++i) {
					if(attrib_indices[i].x() >= attrib_index_counts[i] - 1)
						run &= false;
				}

				run &= seg < num_segments;
				//++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample point
				sample_s += sample_step;

				// update current sample_t
				unsigned next_seg = seg, next_seg_global = global_seg;
				// - find segment the next point is in
				while (next_seg < num_segments && sample_s > param.t_to_s[next_seg_global][15])
				{ next_seg++; next_seg_global++; }
				// - terminate immediately if next sample point is beyond trajectory bound
				if (next_seg >= num_segments) break;
				// - query arclenght parametrization for segment t and offset to get actual global timestamp
				const float sample_t_local = arclen::map(param.t_to_s[next_seg_global], param.s_to_t[next_seg_global], sample_s);
				segtime = segment_time_get(P, tube_traj, next_seg);
				sample_t = segtime.t0 + sample_t_local*(segtime.t1-segtime.t0);
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}
	}

	layer_compile_result compile_glyph_layer(size_t layer_idx, const traj_dataset<float>& data_set, const arclen::parametrization &parametrization, const std::vector<std::string>& attrib_names, const glyph_layer_manager::configuration::layer_configuration& layer_config, const traj_attribute<float>& P, const std::vector<range>& tube_trajs) {

		const AttributeSamplingStrategy sampling_strategy = layer_config.sampling_strategy;
		layer_compile_info lci(layer_config.shape_ptr);

		for(size_t i = 0; i < layer_config.mapped_attributes.size(); ++i) {
			int attrib_idx = layer_config.mapped_attributes[i];
			if(attrib_idx < 0 || size_t(attrib_idx) >= attrib_names.size()) {
				std::cout << "Error: glyph_compiler::compile_glyph_layer - attribute index out of range" << std::endl;
				continue;
			}

			// from the docs: returns an explicitly invalid attribute interface that acts "empty" on all relevant queries
			// if no attribute of the given name exists in the dataset
			const traj_attribute<float>& attrib = data_set.attribute(attrib_names[attrib_idx]);
			lci.mapped_attribs.push_back(&attrib);

			// from the docs: returns an explicitly invalid range that indicates zero samples in the trajectory if the dataset has
			// no trajectory information for the attribute
			lci.attribs_trajs.push_back(&data_set.trajectories(attrib));
		}

		lci.update_attribute_count();

		// Todo: Reserve memory? Might not make a huge or even noticeable difference but try it anyway.
		//lci.attribs.reserve(attribs.size() + ...);
		// reserve the maximum amount of possible segments; actual segment count may be less if some nodes are used multiple times
		lci.ranges.reserve(P.num() - tube_trajs.size());

		switch (sampling_strategy)
		{
			case AttributeSamplingStrategy::kUniformTime:
				compile_glyphs_front_uniform_time(P, tube_trajs, parametrization.t_to_s, layer_config, lci);
				break;
			case AttributeSamplingStrategy::kEquidistant:
				compile_glyphs_front_equidistant(P, tube_trajs, parametrization, layer_config, lci);
				break;
			case AttributeSamplingStrategy::kOriginalSamples:
				compile_glyphs_front_at_samples(P, tube_trajs, parametrization.t_to_s, layer_config, lci);
				break;
			default:
				/* DoNothing() */;
				break;
		}

		layer_compile_result res;
		res.empty = false;
		res.ranges = std::move(lci.ranges);
		res.attribs = std::move(lci.attribs);
		return res;
	}
};
























































class glyph_compiler2 {
public:
	// helper struct for range entries with start index i0 and count n
	struct irange { int i0, n; };

	// helper struct for glyph attributes
	struct glyph_attributes {
		// For each glyph at least 2 attributes (arc length and debug information) are stored.
		static const size_t k_base_count = 2;
		// The number of glyph attributes.
		size_t count = 0;
		std::vector<float> data;

		bool empty() const { return data.empty(); }

		size_t size() const { return data.size(); }

		size_t glyph_count() const {
			return size() / (k_base_count + count);
		}

		void add(const float x) {
			data.push_back(x);
		}

		float& operator [](int idx) {
			return data[idx];
		}

		float operator [](int idx) const {
			return data[idx];
		}

		float last_glyph_s() const {
			if(size() > 0)
				return data[size() - (k_base_count + count)];
			else
				return 0.0f;
		}
	};

	struct layer_compile_result {
		std::vector<irange> ranges;
		glyph_attributes attribs;

		layer_compile_result(size_t mapped_attribute_count) {
			attribs.count = mapped_attribute_count;
		}

		bool empty() const {
			return ranges.empty() || attribs.empty();
		}
	};

	struct result {
		bool success = false;
		std::vector<layer_compile_result> layers;
	};

	bool include_hidden_glyphs = false;
	float length_scale = 1.0f;

	template<typename flt_type>
	struct attribute_trajectories_pair {
		const traj_attribute<float>& attribute;
		const std::vector<range>& trajectories;
	};

	struct global_compile_info {
		const traj_dataset<float>& data_set;
		const std::vector<std::string> attribute_names;
		const arclen::parametrization& parametrization;
		//const attribute_trajectories_pair<float>& position_attribute; // Todo: Find better name if we want to use this.
		const traj_attribute<float>& position_attribute;
		const std::vector<range>& position_trajectories;
	};

	result compile_glyph_attributes(const traj_dataset<float>& data_set, const arclen::parametrization& parametrization, const glyph_layer_manager::configuration& layers_config) {
		result res;
		if(layers_config.layer_configs.empty())
			return res;

		const auto& position_attribute = data_set.positions().attrib;
		const auto& position_trajectories = data_set.trajectories(position_attribute);

		//global_compile_info compile_info = { data_set, data_set.get_attribute_names(), parametrization, position_attribute, position_trajectories };

		// Build seperate range and attribs buffers for each glyph layer.
		// Could be parallelized per layer but this only gives a very moderate speedup (and only if more than one layer is used).
		// Parallelization of the trajectory loop in each layer might be a better bet to improve performance but is non-trivial to implement.
//#pragma omp parallel for
//		for(int layer_idx = 0; layer_idx < layer_count; ++layer_idx) {...}

		for(const auto& layer_config : layers_config.layer_configs) {
			// ToDo: FixMe: Glyphs with no mapped attributes are currently not supported. This prevents placement of glyphs with constant parameters.
			// Trying this would currently result in a runtime error.
			if(!layer_config.mapped_attributes.empty())
				res.layers.push_back(compile_glyph_layer(data_set, parametrization, data_set.get_attribute_names(), position_attribute, position_trajectories, layer_config));
		}

		res.success = true;
		return res;
	}

	bool use_variant2 = false;

private:
	struct layer_compile_info {
		const glyph_shape* shape = nullptr;
		std::vector<const traj_attribute<float>*> mapped_attribs;
		std::vector<const std::vector<range>*> attribs_trajs;

		layer_compile_info(const glyph_shape* shape) : shape(shape) {}
	};

	// clamp value v to in range and remap to out range
	static float clamp_remap(float v, const cgv::vec2 in, const cgv::vec2 out) {
		if(cgv::math::is_zero(in.x() - in.y()))
			return out.x();
		v = cgv::math::clamp(v, in.x(), in.y());
		return cgv::math::map(v, in.x(), in.y(), out.x(), out.y());
	}

	void fill_glyph_params(const std::vector<glyph_layer_manager::configuration::glyph_mapping_triple>& mapping_parameters, const std::vector<float>& attrib_values, std::vector<float>& out_params) {
		for(size_t i = 0; i < mapping_parameters.size(); ++i) {
			const auto& triple = mapping_parameters[i];
			if(triple.type == 0) {
				// constant attribute
				out_params[i] = triple.v->output_range.y();
			} else {
				// use windowing and remapping to get the value of the glyph parameter
				out_params[i] = clamp_remap(attrib_values[triple.idx], triple.v->input_range, triple.v->output_range);
			}
		}
	}











	void place_glyph(
		const glyph_layer_manager::configuration::layer_configuration& layer_config,
		const layer_compile_info& lci,
		const std::vector<cgv::mat4>& arc_lengths,
		const std::vector<float>& attrib_values,
		unsigned global_segment_offset,
		unsigned segment_index,
		unsigned global_segment_index,
		unsigned global_attribute_offset,
		float s,
		float& prev_glyph_size,
		float& last_committed_s,
		std::vector<float>& glyph_params,
		layer_compile_result& result
	) {
		// setup parameters of potential glyph
		fill_glyph_params(layer_config.glyph_mapping_parameters, attrib_values, glyph_params);

		float new_glyph_size = lci.shape->get_size(glyph_params);
		new_glyph_size /= length_scale;

		// infer potential glyph extents
		const float min_dist = result.attribs.empty() ?
			new_glyph_size :
			std::max(new_glyph_size, prev_glyph_size);

		bool include_glyph = result.attribs.glyph_count() == global_attribute_offset || s >= last_committed_s + min_dist;
		include_glyph |= min_dist < 0.0f;

		if(include_glyph || include_hidden_glyphs) {
			auto& cur_range = result.ranges[global_segment_index];
			if(cur_range.n < 1) {
				// first free attribute that falls into this segment
				cur_range.i0 = static_cast<int>(result.attribs.glyph_count());
				cur_range.n = 1;

				// handle overlap to previous segment (this only works for a single previous segment)
				/*if(seg > 0 && arc_lengths[global_seg - 1][15] > s - 0.5f*new_glyph_size) {
					// if there have been no glyphs committed to the previous segment until now, also update its start index
					if(ranges[global_seg - 1].n == 0)
						ranges[global_seg - 1].i0 = cur_range.i0;
					ranges[global_seg - 1].n++;
				}*/

				// handle overlap to the previous segments
				if(segment_index > 0) {
					int prev_seg = static_cast<int>(global_segment_index - 1);
					int min_global_seg = static_cast<int>(global_segment_offset);
					float min_s = s - 0.5f * new_glyph_size;
					if(min_s >= 0.0f) {
						while(prev_seg >= min_global_seg && arc_lengths[prev_seg][15] > min_s) {
						//while(prev_seg >= min_global_seg && param.t_to_s[prev_seg][15] > min_s) {
							// if there have been no glyphs committed to the previous segment until now, also update its start index
							auto& prev_range = result.ranges[prev_seg];
							if(prev_range.n == 0)
								prev_range.i0 = cur_range.i0;
							prev_range.n++;
							prev_seg--;
						}
					}
				}
			} else {
				// one more free attribute that falls into this segment
				cur_range.n++;
				// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
				if(global_segment_index > 0 && new_glyph_size < 0.0f) {
					// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
					result.ranges[global_segment_index - 1].n++;
				}
			}

			// store the new glyph
			result.attribs.add(s);

			int debug_info = include_glyph ? 0 : 1;
			result.attribs.add(*reinterpret_cast<float*>(&debug_info));

			std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(result.attribs.data));
		}

		// Todo: maybe put this outside of this function (but how to differentiate between visible and hidden glyphs?)
		//store the size when this glyph is actually placed
		if(include_glyph) {
			prev_glyph_size = new_glyph_size;
			last_committed_s = s;
		}
	}





	class compile_state {
	public:
		compile_state(
			const traj_attribute<float>& position_attribute,
			const std::vector<range>& position_trajectories,
			const std::vector<cgv::mat4>& arc_lengths,
			const glyph_shape* shape,
			size_t mapped_attribute_count) :
			position_attribute(position_attribute),
			position_trajectories(position_trajectories),
			arc_lengths(arc_lengths),
			shape(shape),
			result(mapped_attribute_count) {

			// Todo: Explain why
			glyph_params.resize(shape->num_size_attribs(), 0.0f);

			// Resize the result to hold one range per segment.
			size_t total_segment_count = position_attribute.num() - position_trajectories.size();
			result.ranges.resize(total_segment_count);

			// Setup first trajectory.
			setup_trajectory();
		}

		bool has_trajectory() const {
			return trajectory_index < position_trajectories.size();
		}

		range get_trajectory() const {
			return current_range;
		}

		void advance_trajectory() {
			// Update global from local state variables.
			trajectory_index++;
			global_attribute_offset += result.attribs.glyph_count();
			global_segment_offset += local_segment_count;
			global_segment_index = global_segment_offset;// +local_segment_index;

			// Reset local state variables.
			current_range = {};
			local_segment_count = 0;
			local_segment_index = 0;
			prev_glyph_size = 0.0f;
			last_committed_s = 0.0f;
			prev_glyph_size = 0.0f;
			last_committed_s = 0.0f;
			local_glyph_index = 0;
			run = false;

			setup_trajectory();
		}

		segment_time<float> advance_segment(float to_t) {
			// Todo: Can we get rid of the duplicate call to segment_time_get?
			auto segtime = segment_time_get(position_attribute, current_range, local_segment_index);
			while(to_t >= segtime.t1) {
				if(local_segment_index >= local_segment_count - 1)
					break;

				increment_segment_index();
				segtime = segment_time_get(position_attribute, current_range, local_segment_index);

				// handle overlap from previous segment
				if(result.ranges[global_segment_index - 1].n > 0) {
					// using half size of previous glyph
					bool overlaps = arc_lengths[global_segment_index][0] < result.attribs.last_glyph_s() + 0.5f * prev_glyph_size;
					// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
					bool has_infinite_size = prev_glyph_size < 0.0f;
					if(overlaps ||has_infinite_size) {
						result.ranges[global_segment_index].i0 = static_cast<int>(result.attribs.glyph_count()) - 1;
						result.ranges[global_segment_index].n = 1;
					}
				}
			}
			return segtime;
		}

		void place_glyph(
			const glyph_layer_manager::configuration::layer_configuration& layer_config,
			const std::vector<float>& attrib_values,
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
					glyph_params[parameter_index] = clamp_remap(attrib_values[mapping_triple.idx], mapping_triple.v->input_range, mapping_triple.v->output_range);
				}
				++parameter_index;
			}


			float new_glyph_size = shape->get_size(glyph_params);
			new_glyph_size /= length_scale;

			// infer potential glyph extents
			const float min_dist = result.attribs.empty() ?
				new_glyph_size :
				std::max(new_glyph_size, prev_glyph_size);

			bool show = result.attribs.glyph_count() == global_attribute_offset || s >= last_committed_s + min_dist || min_dist < 0.0f;

			if(show || show_hidden) {
				auto& cur_range = result.ranges[global_segment_index];
				if(cur_range.n < 1) {
					// first free attribute that falls into this segment
					cur_range.i0 = static_cast<int>(result.attribs.glyph_count());
					cur_range.n = 1;

					// handle overlap to previous segment (this only works for a single previous segment)
					/*if(seg > 0 && arc_lengths[global_seg - 1][15] > s - 0.5f*new_glyph_size) {
						// if there have been no glyphs committed to the previous segment until now, also update its start index
						if(ranges[global_seg - 1].n == 0)
							ranges[global_seg - 1].i0 = cur_range.i0;
						ranges[global_seg - 1].n++;
					}*/

					// handle overlap to the previous segments
					if(local_segment_index > 0) {
						// Todo: Get rid of casts?
						int prev_seg = static_cast<int>(global_segment_index - 1);
						int min_global_seg = static_cast<int>(global_segment_offset);
						float min_s = s - 0.5f * new_glyph_size;
						if(min_s >= 0.0f) {
							while(prev_seg >= min_global_seg && arc_lengths[prev_seg][15] > min_s) {
								//while(prev_seg >= min_global_seg && param.t_to_s[prev_seg][15] > min_s) {
									// if there have been no glyphs committed to the previous segment until now, also update its start index
								auto& prev_range = result.ranges[prev_seg];
								if(prev_range.n == 0)
									prev_range.i0 = cur_range.i0;
								prev_range.n++;
								prev_seg--;
							}
						}
					}
				} else {
					// one more free attribute that falls into this segment
					cur_range.n++;
					// for infinitely sized "glyphs" there always will have been overlap from the previous segment, so the above branch won't have been executed
					if(global_segment_index > 0 && new_glyph_size < 0.0f) {
						// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the previous segment
						result.ranges[global_segment_index - 1].n++;
					}
				}

				// store the new glyph
				result.attribs.add(s);

				// Add an integer mask containing debug info. Currently only the glyph visibility is included with 1 indicating a hidden glyph.
				int debug_info = show ? 0 : 1;
				result.attribs.add(*reinterpret_cast<float*>(&debug_info));

				std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(result.attribs.data));
			}

			//store the size when this glyph is actually placed
			if(show) {
				prev_glyph_size = new_glyph_size;
				last_committed_s = s;
			}
		}

		//size_t get_trajectory_index() const {
		//	return trajectory_index;
		//}

		// Todo: Make private and add getters after debugging.
	public:
		const traj_attribute<float>& position_attribute;
		const std::vector<range>& position_trajectories;
		const std::vector<cgv::mat4>& arc_lengths;
		const glyph_shape* shape = nullptr;
		size_t trajectory_index = 0;
		
		bool run = false;
		range current_range;
		size_t global_attribute_offset = 0;
		size_t global_segment_offset = 0;
		size_t global_segment_index = 0;
		size_t local_segment_count = 0;
		size_t local_segment_index = 0;
		float prev_glyph_size = 0.0f;
		float last_committed_s = 0.0f;
		// the index of the currently processed glyph of one trajectory (hidden or visible); only needed for debugging
		size_t local_glyph_index = 0;

		// scratch buffer to hold mapped glyph parameters that are needed to compute its size
		std::vector<float> glyph_params;

		layer_compile_result result;

		void setup_trajectory() {
			if(has_trajectory()) {
				current_range = position_trajectories[trajectory_index];
				local_segment_count = current_range.n - 1;
				run = local_segment_count > 0;
			}
		}

		void increment_segment_index() {
			local_segment_index++;
			global_segment_index++;// = global_segment_offset + local_segment_index;
		}
	};

	// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
	layer_compile_result compile_glyphs_front_at_samples(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const std::vector<cgv::mat4>& arc_lengths, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		const size_t attribute_count = lci.mapped_attribs.size();
		
		layer_compile_result result(attribute_count);

		// create an index for each attribute
		std::vector<unsigned> attrib_indices(attribute_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<traj_attribute<float>::datapoint_mag> data_points(attribute_count);
		std::vector<float> attrib_values(attribute_count);
		std::vector<bool> has_sample(attribute_count);
		std::vector<float> glyph_params(lci.shape->num_size_attribs());

		// reserve the maximum amount of possible segments; actual segment count may be less if some nodes are used multiple times
		size_t total_segment_count = position_attribute.num() - position_trajectories.size();
		result.ranges.reserve(total_segment_count);

		// - compile data
		unsigned global_segment_offset = 0;
		for(size_t trajectory_index = 0; trajectory_index < position_trajectories.size(); trajectory_index++) {
			const range position_trajectory = position_trajectories[trajectory_index];
			const unsigned segment_count = position_trajectory.n - 1;
			const unsigned global_attribute_offset = static_cast<unsigned>(result.attribs.glyph_count());

			// make sure there is exactly one 'range' entry per segment
			result.ranges.resize(global_segment_offset + segment_count); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// the index of the current segment relative to the current trajectory
			unsigned segment_index = 0;

			// reset the sample availability status
			std::fill(has_sample.begin(), has_sample.end(), false);

			// stores the minimum t over all current attribute sample points in each iteration
			float min_t = std::numeric_limits<float>::max();
			// stores the index of the attribute with the minimum t
			unsigned min_a_idx = 0;

			bool run = segment_count > 0;

			if(run) {
				for(size_t i = 0; i < attrib_indices.size(); ++i) {
					const auto& traj_range = lci.attribs_trajs[i]->at(trajectory_index);
					attrib_indices[i] = traj_range.i0;
					if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
						run = false;
				}
			}

			// following variable only needed for debugging
			unsigned glyph_idx = 0;

			while(run) {
				// the index of the current segment relative to all trajectories
				unsigned global_segment_index = global_segment_offset + segment_index;

				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				min_t = std::numeric_limits<float>::max();

				for(size_t i = 0; i < attribute_count; ++i) {
					auto a = lci.mapped_attribs[i]->signed_magnitude_at(attrib_indices[i]);
					data_points[i] = a;
					if(a.t < min_t) {
						min_a_idx = (unsigned)i;
						min_t = a.t;
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(position_attribute, position_trajectory, segment_index);
				while(min_t >= segtime.t1) {
					if(segment_index >= segment_count - 1)
						break;
					// advance to next segment
					segment_index++;
					global_segment_index = global_segment_offset + segment_index;

					segtime = segment_time_get(position_attribute, position_trajectory, segment_index);
					
					// handle overlap from previous segment
					if(result.ranges[global_segment_index - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							result.ranges[global_segment_index].i0 = static_cast<int>(result.attribs.glyph_count()) - 1;
							result.ranges[global_segment_index].n = 1;
						} else {
							if(arc_lengths[global_segment_index][0] < result.attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								result.ranges[global_segment_index].i0 = static_cast<int>(result.attribs.glyph_count()) - 1;
								result.ranges[global_segment_index].n = 1;
							}
						}
					}
				}

				// commit the attribute if it falls into the current segment
				if((segment_index == segment_count - 1 || min_t >= segtime.t0) && min_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (min_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(arc_lengths[global_segment_index], t_seg);

					for(size_t i = 0; i < attribute_count; ++i) {
						unsigned attrib_idx = attrib_indices[i];

						const auto& a_curr = data_points[i];
						float val = a_curr.val;

						// TODO: make epsilon adjustable
						bool found_sample = abs(min_t - a_curr.t) < 0.001f;
						has_sample[i] = found_sample;

						if(!found_sample && attrib_idx > 0) {
							// get interpolated value
							auto a_prev = lci.mapped_attribs[i]->signed_magnitude_at(attrib_idx - 1);
							float t = (min_t - a_prev.t) / (a_curr.t - a_prev.t);
							val = cgv::math::lerp(a_prev.val, val, t);
						}

						attrib_values[i] = val;
					}

					place_glyph(layer_config, lci, arc_lengths, attrib_values, global_segment_offset, segment_index, global_segment_index, global_attribute_offset, s, prev_glyph_size, last_committed_s, glyph_params, result);
				} else {
					// If the attrib does not fall into the current segment something is out of order.
					// We just increment the attribute index with the minimal timestamp.
					has_sample[min_a_idx] = true;
				}

				// increment indices and check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attribute_count; ++i) {
					const auto &traj_range = lci.attribs_trajs[i]->at(trajectory_index);
					const unsigned max_attrib_index = traj_range.i0 + traj_range.n;
					// only increment indices of attributes that have a sample at the current location (min_a.t)
					if(has_sample[i])
						attrib_indices[i] = std::min(max_attrib_index, ++attrib_indices[i]);
					if(attrib_indices[i] >= max_attrib_index)
						run &= false;
				}

				run &= segment_index < segment_count;
				++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;
			}

			// update auxiliary indices
			global_segment_offset += segment_count;
		}

		// fill the attribute buffer with one glyph entry if it is empty (will cause crash otherwise)
		if(result.attribs.empty())
			result.attribs.data.insert(result.attribs.data.end(), result.attribs.k_base_count + result.attribs.count, 0.0f);

		return result;
	}

	// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
	layer_compile_result compile_glyphs_front_at_samples2(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const std::vector<cgv::mat4>& arc_lengths, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		const size_t attribute_count = lci.mapped_attribs.size();

		// stores an index for each attribute
		std::vector<unsigned> attribute_indices(attribute_count, 0);
		// stores TODO: comment
		std::vector<traj_attribute<float>::datapoint_mag> data_points(attribute_count);
		// stores the attribute values
		std::vector<float> attribute_values(attribute_count);
		// stores if an attribute has a sample at the current location
		std::vector<bool> attribute_has_sample(attribute_count);
		
		compile_state state(position_attribute, position_trajectories, arc_lengths, lci.shape, attribute_count);

		// - compile data
		while(state.has_trajectory()) {
			const range position_trajectory = state.get_trajectory();

			// reset the sample availability status
			std::fill(attribute_has_sample.begin(), attribute_has_sample.end(), false);

			// stores the minimum t over all current attribute sample points in each iteration
			float min_t = std::numeric_limits<float>::max();
			// stores the index of the attribute with the minimum t
			unsigned min_a_idx = 0;

			for(size_t i = 0; i < attribute_indices.size(); ++i) {
				const auto& attribute_trajectory = lci.attribs_trajs[i]->at(state.trajectory_index);
				attribute_indices[i] = attribute_trajectory.i0;

				if(attribute_trajectory.n < 2) // single-sample trajectory, assignment doesn't make sense here
					state.run = false;
			}

			while(state.run) {
				min_t = std::numeric_limits<float>::max();

				for(size_t i = 0; i < attribute_count; ++i) {
					auto a = lci.mapped_attribs[i]->signed_magnitude_at(attribute_indices[i]);
					data_points[i] = a;
					if(a.t < min_t) {
						min_a_idx = (unsigned)i;
						min_t = a.t;
					}
				}

				auto segtime = state.advance_segment(min_t);

				// commit the attribute if it falls into the current segment
				if((state.local_segment_index == state.local_segment_count - 1 || min_t >= segtime.t0) && min_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (min_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(arc_lengths[state.global_segment_index], t_seg);

					for(size_t i = 0; i < attribute_count; ++i) {
						unsigned attrib_idx = attribute_indices[i];

						const auto& a_curr = data_points[i];
						float val = a_curr.val;

						// TODO: make epsilon adjustable
						bool found_sample = abs(min_t - a_curr.t) < 0.001f;
						attribute_has_sample[i] = found_sample;

						if(!found_sample && attrib_idx > 0) {
							// get interpolated value
							auto a_prev = lci.mapped_attribs[i]->signed_magnitude_at(attrib_idx - 1);
							float t = (min_t - a_prev.t) / (a_curr.t - a_prev.t);
							val = cgv::math::lerp(a_prev.val, val, t);
						}

						attribute_values[i] = val;
					}

					state.place_glyph(layer_config, attribute_values, s, length_scale, include_hidden_glyphs);
				} else {
					// If the attrib does not fall into the current segment something is out of order.
					// We just increment the attribute index with the minimal timestamp.
					attribute_has_sample[min_a_idx] = true;
				}

				// increment indices and check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attribute_count; ++i) {
					const auto& attribute_trajectory = lci.attribs_trajs[i]->at(state.trajectory_index);
					const unsigned max_attrib_index = attribute_trajectory.i0 + attribute_trajectory.n;
					// only increment indices of attributes that have a sample at the current location (min_a.t)
					if(attribute_has_sample[i])
						attribute_indices[i] = std::min(max_attrib_index, ++attribute_indices[i]);
					if(attribute_indices[i] >= max_attrib_index)
						state.run = false;
				}

				state.run &= state.local_segment_index < state.local_segment_count;
				++state.local_glyph_index;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;
			}

			state.advance_trajectory();
		}

		// Todo: Is this really needed? If the attrib buffer is empty then the ranges should be as well.
		// fill the attribute buffer with one glyph entry if it is empty (will cause crash otherwise)
		if(state.result.attribs.empty())
			state.result.attribs.data.insert(state.result.attribs.data.end(), state.result.attribs.k_base_count + state.result.attribs.count, 0.0f);

		return state.result;
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_uniform_time(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const std::vector<cgv::mat4>& arc_length, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		// convenience shorthands
		const size_t attrib_count = lci.mapped_attribs.size();
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		const auto& alen = arc_length;

		layer_compile_result result(attrib_count);
		auto& ranges = result.ranges;
		auto& attribs = result.attribs;

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attrib_indices(attrib_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attrib_index_counts(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<float> attrib_values(attrib_count);
		std::vector<float> glyph_params(lci.shape->num_size_attribs());

		// - compile data
		unsigned traj_offset = 0;
		for(size_t trj = 0; trj < position_trajectories.size(); trj++) {
			const auto &tube_traj = position_trajectories[trj];
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0.0f;
			const float sample_step = layer_config.sampling_step;

			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = cgv::uvec2(idx, idx + 1);
				attrib_index_counts[i] = traj_range.i0 + traj_range.n;

				if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
					run &= false;
			}
			run &= seg < num_segments;

			// TODO: make this adapt to data set?
			if(sample_step < 0.005) {
				std::cout << "sample step too low" << std::endl;
				run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attrib_count; ++i) {
				auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attrib_indices[i].y() = attrib_indices[i].x();
			}



			// following variable only needed for debugging
			unsigned glyph_idx = 0;



			while(run) {
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				// iterate over each attribute
				for(size_t i = 0; i < attrib_count; ++i) {
					const auto& mapped_attrib = mapped_attribs[i];
					cgv::uvec2& indices = attrib_indices[i];
					const unsigned count = attrib_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(position_attribute, tube_traj, seg);
				while(sample_t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time_get(position_attribute, tube_traj, ++seg);

					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
							ranges[global_seg].n = 1;
						} else {
							if(alen[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
								ranges[global_seg].n = 1;
							}
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				if((seg == num_segments - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(alen[global_seg], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attrib_count; ++i) {
						const cgv::uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}


					place_glyph(layer_config, lci, alen, attrib_values, traj_offset, seg, global_seg, attribs_traj_offset, s, prev_glyph_size, last_committed_s, glyph_params, result);
				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attrib_count; ++i) {
					if(attrib_indices[i].x() >= attrib_index_counts[i] - 1)
						run &= false;
				}

				run &= seg < num_segments;
				//++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample time point
				sample_t += sample_step;
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}

		return result;
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_uniform_time2(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const std::vector<cgv::mat4>& arc_lengths, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		const size_t attribute_count = lci.mapped_attribs.size();

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attribute_indices(attribute_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attribute_index_counts(attribute_count, 0);
		// stores the attribute values
		std::vector<float> attribute_values(attribute_count);
		
		compile_state state(position_attribute, position_trajectories, arc_lengths, lci.shape, attribute_count);
		const float sample_step = layer_config.sampling_step;

		// - compile data
		while(state.has_trajectory()) {
			const range position_trajectory = state.get_trajectory();

			// stores the current t at which we want to sample the attributes
			float sample_t = 0.0f;
			
			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			for(size_t i = 0; i < attribute_count; ++i) {
				const auto& attribute_trajectory = lci.attribs_trajs[i]->at(state.trajectory_index);
				unsigned idx = attribute_trajectory.i0;
				attribute_indices[i] = cgv::uvec2(idx, idx + 1);
				attribute_index_counts[i] = attribute_trajectory.i0 + attribute_trajectory.n;

				if(attribute_trajectory.n < 2) // single-sample trajectory, assignment doesn't make sense here
					state.run = false;
			}

			// TODO: make this adapt to data set?
			if(sample_step < 0.005f) {
				std::cout << "sample step too low" << std::endl;
				state.run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attribute_count; ++i) {
				auto a = lci.mapped_attribs[i]->signed_magnitude_at(attribute_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attribute_indices[i].y() = attribute_indices[i].x();
			}

			while(state.run) {
				// iterate over each attribute
				for(size_t i = 0; i < attribute_count; ++i) {
					const auto& mapped_attrib = lci.mapped_attribs[i];
					cgv::uvec2& indices = attribute_indices[i];
					const unsigned count = attribute_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				auto segtime = state.advance_segment(sample_t);
				
				// commit the attribute if it falls into the current segment
				if((state.local_segment_index == state.local_segment_count - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(arc_lengths[state.global_segment_index], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attribute_count; ++i) {
						const cgv::uvec2& attrib_idx = attribute_indices[i];

						auto a0 = lci.mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = lci.mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attribute_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					state.place_glyph(layer_config, attribute_values, s, length_scale, include_hidden_glyphs);
				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attribute_count; ++i) {
					if(attribute_indices[i].x() >= attribute_index_counts[i] - 1)
						state.run = false;
				}

				state.run &= state.local_segment_index < state.local_segment_count;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample time point
				sample_t += sample_step;
			}

			state.advance_trajectory();
		}

		return state.result;
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_equidistant(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const arclen::parametrization &parametrization, const glyph_layer_manager::configuration::layer_configuration &layer_config, layer_compile_info &lci) {
		// convenience shorthands
		const size_t attrib_count = lci.mapped_attribs.size();
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		
		layer_compile_result result(attrib_count);
		auto& ranges = result.ranges;
		auto& attribs = result.attribs;

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attrib_indices(attrib_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attrib_index_counts(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<float> attrib_values(attrib_count);
		std::vector<float> glyph_params(lci.shape->num_size_attribs());

		// - compile data
		unsigned traj_offset = 0;
		for(size_t trj = 0; trj < position_trajectories.size(); trj++) {
			const auto &tube_traj = position_trajectories[trj];
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			float prev_glyph_size = 0.0f;
			float last_committed_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0, sample_s = 0;
			const float sample_step = layer_config.sampling_step;

			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = cgv::uvec2(idx, idx + 1);
				attrib_index_counts[i] = traj_range.i0 + traj_range.n;

				if(traj_range.n < 2) // single-sample trajectory, assignment doesn't make sense here
					run &= false;
			}
			run &= seg < num_segments;

			// TODO: make this adapt to data set?
			if(sample_step < 0.005) {
				std::cout << "sample step too low" << std::endl;
				run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attrib_count; ++i) {
				auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attrib_indices[i].y() = attrib_indices[i].x();
			}



			// following variable only needed for debugging
			unsigned glyph_idx = 0;



			while(run) {
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

				// iterate over each attribute
				for(size_t i = 0; i < attrib_count; ++i) {
					const auto& mapped_attrib = mapped_attribs[i];
					cgv::uvec2& indices = attrib_indices[i];
					const unsigned count = attrib_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				// advance segment pointer
				auto segtime = segment_time_get(position_attribute, tube_traj, seg);
				while(sample_t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time_get(position_attribute, tube_traj, ++seg);

					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						// using half size of previous glyph
						if(prev_glyph_size < 0.0f) {
							// "glyphs" with a negative size value are possibly infinite in size and always overlap onto the next segment
							ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
							ranges[global_seg].n = 1;
						} else {
							if(parametrization.t_to_s[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
								ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
								ranges[global_seg].n = 1;
							}
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				if((seg == num_segments - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(parametrization.t_to_s[global_seg], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attrib_count; ++i) {
						const cgv::uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					place_glyph(layer_config, lci, parametrization.t_to_s, attrib_values, traj_offset, seg, global_seg, attribs_traj_offset, s, prev_glyph_size, last_committed_s, glyph_params, result);
				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attrib_count; ++i) {
					if(attrib_indices[i].x() >= attrib_index_counts[i] - 1)
						run &= false;
				}

				run &= seg < num_segments;
				//++glyph_idx;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample point
				sample_s += sample_step;

				// update current sample_t
				unsigned next_seg = seg, next_seg_global = global_seg;
				// - find segment the next point is in
				while (next_seg < num_segments && sample_s > parametrization.t_to_s[next_seg_global][15])
				{ next_seg++; next_seg_global++; }
				// - terminate immediately if next sample point is beyond trajectory bound
				if (next_seg >= num_segments) break;
				// - query arc lenght parametrization for segment t and offset to get actual global timestamp
				const float sample_t_local = arclen::map(parametrization.t_to_s[next_seg_global], parametrization.s_to_t[next_seg_global], sample_s);
				segtime = segment_time_get(position_attribute, tube_traj, next_seg);
				sample_t = segtime.t0 + sample_t_local*(segtime.t1-segtime.t0);
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}

		return result;
	}

	// generate a glyph at uniformly spaced time steps by interpolating attributes
	layer_compile_result compile_glyphs_front_equidistant2(const traj_attribute<float>& position_attribute, const std::vector<range>& position_trajectories, const arclen::parametrization& parametrization, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		const size_t attribute_count = lci.mapped_attribs.size();

		// stores an index pair for each attribute
		std::vector<cgv::uvec2> attribute_indices(attribute_count, cgv::uvec2(0, 1));
		// stores the count of each attribute
		std::vector<unsigned> attribute_index_counts(attribute_count, 0);
		// stores the attribute values
		std::vector<float> attribute_values(attribute_count);

		compile_state state(position_attribute, position_trajectories, parametrization.t_to_s, lci.shape, attribute_count);
		const float sample_step = layer_config.sampling_step;

		// - compile data
		while(state.has_trajectory()) {
			const range position_trajectory = state.get_trajectory();

			// stores the current t at which we want to sample the attributes
			float sample_t = 0.0f, sample_s = 0.0f;
			const float sample_step = layer_config.sampling_step;

			// initialize all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			for(size_t i = 0; i < attribute_count; ++i) {
				const auto& attribute_trajectory = lci.attribs_trajs[i]->at(state.trajectory_index);
				unsigned idx = attribute_trajectory.i0;
				attribute_indices[i] = cgv::uvec2(idx, idx + 1);
				attribute_index_counts[i] = attribute_trajectory.i0 + attribute_trajectory.n;

				if(attribute_trajectory.n < 2) // single-sample trajectory, assignment doesn't make sense here
					state.run = false;
			}

			// TODO: make this adapt to data set?
			if(sample_step < 0.005f) {
				std::cout << "sample step too low" << std::endl;
				state.run = false;
			}

			// test if the first sample time point is before the first attribute sample for each attribute
			for(size_t i = 0; i < attribute_count; ++i) {
				auto a = lci.mapped_attribs[i]->signed_magnitude_at(attribute_indices[i].x());
				if(sample_t < a.t) // if yes, set the indices of this attribute to both point to the very first sample
					attribute_indices[i].y() = attribute_indices[i].x();
			}

			while(state.run) {
				// iterate over each attribute
				for(size_t i = 0; i < attribute_count; ++i) {
					const auto& mapped_attrib = lci.mapped_attribs[i];
					cgv::uvec2& indices = attribute_indices[i];
					const unsigned count = attribute_index_counts[i];

					// increment indices until the sample time point lies between the first and second attribute sample
					while(
						sample_t > mapped_attrib->signed_magnitude_at(indices.y()).t &&
						indices.x() < count - 1
						) {
						indices.x() = indices.y();
						indices.y() = std::min(indices.x() + 1, count - 1);
					}
				}

				auto segtime = state.advance_segment(sample_t);

				// commit the attribute if it falls into the current segment
				if((state.local_segment_index == state.local_segment_count - 1 || sample_t >= segtime.t0) && sample_t <= segtime.t1) {
					// compute segment-relative t and arc length
					const float t_seg = (sample_t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(parametrization.t_to_s[state.global_segment_index], t_seg);

					// interpolate each mapped attribute value
					for(size_t i = 0; i < attribute_count; ++i) {
						const cgv::uvec2& attribute_idx = attribute_indices[i];

						auto a0 = lci.mapped_attribs[i]->signed_magnitude_at(attribute_idx.x());
						auto a1 = lci.mapped_attribs[i]->signed_magnitude_at(attribute_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attribute_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					state.place_glyph(layer_config, attribute_values, s, length_scale, include_hidden_glyphs);
				}

				// check whether the indices of all attributes have reached the end
				for(size_t i = 0; i < attribute_count; ++i) {
					if(attribute_indices[i].x() >= attribute_index_counts[i] - 1)
						state.run = false;
				}

				state.run &= state.local_segment_index < state.local_segment_count;
				//if(glyph_idx >= max_glyph_count)
				//	run = false;

				// increment the sample point
				sample_s += sample_step;

				// update current sample_t
				size_t next_local_segment_index = state.local_segment_index, next_global_segment_index = state.global_segment_index;
				// - find segment the next point is in
				while(next_local_segment_index < state.local_segment_count && sample_s > parametrization.t_to_s[next_global_segment_index][15]) {
					next_local_segment_index++;
					next_global_segment_index++;
				}
				// - terminate immediately if next sample point is beyond trajectory bound
				if(next_local_segment_index >= state.local_segment_count)
					break;
				// - query arclenght parametrization for segment t and offset to get actual global timestamp
				const float sample_t_local = arclen::map(parametrization.t_to_s[next_global_segment_index], parametrization.s_to_t[next_global_segment_index], sample_s);
				segtime = segment_time_get(position_attribute, position_trajectory, next_local_segment_index);
				sample_t = cgv::math::lerp(segtime.t0, segtime.t1, sample_t_local);
			}

			state.advance_trajectory();
		}

		return state.result;
	}

	layer_compile_result compile_glyph_layer(
		const traj_dataset<float>& data_set,
		const arclen::parametrization& parametrization,
		const std::vector<std::string>& attribute_names,
		const traj_attribute<float>& position_attribute,
		const std::vector<range>& position_trajectories,
		const glyph_layer_manager::configuration::layer_configuration& layer_config)
	{
		const AttributeSamplingStrategy sampling_strategy = layer_config.sampling_strategy;
		layer_compile_info lci(layer_config.shape_ptr);

		for(size_t i = 0; i < layer_config.mapped_attributes.size(); ++i) {
			int attrib_idx = layer_config.mapped_attributes[i];
			if(attrib_idx < 0 || size_t(attrib_idx) >= attribute_names.size()) {
				std::cout << "Error: glyph_compiler::compile_glyph_layer - attribute index out of range" << std::endl;
				continue;
			}

			// Todo: Array of structs for mapped_attribs and attribs_trajs?

			// from the docs: returns an explicitly invalid attribute interface that acts "empty" on all relevant queries
			// if no attribute of the given name exists in the dataset
			const traj_attribute<float>& attrib = data_set.attribute(attribute_names[attrib_idx]);
			lci.mapped_attribs.push_back(&attrib);

			// from the docs: returns an explicitly invalid range that indicates zero samples in the trajectory if the dataset has
			// no trajectory information for the attribute
			lci.attribs_trajs.push_back(&data_set.trajectories(attrib));
		}

		// Todo: Last glyph on last trajectory is not placed in version 2 compared to version 1. Why?
		if(use_variant2) {
			switch(sampling_strategy) {
			case AttributeSamplingStrategy::kUniformTime:
				return compile_glyphs_front_uniform_time2(position_attribute, position_trajectories, parametrization.t_to_s, layer_config, lci);
			case AttributeSamplingStrategy::kEquidistant:
				return compile_glyphs_front_equidistant2(position_attribute, position_trajectories, parametrization, layer_config, lci);
			case AttributeSamplingStrategy::kOriginalSamples:
				return compile_glyphs_front_at_samples2(position_attribute, position_trajectories, parametrization.t_to_s, layer_config, lci);
			default:
				return layer_compile_result(0);
			}
		} else {
			switch(sampling_strategy) {
			case AttributeSamplingStrategy::kUniformTime:
				return compile_glyphs_front_uniform_time(position_attribute, position_trajectories, parametrization.t_to_s, layer_config, lci);
			case AttributeSamplingStrategy::kEquidistant:
				return compile_glyphs_front_equidistant(position_attribute, position_trajectories, parametrization, layer_config, lci);
			case AttributeSamplingStrategy::kOriginalSamples:
				return compile_glyphs_front_at_samples(position_attribute, position_trajectories, parametrization.t_to_s, layer_config, lci);
			default:
				return layer_compile_result(0);
			}
		}
	}
};
