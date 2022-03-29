#pragma once

// CGV framework core
#include <cgv/render/render_types.h>

// local includes
#include "arclen_helper.h"
#include "traj_loader.h"
#include "glyph_layer_manager.h"



class glyph_compiler : public cgv::render::render_types {
protected:
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

	// clamp value v to range [r.x,r.y] and remap to [r.z,r.w]
	float clamp_remap(float v, const vec4& r) {
		v = cgv::math::clamp(v, r.x(), r.y());
		float t = 0.0f;
		if(abs(r.x() - r.y()) > std::numeric_limits<float>::epsilon())
			t = (v - r.x()) / (r.y() - r.x());
		return cgv::math::lerp(r.z(), r.w(), t);
	}

	struct layer_compile_info {
		const glyph_shape* current_shape;
		std::vector<const traj_attribute<float>*> mapped_attribs;
		std::vector<const std::vector<range>*> attribs_trajs;
		size_t attrib_count;

		std::vector<irange> ranges;
		glyph_attributes attribs;

		layer_compile_info(const glyph_shape* shape_ptr) : current_shape(shape_ptr) {}
		void update_attribute_count() {
			attrib_count = mapped_attribs.size();
			attribs.count = attrib_count;
		}
	};

	// generate a glyph at every attribute sample location (interpolates attributes if more than one is mapped in this layer)
	void compile_glyphs_front_at_samples(const traj_attribute<float>& P, const std::vector<range>& tube_trajs, const std::vector<mat4>& arc_length, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
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
			float last_commited_s = 0.0f;

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

					// store the number of interpolated attributes for this glyph (debug only)
					unsigned num_interpolated = 0;
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
							++num_interpolated;
						}

						attrib_values[i] = val;
					}

					// setup parameters of potential glyph
					for(size_t i = 0; i < layer_config.glyph_mapping_parameters.size(); ++i) {
						const auto& triple = layer_config.glyph_mapping_parameters[i];
						if(triple.type == 0) {
							// constant attribute
							glyph_params[i] = (*triple.v)[3];
						} else {
							// mapped attribute
							const vec4& ranges = *(triple.v);
							// use windowing and remapping to get the value of the glyph parameter
							glyph_params[i] = clamp_remap(attrib_values[triple.idx], ranges);
						}
					}

					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_commited_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = (unsigned)attribs.glyph_count();
							cur_range.n = 1;

							// handle overlap to previous segment (this only works for a single previous segment)
							/*if(seg > 0 && alen[global_seg - 1][15] > s - 0.5f*new_glyph_size) {
								// if there have been no glyphs comitted to the previous segment until now, also update its start index
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
										// if there have been no glyphs comitted to the previous segment until now, also update its start index
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
						int debug_info = 0;

						debug_info |= num_interpolated;
						debug_info |= min_a_idx << 2;

						if(include_glyph)
							debug_info |= 0x1000;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_commited_s = s;
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
	void compile_glyphs_front_uniform_time(const traj_attribute<float>& P, const std::vector<range>& tube_trajs, const std::vector<mat4>& arc_length, const glyph_layer_manager::configuration::layer_configuration& layer_config, layer_compile_info& lci) {
		// convenience shorthands
		const size_t attrib_count = lci.attrib_count;
		const auto& mapped_attribs = lci.mapped_attribs;
		const auto& attribs_trajs = lci.attribs_trajs;
		auto& ranges = lci.ranges;
		auto& attribs = lci.attribs;
		const auto& alen = arc_length;

		// stores an index pair for each attribute
		std::vector<uvec2> attrib_indices(attrib_count, uvec2(0, 1));
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
			float last_commited_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0.0f;
			const float sample_step = layer_config.sampling_step;

			// initializa all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = uvec2(idx, idx + 1);
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
					uvec2& indices = attrib_indices[i];
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
						const uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					// setup parameters of potential glyph
					for(size_t i = 0; i < layer_config.glyph_mapping_parameters.size(); ++i) {
						const auto& triple = layer_config.glyph_mapping_parameters[i];
						if(triple.type == 0) {
							// constant attribute
							glyph_params[i] = (*triple.v)[3];
						} else {
							// mapped attribute
							const vec4& ranges = *(triple.v);
							// use windowing and remapping to get the value of the glyph parameter
							glyph_params[i] = clamp_remap(attrib_values[triple.idx], ranges);
						}
					}

					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_commited_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = (unsigned)attribs.glyph_count();
							cur_range.n = 1;

							// handle overlap to the previous segments
							if(seg > 0) {
								int prev_seg = static_cast<int>(global_seg - 1);
								int min_global_seg = static_cast<int>(traj_offset);
								float min_s = s - 0.5f*new_glyph_size;
								if(min_s >= 0.0f) {
									while(prev_seg >= min_global_seg && alen[prev_seg][15] > min_s) {
										// if there have been no glyphs comitted to the previous segment until now, also update its start index
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
						int debug_info = 0;

						if(include_glyph)
							debug_info |= 0x1000;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_commited_s = s;
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
		std::vector<uvec2> attrib_indices(attrib_count, uvec2(0, 1));
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
			float last_commited_s = 0.0f;

			// index for the current segment
			unsigned seg = 0;

			// stores the current t at which we want to sample the attributes
			float sample_t = 0, sample_s = 0;
			const float sample_step = layer_config.sampling_step;

			// initializa all attribute index pairs to point to the first two attributes
			// and gather the total count of attributes for this trajectory
			bool run = true;
			for(size_t i = 0; i < attrib_count; ++i) {
				const auto &traj_range = attribs_trajs[i]->at(trj);
				unsigned idx = traj_range.i0;
				attrib_indices[i] = uvec2(idx, idx + 1);
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
					uvec2& indices = attrib_indices[i];
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
						const uvec2& attrib_idx = attrib_indices[i];

						auto a0 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.x());
						auto a1 = mapped_attribs[i]->signed_magnitude_at(attrib_idx.y());

						float denom = a1.t - a0.t;

						float t = 0.0f;
						if(abs(denom) > std::numeric_limits<float>::epsilon())
							t = (sample_t - a0.t) / denom;

						attrib_values[i] = cgv::math::lerp(a0.val, a1.val, t);
					}

					// setup parameters of potential glyph
					for(size_t i = 0; i < layer_config.glyph_mapping_parameters.size(); ++i) {
						const auto& triple = layer_config.glyph_mapping_parameters[i];
						if(triple.type == 0) {
							// constant attribute
							glyph_params[i] = (*triple.v)[3];
						} else {
							// mapped attribute
							const vec4& ranges = *(triple.v);
							// use windowing and remapping to get the value of the glyph parameter
							glyph_params[i] = clamp_remap(attrib_values[triple.idx], ranges);
						}
					}

					float new_glyph_size = lci.current_shape->get_size(glyph_params);
					new_glyph_size /= length_scale;

					// infer potential glyph extents
					const float min_dist = attribs.size() > 0 ?
						std::max(new_glyph_size, prev_glyph_size) :
						new_glyph_size;

					bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_commited_s + min_dist;
					include_glyph |= min_dist < 0.0f;

					if(include_glyph || include_hidden_glyphs) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = (unsigned)attribs.glyph_count();
							cur_range.n = 1;

							// handle overlap to the previous segments
							if(seg > 0) {
								int prev_seg = static_cast<int>(global_seg - 1);
								int min_global_seg = static_cast<int>(traj_offset);
								float min_s = s - 0.5f*new_glyph_size;
								if(min_s >= 0.0f) {
									while(prev_seg >= min_global_seg && param.t_to_s[prev_seg][15] > min_s) {
										// if there have been no glyphs comitted to the previous segment until now, also update its start index
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
						int debug_info = 0;

						if(include_glyph)
							debug_info |= 0x1000;

						attribs.add(*reinterpret_cast<float*>(&debug_info));

						std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));
					}

					//store the size when this glyph is actually placed
					if(include_glyph) {
						prev_glyph_size = new_glyph_size;
						last_commited_s = s;
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
				// - terminate immediatly if next sample point is beyond trajectory bound
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

	void compile_glyph_layer(size_t layer_idx, const traj_dataset<float>& data_set, const arclen::parametrization &parametrization, const std::vector<std::string>& attrib_names, const glyph_layer_manager::configuration::layer_configuration& layer_config, const traj_attribute<float>& P, const std::vector<range>& tube_trajs) {

		const AttributeSamplingStrategy sampling_strategy = layer_config.sampling_strategy;
		layer_compile_info lci(layer_config.shape_ptr);

		for(size_t i = 0; i < layer_config.mapped_attributes.size(); ++i) {
			int attrib_idx = layer_config.mapped_attributes[i];
			if(attrib_idx < 0 || attrib_idx >= attrib_names.size()) {
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

		// reserve memory (might not make a huge or even noticeable difference)
		//lci.attribs.reserve(attribs.size() + ...);
		// reserve the maximum amount of possible segments; actual segment count may be less if some nodes are used multiple times
		lci.ranges.reserve(P.num() - tube_trajs.size());

		switch (sampling_strategy)
		{
			case ASS_UNIFORM:
				compile_glyphs_front_uniform_time(P, tube_trajs, parametrization.t_to_s, layer_config, lci);
				break;
			case ASS_EQUIDIST:
				compile_glyphs_front_equidistant(P, tube_trajs, parametrization, layer_config, lci);
				break;
			case ASS_AT_SAMPLES:
				compile_glyphs_front_at_samples(P, tube_trajs, parametrization.t_to_s, layer_config, lci);
			default:
				/* DoNothing() */;
		}

		layer_filled[layer_idx] = true;
		layer_ranges[layer_idx] = lci.ranges;
		layer_attribs[layer_idx] = lci.attribs;
	}

	void compile_glyph_attributes_impl(const traj_dataset<float> &data_set, const arclen::parametrization &parametrization, const glyph_layer_manager::configuration &layers_config) {
		const auto &P = data_set.positions().attrib;
		const auto &tube_trajs = data_set.trajectories(P);

		auto attrib_names = data_set.get_attribute_names();

		size_t layer_count = layers_config.layer_configs.size();

		layer_filled.resize(layer_count, false);
		layer_ranges.resize(layer_count, std::vector<irange>());
		layer_attribs.resize(layer_count, glyph_attributes());

		// build seperate range and attribs buffers for each glyph layer

		// could be parallelized here but only gives very moderate speedup (and only if more than one alyer is used)
		// parallelization of trajectory loop in each layer might be a better bet to improve performance but is non-trivial to implement
//#pragma omp parallel for
//		for(int layer_idx = 0; layer_idx < layer_count; ++layer_idx) {

		for(size_t layer_idx = 0; layer_idx < layer_count; ++layer_idx) {
			const auto& layer_config = layers_config.layer_configs[layer_idx];

			// skip this layer if it does not have any mapped attributes
			if(layer_config.mapped_attributes.size() == 0) {
				continue;
			}

			compile_glyph_layer(layer_idx, data_set, parametrization, attrib_names, layer_config, P, tube_trajs);
		}
	}

public:
	glyph_compiler() {}
	
	std::vector<bool> layer_filled;
	std::vector<std::vector<irange>> layer_ranges;
	std::vector<glyph_attributes> layer_attribs;

	bool include_hidden_glyphs;
	float length_scale;
	
	bool compile_glyph_attributes(const traj_dataset<float> &data_set, const arclen::parametrization &parametrization, const glyph_layer_manager::configuration &layers_config) {
		bool success = false;
		layer_filled.clear();
		layer_ranges.clear();
		layer_attribs.clear();
		if(layers_config.layer_configs.size() > 0) {
			compile_glyph_attributes_impl(data_set, parametrization, layers_config);
			success = true;
		}
		return success;
	}
};
