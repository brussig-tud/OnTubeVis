#include "voxelizer.h"

int voxelizer::sample_voxel(const ivec3& vidx, const quadratic_bezier_tube& qt) {
	vec3 voxel_min = vg.bounds.ref_min_pnt() + vec3(vidx) * vg.voxel_size;

	vec3 spos = voxel_min + 0.5f * vg.voxel_size;
	float dist = qt.signed_distance(spos);
	if(dist > vg.voxel_half_diag)
		return 0;

	int count = 0;

	for(unsigned k = 0; k < sample_position_offsets.size(); ++k) {
		vec3 spos = voxel_min + sample_position_offsets[k];
		float dist = qt.signed_distance(spos);

		if(dist <= 0.0f)
			++count;
	}

	return count;
}

void voxelizer::voxelize_q_tube(const quadratic_bezier_tube& qt) {
	box3 box = qt.bounding_box(true);

	ivec3 sidx((box.get_min_pnt() - vg.bounds.ref_min_pnt()) / vg.voxel_size);
	ivec3 eidx((box.get_max_pnt() - vg.bounds.ref_min_pnt()) / vg.voxel_size);

	const ivec3& res = vg.resolution;

	sidx = cgv::math::clamp(sidx, ivec3(0), res - 1);
	eidx = cgv::math::clamp(eidx, ivec3(0), res - 1);

	for(int z = sidx.z(); z <= eidx.z(); ++z) {
		for(int y = sidx.y(); y <= eidx.y(); ++y) {
			for(int x = sidx.x(); x <= eidx.x(); ++x) {
				int count = sample_voxel(ivec3(x, y, z), qt);
				float occupancy = subsampling_normalization_factor * static_cast<float>(count);

				int idx = x + res.x() * y + res.x() * res.y() * z;

#pragma omp atomic
				vg.data[idx] += occupancy;
			}
		}
	}
}

void voxelizer::clamp_density() {
	auto& data = vg.data;
	for(size_t i = 0; i < data.size(); ++i)
		data[i] = cgv::math::clamp(data[i], 0.0f, 1.0f);
}

void voxelizer::initialize_voxel_grid(const box3& bbox, unsigned request_resolution) {
	vg.compute_bounding_box(bbox, request_resolution);
	vg.resize_data();
}

void voxelizer::compute_density_volume(const traj_manager<float>::render_data *data_set) {
	if(!data_set) {
		std::cout << "Warning: compute_density_volume received nullptr for data_set." << std::endl;
		return;
	}

	const unsigned num_samples_per_dim = 3;
	const float step = vg.voxel_size / static_cast<float>(num_samples_per_dim);
	const vec3 offset(0.5f * step);

	sample_position_offsets.resize(num_samples_per_dim*num_samples_per_dim*num_samples_per_dim);

	unsigned idx = 0;
	for(unsigned z = 0; z < num_samples_per_dim; ++z) {
		for(unsigned y = 0; y < num_samples_per_dim; ++y) {
			for(unsigned x = 0; x < num_samples_per_dim; ++x) {
				sample_position_offsets[idx++] = offset + vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)) * step;
			}
		}
	}

	subsampling_normalization_factor = 1.0f / static_cast<float>(num_samples_per_dim);

	auto& positions = data_set->positions;
	auto& tangents = data_set->tangents;
	auto& radii = data_set->radii;
	auto& indices = data_set->indices;

#pragma omp parallel for
	for(int i = 0; i < indices.size(); i += 2) {
		unsigned idx_a = indices[i + 0];
		unsigned idx_b = indices[i + 1];

		vec3 p0 = positions[idx_a];
		vec3 p1 = positions[idx_b];
		float r0 = radii[idx_a];
		float r1 = radii[idx_b];
		vec4 t0 = tangents[idx_a];
		vec4 t1 = tangents[idx_b];

		hermite_spline_tube hst = hermite_spline_tube(p0, p1, r0, r1, vec3(t0), vec3(t1), t0.w(), t1.w());

		quadratic_bezier_tube qbt0 = hst.split_to_quadratic_bezier_tube(0);
		quadratic_bezier_tube qbt1 = hst.split_to_quadratic_bezier_tube(1);

		voxelize_q_tube(qbt0);
		voxelize_q_tube(qbt1);
	}

	clamp_density();
}
