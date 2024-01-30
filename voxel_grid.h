#pragma once

#include <vector>
#include <cgv/math/fvec.h>
#include <cgv/media/axis_aligned_box.h>


struct voxel_grid {
	using vec3 = cgv::vec3;
	using ivec3 = cgv::ivec3;
	using box3 = cgv::box3;

	float voxel_size;
	float voxel_half_diag;
	ivec3 resolution;
	box3 bounds;
	std::vector<float> data;

	void compute_bounding_box(const box3& bbox, unsigned request_resolution) {

		vec3 ext = bbox.get_extent();

		// calculate the cube voxel size and the resolution in each dimension
		unsigned max_extent_axis = cgv::math::max_index(ext);
		float max_ext = ext[max_extent_axis];
		voxel_size = max_ext / static_cast<float>(request_resolution);
		voxel_half_diag = 0.5f * sqrt(3.0f) * voxel_size;

		// calculate the number of voxels in each dimension
		int resx = static_cast<int>(ceilf(ext.x() / voxel_size));
		int resy = static_cast<int>(ceilf(ext.y() / voxel_size));
		int resz = static_cast<int>(ceilf(ext.z() / voxel_size));

		resolution = ivec3(resx, resy, resz);
		vec3 grid_ext = vec3(voxel_size) * resolution;
		vec3 grid_min = bbox.get_min_pnt() - 0.5f * (grid_ext - ext);

		bounds.ref_min_pnt() = grid_min;
		bounds.ref_max_pnt() = grid_min + grid_ext;
	}

	void resize_data() {
		data.clear();
		data.resize(resolution.x()*resolution.y()*resolution.z());
	}
};
