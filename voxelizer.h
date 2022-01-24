#pragma once

#include <vector>

#include <cgv/render/render_types.h>

#include "voxel_grid.h"
#include "traj_loader.h"
#include "hermite_spline_tube.h"



class voxelizer : public cgv::render::render_types {
protected:
	voxel_grid vg;

	std::vector<vec3> sample_position_offsets;
	float subsampling_normalization_factor = 1.0f;
	
	int sample_voxel(const ivec3& vidx, const quadratic_bezier_tube& qt);

	void voxelize_q_tube(const quadratic_bezier_tube& qt);

	void clamp_density();

public:
	voxelizer() {}

	~voxelizer() {}

	voxel_grid& ref_voxel_grid() { return vg; }

	void initialize_voxel_grid(const box3& bbox, unsigned request_resolution);

	void compute_density_volume(const traj_manager<float>::render_data *data_set);
};
