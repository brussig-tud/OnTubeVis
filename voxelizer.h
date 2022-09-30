#pragma once

#include <vector>

#include <cgv/render/render_types.h>
#include <cgv/render/context.h>
#include <cgv/render/shader_library.h>
#include <cgv_gl/gl/gl_context.h>

#include "voxel_grid.h"
#include "traj_loader.h"
#include "hermite_spline_tube.h"

class voxelizer : public cgv::render::render_types {
protected:
	cgv::render::shader_program clear_prog;
	cgv::render::shader_program voxelize_prog;
	cgv::render::shader_program clamp_prog;
	cgv::render::shader_program mipmap_prog;

	voxel_grid vg;

	std::vector<vec3> sample_position_offsets;
	float subsampling_normalization_factor = 1.0f;
	
	int sample_voxel(const ivec3& vidx, const quadratic_bezier_tube& qt);

	void voxelize_q_tube(const quadratic_bezier_tube& qt);

	void clamp_density();

	bool load_shader_programs(cgv::render::context& ctx);

	void generate_mipmap(cgv::render::context& ctx, GLuint texture_handle);

public:
	voxelizer() {}

	~voxelizer() {}

	bool init(cgv::render::context& ctx, size_t count);

	voxel_grid& ref_voxel_grid() { return vg; }

	void initialize_voxel_grid(const box3& bbox, unsigned request_resolution);

	void compute_density_volume(const traj_manager<float>::render_data *data_set, const float radius_scale);

	void compute_density_volume_gpu(cgv::render::context& ctx, const traj_manager<float>::render_data *data_set, const float radius_scale, GLuint index_buffer, GLuint data_buffer, cgv::render::texture& tex);
};
