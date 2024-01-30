#pragma once

#include <vector>

#include <cgv/render/context.h>
#include <cgv/render/shader_library.h>
#include <cgv/render/vertex_buffer.h>
#include <cgv_gl/gl/gl_context.h>
#include <cgv_gpgpu/clamp_texture.h>
#include <cgv_gpgpu/fill_texture.h>
#include <cgv_gpgpu/mipmap.h>

#include "voxel_grid.h"
#include "traj_loader.h"
#include "hermite_spline_tube.h"

class voxelizer {
public:
	using vec3 = cgv::vec3;
	using vec4 = cgv::vec4;
	using ivec3 = cgv::ivec3;
	using box3 = cgv::box3;

protected:
	cgv::render::shader_program voxelize_prog;
	
	cgv::gpgpu::clamp_texture clamp_kernel;
	cgv::gpgpu::fill_texture fill_kernel;
	cgv::gpgpu::mipmap mipmap_kernel;

	voxel_grid vg;

	std::vector<vec3> sample_position_offsets;
	float subsampling_normalization_factor = 1.0f;
	
	int sample_voxel(const ivec3& vidx, const quadratic_bezier_tube& qt);

	void voxelize_q_tube(const quadratic_bezier_tube& qt);

	void clamp_density();

	bool load_shader_programs(cgv::render::context& ctx);

public:
	voxelizer() {}

	~voxelizer() {}

	bool init(cgv::render::context& ctx, size_t count);

	void destruct(cgv::render::context& ctx);

	voxel_grid& ref_voxel_grid() { return vg; }

	void initialize_voxel_grid(const box3& bbox, unsigned request_resolution);

	void compute_density_volume(const traj_manager<float>::render_data *data_set, const float radius_scale);

	void compute_density_volume_gpu(cgv::render::context& ctx, const traj_manager<float>::render_data *data_set, const float radius_scale, const cgv::render::vertex_buffer& index_buffer, const cgv::render::vertex_buffer& data_buffer, cgv::render::texture& tex);
};
