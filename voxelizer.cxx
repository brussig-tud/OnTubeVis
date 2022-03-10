#include "voxelizer.h"
#include <cgv_gl/renderer.h>

bool voxelizer::load_shader_programs(cgv::render::context& ctx) {

	bool res = true;
	std::string where = "voxelizer::load_shader_programs()";

	/*shader_define_map distance_defines;
	shader_code::set_define(distance_defines, "ORDER", sort_order, SO_ASCENDING);
	shader_code::set_define(distance_defines, "INITIALIZE_VALUES", value_init_override, true);
	shader_code::set_define(distance_defines, "USE_AUXILIARY_BUFFER", auxiliary_type_def != "", false);
	shader_code::set_define(distance_defines, "VALUE_TYPE_DEFINITION", value_type_def, std::string("uint"));

	if(data_type_def != "")
		distance_defines["DATA_TYPE_DEFINITION"] = data_type_def;
	if(auxiliary_type_def != "")
		distance_defines["AUXILIARY_TYPE_DEFINITION"] = auxiliary_type_def;
	if(key_definition != "")
		distance_defines["KEY_DEFINITION"] = key_definition;*/

	res = res && cgv::glutil::shader_library::load(ctx, clear_prog, "clear", true, where);
	res = res && cgv::glutil::shader_library::load(ctx, voxelize_prog, "voxelize", true, where);
	res = res && cgv::glutil::shader_library::load(ctx, clamp_prog, "clamp", true, where);
	res = res && cgv::glutil::shader_library::load(ctx, mipmap_prog, "mipmap", true, where);

	return res;
}

bool voxelizer::init(cgv::render::context& ctx, size_t count) {

	if(!load_shader_programs(ctx))
		return false;

	return true;
}

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

void voxelizer::compute_density_volume_gpu(cgv::render::context& ctx, const traj_manager<float>::render_data *data_set, cgv::render::texture& tex) {

	if(!data_set) {
		std::cout << "Warning: compute_density_volume_gpu received nullptr for data_set." << std::endl;
		return;
	}

	if(!tex.handle) {
		std::cout << "Warning: compute_density_volume_gpu received invalid texture handle." << std::endl;
		return;
	}

	// uncomment for benchmarking
	/*GLuint time_query = 0;
	GLuint64 elapsed_time;
	glGenQueries(1, &time_query);

	glBeginQuery(GL_TIME_ELAPSED, time_query);*/

	const ivec3& res = vg.resolution;

	// reset the values to zero
	clear_prog.enable(ctx);
	clear_prog.set_uniform(ctx, "res", res);

	const int texture_handle = (const int&)tex.handle - 1;

	glBindImageTexture(0, texture_handle, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);

	glDispatchCompute((GLuint)ceil(res.x() / 4.0f), (GLuint)ceil(res.y() / 4.0f), (GLuint)ceil(res.z() / 4.0f));
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	clear_prog.disable(ctx);

	/*// now voxelize the scene
	const int primitive_count = static_cast<int>(data_set->indices.size() / 2);
	
	voxelize_prog.enable(ctx);
	voxelize_prog.set_uniform(ctx, "primitive_count", primitive_count);
	voxelize_prog.set_uniform(ctx, "vbox_min", vg.bounds.ref_min_pnt());
	voxelize_prog.set_uniform(ctx, "vsize", vg.voxel_size);
	voxelize_prog.set_uniform(ctx, "vrad", 0.5f * vg.voxel_size);
	voxelize_prog.set_uniform(ctx, "vhalfdiag", vdinfo.vhalfdiag);
	voxelize_prog.set_uniform(ctx, "voffset", vdinfo.vsize / 6.0f);
	voxelize_prog.set_uniform(ctx, "vstep", vdinfo.vsize / 3.0f);
	voxelize_prog.set_uniform(ctx, "vvol", vdinfo.vvol);
	voxelize_prog.set_uniform(ctx, "lower_radius_limit", vdinfo.radius_lower_limit);
	//voxelize_prog.set_uniform(ctx, "particle_radius_scale", radius_scale);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, particle_vbo);

	glDispatchCompute((GLuint)ceil(ds.positions.size() / 64.0f), 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	voxelize_prog.disable(ctx);

	// clamp the values to [0,1]
	clamp_prog.enable(ctx);
	clamp_prog.set_uniform(ctx, "particle_count", static_cast<int>(ds.positions.size()));
	clamp_prog.set_uniform(ctx, "resx", static_cast<int>(res.x));
	clamp_prog.set_uniform(ctx, "resy", static_cast<int>(res.y));
	clamp_prog.set_uniform(ctx, "resz", static_cast<int>(res.z));

	glDispatchCompute((GLuint)ceil(res.x / 4.0f), (GLuint)ceil(res.y / 4.0f), (GLuint)ceil(res.z / 4.0f));
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	clamp_prog.disable(ctx);*/

	glBindImageTexture(0, 0, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32F);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);

	// calculate the mipmaps via compute shader
	//generate_mipmap();

	// uncomment for standard mipmap calculation
	glBindTexture(GL_TEXTURE_3D, texture_handle);
	glGenerateMipmap(GL_TEXTURE_3D);
	glBindTexture(GL_TEXTURE_3D, 0);

	// uncomment for benchmarking
	/*glEndQuery(GL_TIME_ELAPSED);

	GLint done = false;
	while(!done) {
	glGetQueryObjectiv(time_query, GL_QUERY_RESULT_AVAILABLE, &done);
	}
	glGetQueryObjectui64v(time_query, GL_QUERY_RESULT, &elapsed_time);

	accumulated_time += elapsed_time / (1000000.0f);
	++runs;

	if(runs > 10) {
		std::cout << "done in " << accumulated_time/static_cast<float>(runs) << " ms" << std::endl;
	}*/
}
