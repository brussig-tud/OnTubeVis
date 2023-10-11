#include "voxelizer.h"

bool voxelizer::load_shader_programs(cgv::render::context& ctx) {

	bool res = true;
	std::string where = "voxelizer::load_shader_programs()";

	res = res && cgv::render::shader_library::load(ctx, voxelize_prog, "voxelize", true, where);

	return res;
}

bool voxelizer::init(cgv::render::context& ctx, size_t count) {

	bool success = true;

	if(!load_shader_programs(ctx))
		success = false;

	clamp_kernel.set_texture_format("r32f");
	fill_kernel.set_value(vec4(0.0f));

	success &= clamp_kernel.init(ctx);
	success &= fill_kernel.init(ctx);
	success &= mipmap_kernel.init(ctx);

	return success;
}

void voxelizer::destruct(cgv::render::context& ctx) {

	voxelize_prog.destruct(ctx);

	clamp_kernel.destruct(ctx);
	fill_kernel.destruct(ctx);
	mipmap_kernel.destruct(ctx);
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
	box.ref_max_pnt() -= 0.005f * vg.voxel_size;

	ivec3 sidx((box.get_min_pnt() - vg.bounds.ref_min_pnt()) / vg.voxel_size);
	ivec3 eidx((box.get_max_pnt() - vg.bounds.ref_min_pnt()) / vg.voxel_size);

	const ivec3& res = vg.resolution;

	sidx = cgv::math::clamp(sidx, ivec3(0), res - 1);
	eidx = cgv::math::clamp(eidx, ivec3(0), res - 1);

	if((eidx.x() - sidx.x()) + (eidx.y() - sidx.y()) + (eidx.z() - sidx.z()) == 0) {
		// primitive lies entirely within the voxel
		int idx = sidx.x() + res.x() * sidx.y() + res.x() * res.y() * sidx.z();

		// approximate the volume of the quadratic bezier tube with two cylinders
		float r0 = 0.5f*(qt.a.rad + qt.b.rad);
		float r1 = 0.5f*(qt.b.rad + qt.c.rad);

		float h0 = length(qt.a.pos - qt.b.pos);
		float h1 = length(qt.b.pos - qt.c.pos);

		float v0 = h0 * float(M_PI)*r0*r0;
		float v1 = h1 * float(M_PI)*r1*r1;

#pragma omp atomic
		vg.data[idx] += (v0 + v1) / (vg.voxel_size*vg.voxel_size*vg.voxel_size);
	} else {
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

void voxelizer::compute_density_volume(const traj_manager<float>::render_data *data_set, const float radius_scale) {
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

	subsampling_normalization_factor = 1.0f / static_cast<float>(num_samples_per_dim*num_samples_per_dim*num_samples_per_dim);

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
		float r0 = radii[idx_a] * radius_scale;
		float r1 = radii[idx_b] * radius_scale;
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

void voxelizer::compute_density_volume_gpu(cgv::render::context& ctx, const traj_manager<float>::render_data *data_set, const float radius_scale, const cgv::render::vertex_buffer& index_buffer, const cgv::render::vertex_buffer& data_buffer, cgv::render::texture& tex) {

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

	fill_kernel.execute(ctx, tex);

	tex.bind_as_image(ctx, 0, 0, false, 0, cgv::render::AccessType::AT_READ_WRITE);

	// now voxelize the scene
	const int primitive_count = static_cast<int>(data_set->indices.size() / 2);
	
	voxelize_prog.enable(ctx);
	voxelize_prog.set_uniform(ctx, "primitive_count", primitive_count);
	voxelize_prog.set_uniform(ctx, "res", vg.resolution);
	voxelize_prog.set_uniform(ctx, "vbox_min", vg.bounds.ref_min_pnt());
	voxelize_prog.set_uniform(ctx, "vsize", vg.voxel_size);
	voxelize_prog.set_uniform(ctx, "vrad", 0.5f * vg.voxel_size);
	voxelize_prog.set_uniform(ctx, "vhalfdiag", vg.voxel_half_diag);
	voxelize_prog.set_uniform(ctx, "vstep", vg.voxel_size / 3.0f);
	voxelize_prog.set_uniform(ctx, "voffset", vg.voxel_size / 6.0f);
	voxelize_prog.set_uniform(ctx, "vvol", vg.voxel_size*vg.voxel_size*vg.voxel_size);
	voxelize_prog.set_uniform(ctx, "radius_scale", radius_scale);

	index_buffer.bind(ctx, cgv::render::VBT_STORAGE, 1);
	data_buffer.bind(ctx, cgv::render::VBT_STORAGE, 2);

	glDispatchCompute((GLuint)ceil(primitive_count / 64.0f), 1, 1);
	glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	voxelize_prog.disable(ctx);

	// clamp the values to [0,1]
	clamp_kernel.execute(ctx, tex);

	// calculate the mipmap via a compute shader
	mipmap_kernel.execute(ctx, tex);

	// uncomment for benchmarking
	/*glEndQuery(GL_TIME_ELAPSED);

	GLint done = false;
	while(!done) {
		glGetQueryObjectiv(time_query, GL_QUERY_RESULT_AVAILABLE, &done);
	}
	glGetQueryObjectui64v(time_query, GL_QUERY_RESULT, &elapsed_time);

	std::cout << "done in " << (elapsed_time / 1000000.0f) / 1000.0f << " s" << std::endl;

	accumulated_time += elapsed_time / (1000000.0f);
	++runs;

	if(runs > 10) {
		std::cout << "done in " << accumulated_time/static_cast<float>(runs) << " ms" << std::endl;
	}*/
}
