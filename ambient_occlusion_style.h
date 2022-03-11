#pragma once

#include <vector>

#include <cgv/render/render_types.h>

#include "voxel_grid.h"



struct ambient_occlusion_style : cgv::render::render_types {
	bool enable = false;
	float sample_offset = 0.04f;
	float sample_distance = 0.8f;
	float strength_scale = 1.0f;

	vec3 texture_offset = vec3(0.0f);
	vec3 texture_scaling = vec3(1.0f);
	vec3 texcoord_scaling = vec3(1.0f);
	float texel_size = 1.0f;

	float cone_angle = 50.0f;
	float angle_factor;
	std::vector<vec3> sample_directions;

	ambient_occlusion_style() {
		generate_sample_directions();
	}

	void derive_voxel_grid_parameters(const voxel_grid& vg) {
		const box3& volume_bbox = vg.bounds;
		const ivec3& volume_resolution = vg.resolution;

		unsigned max_extent_axis = cgv::math::max_index(volume_bbox.get_extent());

		texture_offset = volume_bbox.get_min_pnt();
		texture_scaling = vec3(1.0f) / volume_bbox.get_extent();
		texcoord_scaling = vec3((float)volume_resolution[max_extent_axis]) / vec3(volume_resolution);
		texel_size = 1.0f / volume_resolution[max_extent_axis];
	}

	void generate_sample_directions() {
		sample_directions.resize(3, vec3(0.0f, 1.0f, 0.0f));

		float alpha2 = cgv::math::deg2rad(cone_angle / 2.0f);
		float beta = cgv::math::deg2rad(90.0f - (cone_angle / 2.0f));

		float a = sinf(alpha2);
		float dh = tanf(cgv::math::deg2rad(30.0f)) * a;

		float c = length(vec2(a, dh));

		float b = sqrtf(1 - c * c);

		angle_factor = 2.0f * sinf(alpha2) / sinf(beta);
		sample_directions[0] = vec3(0.0f, b, c);
		sample_directions[1] = vec3(a, b, -dh);
		sample_directions[2] = vec3(-a, b, -dh);
	}
};

#include <cgv/gui/provider.h>

namespace cgv {
namespace gui {

/// define a gui creator for the ambient occlusion style struct
struct ambient_occlusion_style_gui_creator : public gui_creator {
	/// attempt to create a gui and return whether this was successful
	bool create(provider* p, const std::string& label, void* value_ptr, const std::string& value_type, const std::string& gui_type, const std::string& options, bool*) {
		if(value_type != cgv::type::info::type_name<ambient_occlusion_style>::get_name())
			return false;

		ambient_occlusion_style* s_ptr = reinterpret_cast<ambient_occlusion_style*>(value_ptr);
		cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);

		p->add_member_control(b, "Enable", s_ptr->enable, "check");
		p->add_member_control(b, "Sample Offset", s_ptr->sample_offset, "value_slider", "min=0.0;step=0.0001;max=0.2;log=true;ticks=true");
		p->add_member_control(b, "Sample Distance", s_ptr->sample_distance, "value_slider", "min=0.0;step=0.0001;max=1.0;log=true;ticks=true");
		p->add_member_control(b, "Strength Scale", s_ptr->strength_scale, "value_slider", "min=0.0;step=0.0001;max=100.0;log=true;ticks=true");

		connect_copy(
			p->add_member_control(b, "Cone Angle", s_ptr->cone_angle, "value_slider", "min=10.0;step=0.0001;max=90.0;ticks=true")->value_change,
			cgv::signal::rebind(s_ptr, &ambient_occlusion_style::generate_sample_directions));

		return true;
	}
};

#include <cgv_gl/gl/lib_begin.h>

cgv::gui::gui_creator_registration<ambient_occlusion_style_gui_creator> ambient_occlusion_s_gc_reg("ambient_occlusion_style_gui_creator");
}
}
