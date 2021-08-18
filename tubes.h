#pragma once

// C++ STL
#include <vector>
#include <set>

// CGV framework core
#include <cgv/base/node.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>

// CGV OpenGL lib
#include <cgv_gl/volume_renderer.h>

// CGV framework graphics utility
#include <cgv_glutil/frame_buffer_container.h>
#include <cgv_glutil/radix_sort_4way.h>
#include <cgv_glutil/shader_library.h>
#include <cgv_glutil/box_render_data.h>
#include <cgv_glutil/sphere_render_data.h>
#include <cgv_glutil/transfer_function_editor.h>

// local includes
#include "hermite_spline_tube.h"
#include "traj_loader.h"
#include "textured_spline_tube_renderer.h"

using namespace cgv::render;



#include <cgv/gui/mouse_event.h>

class application_plugin :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::provider,
	public cgv::gui::event_handler
{
protected:
	std::vector<cgv::glutil::overlay*> overlays;
	cgv::glutil::overlay* blocking_overlay_ptr;

public:
	application_plugin(const std::string& name) : node(name) {

		blocking_overlay_ptr = nullptr;
	}

	template<class T>
	T* register_overlay() {
		static_assert(std::is_base_of<cgv::glutil::overlay, T>::value, "T must inherit from overlay");
		T* ptr = new T();
		cgv::base::register_object(base_ptr(ptr));
		overlays.push_back(ptr);
		return ptr;
	}

	bool handle(cgv::gui::event& e) {

		if(e.get_kind() == cgv::gui::EID_MOUSE) {
			cgv::gui::mouse_event& me = (cgv::gui::mouse_event&) e;
			cgv::gui::MouseAction ma = me.get_action();

			if(ma == cgv::gui::MA_PRESS) {
				ivec2 mpos(me.get_x(), me.get_y());

				for(auto overlay_ptr : overlays) {
					if(overlay_ptr->is_hit(mpos)) {
						blocking_overlay_ptr = overlay_ptr;
						break;
					}
				}
			}

			bool was_handled = false;

			if(blocking_overlay_ptr) {
				blocking_overlay_ptr->handle(e);
				was_handled = true;
			}

			if(ma == cgv::gui::MA_RELEASE)
				blocking_overlay_ptr = nullptr;

			if(was_handled)
				return true;
			else
				return handle_all(e);
		}
	}
	
	virtual bool handle_all(cgv::gui::event& e) = 0;
};





////
// Plugin definition

/// baseline visualization plugin for arbitrary trajectory data as tubes using the framework tube renderers and
/// trajectory loading facilities
class tubes :
	//public cgv::base::node,             // derive from node to integrate into global tree structure and to store a name
	public cgv::base::argument_handler, // derive from argument handler to be able to process custom arguments
	//public cgv::gui::provider,          // derive from provider to obtain a GUI tab
	//public cgv::gui::event_handler,     // derive from event handler to be able to directly react to user interaction
	//public cgv::render::drawable        // derive from drawable for being able to render
	public application_plugin
{
public:
	/// real number type
	//typedef float real;

	struct voxel_grid {
		float voxel_size;
		ivec3 resolution;
		box3 bounds;
		std::vector<float> data;

		void compute_bounding_box(const box3& bbox, unsigned request_resolution) {

			vec3 ext = bbox.get_extent();

			// calculate the cube voxel size and the resolution in each dimension
			int max_ext_axis = cgv::math::max_index(ext);
			float max_ext = ext[max_ext_axis];
			voxel_size = max_ext / static_cast<float>(request_resolution);

			// calculate the number of voxels in each dimension
			unsigned resx = static_cast<unsigned>(ceilf(ext.x() / voxel_size));
			unsigned resy = static_cast<unsigned>(ceilf(ext.y() / voxel_size));
			unsigned resz = static_cast<unsigned>(ceilf(ext.z() / voxel_size));

			resolution = vec3(resx, resy, resz);
			vec3 grid_ext = vec3(voxel_size) * resolution;
			vec3 grid_min = bbox.get_min_pnt() - 0.5f * (grid_ext - ext);

			bounds.ref_min_pnt() = grid_min;
			bounds.ref_max_pnt() = grid_min + grid_ext;
		}
	};

	struct ambient_occlusion_style {
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
			generate_sample_dirctions();
		}

		void derive_voxel_grid_parameters(const voxel_grid& vg) {
			const box3& volume_bbox = vg.bounds;
			const ivec3& volume_resolution = vg.resolution;

			unsigned max_extent_axis = cgv::math::max_index(volume_bbox.get_extent());

			texture_offset = volume_bbox.get_min_pnt();
			texture_scaling = vec3(1.0f) / volume_bbox.get_extent();
			texcoord_scaling = vec3(volume_resolution[max_extent_axis]) / vec3(volume_resolution);
			texel_size = 1.0f / volume_resolution[max_extent_axis];
		}

		void generate_sample_dirctions() {
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

protected:


	cgv::glutil::transfer_function_editor* tf_editor_ptr;









	/// store a pointer to the view for fast access
	view* view_ptr = nullptr;
	
	/// store the current OpenGL viewport configuration
	GLint viewport[4];

	/// path of the dataset to load - can be either a directory or a single file
	std::string datapath;

	/// misc configurable fields
	struct {
		/// proxy for controlling fltk_gl_view::instant_redraw
		bool instant_redraw_proxy = false;

		/// proxy for controlling stereo_view_interactor::fix_view_up_dir
		bool fix_view_up_dir_proxy = false;
	} misc_cfg;

	/// drag-n-drop state fields
	struct {
		/// current mouse position
		ivec2 pos;

		/// current drag-n-drop string
		std::string text;

		/// list of filenames extracted from @ref #text
		std::vector<std::string> filenames;
	} dnd;

	/// dataset state fields
	struct {
		/// set of filepaths for loading
		std::set<std::string> files;
	} dataset;

	// TODO: comment and cleanup members
	cgv::glutil::frame_buffer_container fbc;
	cgv::glutil::shader_library shaders;
	volume_render_style vstyle;

	/// rendering state fields
	struct {
		/// render style for the textured spline tubes
		textured_spline_tube_render_style style;
		
		/// accessor for the render data generated by the trajectory manager
		const traj_manager<float>::render_data *data;

		/// segment-wise arclength approximations (cubic bezier curves returning global
		/// trajectory arclength at the segment)
		std::vector<vec4> arclen_data;

		/// GPU-side storage buffer mirroring the \ref #arclen_data .
		vertex_buffer arclen_sbo;

		/// shared attribute array manager used by both renderers
		attribute_array_manager aam;

		/// the gpu sorter used to reorder the indices according to their corresponding segment visibility order
		cgv::glutil::gpu_sorter* sorter = nullptr;

		/// whether to sort the segemnts, which is used to boost performance together with conservative depth testing
		bool sort = true;

		float percentage = 1.0f;
	} render;

	/// trajectory manager
	traj_manager<float> traj_mgr;

	box3 bbox;
	
	bool show_volume = false;

	/// test texture
	texture tex;
	texture density_tex;
	texture tf_tex;

	voxel_grid density_volume;
	ambient_occlusion_style ao_style;

	void set_view(void);
	void update_attribute_bindings(void);
	void calculate_bounding_box(void);

	vec2 sd_quadratic_bezier(const vec3& A, const vec3& B, const vec3& C, const vec3& pos);

	int sample_voxel(const ivec3& vidx, const hermite_spline_tube::q_tube& qt);
	void voxelize_q_tube(const hermite_spline_tube::q_tube& qt);
	void create_density_volume(context& ctx, unsigned resolution);

	/// draw methods
	void draw_dnd(context& ctx);
	void draw_trajectories(context& ctx);
	void draw_density_volume(context& ctx);

	/// helper methods
	bool load_transfer_function(context& ctx);

public:
	tubes();
	std::string get_type_name() const { return "tubes"; }
	void handle_args(std::vector<std::string> &args);

	void clear(context& ctx);

	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void stream_help(std::ostream& os);
	void stream_stats(std::ostream& os) {}

	bool handle_all(cgv::gui::event& e);
	void on_set(void* member_ptr);

	bool init(context& ctx);
	void init_frame(context& ctx);
	void draw(context& ctx);

	void create_gui();

	//bool on_exit_request();
};
