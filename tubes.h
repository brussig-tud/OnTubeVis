#pragma once

// C++ STL
#include <vector>
#include <set>

// CGV framework core
#include <cgv/base/node.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/utils/stopwatch.h>

// CGV OpenGL lib
#include <cgv_gl/volume_renderer.h>

// CGV framework graphics utility
#include <cgv_glutil/application_plugin.h>
#include <cgv_glutil/box_render_data.h>
#include <cgv_glutil/frame_buffer_container.h>
#include <cgv_glutil/navigator.h>
#include <cgv_glutil/radix_sort_4way.h>
#include <cgv_glutil/shader_library.h>
#include <cgv_glutil/sphere_render_data.h>
#include <cgv_glutil/color_scale_editor.h>
#include <cgv_glutil/transfer_function_editor.h>

// local includes
#include "traj_loader.h"
#include "demo.h" // interactive testbed helper classes and data
#include "hermite_spline_tube.h"
#include "voxelizer.h"
#include "ambient_occlusion_style.h"
#include "glyph_layer_manager.h"
#include "textured_spline_tube_renderer.h"



using namespace cgv::render;

////
// Plugin definition

/// baseline visualization plugin for arbitrary trajectory data as tubes using the framework tube renderers and
/// trajectory loading facilities
class tubes :
	public cgv::base::argument_handler, // derive from argument handler to be able to process custom arguments
	public cgv::glutil::application_plugin	// derive from application plugin, which is a node, drawable, gui provider and event handler and can handle overlays
{
public:

	/// data layout for per-node attributes within the attribute render SSBO
	struct node_attribs {
		vec4 pos_rad;
		vec4 color;
		vec4 tangent;
	};

	/// data layout for an attribute value sampled independently from the geometry nodes
	template <class T>
	struct free_attrib
	{
		/// default constructor
		inline free_attrib() {}
		/// convenience constructor
		inline free_attrib(float s, const T& value) : s(s), value(value) {}

		/// arclength along trajectory at which the value was sampled
		float s;
		/// actual attribute value
		T value;
	};

	/// data layout for glyph visual attribute values
	struct glyph_attribs
	{
		/// arclength along trajectory at which the glyph should be placed
		float s;
		/// radius properties
		float radius0, radius1;
		// angle properties
		float angle0, angle1;

		/// infer glyph dimensions (ToDo: should be dependent on glyph type)
		struct extent { float rad, diam; };
		static extent calc_radius (glyph_attribs glyph, float length_scale)
		{
			extent e;
			e.rad = std::min(glyph.radius0+glyph.radius1, 1.25f*glyph.radius0) / length_scale;
			e.diam = e.rad + e.rad;
			return e;
		}
	};

	cgv::type::DummyEnum voxel_grid_resolution;

	

protected:
	cgv::glutil::color_scale_editor* cs_editor_ptr;
	cgv::glutil::transfer_function_editor* tf_editor_ptr;
	cgv::glutil::navigator* navigator_ptr;


	enum GridMode {
		GM_NONE = 0,
		GM_COLOR = 1,
		GM_NORMAL = 2,
		GM_COLOR_NORMAL = 3
	};

	struct grid_parameters {
		vec2 scaling;
		float thickness;
		float blend_factor;
	};

	GridMode grid_mode;
	rgba grid_color;
	cgv::type::DummyEnum grid_normal_settings;
	bool grid_normal_inwards;
	bool grid_normal_variant;
	float normal_mapping_scale;
	std::vector<grid_parameters> grids;
	bool enable_fuzzy_grid;

	struct attribute_mapping_parameters {
		GlyphType glyph_type = GT_CIRCLE;
		bool curvature_correction = true;
		float radius0 = 2/3.f;
		float radius1 = 1/4.f;
		float angle0 = 215.0f;
		float angle1 = 100.0f;
		float length_scale = 1.0;
	} am_parameters;

	// global tube render settings
	bool override_attribute_values = false; // only for testing purposes (uses values controlled by GUI sliders)
	float antialias_radius = 0.5f;
	shader_define_map tube_shading_defines;






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

		/// proxy for controlling context::enable_vsynch through fltk_gl_view
		bool vsync_proxy = true;

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

		/// generated demo dataset
		std::vector<demo::trajectory> demo_trajs;
	} dataset;

	// TODO: comment and cleanup members
	cgv::glutil::frame_buffer_container fbc;
	cgv::glutil::shader_library shaders;
	volume_render_style vstyle;

	/// rendering state fields
	struct {
		/// render style for the textured spline tubes
		textured_spline_tube_render_style style;
		
		/// render data generated by the trajectory manager
		const traj_manager<float>::render_data *data;

		/// segment-wise arclength approximations (set of 4 cubic bezier curves returning global
		/// trajectory arclength at the segment, packed into the columns of a 4x4 matrix)
		std::vector<mat4> arclen_data;

		/// GPU-side storage buffer mirroring the \ref #arclen_data .
		vertex_buffer arclen_sbo;

		/// GPU-side render attribute buffer.
		vertex_buffer render_sbo;

		/// GPU-side storage buffer storing independently sampled attribute data.
		vertex_buffer attribs_sbo;

		/// GPU-side storage buffer indexing the independently sampled attributes per tube segment.
		vertex_buffer aindex_sbo;

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

	/// whether to display the interactive testbed
	bool show_demo;


	cgv::glutil::sphere_render_data<> srd;
	vec3 test_eye = vec3(5.0f, 0.5f, 5.0f);
	vec3 test_dir = vec3(0.0f, 0.0f, -1.0f);

	// Benchmark members
	bool do_benchmark;
	bool benchmark_running;
	cgv::utils::stopwatch benchmark_timer;
	unsigned total_frames;
	vec3 initial_eye_pos;
	vec3 initial_focus;
	double last_seconds_since_start;





	glyph_layer_manager glyph_layer_mgr;



	bool show_volume = false;
	box3 bbox;
	texture density_tex;
	texture tf_tex;

	voxelizer density_volume;
	ambient_occlusion_style ao_style, ao_style_bak; // the latter is used to restore defaults after demo data is unloaded

	glyph_layer_manager::shader_configuration glyph_layers_shader_config;






	bool compile_glyph_attribs (void);

	void set_view(void);
	void update_grid_ratios(void);
	void update_attribute_bindings(void);
	void calculate_bounding_box(void);

	int sample_voxel(const ivec3& vidx, const quadratic_bezier_tube& qt);
	void voxelize_q_tube(const quadratic_bezier_tube& qt);
	void create_density_volume(context& ctx, unsigned resolution);

	/// draw methods
	void draw_dnd(context& ctx);
	void draw_trajectories(context& ctx);
	void draw_density_volume(context& ctx);

	/// helper methods
	bool load_transfer_function(context& ctx);
	shader_define_map build_tube_shading_defines();

	void create_vec3_gui(const std::string& name, vec3& value, float min = 0.0f, float max = 1.0f);

public:
	tubes();
	std::string get_type_name() const { return "tubes"; }
	void handle_args(std::vector<std::string> &args);

	void clear(context& ctx);

	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void stream_help(std::ostream& os);
	void stream_stats(std::ostream& os) {}

	bool handle_event(cgv::gui::event& e);
	void on_set(void* member_ptr);

	bool init(context& ctx);
	void init_frame(context& ctx);
	void draw(context& ctx);

	void create_gui();
};
