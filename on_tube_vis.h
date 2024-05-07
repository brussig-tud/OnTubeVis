#pragma once

// C++ STL
#include <vector>
#include <set>

// CGV framework core
#include <cgv/base/node.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/help_message.h>
#include <cgv/gui/file_helper.h>
#include <cgv/render/drawable.h>
#include <cgv/render/managed_frame_buffer.h>
#include <cgv/render/shader_library.h>
#include <cgv/utils/stopwatch.h>

// CGV OpenGL lib
#include <cgv_gl/box_wire_render_data.h>
#include <cgv_gl/cone_render_data.h>
#include <cgv_gl/sphere_render_data.h>
#include <cgv_gl/volume_renderer.h>

// CGV framework application utility
#include <cgv_app/application_plugin.h>
#include <cgv_app/color_map_editor.h>
#include <cgv_app/navigator.h>
#include <cgv_app/performance_monitor.h>

// CGV framework GPU algorithms
#include <cgv_gpgpu/visibility_sort.h>

// CGV framework post processing algorithms
#include <cgv_post/temporal_anti_aliasing.h>

// local includes
#include "traj_loader.h"
#include "arclen_helper.h"
#include "demo.h" // interactive testbed helper classes and data
#include "attrib_handle_manager.h"
#include "voxelizer.h"
#include "ambient_occlusion_style.h"
#include "visualization_variables_info.h"
#include "glyph_layer_manager.h"
#include "color_map_manager.h"
#include "color_legend_manager.h"
#include "layer_config_io.h"
#include "textured_spline_tube_renderer.h"
#include "color_map_viewer.h"
#include "mapping_legend.h"
#include "render_types.h"
#include "gpumem.h"
#ifdef RTX_SUPPORT
#include "optix_integration.h"
#include "optixtracer_textured_spline_tube.h"
#endif


// define GridMode outside of main on_tube_vis class to be able to use it with type reflection
enum GridMode {
	GM_NONE = 0,
	GM_COLOR = 1,
	GM_NORMAL = 2,
	GM_COLOR_AND_NORMAL = 3
};

namespace cgv {
namespace reflect {

// define custom reflection traits for the GridMode
enum_reflection_traits<GridMode> get_reflection_traits(const GridMode&);

}
}



using namespace cgv::render;

////
// Plugin definition

/// baseline visualization plugin for arbitrary trajectory data as tubes using the framework tube renderers and trajectory loading facilities
class on_tube_vis :
	public cgv::base::argument_handler, // derive from argument handler to be able to process custom arguments
	public cgv::app::application_plugin	// derive from application plugin, which is a node, drawable, gui provider and event handler and can handle overlays
{
public:
	using vec2 = cgv::vec2;
	using vec3 = cgv::vec3;
	using vec4 = cgv::vec4;
	using ivec2 = cgv::ivec2;
	using ivec3 = cgv::ivec3;
	using uvec2 = cgv::uvec2;
	using vecn = cgv::vecn;
	using mat4 = cgv::mat4;
	using box3 = cgv::box3;
	using rgb = cgv::rgb;
	using rgba = cgv::rgba;

	cgv::type::DummyEnum voxel_grid_resolution;

protected:
#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	/// List of implemented primitives for OptiX raytracing
	enum OptixPrimitive
	{
		OPR_RUSSIG = 0,
		OPR_PHANTOM = 1,
		OPR_BUILTIN = 2,
		OPR_BUILTIN_CUBIC = 3
	};

	/// OptiX debug output options
	enum OptixDebugVisualization
	{
		OXV_OFF = 0,
		OXV_ALBEDO = 1,
		OXV_DEPTH = 2,
		OXV_NORMAL_TANGENT = 3
	};

	// state
	struct
	{
		// keep track of initialization state
		bool initialized = false;

		// use OptiX to raycast tubes instead of OpenGL rasterization
		bool enabled = false;

		// result output mode
		OptixDebugVisualization debug = OXV_OFF;

		// Wether to show BLAS bounding volumes (Russig intersector only for now)
		bool debug_bvol = false;

		// Whether to render a hologram
		bool holographic = false, unproject_mode_dbg=false;

		// Eye selector for testing stereo transform with normal non-holo/non-stereo mode
		float holo_eye = 0;

		// remembers whether TAA was enabeld since it currently needs to be force-disabled for holographic rendering
		bool prev_TAA_state;

		// OptiX device context
		OptixDeviceContext context = nullptr;

		// sphere-based hermite spline tube intersector by Russig et al.
		optixtracer_textured_spline_tube_russig tracer_russig;

		// Custom implementation of the phantom-ray-hair-intersector for disc-based cubic spline tubes
		optixtracer_textured_spline_tube_phantom tracer_phantom;

		// Optix-builtin phantom-ray-hair-intersector for disc-based quadratic spline tubes
		optixtracer_textured_spline_tube_builtin tracer_builtin;

		// Optix-builtin phantom-ray-hair-intersector for disc-based cubic spline tubes
		optixtracer_textured_spline_tube_builtincubic tracer_builtin_cubic;

		// the intersector/primitive to use for raytracing
		OptixPrimitive primitive = OPR_RUSSIG;
		optixtracer_textured_spline_tube *tracer = &tracer_russig;

		// SSBO interop resource handles
		cudaGraphicsResource *sbo_nodes = nullptr;
		cudaGraphicsResource *sbo_nodeids = nullptr;
		cudaGraphicsResource *sbo_alen = nullptr;

		// GL interop
		CUstream stream = nullptr;
		cuda_output_buffer<float4> outbuf_albedo;
		cuda_output_buffer<float3> outbuf_position;
		cuda_output_buffer<float3> outbuf_normal;
		cuda_output_buffer<float3> outbuf_tangent;
		cuda_output_buffer<float1> outbuf_depth;

		// framebuffer attachment references (except depth, which is a texture we own for an NVIDIA driver bug workaround)
		struct {
			texture *albedo, *position, *normal, *tangent, depth;
		} fb;
	} optix;

	void optix_cleanup (void);
	void optix_unregister_resources (void);

	bool optix_ensure_init (context &ctx);

	bool optix_init (void);
	bool optix_register_resources (context &ctx);

	void optix_draw_trajectories (context &ctx);

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif

	cgv::app::color_map_editor_ptr cm_editor_ptr;
	cgv::app::color_map_editor_ptr tf_editor_ptr;
	cgv::app::navigator_ptr navigator_ptr;
	cgv::data::ref_ptr<color_map_viewer> cm_viewer_ptr;
	cgv::data::ref_ptr<mapping_legend> mapping_legend_ptr;
	cgv::app::performance_monitor_ptr perfmon_ptr;
	bool show_mapping_legend = true;
	bool show_color_map_viewer = false;
	bool show_navigator = false;
	bool show_performance_monitor = false;

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

	/// shader defines for the deferred shading pass
	shader_define_map tube_shading_defines;

	/// store the current OpenGL viewport configuration
	GLint viewport[4];

	/// GUI help message
	cgv::gui::help_message help;

	/// file helper for path of the dataset to load - can be either a directory or a single file
	cgv::gui::file_helper datapath_helper;

	/// misc configurable fields
	struct {
		/// proxy for controlling fltk_gl_view::instant_redraw
		bool instant_redraw_proxy = false;

		/// proxy for controlling context::enable_vsynch through fltk_gl_view
		bool vsync_proxy = false;

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

		/// whether the RTLola drone flight demo dataset was detected
		bool is_rtlola = false;

		/// whether to show the map when the RTLola drone flight demo dataset was detected
		bool rtlola_show_map = true;

		/// the map texture that will be displayed under the RTLola drone flight demo dataset
		texture rtlola_map_tex;

		/// VAO used to draw the map for the RTLola drone flight demo dataset
		attribute_array_binding rtlola_map_vao;

		/// vertex buffer for the quad containing the map for the RTLola drone flight demo dataset
		vertex_buffer rtlola_map_vbo;
	} dataset;

	cgv::post::temporal_anti_aliasing taa;
	
	cgv::render::managed_frame_buffer fbc;
	cgv::render::shader_library shaders;
	volume_render_style vstyle;
	cgv::render::gl_color_map volume_tf;

	// playback
	struct {
		bool active = false;
		bool repeat = false;
		double tstart = 0.;
		double tend = 1.;
		double speed = 1.;

		bool follow = false;
		unsigned follow_traj = 0;
		unsigned follow_last_nid = 0;

		double time_active = 0.;
		cgv::utils::stopwatch timer = &time_active;
	} playback;

	void playback_rewind() {
		render.style.max_t = (float)playback.tstart;
		on_set(&render.style.max_t);
	}
	void playback_reset_ds() {
		playback.active = false; update_member(&playback.active);
		render.style.max_t = render.style.data_t_minmax.second;
		on_set(&render.style.max_t);
	}

	void toggle_tube_ribbon()
	{
		if(render.style.is_tube())
			render.style.line_primitive = ui_state.tr_toggle.last_ribbon_primitive;
		else
			render.style.line_primitive = ui_state.tr_toggle.last_tube_primitive;

		ui_state.tr_toggle.was_toggled = true;
		on_set(&render.style.line_primitive);
	}

	std::string get_tube_ribbon_toggle_label()
	{
		std::string label = "Current: ";
		label += render.style.is_tube() ? "tubes" : "ribbons";
		label += " (toggle)";
		return label;
	}

	void update_tube_ribbon_toggle()
	{
		ui_state.tr_toggle.button->set("label", get_tube_ribbon_toggle_label());
	}

	bool show_bbox = false;
	bool show_wireframe_bbox = true;
	cgv::render::box_render_data<> bbox_rd;
	cgv::render::box_wire_render_data<> bbox_wire_rd;

	vec3 last_sort_pos;
	vec3 last_sort_dir;

	struct on_tube_visualization {
		glyph_layer_manager::configuration config;
		glyph_layer_manager manager;
		std::shared_ptr<visualization_variables_info> variables;

		on_tube_visualization(cgv::base::base_ptr base_ptr) : manager(base_ptr) {
			variables = std::make_shared<visualization_variables_info>();
		}
	};

	struct render_state;

	/// Manages ring buffer content for a single trajectory.
	class ringbuf_trajectory {
	private:
		/// Constant to indicate the lack of a buffer entry.
		static constexpr gpumem::index_type no_index = -1;

		/// The absolute index of the last entry in the node buffer belonging to this trajectory.
		gpumem::index_type last_node_idx = no_index;

		float arc_length = 0;

	public:
		/// Extend the trajectory by one node at the end.
		void append_node(const node_attribs &node, render_state &render);
	};

	/// rendering state fields
	struct render_state {
		/// render style for the textured spline tubes
		textured_spline_tube_render_style style;
		
		/// render data generated by the trajectory manager
		const traj_manager<float>::render_data *data;

		/// the on-tube visualization layers for each loaded dataset
		std::vector<on_tube_visualization> visualizations;

		/// segment-wise arclength approximations (set of 4 cubic bezier curves returning global
		/// trajectory arclength at the segment, packed into the columns of a 4x4 matrix)
		arclen::parametrization arclen_data;

		/// GPU ring buffer containing trajectory nodes.
		gpumem::ring_buffer<node_attribs> node_buffer;

		/// GPU ring buffer containing segments defined as pairs of absolute indices into #node_buffer.
		gpumem::ring_buffer<uvec2> segment_buffer;

		/// GPU buffer containing segment-wise arclength parametrization.
		/// Entries correspond to #segment_buffer.
		gpumem::array<mat4> t_to_s;

		/// GPU-side storage buffer mirroring the \ref #arclen_data .
		vertex_buffer arclen_sbo;

		/// GPU-side render attribute buffer.
		vertex_buffer render_sbo;

		/// GPU-side storage buffers storing independently sampled attribute data.
		std::vector<vertex_buffer> attribs_sbos;

		/// GPU-side storage buffers indexing the independently sampled attributes per tube segment.
		std::vector<vertex_buffer> aindex_sbos;

		/// shared attribute array manager used by both renderers
		attribute_array_manager aam;

		/// the gpu sorter used to reorder the indices according to their corresponding segment visibility order
		cgv::gpgpu::visibility_sort sorter;

		/// Managers for trajectory data stored in ring buffers, along with the index range each trajectory will be
		/// generated from.
		std::vector<std::pair<ringbuf_trajectory, uvec2>> trajectories;

		/// Fence placed directly after the last draw command.
		GLsync draw_fence;

		/// Append all data points up to the current timestamp to their respective trajectory.
		void extend_trajectories ();

		/// Remove all data points up to the given timestamp from their respective trajectory.
		void trim_trajectories (float cutoff_time);
	} render;
	int render_gui_dummy = 0;

	/// trajectory manager
	traj_manager<float> traj_mgr;

	/// attribute handle manager
	attrib_handle_manager<float> ah_mgr;

	/// color map manager
	color_map_manager color_map_mgr;

	/// color map legend manager
	color_legend_manager color_legend_mgr;
	bool update_legends = false; // flag indicating whether the color and mapping legends need updating during init_frame

	/// benchmark state fields
	struct {
		/// whether a benchmark run is requested
		bool requested = false;
		/// whether a benchmark is currently running
		bool running = false;
		/// timer to count elapsed time
		cgv::utils::stopwatch timer;
		/// counter for rendered frames
		unsigned total_frames;
		/// store last seconds since the start of the run
		double last_seconds_since_start;

		double sort_time_total = 0.0;
		unsigned num_sorts = 0;
	} benchmark;

	/// the different debug render modes
	enum DebugRenderMode {
		DRM_NONE,
		DRM_NODES,
		DRM_SEGMENTS,
		DRM_NODES_SEGMENTS,
		DRM_VOLUME
	};

	/// debug state fields
	struct {
		DebugRenderMode render_mode = DRM_NONE;

		/// debug render data
		struct {
			cgv::render::sphere_render_data<> nodes;
			cgv::render::cone_render_data<> segments;
		} geometry;
		
		/// whether to higlight individual segments in the textured spline tube renderer
		bool highlight_segments = false;
		/// whether to show glyphs that are normally not drawn due to overlap in a transparent fashion
		bool show_hidden_glyphs = false;

		/// whether to sort the segments, which is used to boost performance together with conservative depth testing
		/// TODO: Sorting currently does not work with ring buffers.
		bool sort = false;
		/// whether to only sort after significant view changes instead of every redraw
		bool lazy_sort = true;
		/// whether to foirce the initial draw order of segments as defined in the data set (overrides sort setting)
		bool force_initial_order = false;
		/// whether to limit the render count
		bool limit_render_count = false;
		/// percentage of rendered segments
		float render_percentage = 1.0f;
		/// amount of rendered segments
		size_t render_count = 0;
		/// total segment count
		size_t segment_count = 0;

		double far_extent_factor = 0.8;
		double near_extent_factor = 0.3;
		bool near_view = false;
	} debug;

	/// misc state needed for handling all kinds of user interaction (like "smart" convenience toggles)
	struct {
		/// "smart" tube/ribbon toggle
		struct {
			/// stores the most recent "explicitly" selected rasterization tube primitive type
			textured_spline_tube_render_style::LinePrimitive last_tube_primitive = textured_spline_tube_render_style::LP_TUBE_RUSSIG;
			/// stores the most recent "explicitly" selected ribbon tube primitive type
			textured_spline_tube_render_style::LinePrimitive last_ribbon_primitive = textured_spline_tube_render_style::LP_RIBBON_RAYCASTED;
			/// flags that the last change to the selected line primitive was from this toggle
			bool was_toggled = false;
			/// convencience function for check-and-resetting the toggle flag
			bool check_toggled (void) { const bool toggled=was_toggled; was_toggled=false; return toggled; }
			/// reference to the GUI button attached to the toggle
			cgv::gui::button_ptr button;
		} tr_toggle;
	} ui_state;

	bool benchmark_mode = false;
	bool benchmark_mode_setup = false;
	
	/// layer configuration file handling fields
	cgv::gui::file_helper layer_config_file_helper;
	bool layer_config_has_unsaved_changes = false;

	bool voxelize_gpu = true;
	box3 bbox;
	texture density_tex;
	texture tf_tex;

	voxelizer density_volume;
	ambient_occlusion_style ao_style, ao_style_bak; // the latter is used to restore defaults after demo data is unloaded

	bool save_layer_configuration(const std::string& file_name);
	bool read_layer_configuration(const std::string& file_name);

	void update_glyph_layer_managers(void);
	void glyphs_out_of_date(bool state);
	bool compile_glyph_attribs(void);
	double change_time = 0.0;
	double recalc_delay = 0.2;
	bool has_changed = false;
	void timer_event(double, double);

	void set_view(void);
	void update_grid_ratios(void);
	void update_attribute_bindings(void);
	void update_debug_attribute_bindings(void);
	void calculate_bounding_box(void);

	void create_density_volume(context& ctx, unsigned resolution);

	/// draw methods
	void draw_dnd(context& ctx);
	void draw_trajectories(context& ctx);
	void draw_density_volume(context& ctx);

	/// helper methods
	shader_define_map build_tube_shading_defines();
	void on_register();
	void create_vec3_gui(const std::string& name, vec3& value, float min = 0.0f, float max = 1.0f);

public:
	on_tube_vis();
	~on_tube_vis();

	std::string get_type_name() const { return "on_tube_vis"; }
	void handle_args(std::vector<std::string> &args);

	void clear(context& ctx);

	bool self_reflect(cgv::reflect::reflection_handler& rh);
	void stream_help(std::ostream& os);
	void stream_stats(std::ostream& os) {}

	bool handle_event(cgv::gui::event& e);
	void handle_color_map_change();
	void handle_transfer_function_change();
	void handle_member_change(const cgv::utils::pointer_test& m);
	bool on_exit_request();

	bool init(context& ctx);
	void init_frame(context& ctx);
	void draw(context& ctx);
	void after_finish(context& ctx);

	void create_gui();
};
