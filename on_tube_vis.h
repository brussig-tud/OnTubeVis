#pragma once

// C++ STL
#include <vector>
#include <set>
#include <optional>
#include <mutex>
#include <condition_variable>

// CGV framework core
#include <cgv/base/group.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/help_message.h>
#include <cgv/gui/file_helper.h>
#include <cgv/render/managed_frame_buffer.h>
#include <cgv/render/shader_library.h>
#include <cgv/utils/stopwatch.h>

// CGV OpenGL lib
#include <cgv_gl/box_wire_render_data.h>
#include <cgv_gl/cone_render_data.h>
#include <cgv_gl/sphere_render_data.h>
#include <cgv_gl/volume_renderer.h>

// CGV framework application utility
#include <cgv_app/color_map_editor.h>
#include <cgv_app/navigator.h>
#include <cgv_app/performance_monitor.h>

// CGV framework post processing algorithms
#include <cgv_post/temporal_anti_aliasing.h>

// OnTubeVis private streaming API
#include "api/state/core.h"

// local includes
#include "traj_loader.h"
#include "demo.h" // interactive testbed helper classes and data
#include "attrib_handle_manager.h"
#include "voxelizer.h"
#include "color_map_manager.h"
#include "color_legend_manager.h"
#include "textured_spline_tube_renderer.h"
#include "color_map_viewer.h"
#include "mapping_legend.h"
#include "render/state.h"
#include "otv_client.h"
#ifdef RTX_SUPPORT
#include "optix_integration.h"
#include "optixtracer_textured_spline_tube.h"
#endif


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
	public cgv::base::group,			// derive from group to support child nodes (needed for overlays)
	public cgv::gui::event_handler,		// derive from event handler to receive input events
	public cgv::gui::provider,			// derive from gui provider to have gui controls
	public cgv::render::drawable		// derive from drawable to allow drawing in the GL context
{
	friend otv::otv_client;

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

	// service control flow
	static std::mutex init_mtx;
	static std::condition_variable init_cv;
	bool non_service_init_signaled = false;
	void signal_non_service_init (void);
	bool session_starting = false;
	bool session_active = false;
	bool session_first_node = true;
	bool session_taa_keep_sampling = false;
	unsigned session_taa_missing_samples = 0;
	unsigned session_glyphbuf_size = 0;
	unsigned session_sample_count_factor = 0;

	// API command endpoints
	void start_new_streaming_session (const VisSetup &vis_setup);

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

	view* view_ptr = nullptr;

	// don't load any dataset, disable most GUIs
	bool run_as_service = false;

	// for deferring populating the initial dataset and on-tube layers to display until the first frame is rendered
	bool data_init_pending = true;
	std::optional<std::string> layer_cfg_init_pending;

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

	/// tube shading settings
	tube_shading_settings tube_shading;

protected:
	/// shader defines for the deferred shading pass
	shader_compile_options tube_shading_options;

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

public:
	bool toggle_taa_proxy = true;
	cgv::post::temporal_anti_aliasing taa;

protected:
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
		render.style.max_t = client.playback_t = (float)playback.tstart;
		on_set(&render.style.max_t);
		/*on_set(&datapath_helper.file_name);
		on_set(&layer_config_file_helper.file_name);*/
	}
	void playback_reset_ds() {
		playback.active = false; update_member(&playback.active);
		render.style.max_t = client.playback_t = render.style.data_t_minmax.second;
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

	std::string get_tube_ribbon_toggle_label() const
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

public:
	cgv::render::box_render_data<> bbox_rd;
	cgv::render::box_wire_render_data<> bbox_wire_rd;

protected:
	vec3 last_sort_pos;
	vec3 last_sort_dir;

public:
	float override_cap_clip_distance_proxy;
	std::optional<float> override_cap_clip_distance;
	float buf_size_fract = 1;
	float glyph_buf_mult = 8;
	otv::render_state render;
	otv::otv_client  client{render};
	bool streaming_start_extrapol = false;

protected:
	int render_gui_dummy = 0;

public:
	/// trajectory manager
	traj_manager<float> traj_mgr;

protected:
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
	struct debug_settings {
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
	ambient_occlusion_style ao_style_bak; // used to restore defaults after demo data is unloaded

	bool save_layer_configuration(const std::string& file_name);
	bool read_layer_configuration(const std::string& file_name);

	void update_glyph_layer_managers(void);
	void glyphs_out_of_date(bool state);
	bool compile_glyph_attribs();
	double change_time = 0.0;
	double recalc_delay = 0.2;
	bool has_changed = false;
	void timer_event(double, double);
	void update_scene_extents (void);
	void set_view(void);

	void ensure_initial_dataset(context& ctx);
	void update_dataset(context &ctx, bool cause_new_session=true);
	bool update_visualizations(bool may_cause_new_session=true);
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

	bool handle(cgv::gui::event& e);
	void handle_color_map_change();
	void handle_transfer_function_change();
	void on_set(void* member_ptr);
	void quit();
	bool on_exit_request();

	bool init(context& ctx);
	void init_frame(context& ctx);
	void draw(context& ctx);
	void after_finish(context& ctx);

	void create_gui();
};


/// The OnTubeVis singleton instance implementing the streaming API
extern on_tube_vis *otv_instance;
