#pragma once

// C++ STL
#include <vector>
#include <set>

// CGV framework core
#include <cgv/base/group.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/help_message.h>
#include <cgv/gui/file_helper.h>
#include <cgv/media/transfer_function.h>
#include <cgv/render/color_scale_adapter.h>
#include <cgv/render/drawable.h>
#include <cgv/render/managed_frame_buffer.h>
#include <cgv/render/shader_library.h>
#include <cgv/utils/stopwatch.h>

// CGV OpenGL lib
#include <cgv_gl/box_wire_render_data.h>
#include <cgv_gl/cone_render_data.h>
#include <cgv_gl/sphere_render_data.h>
#include <cgv_gl/volume_renderer.h>

// CGV framework overlays
#include <cgv_overlay/navigator.h>
#include <cgv_overlay/performance_monitor.h>
#include <cgv_overlay/transfer_function_editor.h>

// CGV framework GPU algorithms
#include <cgv_gpgpu/visibility_sort.h>

// CGV framework post processing algorithms
#include <cgv_post/temporal_anti_aliasing.h>

// CGV framework plugins
// - stereo_view_interactor for controlling/listening to camera changes
#include <plugins/crg_stereo_view/stereo_view_interactor.h>
// - screenshot plugin for saving/restoring scenes
#include <plugins/screenshot/screenshot.h>

// local includes
#include "traj_loader.h"
#include "arclen/main.h"
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
#ifdef RTX_SUPPORT
#include "optix_integration.h"
#include "optixtracer_textured_spline_tube.h"
#endif
#include "userstudies/ribbons_vs_tubes/trial.h"


// define GridMode outside of main on_tube_vis class to be able to use it with type reflection
enum class GridMode : unsigned {
	kNone = 0,
	kColor = 1,
	kNormal = 2,
	kColorAndNormal = 3
};

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

	/// data layout for per-node attributes within the attribute render SSBO
	struct node_attribs {
		vec4 pos_rad;
		vec4 color;
		vec4 tangent;
		vec4 t; // only uses .x component to store t, yzw are reserved for future use
	};

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
			cgv::render::texture *albedo, *position, *normal, *tangent, depth;
		} fb;
	} optix;

	void optix_cleanup (void);
	void optix_unregister_resources (void);

	bool optix_ensure_init (cgv::render::context &ctx);

	bool optix_init (void);
	bool optix_register_resources (cgv::render::context &ctx);

	void optix_draw_trajectories (cgv::render::context &ctx);

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif

	// the number of supported glyph mapping layers which is currently fixed at 4
	static const size_t k_supported_layer_count = 4;

	cgv::render::view* view_ptr = nullptr;

	cgv::overlay::transfer_function_editor_ptr cm_editor_ptr;
	cgv::overlay::transfer_function_editor_ptr tf_editor_ptr;
	cgv::overlay::navigator_ptr navigator_ptr;
	cgv::data::ref_ptr<color_map_viewer> cm_viewer_ptr;
	cgv::data::ref_ptr<mapping_legend> mapping_legend_ptr;
	cgv::overlay::performance_monitor_ptr perfmon_ptr;
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
	cgv::render::shader_compile_options tube_shading_options;

	/// store the current OpenGL viewport configuration
	GLint viewport[4] = { 0, 0, 0, 0 };

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
		cgv::render::texture rtlola_map_tex;

		/// VAO used to draw the map for the RTLola drone flight demo dataset
		cgv::render::attribute_array_binding rtlola_map_vao;

		/// vertex buffer for the quad containing the map for the RTLola drone flight demo dataset
		cgv::render::vertex_buffer rtlola_map_vbo;
	} dataset;

	cgv::post::temporal_anti_aliasing taa;
	
	cgv::render::managed_frame_buffer fbc;
	cgv::render::shader_library shaders;
	cgv::render::volume_render_style vstyle;
	std::shared_ptr<cgv::media::transfer_function> volume_tf = std::make_shared<cgv::media::transfer_function>();
	cgv::render::color_scale_adapter volume_tf_adapter;

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

	// user studies
	struct {
		userstudies::trial *active_trial = nullptr;
		userstudies::RvT::trial ribbons_vs_tubes_trial;
	} user_studies;

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

	/// rendering state fields
	struct {
		/// render style for the textured spline tubes
		cgv::render::textured_spline_tube_render_style style;
		
		/// render data generated by the trajectory manager
		const traj_manager<float>::render_data *data;

		/// the on-tube visualization layers for each loaded dataset
		std::vector<on_tube_visualization> visualizations;

		/// segment-wise arclength approximations (set of 4 cubic bezier curves returning global
		/// trajectory arclength at the segment, packed into the columns of a 4x4 matrix)
		arclen::parametrization arclen_data;

		/// GPU-side storage buffer mirroring the \ref #arclen_data .
		cgv::render::vertex_buffer arclen_sbo;

		/// GPU-side render attribute buffer.
		cgv::render::vertex_buffer render_sbo;

		/// GPU-side storage buffers storing independently sampled attribute data.
		std::vector<cgv::render::vertex_buffer> attribs_sbos;

		/// GPU-side storage buffers indexing the independently sampled attributes per tube segment.
		std::vector<cgv::render::vertex_buffer> aindex_sbos;

		/// shared attribute array manager used by both renderers
		cgv::render::attribute_array_manager aam;

		/// the gpu sorter used to reorder the indices according to their corresponding segment visibility order
		cgv::gpgpu::visibility_sort sorter;
	} render;
	int render_gui_dummy = 0;
	cgv::render::textured_spline_tube_render_style::AttribMode attrib_mode_bak = render.style.attrib_mode;

	/// trajectory manager
	traj_manager<float> traj_mgr;

	/// attribute handle manager
	attrib_handle_manager<float> ah_mgr;

	/// color map manager
	color_map_manager color_map_mgr;

	/// color map legend manager
	color_legend_manager color_legend_mgr;
	bool update_legends = false; // flag indicating whether the color and mapping legends need updating during init_frame

	unsigned scene_switch_state = 0;
	bool unlock_after_scene_switch = true;
	signed selected_scene = -1;
	screenshot *screenshot_ptr = nullptr;

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
	enum class DebugRenderMode {
		kDisabled,
		kNodes,
		kSegments,
		kNodesAndSegments,
		kVolume
	};

	/// debug state fields
	struct {
		DebugRenderMode render_mode = DebugRenderMode::kDisabled;

		/// debug render data
		struct {
			cgv::render::sphere_render_data<> nodes;
			cgv::render::cone_render_data<> segments;
		} geometry;
		
		// whether to print out information about placed glyphs like count and attribute count after compiling the layers
		bool print_glyph_information = false;
		/// whether to higlight individual segments in the textured spline tube renderer
		bool highlight_segments = false;
		/// whether to show glyphs that are normally not drawn due to overlap in a transparent fashion
		bool show_hidden_glyphs = false;

		/// whether to sort the segments, which is used to boost performance together with conservative depth testing
		bool sort = true;
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
			cgv::render::textured_spline_tube_render_style::LinePrimitive last_tube_primitive = cgv::render::textured_spline_tube_render_style::LP_TUBE_RUSSIG;
			/// stores the most recent "explicitly" selected ribbon tube primitive type
			cgv::render::textured_spline_tube_render_style::LinePrimitive last_ribbon_primitive = cgv::render::textured_spline_tube_render_style::LP_RIBBON_RAYCASTED;
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
	cgv::render::texture density_tex;
	cgv::render::texture tf_tex;

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
	void initialize_sorter(void);
	void calculate_bounding_box(void);

	void create_density_volume(cgv::render::context& ctx, unsigned resolution);

	/// draw methods
	void draw_dnd(cgv::render::context& ctx);
	void draw_trajectories(cgv::render::context& ctx);
	void draw_density_volume(cgv::render::context& ctx);

	/// helper methods
	cgv::render::shader_compile_options build_tube_shading_options();
	void on_register() override;
	void create_vec3_gui(const std::string& name, vec3& value, float min = 0.0f, float max = 1.0f);

	void on_view_interaction (const view_interaction &interaction);
	void handle_screenshot_change (screenshot::event &event);

public:
	on_tube_vis();
	~on_tube_vis();

	std::string get_type_name() const override { return "on_tube_vis"; }
	void handle_args(std::vector<std::string> &args) override;

	bool init(cgv::render::context& ctx) override;
	void clear(cgv::render::context& ctx) override;

	bool self_reflect(cgv::reflect::reflection_handler& rh) override;
	void stream_help(std::ostream& os) override;
	void stream_stats(std::ostream& os) override {}

	bool handle(cgv::gui::event& e) override;
	void handle_color_map_change();
	void on_set(void* member_ptr) override;
	bool on_exit_request() override;

	void init_frame(cgv::render::context& ctx) override;
	void draw(cgv::render::context& ctx) override;
	void after_finish(cgv::render::context& ctx) override;

	void create_gui() override;
};
