
// Implemented header
#include "on_tube_vis.h"

// C++ STL
#include <filesystem>

// CGV framework core
#include <cgv/defines/quote.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/trigger.h>
#include <cgv/signal/rebind.h>
#include <cgv/gui/theme_info.h>
#include <cgv/math/ftransform.h>
#include <cgv/media/image/image_reader.h>
#include <cgv/utils/advanced_scan.h>

// CGV framework application utility
#include <cgv_app/color_map_reader.h>
#include <cgv_app/color_map_writer.h>

// CGV framework plugins
// - fltk_gl_view for controlling instant redraw
#include <plugins/cg_fltk/fltk_gl_view.h>
// - stereo_view_interactor for controlling fix_view_up_dir
#include <plugins/crg_stereo_view/stereo_view_interactor.h>

// CGV framework 3rd party libraries
#include <3rd/xml/tinyxml2/tinyxml2.h>
#include <3rd/xml/cgv_xml/query.h>

// Local includes
#include "arclen_helper.h"
#include "glyph_compiler.h"
#ifdef RTX_SUPPORT
#include "cuda/optix_interface.h"


// ###############################
// ### BEGIN: OptiX integration
// ###############################

void optix_log_cb (unsigned int lvl, const char *tag, const char *msg, void* /* cbdata */) {
	std::cerr << "["<<std::setw(2)<<lvl<<"]["<<std::setw(12)<<tag<<"]: " << msg << std::endl;
}

// ###############################
// ###  END:  OptiX integration
// ###############################
#endif


/* TODOs:
* > Find way of adjusting view z-Near to prevent z-fighting artifacts that occur with the default
*   value (which is often much too small).
* 
* > Check order of primitives behind the camera after sorting by distance and possibly prevent
*   drawing of invisible primitives (probably irrelevant)?
* 
* > Resolve bug in star and line plot. The first non-empty/mapped entry will always get mapped
*   to the first color. Example for star plot: map only axis 2, so axis 0 and 1 are unmapped.
*   Then the color from axis 0 will be used for the mapped axis 2 while the color from axis 2
*   is ignored.
*/
#include <cgv/gui/application.h>

namespace cgv {
namespace reflect {

enum_reflection_traits<GridMode> get_reflection_traits(const GridMode&) {
	return enum_reflection_traits<GridMode>("GM_NONE,GM_COLOR,GM_NORMAL,GM_COLOR_AND_NORMAL");
}

}
}

void on_tube_vis::on_register()
{
	cgv::gui::application::get_window(0)->set("title", "OnTubeVis");
}


on_tube_vis::on_tube_vis() : application_plugin("OnTubeVis"), color_legend_mgr(this)
{
	// adjust geometry and grid style defaults
	render.style.material.set_brdf_type(
		(cgv::media::illum::BrdfType)(cgv::media::illum::BrdfType::BT_STRAUSS_DIFFUSE
			| cgv::media::illum::BrdfType::BT_COOK_TORRANCE)
	);
	render.style.material.set_roughness(0.25);
	render.style.material.set_metalness(0.25);
	render.style.material.set_ambient_occlusion(0.75);
	render.style.material.set_specular_reflectance({ 0.05f, 0.05f, 0.05f });
	render.style.use_conservative_depth = true;
	
	bbox_rd.style.culling_mode = cgv::render::CM_FRONTFACE;
	bbox_rd.style.illumination_mode = cgv::render::IM_TWO_SIDED;
	bbox_rd.style.surface_color = rgb(0.7f);

	vstyle.enable_depth_test = false;

	debug.geometry.segments.style.rounded_caps = true;

	grids.resize(2);
	grids[0].scaling = vec2(1.0f, 1.0f);
	grids[0].thickness = 0.05f;
	grids[0].blend_factor = 0.5f;
	grids[1].scaling = vec2(4.0f);
	grids[1].thickness = 0.1f;
	grids[1].blend_factor = 0.333f;
	grid_color = rgba(0.25f, 0.25f, 0.25f, 0.75f);
	grid_mode = GM_COLOR_AND_NORMAL;
	grid_normal_settings = (cgv::type::DummyEnum)1u;
	grid_normal_inwards = true;
	grid_normal_variant = true;
	normal_mapping_scale = 1.0f;
	enable_fuzzy_grid = false;

	// set default voxel grid resolution
#ifdef _DEBUG
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(32u);
#else
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(128u);
#endif

	shaders.add("tube_shading", "textured_spline_tube_shading.glpr");

	// add framebuffer attachments needed for deferred rendering
	fbc.add_attachment("depth", "uint32[D]");
	fbc.add_attachment("albedo", "flt32[R,G,B,A]");
	fbc.add_attachment("position", "flt32[R,G,B]");
	fbc.add_attachment("normal", "flt32[R,G,B]");
	fbc.add_attachment("tangent", "flt32[R,G,B]");

	// register overlay widgets
	mapping_legend_ptr = register_overlay<mapping_legend>("Mapping Legend");

	cm_editor_ptr = register_overlay<cgv::app::color_map_editor>("Color Scales");
	cm_editor_ptr->set_visibility(false);
	cm_editor_ptr->gui_options.create_default_tree_node = false;
	cm_editor_ptr->set_on_change_callback(std::bind(&on_tube_vis::handle_color_map_change, this));

	tf_editor_ptr = register_overlay<cgv::app::color_map_editor>("Transfer Function");
	tf_editor_ptr->set_visibility(false);
	tf_editor_ptr->set_opacity_support(true);
	tf_editor_ptr->set_on_change_callback(std::bind(&on_tube_vis::handle_transfer_function_change, this));

	navigator_ptr = register_overlay<cgv::app::navigator>("Navigator");
	navigator_ptr->set_visibility(false);
	navigator_ptr->gui_options.show_layout_options = false;
	navigator_ptr->set_overlay_alignment(cgv::app::overlay::AO_START, cgv::app::overlay::AO_END);
	navigator_ptr->set_size(100);
	
	cm_viewer_ptr = register_overlay<color_map_viewer>("Color Scale Viewer");
	cm_viewer_ptr->set_visibility(false);

	perfmon_ptr = register_overlay<cgv::app::performance_monitor>("Performance Monitor");
	perfmon_ptr->set_visibility(false);
	perfmon_ptr->set_show_background(false);
	perfmon_ptr->enable_monitoring_only_when_visible(true);
	
	layer_config_file_helper = cgv::gui::file_helper(this, "Open/Save Layer Configuration", cgv::gui::file_helper::Mode::kOpenAndSave);
	layer_config_file_helper.add_filter("Layer Configuration XML", "xml");

	// setup datapath input control
	datapath_helper = cgv::gui::file_helper(this, "Open Trajectory Data", cgv::gui::file_helper::Mode::kOpen);
	datapath_helper.add_multi_filter("All Trajectory Files", {"bezdat", "csv", "sepia", "ppcdf", "ipcdf", "tgen"});
	datapath_helper.add_filter("Bezier Splines", "bezdat");
	datapath_helper.add_filter("CSV", "csv");
	datapath_helper.add_filter("Sepia Trajectories", "sepia");
	datapath_helper.add_filter("On-Board Diagnostics", "ppcdf");
	datapath_helper.add_filter("On-Board Diagnostics Extended", "ipcdf");
	datapath_helper.add_filter("Trajectory Generator Config", "tgen");

	// fill help message info
	help.add_line("OnTubeVis Help");
	help.add_line("");
	help.add_line("Load Datasets and Configurations");
	help.add_line("Open a dataset file or folder containing datasets through the \"Data Path\" input. Open a configuration through the \"Configuration\" input. Alternatively, Drag & Drop supported data files or configurations onto the window.");

	help.add_line("");
	help.add_line("Keybindings");

	help.add_line("View:");
	help.add_bullet_point(".\t\t\t : Near");
	help.add_bullet_point(",\t\t\t : Far");
	help.add_bullet_point("CTRL+Space\t : Reset");

	help.add_line("Scene:");
	help.add_bullet_point("NumP. Enter\t : Toggle tube/ribbon mode");
	help.add_bullet_point("B\t\t\t : Toggle bounding box");
	help.add_bullet_point("W\t\t\t : Toggle wireframe bounding box");
	help.add_bullet_point("R\t\t\t : Double tube radius/ribbon width");
	help.add_bullet_point("SHIFT+R\t\t : Halve tube radius/ribbon width");
	help.add_bullet_point("G\t\t\t : Cycle grid modes");
	help.add_bullet_point("NumPad 0\t : Toggle satellite image map (only for dataset \"rtlola_droneflight\")");

	help.add_line("Rendering:");
	help.add_bullet_point("A\t\t\t : Toggle ambient occlusion");
	help.add_bullet_point("T\t\t\t : Toggle temporal anti-aliasing");

#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	help.add_bullet_point("CTRL+O\t\t : Toggle OptiX");
	help.add_bullet_point("P\t\t\t : Cycle OptiX primitive");
	help.add_bullet_point("CTRL+P\t\t : Toggle OptiX unproject mode");
	help.add_bullet_point("H\t\t\t : Toggle holographic rendering (OptiX only)");
	
	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif

	help.add_line("Playback:");
	help.add_bullet_point("Space\t\t : Play/Pause");
	help.add_bullet_point("Backspace\t : Rewind");
	help.add_bullet_point("End\t\t\t : Skip to end");
	help.add_bullet_point("Home\t\t : Follow active trajectory");

	help.add_line("Widgets:");
	help.add_bullet_point("M\t\t\t : Toggle color scale preview");
	help.add_bullet_point("N\t\t\t : Toggle navigator");

	help.add_line("Benchmark:");
	help.add_bullet_point("[1-4]\t\t : Select preset");
	help.add_bullet_point("CTRL+B\t\t : Start/Abort");

	// connect animation timer callback
	connect(cgv::gui::get_animation_trigger().shoot, this, &on_tube_vis::timer_event);

#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	// add display shader to library
	shaders.add("optix_display", "optix_display.glpr");

	optix.prev_TAA_state = taa.is_enabled();

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif
}

on_tube_vis::~on_tube_vis()
{
#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	// perform cleanup routine
	optix_cleanup();

	// shutdown optix
	if (optix.context)
	{
		OPTIX_CHECK(optixDeviceContextDestroy(optix.context));
		optix.context = nullptr;
	}

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif
}

void on_tube_vis::handle_args (std::vector<std::string> &args)
{
	// look out for potential dataset files/dirs
	std::vector<unsigned> arg_ids;
	for (unsigned i=0; i<(unsigned)args.size(); i++)
		if (traj_mgr.can_load(args[i]))
		{
			// this appears to be a dataset file we're supposed to load
			dataset.files.emplace(args[i]);
			arg_ids.emplace_back(i);
		}

	// process our arguments (if any)
	if (!arg_ids.empty())
	{
		// remove the arguments we grabbed
		for (signed i=(signed)arg_ids.size()-1; i>=0; i--)
			args.erase(args.begin() + arg_ids[i]);
		// announce change in dataset_fn
		on_set(&dataset);
	}
}

void on_tube_vis::clear(cgv::render::context &ctx) {
	// decrease reference count of the renderers by one
	ref_textured_spline_tube_renderer(ctx, -1);
	ref_volume_renderer(ctx, -1);

	bbox_rd.destruct(ctx);
	bbox_wire_rd.destruct(ctx);

	debug.geometry.nodes.destruct(ctx);
	debug.geometry.segments.destruct(ctx);

	shaders.clear(ctx);
	fbc.destruct(ctx);
	
	color_map_mgr.destruct(ctx);

	render.sorter.destruct(ctx);

	density_volume.destruct(ctx);

	taa.destruct(ctx);
}

bool on_tube_vis::self_reflect (cgv::reflect::reflection_handler &rh)
{
	return
		rh.reflect_member("datapath", datapath_helper.file_name) &&
		rh.reflect_member("layer_config_file", layer_config_file_helper.file_name) && // ToDo: figure out proper reflection name
		rh.reflect_member("show_hidden_glyphs", debug.show_hidden_glyphs) &&
		rh.reflect_member("render_style", render.style) &&
		rh.reflect_member("attrib_mode", (unsigned&)render.style.attrib_mode) &&
		rh.reflect_member("bounding_geometry", render.style.bounding_geometry) &&
		rh.reflect_member("bounding_box_color", bbox_rd.style.surface_color) &&
		rh.reflect_member("show_bounding_box", show_bbox) &&
		rh.reflect_member("show_wireframe_box", show_wireframe_bbox) &&
		rh.reflect_member("grid_mode", grid_mode) &&
		rh.reflect_member("grid_normal_settings", grid_normal_settings) &&
		rh.reflect_member("grid_normal_inwards", grid_normal_inwards) &&
		rh.reflect_member("grid_normal_variant", grid_normal_variant) &&
		rh.reflect_member("voxelize_gpu", voxelize_gpu) &&
#ifdef RTX_SUPPORT
		rh.reflect_member("use_optix", optix.enabled) &&
		rh.reflect_member("optix_primitive", optix.primitive) &&
		rh.reflect_member("optix_debug_mode", optix.debug) &&
		rh.reflect_member("optix_holographic", optix.holographic) &&
#endif
		rh.reflect_member("instant_redraw_proxy", misc_cfg.instant_redraw_proxy) &&
		rh.reflect_member("vsync_proxy", misc_cfg.vsync_proxy) &&
		rh.reflect_member("fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy) &&
		rh.reflect_member("benchmark_mode", benchmark_mode);
}

void on_tube_vis::stream_help (std::ostream &os) {
	os << "on_tube_vis: adapt <R>adius, <B>ounding box, <W>ire bounding box, <NUM_Enter> toggle tube/ribbon" << std::endl;
}

#define SET_MEMBER(m, v) m = v; update_member(&m);

bool on_tube_vis::handle_event(cgv::gui::event &e) {

	unsigned et = e.get_kind();
	unsigned char modifiers = e.get_modifiers();

	if(et == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = (cgv::gui::key_event&) e;
		cgv::gui::KeyAction ka = ke.get_action();

		bool handled = false;

		if(ka == cgv::gui::KA_PRESS) {
			switch(ke.get_key()) {
			case ',':
				std::cout << "View setting: far" << std::endl;
				debug.near_view = false;
				set_view();
				handled = true;
				break;
			case '.':
				std::cout << "View setting: near" << std::endl;
				debug.near_view = true;
				set_view();
				handled = true;
				break;
			case '1':
				std::cout << "Benchmark setup: previous (proxy gemometry = full box; conservative depth = off; sorting = off; AO = off)" << std::endl;
				SET_MEMBER(render.style.bounding_geometry, textured_spline_tube_render_style::BG_BOX);
				SET_MEMBER(render.style.use_cubic_tangents, false);
				SET_MEMBER(render.style.use_conservative_depth, false);
				SET_MEMBER(debug.sort, false);
				if(!debug.force_initial_order) {
					SET_MEMBER(debug.force_initial_order, true);
					on_set(&debug.force_initial_order);
				}
				SET_MEMBER(ao_style.enable, false);
				on_set(&ao_style.enable);
				handled = true;
				break;
			case '2':
				std::cout << "Benchmark setup: ours plain (no sorting) (proxy gemometry = aligned box billboard; conservative depth = on; sorting = off; AO = off)" << std::endl;
				SET_MEMBER(render.style.bounding_geometry, textured_spline_tube_render_style::BG_ALIGNED_BOX_BILLBOARD);
				SET_MEMBER(render.style.use_cubic_tangents, true);
				SET_MEMBER(render.style.use_conservative_depth, true);
				SET_MEMBER(debug.sort, false);
				if(!debug.force_initial_order) {
					SET_MEMBER(debug.force_initial_order, true);
					on_set(&debug.force_initial_order);
				}
				SET_MEMBER(ao_style.enable, false);
				on_set(&ao_style.enable);
				handled = true;
				break;
			case '3':
				std::cout << "Benchmark setup: ours plain (sorting) (proxy gemometry = aligned box billboard; conservative depth = on; sorting = on; AO = off)" << std::endl;
				SET_MEMBER(render.style.bounding_geometry, textured_spline_tube_render_style::BG_ALIGNED_BOX_BILLBOARD);
				SET_MEMBER(render.style.use_cubic_tangents, true);
				SET_MEMBER(render.style.use_conservative_depth, true);
				SET_MEMBER(debug.sort, true);
				if(debug.force_initial_order) {
					SET_MEMBER(debug.force_initial_order, false);
					on_set(&debug.force_initial_order);
				}
				SET_MEMBER(ao_style.enable, false);
				on_set(&ao_style.enable);
				handled = true;
				break;
			case '4':
				std::cout << "Benchmark setup: ours full (proxy gemometry = aligned box billboard; conservative depth = on; sorting = on; AO = on)" << std::endl;
				SET_MEMBER(render.style.bounding_geometry, textured_spline_tube_render_style::BG_ALIGNED_BOX_BILLBOARD);
				SET_MEMBER(render.style.use_cubic_tangents, true);
				SET_MEMBER(render.style.use_conservative_depth, true);
				SET_MEMBER(debug.sort, true);
				if(debug.force_initial_order) {
					SET_MEMBER(debug.force_initial_order, false);
					on_set(&debug.force_initial_order);
				}
				SET_MEMBER(ao_style.enable, true);
				on_set(&ao_style.enable);
				handled = true;
				break;
			case 'A':
				ao_style.enable = !ao_style.enable;
				std::cout << "Ambient occlusion: " << (ao_style.enable ? "on" : "off") << std::endl;
				on_set(&ao_style.enable);
				handled = true;
				break;
			case 'B':
				if(modifiers == cgv::gui::EM_CTRL) {
					if(benchmark.running) {
						std::cout << "Aborting benchmark..." << std::endl;
						benchmark.running = false;
						benchmark.requested = false;
					} else {
						std::cout << "Starting benchmark..." << std::endl;
						benchmark.requested = true;
					}
				} else {
					show_bbox = !show_bbox;
					on_set(&show_bbox);
				}
				handled = true;
				break;
			case 'G':
				grid_mode = static_cast<GridMode>((static_cast<int>(grid_mode) + 1) % 4);
				on_set(&grid_mode);
				handled = true;
				break;
		#ifdef RTX_SUPPORT
			case 'H':
				optix.holographic = !optix.holographic;
				on_set(&optix.holographic);
				handled = true;
				break;
		#endif
			case 'N':
				if(navigator_ptr) {
					show_navigator = !show_navigator;
					update_member(&show_navigator);
					navigator_ptr->set_visibility(show_navigator);
					handled = true;
				}
				break;
			case 'M':
				if(cm_viewer_ptr) {
					show_color_map_viewer = !show_color_map_viewer;
					update_member(&show_color_map_viewer);
					cm_viewer_ptr->set_visibility(show_color_map_viewer);
					handled = true;
				}
				break;
			case 'R':
				if (modifiers == 0)
					render.style.radius_scale *= 2.0f;
				else if (modifiers == cgv::gui::EM_SHIFT)
					render.style.radius_scale *= 0.5f;
				on_set(&render.style.radius_scale);
				handled = true;
				break;
			case 'T':
				taa.set_enabled(!taa.is_enabled());
				on_set(&taa);
				handled = true;
				break;
			case 'O':
			#ifdef RTX_SUPPORT
				if (modifiers == cgv::gui::EM_CTRL) {
					optix.enabled = !optix.enabled;
					on_set(&optix.enabled);
					handled = true;
				}
			#endif
				break;
		#ifdef RTX_SUPPORT
			case 'P':
				if (modifiers == cgv::gui::EM_CTRL) {
					optix.unproject_mode_dbg = !optix.unproject_mode_dbg;
					on_set(&optix.holographic); std::cerr << "unproject dbg: "<<optix.unproject_mode_dbg << std::endl;
				}
				else {
					optix.primitive = (OptixPrimitive)((((unsigned)optix.primitive)+1) % 4);
					on_set(&optix.primitive); std::cerr << "primitive type: "<<optix.primitive << std::endl;
				}
				handled = true;
				break;
		#endif
			case 'W':
				show_wireframe_bbox = !show_wireframe_bbox;
				on_set(&show_wireframe_bbox);
				handled = true;
				break;
			case cgv::gui::Keys::KEY_Num_0:
				dataset.rtlola_show_map = !dataset.rtlola_show_map;
				taa.reset();
				handled = true;
				break;
			case cgv::gui::Keys::KEY_Num_Enter:
				{ const bool new_state = render.style.is_tube();
				  ui_state.tr_toggle.control->check_and_set_value(new_state); }
				handled = true;
				break;
			case cgv::gui::Keys::KEY_Space:
				if(modifiers == 0) {
					playback.active = !playback.active;
					on_set(&playback.active);
					handled = true;
				}
				break;
			case cgv::gui::Keys::KEY_Home:
				playback.follow = !playback.follow;
				on_set(&playback.follow);
				handled = true;
				break;
			case cgv::gui::Keys::KEY_Back_Space:
				playback_rewind();
				handled = true;
				break;
			case cgv::gui::Keys::KEY_End:
				playback_reset_ds();
				handled = true;
				break;
			default:
				break;
			}
		}
		else if (ka == cgv::gui::KA_REPEAT && ke.get_key() == cgv::gui::Keys::KEY_Num_0)
		{
			handled = true;
		}

		if(handled) {
			post_redraw();
			return true;
		}
	}

	if(et == cgv::gui::EID_MOUSE) {
		cgv::gui::mouse_event &me = static_cast<cgv::gui::mouse_event&>(e);
		// select drag and drop events only
		if((me.get_flags() & cgv::gui::EF_DND) != 0) switch(me.get_action()) {
		case cgv::gui::MA_ENTER:
		{
			// store (and process) dnd text on enter event, since it's not available in drag events
			dnd.text = me.get_dnd_text();
			std::vector<cgv::utils::line> lines;
			cgv::utils::split_to_lines(dnd.text, lines, true);
			dnd.filenames.reserve(lines.size());
			for(const auto &line : lines)
				dnd.filenames.emplace_back(cgv::utils::to_string(line));
			return true;
		}

		case cgv::gui::MA_DRAG:
		{
			// during dragging keep track of drop position and redraw
			if(auto ctx_ptr = get_context())
				dnd.pos = ivec2(me.get_x(), ctx_ptr->get_height() - me.get_y() - 1);
			post_redraw();
			return true;
		}

		case cgv::gui::MA_LEAVE:
		{
			// when mouse leaves window, cancel the dnd operation (and redraw to clear the dnd indicator
			// onscreen text)
			dnd.text.clear();
			dnd.filenames.clear();
			post_redraw();
			return true;
		}

		case cgv::gui::MA_RELEASE:
		{
			// process the files that where dropped onto the window
			bool try_load_dataset = true;
			if(dnd.filenames.size() == 1) {
				std::string extension = cgv::utils::file::get_extension(dnd.filenames[0]);
				if(cgv::utils::to_upper(extension) == "XML") {
					layer_config_file_helper.file_name = dnd.filenames[0];
					layer_config_file_helper.update();
					on_set(&layer_config_file_helper.file_name);
					try_load_dataset = false;
				}
			}
			if(try_load_dataset) {
				dataset.files.clear();
				for(const std::string &filename : dnd.filenames)
					dataset.files.emplace(filename);
				on_set(&dataset);
			}
			dnd.filenames.clear();
			dnd.text.clear();
			return true;
		}

		default:
			/* DoNothing() */;
		}
	}
	return false;
}

void on_tube_vis::handle_color_map_change() {
	if(cm_editor_ptr) {
		color_map_mgr.update_texture(*get_context());
		if(cm_viewer_ptr)
			cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}
}

void on_tube_vis::handle_transfer_function_change() {
	if(tf_editor_ptr)
		volume_tf.generate_texture(*get_context());
}

void on_tube_vis::handle_member_change(const cgv::utils::pointer_test& m) {

	// control flags
	bool do_full_gui_update = false, data_set_changed = false, from_demo = false, reset_taa = true;

	// internal state flags
	// - configurable datapath
	if(m.is(datapath_helper.file_name))
	{
		const auto& file_name = datapath_helper.file_name;
		if(!file_name.empty())
		{
			from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
			if (traj_mgr.can_load(file_name))
			{
				traj_mgr.clear();
				cgv::utils::stopwatch s(true);
				std::cout << "Reading data set from " << file_name << " ..." << std::endl;

				if(traj_mgr.load(file_name) != -1) {
					std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
					dataset.files.clear();
					dataset.files.emplace(file_name);
					data_set_changed = true;
				}
			}
		}
	}
	// - non-configurable dataset logic
	else if(m.is(dataset)) {
		from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
		// clear current dataset
		datapath_helper.file_name.clear();
		traj_mgr.clear();

		// load new data
		bool loaded_something = false;
		for(const auto& file : dataset.files) {
			cgv::utils::stopwatch s(true);
			std::cout << "Reading data set from " << file << " ..." << std::endl;
			loaded_something = traj_mgr.load(file) != -1 || loaded_something;
			std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
		}
		update_member(&datapath_helper.file_name);

		// update render state
		if(loaded_something)
			data_set_changed = true;
	}

	if(data_set_changed) {
		render.data = &(traj_mgr.get_render_data());
		if(from_demo) {
			ao_style = ao_style_bak;	// reset from handcrafted AO settings
			update_member(&ao_style);
		}
		if(traj_mgr.dataset(0).name().compare("rtlola_droneflight") == 0)
			dataset.is_rtlola = true;

		// print out attribute statistics
		const auto& ds = traj_mgr.dataset(0);
		std::cerr << "Avg. segment length: "<<ds.avg_segment_length() << std::endl
		          << "Data attributes:" << std::endl;
		for(const auto& a : ds.attributes())
			std::cerr << " - [" << a.first << "] - " << a.second.get_timestamps().size() << " samples" << std::endl;
		std::cerr << std::endl;
		// TODO: For some datasets, e.g. fisch_wehr_streamline.0.csv, this results in a much too small value and
		// caps are subsequently clipped at too short distances. The resulting cracks get smoothed out by the anti-
		// aliasing and are thus only hardly noticeable.
		SET_MEMBER(render.style.cap_clip_distance, ds.avg_segment_length() * 20.f);
#ifdef RTX_SUPPORT
		// ###############################
		// ### BEGIN: OptiX integration
		// ###############################

		if(optix.initialized)
			optix_unregister_resources();

		// ###############################
		// ###  END:  OptiX integration
		// ###############################
#endif
		render.style.max_t = render.style.data_t_minmax.second; // <-- make sure we initially display the whole newly loaded dataset
		update_attribute_bindings();
		update_grid_ratios();

		update_glyph_layer_managers();

		compile_glyph_attribs();
		ah_mgr.set_dataset(traj_mgr.dataset(0));

		context& ctx = *get_context();
		tube_shading_defines = build_tube_shading_defines();
		shaders.reload(ctx, "tube_shading", tube_shading_defines);

		// reset glyph layer configuration file
		layer_config_file_helper.file_name = "";
		layer_config_has_unsaved_changes = false;
		layer_config_file_helper.update();
		on_set(&layer_config_has_unsaved_changes);
#ifdef RTX_SUPPORT
		// ###############################
		// ### BEGIN: OptiX integration
		// ###############################

		if(optix.initialized) {
			optix.tracer_russig.update_accelds(render.data);
			optix.tracer_phantom.update_accelds(render.data);
			optix.tracer_builtin.update_accelds(render.data);
			optix.tracer_builtin_cubic.update_accelds(render.data);
			optix_register_resources(ctx);
		}

		// ###############################
		// ###  END:  OptiX integration
		// ###############################
#endif
		do_full_gui_update = true;
	}

	// render settings
	if(m.one_of(debug.highlight_segments,
				ao_style.enable,
				grid_mode,
				grid_normal_settings,
				grid_normal_inwards,
				grid_normal_variant,
				enable_fuzzy_grid)) {
		shader_define_map defines = build_tube_shading_defines();
		if(defines != tube_shading_defines) {
			context& ctx = *get_context();
			tube_shading_defines = defines;
			shaders.reload(ctx, "tube_shading", tube_shading_defines);
		}
	}

	// - debug render setting
	if(m.is(debug.force_initial_order)) {
		update_attribute_bindings();
	}

	if(m.is(debug.render_percentage)) {
		debug.render_count = static_cast<size_t>(debug.render_percentage * debug.segment_count);
		update_member(&debug.render_count);
	}

	if(m.is(debug.render_count)) {
		debug.render_percentage = static_cast<float>(debug.render_count) / static_cast<float>(debug.segment_count);
		update_member(&debug.render_percentage);
	}

	if(m.is(debug.render_mode)) {
		do_full_gui_update = true;
		update_debug_attribute_bindings();
	}

	// voxelization settings
	if(m.one_of(voxel_grid_resolution, voxelize_gpu, render.style.radius_scale)) {
		context& ctx = *get_context();
		voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(cgv::math::clamp(static_cast<unsigned>(voxel_grid_resolution), 16u, 512u));
		create_density_volume(ctx, voxel_grid_resolution);

		if(m.is(render.style.radius_scale)) {
			update_grid_ratios();
			glyphs_out_of_date(true);
		}
	}

	// visualization settings
	if(m.is(render.visualizations.front().manager)) {
		auto& glyph_layer_mgr = render.visualizations.front().manager;
		auto& glyph_layers_config = render.visualizations.front().config;
		const auto action = glyph_layer_mgr.action_type();
		bool changes = false;
		if(action == AT_CONFIGURATION_CHANGE) {
			glyph_layers_config = glyph_layer_mgr.get_configuration();

			context& ctx = *get_context();
			tube_shading_defines = build_tube_shading_defines();
			shaders.reload(ctx, "tube_shading", tube_shading_defines);

			compile_glyph_attribs();

			changes = true;
			do_full_gui_update = true;
		} else if(action == AT_CONFIGURATION_VALUE_CHANGE) {
			glyph_layers_config = glyph_layer_mgr.get_configuration();
			glyphs_out_of_date(true);
			changes = true;
		} else if(action == AT_MAPPING_VALUE_CHANGE) {
			glyphs_out_of_date(true);
			changes = true;
		}

		if(changes) {
			layer_config_has_unsaved_changes = true;
			on_set(&layer_config_has_unsaved_changes);
			update_legends = true;
		}
	}

	if(m.is(color_map_mgr)) {
		switch(color_map_mgr.action_type()) {
		case AT_CONFIGURATION_CHANGE:
		{
			if(cm_editor_ptr) {
				const auto& color_maps = color_map_mgr.ref_color_maps();
				const auto* edit_cm_ptr = cm_editor_ptr->get_color_map();
				for(size_t i = 0; i < color_maps.size(); ++i) {
					if(&color_maps[i].cm == edit_cm_ptr) {
						// TODO: use smart pointers for color map and in manager
						cm_editor_ptr->set_color_map(nullptr);
						cm_editor_ptr->set_visibility(false);
					}
				}
			}
			// TODO: remove commented line
			render.visualizations.front().variables->set_color_map_names(color_map_mgr.get_names());

			color_map_mgr.update_texture(*get_context());
			if(cm_viewer_ptr) {
				cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
				cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
			}

			do_full_gui_update = true;
			break;
		}
		case AT_EDIT_REQUEST:
			if(cm_editor_ptr) {
				int idx = color_map_mgr.edit_index();
				if(idx > -1 && idx < color_map_mgr.ref_color_maps().size()) {
					cm_editor_ptr->set_color_map(&(color_map_mgr.ref_color_maps()[idx].cm));
					cm_editor_ptr->set_visibility(true);
				}
			}
			break;
		default: break;
		}
		// TODO: add case for value change action type
		layer_config_has_unsaved_changes = true;
		on_set(&layer_config_has_unsaved_changes);
	}

	if(m.is(layer_config_file_helper.file_name)) {
		std::string& file_name = layer_config_file_helper.file_name;

		if(layer_config_file_helper.save()) {
			layer_config_file_helper.ensure_extension("xml");

			if(layer_config_file_helper.compare_extension("xml")) {
				if(save_layer_configuration(file_name)) {
					layer_config_has_unsaved_changes = false;
					on_set(&layer_config_has_unsaved_changes);
				} else {
					std::cout << "Error: Could not write glyph_ layer configuration to file: " << file_name << std::endl;
				}
			} else {
				std::cout << "Please specify a xml file name." << std::endl;
			}
		} else {
			/*
			#ifndef CGV_FORCE_STATIC
					// TODO: implemenmt
					std::cout << "IMPLEMENT" << std::endl;
					//std::filesystem::path file_path(file_name);
					//if(file_path.is_relative()) {
					//	std::string debug_file_name = QUOTE_SYMBOL_VALUE(INPUT_DIR);
					//	file_name = debug_file_name + "/" + file_name;
					//}
			#endif
			*/

			std::string extension = cgv::utils::file::get_extension(file_name);
			// only try to read the filename if it ends with an xml extension
			if(layer_config_file_helper.compare_extension("xml")) {
				if(read_layer_configuration(file_name)) {
					layer_config_has_unsaved_changes = false;
					on_set(&layer_config_has_unsaved_changes);
				} else {
					std::cout << "Error: could not read glyph layer configuration from " << file_name << std::endl;
				}
			}
		}

	}

	if(m.is(layer_config_has_unsaved_changes)) {
		auto ctrl = find_control(layer_config_file_helper.file_name);
		if(ctrl)
			ctrl->set("text_color", layer_config_has_unsaved_changes ? cgv::gui::theme_info::instance().warning_hex() : "");
	}

	if(m.is(debug.show_hidden_glyphs))
		compile_glyph_attribs();

	// playback controls
	if(m.is(playback.active) && playback.active) {
		playback.timer.add_time();
		render.style.max_t =
			render.style.max_t >= (float)playback.tend ? (float)playback.tstart : render.style.max_t;
		const auto& ds = traj_mgr.dataset(0);
		const auto& pos = ds.positions();
		const auto& traj_range = ds.trajectories(pos.attrib)[playback.follow_traj];
		playback.follow_last_nid = find_sample(pos.attrib, traj_range, render.style.max_t);
	}
	if(m.is(playback.follow_traj)) {
		const auto& ds = traj_mgr.dataset(0);
		const auto& pos = ds.positions();
		const auto& traj_range = ds.trajectories(pos.attrib)[playback.follow_traj];
		playback.follow_last_nid = find_sample(pos.attrib, traj_range, render.style.max_t);
	}

	// widget controls
	if(m.is(show_mapping_legend)) {
		if(mapping_legend_ptr)
			mapping_legend_ptr->set_visibility(show_mapping_legend);
	}
	
	if(m.is(show_color_map_viewer)) {
		if(cm_viewer_ptr)
			cm_viewer_ptr->set_visibility(show_color_map_viewer);
	}

	if(m.is(show_navigator)) {
		if(navigator_ptr)
			navigator_ptr->set_visibility(show_navigator);
	}

	if(m.is(show_performance_monitor)) {
		if(perfmon_ptr)
			perfmon_ptr->set_visibility(show_performance_monitor);
	}

	// misc settings
	// - instant redraw
	if(m.is(misc_cfg.instant_redraw_proxy))
		// ToDo: handle the (virtually impossible) case that some other plugin than cg_fltk provides the gl_context
		dynamic_cast<fltk_gl_view*>(get_context())->set_void("instant_redraw", "bool", m.ptr);
	// - vsync
	if(m.is(misc_cfg.vsync_proxy))
		// ToDo: handle the (virtually impossible) case that some other plugin than cg_fltk provides the gl_context
		dynamic_cast<fltk_gl_view*>(get_context())->set_void("vsync", "bool", m.ptr);
	// - fix view up dir
	else if(m.is(misc_cfg.fix_view_up_dir_proxy))
		dynamic_cast<node*>(find_view_as_node())->set("fix_view_up_dir", misc_cfg.fix_view_up_dir_proxy);

	// In case of timestep thresholding we don't want to reset TAA
	if(m.is(render.style.max_t))
		reset_taa = false;

	if(m.is(render.style.line_primitive))
	{
		// perform smart toggle bookkeeping
		if (!ui_state.tr_toggle.check_toggled()) {
			if (render.style.is_tube())
				ui_state.tr_toggle.last_tube_primitive = render.style.line_primitive;
			else
				ui_state.tr_toggle.last_ribbon_primitive = render.style.line_primitive;
		}
		reset_taa = true;
	}

#ifdef RTX_SUPPORT
	// ###############################
	/* ### BEGIN: OptiX integration */ {
	// ###############################

	const auto push_optix_holo_taa_state = [this]() {
		optix.prev_TAA_state = taa.is_enabled();
		taa.set_enabled(false);
		context& ctx = *get_context();
		const ivec2 fbsize(ctx.get_width() * 3, ctx.get_height());
		fbc.set_size(fbsize);
		fbc.ensure(ctx);
		optix.fb.depth.set_resolution(0, fbsize.x());
		optix.fb.depth.set_resolution(1, fbsize.y());
		optix.fb.depth.ensure_state(ctx);
	};
	const auto pop_optix_holo_taa_state = [this]() {
		taa.set_enabled(optix.prev_TAA_state);
		context& ctx = *get_context();
		const ivec2 fbsize(ctx.get_width() * 3, ctx.get_height());
		fbc.set_size({ (int)ctx.get_width(), (int)ctx.get_height() });
		fbc.ensure(ctx);
		optix.fb.depth.set_resolution(0, fbsize.x());
		optix.fb.depth.set_resolution(1, fbsize.y());
		optix.fb.depth.ensure_state(ctx);
	};

	if(m.member_of(taa)) {
		optix.prev_TAA_state = taa.is_enabled();
	} else if(m.is(optix.enabled)) {
		do_full_gui_update = true;
		if(optix.enabled) {
			if(optix_ensure_init(*get_context())) {
				if(optix.holographic)
					push_optix_holo_taa_state();
			} else
				optix.enabled = false;
		} else {
			pop_optix_holo_taa_state();
		}
	} else if(m.is(optix.primitive)) {
		if(optix.primitive == OPR_PHANTOM)
			optix.tracer = &optix.tracer_phantom;
		else if(optix.primitive == OPR_BUILTIN)
			optix.tracer = &optix.tracer_builtin;
		else if(optix.primitive == OPR_BUILTIN_CUBIC)
			optix.tracer = &optix.tracer_builtin_cubic;
		else
			optix.tracer = &optix.tracer_russig;
	} else if(m.is(optix.holographic) && optix.enabled) {
		if(optix.holographic)
			push_optix_holo_taa_state();
		else
			pop_optix_holo_taa_state();
		do_full_gui_update = true;
	}

	// ###############################
	/* ###  END:  OptiX integration */ }
	// ###############################
#endif

	// default implementation for all members
	// - update GUI
	if(do_full_gui_update)
		post_recreate_gui();
	// - reset TAA
	if(reset_taa)
		taa.reset();
	else
		taa.reset_static_frame_count(); // Just make sure we keep multisampling
}

bool on_tube_vis::on_exit_request() {
	// TODO: does not seem to fire when window is maximized?
#ifndef _DEBUG
	if(layer_config_has_unsaved_changes) {
		return cgv::gui::question("The glyph layer configuration has unsaved changes. Are you sure you want to quit?");
	}
#endif
	return true;
}

bool on_tube_vis::save_layer_configuration(const std::string& file_name) {

	const auto& visualization = render.visualizations.front();

	return layer_configuration_io::write_layer_configuration(file_name, visualization.variables, visualization.manager, color_map_mgr);
}

bool on_tube_vis::read_layer_configuration(const std::string& file_name) {

	auto& visualization = render.visualizations.front();

	if(layer_configuration_io::read_layer_configuration(file_name, visualization.variables, visualization.manager, color_map_mgr)) {
		// update the dependent members
		color_map_mgr.update_texture(*get_context());
		if(cm_viewer_ptr) {
			cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
			cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
		}

		visualization.manager.notify_configuration_change();
		return true;
	}

	return false;
}

void on_tube_vis::update_glyph_layer_managers() {
	if(!traj_mgr.has_data()) {
		std::cout << "Warning: update_glyph_layer_managers - trajectory manager has no data" << std::endl;
		return;
	}

	render.visualizations.clear();
	for (unsigned i=0; i< traj_mgr.num_datasets(); i++)
	{
		const auto& dataset = traj_mgr.dataset(i);
		auto attrib_names = dataset.get_attribute_names();
		std::vector<vec2> attrib_ranges;

		// collect value ranges for available attributes
		for(size_t i = 0; i < attrib_names.size(); ++i) {
			const auto& attrib = traj_mgr.dataset(0).attribute(attrib_names[i]);
			vec2 range(attrib.min(), attrib.max());
			attrib_ranges.push_back(range);
		}

		auto &new_layer_config = render.visualizations.emplace_back(this);
		// set new information of available attributes and ranges
		new_layer_config.variables->set_attribute_names(attrib_names);
		new_layer_config.variables->set_attribute_ranges(attrib_ranges);
		new_layer_config.variables->set_color_map_names(color_map_mgr.get_names());
		// clear old configuration of glyph layers and reset shader
		new_layer_config.manager.clear();
		// set visualization variable information in glyph layer manager
		new_layer_config.manager.set_visualization_variables(new_layer_config.variables);
	}
}

void on_tube_vis::timer_event(double t, double dt)
{
	if (has_changed && t > change_time + recalc_delay)
		compile_glyph_attribs();
}

void on_tube_vis::glyphs_out_of_date(bool state) 
{
	auto ctrl = find_element("Compile Attributes");
	if (state) {
		change_time = cgv::gui::get_animation_trigger().get_current_time();
		has_changed = true;
	}
	else {
		has_changed = false;
	}
	if(ctrl)
		ctrl->set("color", state ? cgv::gui::theme_info::instance().warning_hex() : "");
}

bool on_tube_vis::compile_glyph_attribs (void)
{
	bool success = false;
	for (unsigned ds_idx=0; ds_idx<traj_mgr.num_datasets(); ds_idx++)
	{
		const auto &ds = traj_mgr.dataset(ds_idx);
		auto &ds_config = render.visualizations[ds_idx];
		if(ds_config.config.layer_configs.size() > 0)
		{
			cgv::utils::stopwatch s(true);
			std::cout << "Compiling glyph attributes for dataset "<<ds_idx<<" '"<<ds.name()<<"'... ";

			glyph_compiler gc;
			gc.length_scale = render.style.length_scale;
			gc.include_hidden_glyphs = debug.show_hidden_glyphs;

			const auto &dataset = traj_mgr.dataset(0);

			success = gc.compile_glyph_attributes(dataset, render.arclen_data, ds_config.config);

			// get context
			const auto &ctx = *get_context();

			for(size_t layer_idx = 0; layer_idx < gc.layer_filled.size(); ++layer_idx) {
				if(gc.layer_filled[layer_idx]) {
					const auto& ranges = gc.layer_ranges[layer_idx];
					const auto& attribs = gc.layer_attribs[layer_idx];
					// - sanity check
					{
						const float num_ranges = (float)ranges.size(), num_segs = float(render.data->indices.size()) / 2;
						assert(num_ranges == num_segs);
					}

					// - upload
					vertex_buffer& attribs_sbo = render.attribs_sbos[layer_idx];
					vertex_buffer& aindex_sbo = render.aindex_sbos[layer_idx];

					// ...attrib nodes
					attribs_sbo.destruct(ctx);
					if(!attribs_sbo.create(ctx, attribs.data))
						std::cerr << "!!! unable to create glyph attribute Storage Buffer Object !!!" << std::endl << std::endl;

					// ...index ranges
					aindex_sbo.destruct(ctx);
					if(!aindex_sbo.create(ctx, ranges))
						std::cerr << "!!! unable to create glyph index ranges Storage Buffer Object !!!" << std::endl << std::endl;
				}
			}

			std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
			glyphs_out_of_date(false);
		}
	}

	if(success) {
		taa.reset();
		post_redraw();
	}

	return success;
}

bool on_tube_vis::init (cgv::render::context &ctx)
{
	// generate demo dataset
	// - demo AO settings
	ao_style_bak = ao_style;
	ao_style.strength_scale = 15.0f;
	update_member(&ao_style);
	// - demo geometry
	constexpr unsigned seed = 11;
#ifdef _DEBUG
	constexpr unsigned num_trajectories = 3;
	constexpr unsigned num_nodes = 16;
#else
	constexpr unsigned num_trajectories = 256; // 1
	constexpr unsigned num_nodes = 256; // 32
#endif
	for (unsigned i=0; i<num_trajectories; i++)
		dataset.demo_trajs.emplace_back(demo::gen_trajectory(num_nodes, seed+i));
	traj_mgr.add_dataset(
		demo::compile_dataset(dataset.demo_trajs)
	);
	// - print out attribute statistics
	std::cerr << "Data attributes:" << std::endl;
	for (const auto& a : traj_mgr.dataset(0).attributes())
		std::cerr << " - ["<<a.first<<"] - "<<a.second.get_timestamps().size()<<" samples" << std::endl;
	std::cerr << std::endl;

	// increase reference count of the renderers by one
	auto &tstr = ref_textured_spline_tube_renderer(ctx, 1);
	auto &vr = ref_volume_renderer(ctx, 1);
	bool success = tstr.ref_prog().is_linked() && vr.ref_prog().is_linked();

	// load all shaders in the library
	success &= shaders.load_all(ctx);

	// prepare render-time dataset state
	for (const auto &ds : traj_mgr.datasets()) {
		auto &vis = render.visualizations.emplace_back(this);
		vis.config = vis.manager.get_configuration();
	}

	tube_shading_defines = build_tube_shading_defines();
	shaders.reload(ctx, "tube_shading", tube_shading_defines);

	// init shared attribute array manager
	success &= render.aam.init(ctx);

	success &= density_volume.init(ctx, 0);

	render.sorter.set_data_type_override("vec4 pos_rad; vec4 color; vec4 tangent; vec4 t;");
	render.sorter.set_auxiliary_type_override("uint a_idx; uint b_idx;");

	std::string key_definition =
		R"(aux_type indices = aux_values[idx]; \
		data_type a = data[indices.a_idx]; \
		data_type b = data[indices.b_idx]; \
		vec3 pa = a.pos_rad.xyz; \
		vec3 pb = b.pos_rad.xyz; \
		\
		vec3 x = 0.5*(pa + pb); \
		vec3 eye_to_pos = x - eye_pos; \
		float ddv = dot(eye_to_pos, view_dir); \
		float key = (ddv < 0.0 ? -1.0 : 1.0) * dot(eye_to_pos, eye_to_pos);)";

	render.sorter.set_key_definition_override(key_definition);

	// Initialize the last sort position and direction to zero to force a sorting step before the first draw call
	last_sort_pos = vec3(0.0f);
	last_sort_dir = vec3(0.0f);

	bbox_rd.init(ctx);
	bbox_wire_rd.init(ctx);

	// initialize debug geometry
	debug.geometry.nodes.init(ctx);
	debug.geometry.segments.init(ctx);

	// enable ambient occlusion
	ao_style.enable = true;

	// init data-dependent render state
	update_attribute_bindings();
	update_grid_ratios();

	// init color maps
	// - manager
	color_map_mgr.init(ctx);

	auto load_color_maps = [this](const std::string& file_name) {
		cgv::app::color_map_reader::result color_maps;
		if(cgv::app::color_map_reader::read_from_xml_file(file_name, color_maps))
			for(const auto& entry : color_maps)
				color_map_mgr.add_color_map(entry.first, entry.second, false);
	};

	auto load_color_maps_from_directory = [this, &load_color_maps](const std::string& dir_name) {
		if(std::filesystem::exists(dir_name)) {
			for(const auto& entry : std::filesystem::directory_iterator(dir_name)) {
				std::filesystem::path entry_path = entry.path();
				// only take xml files
				if(entry_path.extension() == ".xml")
					load_color_maps(entry_path.string());
			}
		}
	};

	// - load sequential and diverging color maps from the resource directory
	load_color_maps_from_directory("res/color_maps/sequential");
	load_color_maps_from_directory("res/color_maps/diverging");
	
	// - load sequential rainbow and turbo color maps
	load_color_maps("res/color_maps/rainbow.xml");
	load_color_maps("res/color_maps/turbo.xml");

	color_map_mgr.update_texture(ctx);
	if(cm_viewer_ptr) {
		cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
		cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	update_glyph_layer_managers();
	compile_glyph_attribs();
	ah_mgr.set_dataset(traj_mgr.dataset(0));

	volume_tf.init(ctx);

	// initialize temporal anti-aliasing
	success &= taa.init(ctx);
	taa.set_jitter_sample_count(8);
	taa.set_mix_factor(0.125f);
	taa.set_fxaa_mix_factor(0.5f);

	// use white background for paper screenshots
	//ctx.set_bg_color(1.0f, 1.0f, 1.0f, 1.0f);

	////
	// RTLola drone flight demo dataset map

	// vertex structure for interleaved storage
	struct rtlola_map_vertex {
		vec4 pos;
		vec4 color;
		vec2 texcoord;
	};

	// vertex buffer element descriptors for each of our struct member types
	const auto vec4desc = element_descriptor_traits<vec4>::get_type_descriptor(vec4());
	const auto vec2desc = element_descriptor_traits<vec2>::get_type_descriptor(vec2());

	// setup VBO and VAO
	const shader_program &default_shader = ctx.ref_default_shader_program(true /* <-- texture support */);
	success &= dataset.rtlola_map_vao.create(ctx);
	success &= dataset.rtlola_map_vbo.create(ctx, std::vector<rtlola_map_vertex>{
		{{-2544.4730907677954f, 581.5f, -837.50963767276119f, 1.f}, {1.f}, {0.f, 0.f}},
		{{2004.4304399613477f, 581.5f, -837.50963767276119f, 1.f}, {1.f}, {1.f, 0.f}},
		{{-2544.4730907677954f, 581.5f, 2506.1411020040873f, 1.f}, {1.f}, {0.f, 1.f}},
		{{2004.4304399613477f, 581.5f, 2506.1411020040873f, 1.f}, {1.f}, {1.f, 1.f}}
	});
	success &= dataset.rtlola_map_vao.bind_attribute_array(
		ctx, default_shader, "position", vec4desc,
		dataset.rtlola_map_vbo, 0, 4, sizeof(rtlola_map_vertex)
	);
	success &= dataset.rtlola_map_vao.bind_attribute_array(
		ctx, default_shader, "color", vec4desc,
		dataset.rtlola_map_vbo, sizeof(vec4), 4, sizeof(rtlola_map_vertex)
	);
	success &= dataset.rtlola_map_vao.bind_attribute_array(
		ctx, default_shader, "texcoord", vec2desc,
		dataset.rtlola_map_vbo, 2*sizeof(vec4), 4, sizeof(rtlola_map_vertex)
	);

	// load map texture
#ifdef CGV_FORCE_STATIC
	// TODO: why is loading embedded image files in case of single-exe builds not handled automatically?
	success &= dataset.rtlola_map_tex.create_from_image(ctx, "res://rtlola_droneflight.png");
#else
	success &= dataset.rtlola_map_tex.create_from_image(ctx, "res/rtlola_droneflight.png");
#endif


#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	if (optix.enabled && !optix_ensure_init(ctx)) {
		optix.enabled = false;
		update_member(&optix.enabled);
		success = false;
	}

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif


	// done
	return success;
}

#ifdef RTX_SUPPORT
// ###############################
// ### BEGIN: OptiX integration
// ###############################

void on_tube_vis::optix_cleanup (void)
{
	optix.tracer_russig.destroy();
	optix.tracer_phantom.destroy();
	optix.tracer_builtin.destroy();
	optix.tracer_builtin_cubic.destroy();
	CUDA_SAFE_DESTROY_STREAM(optix.stream);
}

void on_tube_vis::optix_unregister_resources (void)
{
	CUDA_SAFE_UNREGISTER(optix.sbo_alen);
	CUDA_SAFE_UNREGISTER(optix.sbo_nodeids);
	CUDA_SAFE_UNREGISTER(optix.sbo_nodes);
}

bool on_tube_vis::optix_ensure_init (context &ctx)
{
	// report OK if nothing needs to be done
	if (optix.initialized)
		return true;

	// initialize CUDA
	CUDA_CHECK_FAIL(cudaFree(0));

	// initialize OptiX
	OPTIX_CHECK_FAIL(optixInit());

	// specify OptiX device context options
	OptixDeviceContextOptions options = {};
	options.logCallbackFunction = &optix_log_cb;
	options.logCallbackLevel = 4;
	std::cerr << std::endl; // <-- Make sure the initial CUDA/OptiX message stream is preceded by an empty line

	// associate a CUDA context (and therefore a specific GPU) with this device context
	bool success = true;
	OPTIX_CHECK_SET(
		optixDeviceContextCreate(
			0, // zero means take the default context (i.e. first best compatible device)
			&options, &optix.context
		),
		success
	);
	if (!success)
		return false;

	// setup optix launch environment
	if (traj_mgr.has_data()) {
		optix.tracer_russig = optixtracer_textured_spline_tube_russig::build(optix.context, render.data);
		success = success && optix.tracer_russig.built();
		optix.tracer_phantom = optixtracer_textured_spline_tube_phantom::build(optix.context, render.data);
		success = success && optix.tracer_phantom.built();
		optix.tracer_builtin = optixtracer_textured_spline_tube_builtin::build(optix.context, render.data);
		success = success && optix.tracer_builtin.built();
		optix.tracer_builtin_cubic = optixtracer_textured_spline_tube_builtincubic::build(optix.context, render.data);
		success = success && optix.tracer_builtin_cubic.built();
		success = success && optix_register_resources(ctx);
	}
	const int w = ctx.get_width() * (optix.holographic ? 3 : 1);
	success = success && optix.outbuf_albedo.reset(CUOutBuf::GL_INTEROP, w, ctx.get_height());
	success = success && optix.outbuf_position.reset(CUOutBuf::GL_INTEROP, w, ctx.get_height());
	success = success && optix.outbuf_normal.reset(CUOutBuf::GL_INTEROP, w, ctx.get_height());
	success = success && optix.outbuf_tangent.reset(CUOutBuf::GL_INTEROP, w, ctx.get_height());
	success = success && optix.outbuf_depth.reset(CUOutBuf::GL_INTEROP, w, ctx.get_height());

	// create CUDA operations stream
	CUDA_CHECK_FAIL(cudaStreamCreate(&optix.stream));
	optix.outbuf_albedo.set_stream(optix.stream);
	optix.outbuf_position.set_stream(optix.stream);
	optix.outbuf_normal.set_stream(optix.stream);
	optix.outbuf_tangent.set_stream(optix.stream);
	optix.outbuf_depth.set_stream(optix.stream);

	// connect to framebuffer
	fbc.ensure(ctx);
	optix.fb.albedo = fbc.attachment_texture_ptr("albedo");
	optix.fb.position = fbc.attachment_texture_ptr("position");
	optix.fb.normal = fbc.attachment_texture_ptr("normal");
	optix.fb.tangent = fbc.attachment_texture_ptr("tangent");

	// done!
	optix.initialized = success;
	return success;
}

bool on_tube_vis::optix_register_resources (context &ctx)
{
	// unregister previous version (when our function is called, the SSBOs were very likely exchanged for new ones)
	optix_unregister_resources();

	// track success - we don't immediately fail and return since the code in this function is not robust to failure
	// (i.e. doesn't use RAII) so doing that could potentially result in both host and device memory leaks
	bool success = true;

	// register the SSBOs with CUDA
	// - node data
	const GLuint nodes_handle = (const int&)render.render_sbo.handle-1;
	CUDA_CHECK_SET(
		cudaGraphicsGLRegisterBuffer(&optix.sbo_nodes, nodes_handle, cudaGraphicsRegisterFlagsReadOnly),
		success
	);
	// - node indices
	const auto nodeids_vbo = ref_textured_spline_tube_renderer(ctx).get_vertex_buffer_ptr(ctx, render.aam, "node_ids");
	CUDA_CHECK_SET(
		cudaGraphicsGLRegisterBuffer(&optix.sbo_nodeids, (const int&)(nodeids_vbo->handle)-1, cudaGraphicsRegisterFlagsReadOnly),
		success
	);
	// - arclength
	const GLuint alen_handle = (const int&)render.arclen_sbo.handle-1;
	CUDA_CHECK_SET(
		cudaGraphicsGLRegisterBuffer(&optix.sbo_alen, alen_handle, cudaGraphicsRegisterFlagsReadOnly),
		success
	);

	// done!
	return success;
}

void on_tube_vis::optix_draw_trajectories (context &ctx)
{
	// constants
	const vec4 eyespace_origin(0, 0, 0, 1);


	////
	// Launch OptiX

	/* local scope */ {
		// instantiante RT params
		curve_rt_params params;

		// prepare camera info
		const auto  sview = (stereo_view*)view_ptr;
		const mat4  &MV    = MAT4(params.cam_MV) = ctx.get_modelview_matrix(),
		            &P     = MAT4(params.cam_P) = ctx.get_projection_matrix(),
		            &invMV = MAT4(params.cam_invMV) = cgv::math::inv(MV),
		            &invP  = MAT4(params.cam_invP) = cgv::math::inv(P);
		const float aspect = (float)ctx.get_width()/ctx.get_height(),
		            stereo_eye_dist = (float)sview->get_eye_distance()/*,
		            optixV_len = (float)view_ptr->get_tan_of_half_of_fovy(true),
		            optixU_len = optixV_len * aspect*/;
		const vec2  screen_ext((float)sview->get_y_extent_at_focus()*aspect, (float)sview->get_y_extent_at_focus());
		const vec3  cyclops_worldspace = sview->get_eye(), eye = vec3_from_vec4h(invMV * eyespace_origin);
		const mat4  MV_cam = cgv::math::look_at4<float>(
		            	cyclops_worldspace, sview->get_focus(), sview->get_view_up_dir()
		            )/*,
		            optixW = cgv::math::normalize(view_ptr->get_view_dir()),
		            optixV = cgv::math::normalize(view_ptr->get_view_up_dir()) * optixV_len,
		            optixU = cgv::math::normalize(cgv::math::cross(optixW, optixV)) * optixU_len*/;

		// helper function (careful - they need some fields of 'params' to be preset before being called)
		const auto stereo_projection_matrix = [&params, &screen_ext, stereo_eye_dist] (float e, const mat4 &P_base) -> mat4 {
			/*mat4 ret = P_base;
			ret(0, 2) = -e * stereo_eye_dist;
			return ret;
			*/return
				cgv::math::stereo_frustum_screen4<double>(
					e, stereo_eye_dist, screen_ext.x(), screen_ext.y(), params.parallax_zero_depth,
					params.cam_clip.x, params.cam_clip.y
				);
		};
		const auto stereo_modelview_matrix = [&ctx, &params, &MV_cam, &screen_ext, stereo_eye_dist]
		                                     (float e) -> mat4 {
			ctx.push_modelview_matrix();
			ctx.set_modelview_matrix(cgv::math::identity4<double>());
			ctx.mul_modelview_matrix(cgv::math::stereo_translate_screen4<float>(e, stereo_eye_dist, screen_ext.x()));
			ctx.mul_modelview_matrix(MV_cam);
			const mat4 ret = ctx.get_modelview_matrix();
			ctx.pop_modelview_matrix();
			return ret;
		};

		// setup params for our launch
		// - obtain values from selected tracer
		const auto &lp = optix.tracer->ref_launch_params();
		// - map our shared SSBOs (ToDo: include output surfaces in this batched mapping call)
		cudaGraphicsResource_t inbufs[] = {optix.sbo_nodes, optix.sbo_nodeids, optix.sbo_alen};
		CUDA_CHECK(cudaGraphicsMapResources(3, inbufs, optix.stream));
		// - set values
		
		params.nodes = nullptr; {
			size_t size;
			CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void**>(&params.nodes), &size, optix.sbo_nodes));
		};
		params.node_ids = nullptr; {
			size_t size;
			CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void**>(&params.node_ids), &size, optix.sbo_nodeids));
		};
		params.alen = nullptr; {
			size_t size;
			CUDA_CHECK(cudaGraphicsResourceGetMappedPointer(reinterpret_cast<void**>(&params.alen), &size, optix.sbo_alen));
		};
		params.positions = lp.positions;
		params.radii = lp.radii;
		params.albedo = optix.outbuf_albedo.map();
		params.position = optix.outbuf_position.map();
		params.normal = optix.outbuf_normal.map();
		params.tangent = optix.outbuf_tangent.map();
		params.depth = optix.outbuf_depth.map();
		params.cubic_tangents = render.style.use_cubic_tangents;
		params.max_t = render.style.max_t;
		params.taa_subframe_id = taa.is_enabled() ? taa.get_current_accumulation_count() : 0;
		params.taa_jitter_scale = taa.get_jitter_scale();
		params.viewport_dims = { ctx.get_width(), ctx.get_height() };
		params.framebuf_dims = {
			params.viewport_dims.x * (optix.holographic ? 3 : 1),
			params.viewport_dims.y
		};
		params.show_bvol = optix.debug_bvol;
		params.accelds = lp.accelds;
		params.cam_eye = to_float3(cyclops_worldspace);
		/*{ const auto cyclops = MV.mul_pos(cyclops_worldspace);
		  params.cam_cyclops_eyespace = to_float3h(cyclops); }*/
		params.cam_clip = {znear_from_proj4(P), zfar_from_proj4(P)};
		MAT4(params.cam_N) = cgv::math::transpose(invMV);
		params.holo = (Holo)optix.holographic;
		params.unproject_mode_dbg = optix.unproject_mode_dbg;
		params.screen_size = to_float2(screen_ext);
		params.holo_eye = optix.holo_eye;
		params.holo_eyes_dist = stereo_eye_dist;
		params.parallax_zero_depth = (float)sview->get_parallax_zero_depth();
		MAT4(params.holo_MV_left)  = stereo_modelview_matrix(-1.f);
		MAT4(params.holo_invMV_left) = cgv::math::inv(MAT4(params.holo_MV_left));
		MAT4(params.holo_MV_right) = stereo_modelview_matrix( 1.f);
		MAT4(params.holo_invMV_right) = cgv::math::inv(MAT4(params.holo_MV_right));
		mat4 &Pl = MAT4(params.holo_P_left)   = stereo_projection_matrix(-1.f, P);
		MAT4(params.holo_invP_left) = cgv::math::inv(MAT4(params.holo_P_left));
		mat4 &Pr = MAT4(params.holo_P_right)  = stereo_projection_matrix( 1.f, P);
		mat4 PC = stereo_projection_matrix(0, P), PCi = cgv::math::lerp(Pl, Pr, .5f);
		//const_cast<mat4&>(P) = PCi; const_cast<mat4&>(invP) = cgv::math::inv(P);
		/*mat4 invPl = cgv::math::inv(Pl), invPC = cgv::math::inv(PC), invPr = cgv::math::inv(Pr);
		std::cerr << "=======================================================================================" << std::endl;
		std::cerr << "P:" << std::endl << P << std::endl;
		std::cerr << "PC:" << std::endl << PC << std::endl;
		std::cerr << "PCi:" << std::endl << PCi << std::endl;
		std::cerr << " -----------------------------------------------" << std::endl;
		std::cerr << "Pl:" << std::endl << Pl << std::endl;
		std::cerr << "Pr:" << std::endl << Pr << std::endl;
		std::cerr << " -----------------------------------------------" << std::endl;
		std::cerr << "invPl:" << std::endl << invPl << std::endl;
		std::cerr << "invPC:" << std::endl << invPC << std::endl;
		std::cerr << "invPr:" << std::endl << invPr << std::endl;
		std::cerr << "=======================================================================================" << std::endl;*/
		MAT4(params.holo_invP_right) = cgv::math::inv(MAT4(params.holo_P_right));
		MAT4(params.holo_invMVP_left) = cgv::math::inv(MAT4(params.holo_P_left)) * MAT4(params.holo_MV_left);
		MAT4(params.holo_invMVP_right) = cgv::math::inv(MAT4(params.holo_P_right)) * MAT4(params.holo_MV_right);
		/*params.holo_eye_left = MAT4(params.holo_invMV_left) * 
		params.holo_eye_right = MAT4(params.holo_invMV_right) * */
		// - upload to device
		CUDA_CHECK(cudaMemcpy(reinterpret_cast<void *>(lp.params), &params, lp.params_size, cudaMemcpyHostToDevice));

		// Launch!
		OPTIX_CHECK(optixLaunch(
			lp.pipeline, optix.stream, lp.params, lp.params_size, lp.sbt,
			params.framebuf_dims.x, params.framebuf_dims.y, /*depth=*/1
		));
		CUDA_SYNC_CHECK();

		// clean up
		optix.outbuf_depth.unmap();
		optix.outbuf_tangent.unmap();
		optix.outbuf_normal.unmap();
		optix.outbuf_position.unmap();
		optix.outbuf_albedo.unmap();
		CUDA_CHECK(cudaGraphicsUnmapResources(3, inbufs, optix.stream));

		// transfer OptiX render result into our result texture
		optix.outbuf_albedo.into_texture(ctx, optix.fb.albedo);
		optix.outbuf_position.into_texture(ctx, optix.fb.position);
		optix.outbuf_normal.into_texture(ctx, optix.fb.normal);
		optix.outbuf_tangent.into_texture(ctx, optix.fb.tangent);
		optix.outbuf_depth.into_texture(ctx, optix.fb.depth);
	}


	////
	// Display results

	if (optix.debug)
	{
		// obtain the OptiX display shader program
		static shader_program &display_prog = shaders.get("optix_display");		

		// configure shader and bind optix output textures
		display_prog.enable(ctx);
		display_prog.set_uniform(ctx, "debug", (int)optix.debug);
		optix.fb.albedo->enable(ctx, 0);
		optix.fb.position->enable(ctx, 1);
		optix.fb.normal->enable(ctx, 2);
		optix.fb.tangent->enable(ctx, 3);
		optix.fb.depth.enable(ctx, 4);

		// blend the display result onto the main framebuffer
		glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		glDisable(GL_BLEND);

		// cleanup
		optix.fb.depth.disable(ctx);
		optix.fb.tangent->disable(ctx);
		optix.fb.normal->disable(ctx);
		optix.fb.position->disable(ctx);
		optix.fb.albedo->disable(ctx);
		display_prog.disable(ctx);
	}
}

// ###############################
// ###  END:  OptiX integration
// ###############################
#endif

void on_tube_vis::init_frame (cgv::render::context &ctx)
{
	// TODO: remove once all relevant view interactors provided by the framework properly fix the up-vector
	/*if (misc_cfg.fix_view_up_dir_proxy && view_ptr)
		view_ptr->set_view_up_dir(0, 1, 0);*/

	// update color and mapping legends if necessary
	if (update_legends) {
		color_legend_mgr.compose(
			ctx, traj_mgr.dataset(0), color_map_mgr, render.visualizations.front().manager.ref_glyph_attribute_mappings()
		);

		if(mapping_legend_ptr)
			mapping_legend_ptr->update(traj_mgr.dataset(0), render.visualizations.front().manager);

		update_legends = false;
	}

	// keep the framebuffer up to date with the viewport size
	fbc.ensure(ctx);
	
	taa.ensure(ctx);

#ifdef RTX_SUPPORT
	// ###############################
	// ### BEGIN: OptiX integration
	// ###############################

	if (optix.initialized && optix.enabled)
	{
		// keep the optix interop buffer up to date with the viewport size
		const unsigned w = ctx.get_width() * (optix.holographic ? 3 : 1),
		               h = ctx.get_height();
		optix.outbuf_albedo.resize(w, h);
		optix.outbuf_position.resize(w, h);
		optix.outbuf_normal.resize(w, h);
		optix.outbuf_tangent.resize(w, h);
		optix.outbuf_depth.resize(w, h);
	}

	// ###############################
	// ###  END:  OptiX integration
	// ###############################
#endif
	// query the current viewport dimensions as this is needed for multiple draw methods
	glGetIntegerv(GL_VIEWPORT, viewport);

	if (playback.active)
	{
		const double prev = playback.time_active;
		playback.timer.add_time();
		render.style.max_t = render.style.max_t + float((playback.time_active-prev)*playback.speed);
		update_member(&render.style.max_t);
		if (render.style.max_t >= playback.tend)
		{
			if (playback.repeat)
				render.style.max_t = (float)playback.tstart;
			else {
				render.style.max_t = (float)playback.tend;
				playback.active = false;
				update_member(&playback.active);
			}
		}
		if (playback.follow && playback.active)
		{
			const auto &ds = traj_mgr.dataset(0);
			const auto &[pos_attrib, pos_data] = ds.positions();
			const auto &traj_range = ds.trajectories(pos_attrib)[playback.follow_traj];
			const unsigned nid = find_sample_linear(
				pos_attrib, traj_range, render.style.max_t, playback.follow_last_nid
			),
			nid_next = std::min(nid+1, traj_range.i0+traj_range.n-1);
			const float t0 = pos_data.timestamps[nid];
			view_ptr->set_focus(cgv::math::lerp(
				ds.mapped_position(nid), ds.mapped_position(nid_next),
				(render.style.max_t-t0) / (pos_data.timestamps[nid_next]-t0)
			));
		}
	}

	if(benchmark.requested) {
		if(!misc_cfg.instant_redraw_proxy) {
			misc_cfg.instant_redraw_proxy = true;
			on_set(&misc_cfg.instant_redraw_proxy);
		}
		if(misc_cfg.vsync_proxy) {
			misc_cfg.vsync_proxy = false;
			on_set(&misc_cfg.vsync_proxy);
		}

		if(!benchmark.running) {
			benchmark.running = true;
			benchmark.timer.restart();

			benchmark.total_frames = 0u;
			benchmark.last_seconds_since_start = 0.0;
			benchmark.sort_time_total = 0.0;
			benchmark.num_sorts = 0;
		}
	}

	if(!benchmark_mode_setup && benchmark_mode) {
		misc_cfg.instant_redraw_proxy = true;
		on_set(&misc_cfg.instant_redraw_proxy);
		misc_cfg.vsync_proxy = false;
		on_set(&misc_cfg.vsync_proxy);
		show_bbox = false;
		update_member(&show_bbox);
		show_wireframe_bbox = false;
		update_member(&show_wireframe_bbox);
		cm_viewer_ptr->set_visibility(false);
		navigator_ptr->set_visibility(false);
		perfmon_ptr->set_visibility(false);
		debug.far_extent_factor = 0.4;
		debug.near_extent_factor = 0.333333*debug.far_extent_factor;
		set_view();
		//grid_mode = GM_COLOR_NORMAL;
		//on_set(&grid_mode);
		benchmark_mode_setup = true;
	}
}

void on_tube_vis::draw (cgv::render::context &ctx)
{
	if(!view_ptr) return;

	// draw dataset using selected render mode
	if(traj_mgr.has_data())
	{
		taa.begin(ctx);

		int debug_idx_count = static_cast<int>(render.data->indices.size());
		if(debug.limit_render_count)
			debug_idx_count = static_cast<int>(2 * debug.render_count);

		switch(debug.render_mode) {
		case DRM_NONE:
			draw_trajectories(ctx);
			break;
		case DRM_NODES:
			debug.geometry.nodes.render(ctx, 0, debug_idx_count);
			break;
		case DRM_SEGMENTS:
			debug.geometry.segments.render(ctx, 0, debug_idx_count);
			break;
		case DRM_NODES_SEGMENTS:
			debug.geometry.nodes.render(ctx, 0, debug_idx_count);
			debug.geometry.segments.render(ctx, 0, debug_idx_count);
			break;
		case DRM_VOLUME:
			draw_density_volume(ctx);
			break;
		default:
			break;
		}

		if (dataset.is_rtlola && dataset.rtlola_show_map)
		{
			shader_program &default_shader = ctx.ref_default_shader_program(true /* <-- texture support */);

			dataset.rtlola_map_tex.enable(ctx, 0);
			default_shader.enable(ctx);

			glDisable(GL_CULL_FACE);
			dataset.rtlola_map_vao.enable(ctx);
			glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
			dataset.rtlola_map_vao.disable(ctx);
			glEnable(GL_CULL_FACE);

			default_shader.disable(ctx);
			dataset.rtlola_map_tex.disable(ctx);
		}

		if (show_wireframe_bbox)
			bbox_wire_rd.render(ctx);

		if (show_bbox)
			bbox_rd.render(ctx);

		taa.end(ctx);
	}
	
	// display drag-n-drop information, if a dnd operation is in progress
	if(!dnd.text.empty())
		draw_dnd(ctx);
}

void on_tube_vis::after_finish(context& ctx) {

	if(initialize_view_ptr()) {
		// do one-time initialization that needs the view if necessary
		set_view();
		ensure_selected_in_tab_group_parent();

		// set one of the loaded color maps as the transfer function for the volume renderer
		if(tf_editor_ptr) {
			auto& cmcs = color_map_mgr.ref_color_maps();
			for(auto& cmc : cmcs) {
				if(cmc.name == "imola") {
					for(const auto& p : cmc.cm.ref_color_points())
						volume_tf.add_color_point(p.first, p.second);

					volume_tf.add_opacity_point(0.0f, 0.0f);
					volume_tf.add_opacity_point(1.0f, 1.0f);

					tf_editor_ptr->set_color_map(&volume_tf);
					break;
				}
			}
		}

		taa.set_view(view_ptr);
	}

	if(benchmark.running) {
		++benchmark.total_frames;
		double benchmark_time = 5.0;

		double seconds_since_start = benchmark.timer.get_elapsed_time();
		double alpha = (seconds_since_start - benchmark.last_seconds_since_start) / benchmark_time;
		benchmark.last_seconds_since_start = seconds_since_start;

		double depth = cgv::math::length(view_ptr->get_eye() - view_ptr->get_focus());

		view_ptr->rotate(0.0, cgv::math::deg2rad(360.0 * alpha), depth);

		if(seconds_since_start >= benchmark_time) {
			benchmark.running = false;
			benchmark.requested = false;
			update_member(&benchmark.requested);

			double avg_fps = (double)benchmark.total_frames / seconds_since_start;

			std::stringstream ss;
			ss.precision(2);
			ss << std::fixed;
			ss << "Average FPS: " << avg_fps << " | " << (1000.0f / avg_fps) << "ms";
			
			//ss << std::endl;
			//ss << "Sorted " << benchmark.num_sorts << " times with mean duration of " << (benchmark.sort_time_total / static_cast<double>(benchmark.num_sorts)) << "ms" << std::endl;

			std::cout << ss.str() << std::endl;
		}
	}
}

void on_tube_vis::create_gui(void)
{
	const auto add_heading = [&](const std::string& name) {
		add_decorator(name, "heading", "level=1");
	};

	const auto add_section_heading = [&](const std::string& name, int level) {
		align("%y+=10");
		add_decorator(name, "heading", "level=" + std::to_string(level), "\n%y-=14");
		add_decorator("", "separator", "", "\n%y-=8");
	};

	// Scene settings
	add_heading("Scene");

	help.create_gui(this);
	
	datapath_helper.create_gui("Data Path");

	add_member_control(this, "Bounds", bbox_rd.style.surface_color, "", "w=20", " ");
	add_member_control(this, "Box", show_bbox, "toggle", "w=83", "%x+=2");
	add_member_control(this, "Wireframe", show_wireframe_bbox, "toggle", "w=83");
	/* Quick tube/ribbon toggle */ {
		static bool dummy = render.style.is_ribbon();
		ui_state.tr_toggle.control = add_control("Current: tubes (toggle)", dummy, "toggle", "");
		if (ui_state.tr_toggle.control)
			connect_copy(
				ui_state.tr_toggle.control->value_change,
				cgv::signal::rebind(this, &on_tube_vis::toggle_tube_ribbon)
			);
	}

	if(begin_tree_node("Playback", playback, false)) {
		align("\a");
		const auto& [tmin, tmax] = render.data->t_minmax;
		const std::string tmin_str = std::to_string(tmin), tmax_str = std::to_string(tmax),
			step_str = std::to_string((tmax - tmin) / 10000.f);

		connect_copy(
			add_button("@|<", "tooltip='(Backspace) Resets playback time to start.';w=50", " ")->click,
			cgv::signal::rebind(this, &on_tube_vis::playback_rewind)
		);
		add_member_control(
			this, "@play", playback.active, "toggle",
			"tooltip='(Space) Controls whether to animate the dataset(s) within the set timeframe.';w=76", " "
		);
		connect_copy(
			add_button("@>|", "tooltip='(End) Cancels playback and displays the full data.';w=50")->click,
			cgv::signal::rebind(this, &on_tube_vis::playback_reset_ds)
		);

		add_member_control(this, "Playback Speed", playback.speed, "value_slider", "min=0.01;max=1000;step=0.01;ticks=true;log=true");
		add_member_control(this, "Timeframe Start", playback.tstart, "value_slider", "min=" + tmin_str + ";max=" + tmax_str + ";step=" + step_str + ";ticks=false");
		add_member_control(this, "Timeframe End", playback.tend, "value_slider", "min=" + tmin_str + ";max=" + tmax_str + ";step=" + step_str + ";ticks=false");
		add_member_control(this, "Repeat", playback.repeat, "check");
		add_member_control(this, "Follow Trajectory", playback.follow, "check", "tooltip='(Home) If enabled the view follows the trajectory.'");
		add_member_control(
			this, "Trajectory ID", playback.follow_traj, "value_slider",
			"min=0;max=" + std::to_string(render.data->datasets[0].trajs.size() - 1) + ";step=1;ticks=true"
		);
		align("\b");
		end_tree_node(playback);
	}

	add_decorator("", "separator");
	add_heading("Visualization");

	// Attribute mapping settings
	for(unsigned ds = 0; ds < (unsigned)render.visualizations.size(); ds++) {
		if(begin_tree_node("Attributes '" + traj_mgr.dataset(ds).name() + "'", render.visualizations[ds].manager, true)) {
			align("\a");
			layer_config_file_helper.create_gui("Configuration", layer_config_has_unsaved_changes ? "text_color=" + cgv::gui::theme_info::instance().warning_hex() : "");
			align("%y-=8");
			add_decorator("", "separator", "", "\n%y-=8");
			connect_copy(add_button("Compile Attributes")->click, cgv::signal::rebind(this, &on_tube_vis::compile_glyph_attribs));
			render.visualizations.front().manager.create_gui(this, *this);
			align("\b");
			end_tree_node(render.visualizations.front().manager);
		}
	}

	add_decorator("", "separator");

	if(begin_tree_node("Grid", grids, false)) {
		align("\a");
		add_member_control(this, "Mode", grid_mode, "dropdown", "enums='None, Color, Normal, Color + Normal'");
		add_member_control(this, "Fuzzy coloring", enable_fuzzy_grid, "check");
		add_member_control(this, "Color", grid_color);
		add_member_control(this, "Normal Type", grid_normal_settings, "dropdown", "enums='0,1,2,3'");
		add_member_control(this, "Inward", grid_normal_inwards, "check");
		add_member_control(this, "Variant", grid_normal_variant, "check");
		add_member_control(this, "Normal Scale", normal_mapping_scale, "value_slider", "min=0;max=1;step=0.001;ticks=true");
		for(size_t i = 0; i < grids.size(); ++i) {
			add_decorator("Grid " + std::to_string(i), "heading", "level=3");
			add_member_control(this, "Scaling U", grids[i].scaling[0], "value_slider", "min=1;max=50;step=0.5;ticks=true");
			add_member_control(this, "Scaling V", grids[i].scaling[1], "value_slider", "min=1;max=50;step=0.5;ticks=true");
			add_member_control(this, "Thickness", grids[i].thickness, "value_slider", "min=0;max=1;step=0.01;ticks=true");
			add_member_control(this, "Blend Factor", grids[i].blend_factor, "value_slider", "min=0;max=1;step=0.01;ticks=true");
		}
		align("\b");
		end_tree_node(grids);
	}

	add_decorator("", "separator");

	// Rendering settings
	if(begin_tree_node("Rendering", render_gui_dummy, false)) {
		align("\a");

#ifdef RTX_SUPPORT
		add_member_control(this, "Enable OptiX Raycasting", optix.enabled, "check");

		if(optix.enabled) {
			if(begin_tree_node("OptiX Settings", optix.enabled, false)) {
				align("\a");
				add_member_control(this, "Tube Primitive", optix.primitive, "dropdown", "enums='Russig,Phantom,Built-in,Built-in (cubic)'");
				add_member_control(this, "Debug Visualization", optix.debug, "dropdown", "enums='Off,Albedo,Depth,Tangent + Normal'");
				add_member_control(this, "Show BLAS Bounding Volumes", optix.debug_bvol, "check");
				add_member_control(this, "Render as Hologram", optix.holographic, "check");
				add_member_control(this, "Holo-Eye Test", optix.holo_eye, "value_slider", "min=-1;max=1;step=0.0625;ticks=true");
				align("\b");
				end_tree_node(optix.enabled);
			}
		}
#endif

		if(begin_tree_node("Style", render.style, false)) {
			align("\a");
			add_gui("", render.style);
			align("\b");
			end_tree_node(render.style);
		}

		if(begin_tree_node("Anti-Aliasing", taa, false)) {
			align("\a");
			taa.create_gui(this);
			align("\b");
			end_tree_node(taa);
		}

		if(begin_tree_node("Ambient Occlusion", ao_style, false)) {
			align("\a");
			add_gui("ao_style", ao_style);
			add_member_control(this, "Voxel Grid Resolution", voxel_grid_resolution, "dropdown", "enums='16=16, 32=32, 64=64, 128=128, 256=256, 512=512'");
			add_member_control(this, "Voxelize using GPU", voxelize_gpu, "check");
			align("\b");
			end_tree_node(ao_style);
		}

		align("\b");
		end_tree_node(render_gui_dummy);
	}
	
	add_decorator("", "separator");

	// Color scale manager and editor
	integrate_object_gui(cm_editor_ptr);
	if(cm_editor_ptr->begin_overlay_gui()) {
		color_map_mgr.create_gui(this, *this);
		cm_editor_ptr->create_gui();
		cm_editor_ptr->end_overlay_gui();
	}

	add_decorator("", "separator");
	
	// Overlay widgets
	add_member_control(this, "Layer Legend", show_mapping_legend, "toggle", "tooltip='Toggle visibility of the layer mapping legend.';w=98", "%x+=4");
	add_member_control(this, "Color Scales", show_color_map_viewer, "toggle", "tooltip='Toggle visibility of the color scale preview.';w=98", "\n%y-=5");
	add_member_control(this, "Navigator", show_navigator, "toggle", "tooltip='Toggle visibility of the navigator cube.';w=98", "%x+=4");
	add_member_control(this, "Performance", show_performance_monitor, "toggle", "tooltip='Toggle visibility of the performance monitor.';w=98");

	add_decorator("", "separator");

	// Debug settings contractable section
	if(begin_tree_node("(Debug)", benchmark, false)) {
		align("\a");

		add_section_heading("Tools", 3);
		add_member_control(
			this, "Instant Redraw", misc_cfg.instant_redraw_proxy, "toggle",
			"tooltip='Controls the instant redraw state of the FLTK GL window.'"
		);
		align("%y-=6");
		add_member_control(
			this, "VSync", misc_cfg.vsync_proxy, "toggle",
			"tooltip='Controls the vsync state of the FLTK GL window.'"
		);
		align("%y-=6");
		add_member_control(
			this, "Fix View Up Dir", misc_cfg.fix_view_up_dir_proxy, "toggle",
			"tooltip='Controls the \"fix_view_up_dir\" state of the view interactor.'"
		);
		
		add_section_heading("Render Mode", 3);
		add_member_control(this, "", debug.render_mode, "dropdown", "enums='Default,Nodes,Segments,Nodes + Segments,Volume'");
		
		if(debug.render_mode == DRM_VOLUME) {
			if(begin_tree_node("Volume Style", vstyle, false, "level=3")) {
				align("\a");
				add_gui("vstyle", vstyle);

				inline_object_gui(tf_editor_ptr);

				align("\b");
				end_tree_node(vstyle);
			}
		}

		add_member_control(this, "Show Segments", debug.highlight_segments, "check");
		add_member_control(this, "Show Hidden Glyphs", debug.show_hidden_glyphs, "check");

		add_section_heading("Sorting", 3);
		add_member_control(this, "Sort by Distance", debug.sort, "check");
		add_member_control(this, "Lazy Sort", debug.lazy_sort, "check", "tooltip='Sort only after significant view changes.'");
		add_member_control(this, "Force Initial Order", debug.force_initial_order, "check");

		add_section_heading("Render Count", 3);
		add_member_control(this, "Limit", debug.limit_render_count, "check");
		add_member_control(this, "Percentage", debug.render_percentage, "value_slider", "min=0.0;step=0.001;max=1.0;ticks=true");
		add_member_control(this, "Count", debug.render_count, "value", "w=70;min=0;step=1;max=" + std::to_string(debug.segment_count), " ");
		add_member_control(this, "", debug.render_count, "wheel", "w=120;min=0;step=1;max=" + std::to_string(debug.segment_count));
		
		add_section_heading("Benchmark", 3);
		add_member_control(this, "Start", benchmark.requested, "toggle", "");

		align("\b");
		end_tree_node(benchmark);
	}
}

void on_tube_vis::create_vec3_gui(const std::string& name, vec3& value, float min, float max) {

	std::string value_config = "w=55;min=" + std::to_string(min) + ";max=" + std::to_string(max);
	std::string slider_config = "w=55;min=" + std::to_string(min) + ";max=" + std::to_string(max) + ";step=0.0001;ticks=true";

	add_member_control(this, name, value[0], "value", value_config, " ");
	add_member_control(this, "", value[1], "value", value_config, " ");
	add_member_control(this, "", value[2], "value", value_config);
	add_member_control(this, "", value[0], "slider", slider_config, " ");
	add_member_control(this, "", value[1], "slider", slider_config, " ");
	add_member_control(this, "", value[2], "slider", slider_config);
}

void on_tube_vis::set_view(void)
{
	if (!view_ptr || !traj_mgr.has_data()) return;

	view_ptr->set_focus(bbox.get_center());
	double extent_factor = debug.near_view ? debug.near_extent_factor : debug.far_extent_factor;
	view_ptr->set_y_extent_at_focus(extent_factor * (double)length(bbox.get_extent()));

	auto* cview_ptr = dynamic_cast<cgv::render::clipped_view*>(view_ptr);
	if(cview_ptr) {
		// extent the bounding box to prevent accidental clipping of proxy geometry which could happen in certain scenarios.
		box3 clip_bbox = bbox;
		vec3 extent = bbox.get_extent();
		clip_bbox.ref_min_pnt() -= 0.25f*extent;
		clip_bbox.ref_max_pnt() += 0.25f*extent;
		cview_ptr->set_scene_extent(clip_bbox);
	}

	taa.reset();
}

void on_tube_vis::update_grid_ratios(void) {
	auto& ctx = *get_context();

	if (!render.data->datasets.empty())
	{
		uint64_t num = 0;
		double sum = 0;
		for (const auto &ds : render.data->datasets)
		{
			num += ds.irange.n;
			sum += ds.irange.n * double(ds.irange.med_radius);
		}
		double mean_rad = sum / double(num);
		// adjust by the current radius scale
		mean_rad *= render.style.radius_scale;
		// we base everything on the mean of all trajectory median radii
		float inv_mean_rad = 1.f / static_cast<float>(mean_rad);
		render.style.length_scale = inv_mean_rad;
		grids[0].scaling.x() = inv_mean_rad;
		//grids[0].scaling.y() = grids[0].scaling.x()/4;
		grids[0].scaling.y() = 1.0f;
		grids[1].scaling.x() = grids[0].scaling.x()*4;
		//grids[1].scaling.y() = grids[0].scaling.x();
		grids[1].scaling.y() = 4.0f;
		for (unsigned i=0; i<2; i++)
		{
			update_member(&(grids[i].scaling[0]));
			update_member(&(grids[i].scaling[1]));
		}
		update_member(&render.style.length_scale);
	}
}

void on_tube_vis::update_attribute_bindings(void) {
	auto &ctx = *get_context();

	if (traj_mgr.has_data())
	{
		calculate_bounding_box(); 
		set_view();

		// update min/max timestamps
		auto &[tmin, tmax] = render.style.data_t_minmax;
		float pct = (render.style.max_t-tmin) / (tmax-tmin);
		render.style.data_t_minmax = render.data->t_minmax;
		render.style.max_t = cgv::math::clamp(tmin + pct*(tmax-tmin), tmin, tmax);
		playback.tstart = tmin;
		playback.tend = tmax;
		playback.follow_traj = std::min(
			playback.follow_traj, (unsigned)render.data->datasets[0].trajs.size()-1
		);

		// Clear range and attribute buffers for glyph layers
		for(size_t i = 0; i < render.aindex_sbos.size(); ++i)
			render.aindex_sbos[i].destruct(ctx);
		for(size_t i = 0; i < render.attribs_sbos.size(); ++i)
			render.attribs_sbos[i].destruct(ctx);
		
		render.aindex_sbos.clear();
		render.aindex_sbos.resize(4);
		render.attribs_sbos.clear();
		render.attribs_sbos.resize(4);

		// Recompute arclength parametrization
		cgv::utils::stopwatch s(true);
		std::cout << "Computing arclength parametrization... ";

		render.arclen_sbo.destruct(ctx);
		render.arclen_data = arclen::compute_parametrization(traj_mgr);
		render.arclen_sbo = arclen::upload_renderdata(ctx, render.arclen_data.t_to_s);
		
		std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

		s.restart();
		std::cout << "Uploading vertex attributes... ";

		// Upload render attributes
		render.render_sbo.destruct(ctx);
		// - compile data
		const size_t num_nodes = render.data->positions.size();
		std::vector<node_attribs> render_attribs; render_attribs.reserve(num_nodes);
		for (size_t i=0; i<num_nodes; i++)
		{
			// convenience shortcut
			const auto &col = render.data->colors[i];
			// interleave and commit
			render_attribs.emplace_back(node_attribs{
				/* .pos_rad */  vec4(render.data->positions[i], render.data->radii[i]),
				/* .color */    vec4(col.R(), col.G(), col.B(), 1),
				/* .tangent */  render.data->tangents[i],  // <- does already contain radius deriv. in w-component
				/* .t */        vec4(render.data->timestamps[i], 0, 0, 0)
			});
		}
		// - upload
		cgv::render::vertex_buffer new_sbo(cgv::render::VBT_STORAGE, cgv::render::VBU_STATIC_READ);
		if (!new_sbo.create(ctx, render_attribs))
			std::cerr << "!!! unable to create render attribute Storage Buffer Object !!!" << std::endl << std::endl;
		render.render_sbo = std::move(new_sbo);

		unsigned node_indices_count = (unsigned)render.data->indices.size();
		unsigned segment_count = node_indices_count / 2;

		std::vector<unsigned> segment_indices(segment_count);

		for(unsigned i = 0; i < segment_indices.size(); ++i)
			segment_indices[i] = i;

		// Upload render attributes to legacy buffers
		auto &tstr = ref_textured_spline_tube_renderer(ctx);
		tstr.enable_attribute_array_manager(ctx, render.aam);
		tstr.set_node_id_array(ctx, reinterpret_cast<const uvec2*>(render.data->indices.data()), segment_count, sizeof(uvec2));
		tstr.set_indices(ctx, segment_indices);
		tstr.disable_attribute_array_manager(ctx, render.aam);

		if(!render.sorter.init(ctx, render.data->indices.size() / 2))
			std::cout << "Could not initialize gpu sorter" << std::endl;

		std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

		debug.segment_count = segment_count;
		debug.render_percentage = 1.0f;
		debug.render_count = segment_count;
		update_member(&debug.render_percentage);
		update_member(&debug.render_count);

		// Generate the density volume (uses GPU buffer data so we need to do this after upload)
		create_density_volume(ctx, voxel_grid_resolution);
	}

	// reset the last sort pos and direction to zero when the render data changed to force a sorting step
	last_sort_pos = vec3(0.0f);
	last_sort_dir = vec3(0.0f);
}

void on_tube_vis::update_debug_attribute_bindings() {
	auto &ctx = *get_context();

	auto& nodes = debug.geometry.nodes;
	auto& segments = debug.geometry.segments;

	nodes.clear();
	segments.clear();

	if(traj_mgr.has_data()) {
		float radius_scale = debug.render_mode == DRM_NODES_SEGMENTS ? 0.5f : 1.0f;

		// Create render data for debug views if requested
		if(debug.render_mode == DRM_NONE) {
			// do an early transfer to free GPU memory, since the render function of this data will not be called anymore
			nodes.early_transfer(ctx, ref_sphere_renderer(ctx));
			segments.early_transfer(ctx, ref_cone_renderer(ctx));
		} else {
			const size_t num_nodes = render.data->positions.size();
			for(size_t i = 0; i < num_nodes; i++) {
				// convenience shortcut
				const auto &col = render.data->colors[i];

				nodes.add(render.data->positions[i], col, render.data->radii[i]);
				segments.add_position(render.data->positions[i]);
				segments.add_radius(radius_scale * render.data->radii[i]);
				segments.add_color(col);
			}
			// also use index buffer for nodes even though this means we render most nodes twice
			// this is necessary for correct usage of the render count limit
			nodes.indices = render.data->indices;
			segments.indices = render.data->indices;
		}
	}
}

void on_tube_vis::calculate_bounding_box(void) {

	cgv::utils::stopwatch s(true);
	std::cout << "Calculating bounding box... ";

	bbox.invalidate();
	
	if(traj_mgr.has_data()) {
		// ensure render data
		render.data = &(traj_mgr.get_render_data());
		auto& positions = render.data->positions;
		auto& tangents = render.data->tangents;
		auto& radii = render.data->radii;
		auto& indices = render.data->indices;

		for(unsigned i = 0; i < indices.size(); i += 2) {
			unsigned idx_a = indices[i + 0];
			unsigned idx_b = indices[i + 1];

			vec3 p0 = positions[idx_a];
			vec3 p1 = positions[idx_b];
			float r0 = radii[idx_a];
			float r1 = radii[idx_b];
			vec4 t0 = tangents[idx_a];
			vec4 t1 = tangents[idx_b];

			hermite_spline_tube hst = hermite_spline_tube(p0, p1, r0, r1, vec3(t0), vec3(t1), t0.w(), t1.w());
			bbox.add_axis_aligned_box(hst.bounding_box(true));
		}
	}

	bbox_wire_rd.clear();
	bbox_wire_rd.add(bbox.get_center(), bbox.get_extent(), rgb(0.75f));

	bbox_rd.clear();
	bbox_rd.add(bbox.get_center(), bbox.get_extent());

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
	std::cout << "Bounding box min: " << bbox.get_min_pnt() << "| max: " << bbox.get_max_pnt() << "| extent: " << bbox.get_extent() << std::endl;
}

void on_tube_vis::create_density_volume(context& ctx, unsigned resolution) {

	density_volume.initialize_voxel_grid(bbox, resolution);
	ivec3 res = density_volume.ref_voxel_grid().resolution;
	std::cout << "Generating density volume with resolution (" << res.x() << ", " << res.y() << ", " << res.z() << ")... ";

	if(density_tex.is_created()) {
		if(res.x() != density_tex.get_width() || res.y() != density_tex.get_height() || res.z() != density_tex.get_depth()) {
			density_tex.destruct(ctx);
		}
	}

	if(!density_tex.is_created()) {
		std::vector<float> density_data(res.x()*res.y()*res.z(), 0.0f);

		cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res.x(), res.y(), res.z(), TI_FLT32, cgv::data::CF_R), density_data.data());
		density_tex = texture("flt32[R]", TF_LINEAR, TF_LINEAR_MIPMAP_LINEAR, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER);
		density_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);
		density_tex.create(ctx, dv, 0);
		density_tex.create_mipmaps(ctx);
	}

	cgv::utils::stopwatch s(true);

	if(voxelize_gpu) {
		// prepare index buffer pointer
		auto &tstr = ref_textured_spline_tube_renderer(ctx);
		const vertex_buffer* node_idx_buffer_ptr = tstr.get_vertex_buffer_ptr(ctx, render.aam, "node_ids");
		if(node_idx_buffer_ptr && render.render_sbo.is_created())
			density_volume.compute_density_volume_gpu(ctx, render.data, render.style.radius_scale, *node_idx_buffer_ptr, render.render_sbo, density_tex);
	} else {
		density_volume.compute_density_volume(render.data, render.style.radius_scale);

		std::vector<float>& density_data = density_volume.ref_voxel_grid().data;

		cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res.x(), res.y(), res.z(), TI_FLT32, cgv::data::CF_R), density_data.data());
		density_tex.replace(ctx, 0, 0, 0, dv);
		density_tex.generate_mipmaps(ctx);
	}

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

	ao_style.derive_voxel_grid_parameters(density_volume.ref_voxel_grid());
}

void on_tube_vis::draw_dnd(context& ctx) {
	const auto& ti = cgv::gui::theme_info::instance();

	//static const rgb dnd_col(1, 0.5f, 0.5f);
	static const rgb dnd_col = ti.highlight();
	// compile the text we're going to draw and gather its approximate dimensions at the same time
	float w = 0, s = ctx.get_current_font_size();
	std::stringstream dnd_drawtext;
	dnd_drawtext << "Load ";

	if(dnd.filenames.size() == 1) {
		std::string extension = cgv::utils::file::get_extension(dnd.filenames[0]);
		if(cgv::utils::to_upper(extension) == "XML")
			dnd_drawtext << "glyph layer configuration";
		else
			dnd_drawtext << "dataset";
	} else {
		dnd_drawtext << "dataset";
	}
	
	dnd_drawtext << ":" << std::endl;
	{
		std::string tmp;
		for(const std::string &filename : dnd.filenames) {
			tmp = "   "; tmp += filename;
			w = std::max(w, ctx.get_current_font_face()->measure_text_width(tmp, s));
			dnd_drawtext << tmp << std::endl;
		}
	}
	float h = dnd.filenames.size()*s + s;
	// calculate actual position at which to place the text
	ivec2 pos = dnd.pos,
		overflow(viewport[0] + viewport[2] - dnd.pos.x() - int(std::ceil(w)),
			viewport[1] + viewport[3] - dnd.pos.y() - int(std::ceil(h)));
	// - first, try to prevent truncation at the right and bottom borders
	if(overflow.x() < 0)
		pos.x() = std::max(1, pos.x() + overflow.x());
	if(overflow.y() < 0)
		pos.y() = std::max(1, pos.y() + overflow.y());
	// - then, absolutely prevent truncation at the top border
	pos.y() = std::max(viewport[1] + signed(s), pos.y());
	// draw the text
	ctx.push_pixel_coords();
	ctx.set_color(dnd_col);
	ctx.set_cursor(vecn(float(pos.x()), float(pos.y())), "", cgv::render::TA_TOP_LEFT);
	ctx.output_stream() << dnd_drawtext.str();
	ctx.output_stream().flush();
	ctx.pop_pixel_coords();
}

void on_tube_vis::draw_trajectories(context& ctx)
{
	// common init
	// - view-related info
	const vec3 &cyclopic_eye = view_ptr->get_eye();
	const vec3 &view_dir = view_ptr->get_view_dir();
	const vec3 &view_up_dir = view_ptr->get_view_up_dir();

	vec2 viewport_size(
		static_cast<float>(fbc.ref_frame_buffer().get_width()),
		static_cast<float>(fbc.ref_frame_buffer().get_height())
	);
	// - spline stube renderer setup relevant to deferred shading pass
	auto &tstr = ref_textured_spline_tube_renderer(ctx);
	tstr.set_render_style(render.style);
	// - the depth texture to use
	//   (workaround for longstanding NVIDIA driver bug preventing GPU-internal PBO transfers to GL_DEPTH_COMPONENT formats)
#ifdef RTX_SUPPORT
	texture &tex_depth = (optix.enabled && optix.initialized) ? optix.fb.depth : *fbc.attachment_texture_ptr("depth");
#else
	texture &tex_depth = *fbc.attachment_texture_ptr("depth");
#endif
	// - node attribute data needed by both rasterization and raytracing
	const vertex_buffer* node_idx_buffer_ptr = tstr.get_vertex_buffer_ptr(ctx, render.aam, "node_ids");
	
#ifdef RTX_SUPPORT
	if (!optix.enabled || !optix.initialized)
#endif
	{
		// enable drawing framebuffer
		fbc.enable(ctx);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render tubes
		auto &tstr = ref_textured_spline_tube_renderer(ctx);

		// prepare index buffer pointer
		const vertex_buffer* segment_idx_buffer_ptr = tstr.get_index_buffer_ptr(render.aam);

		if(!render.render_sbo.is_created() ||
			!render.arclen_sbo.is_created() ||
			segment_idx_buffer_ptr == nullptr ||
			node_idx_buffer_ptr == nullptr)
			return;

		// onyl perform a new visibility sort step when the view configuration deviates significantly
		bool do_sort = false;
		float pos_angle = dot(normalize(last_sort_pos), normalize(cyclopic_eye));
		float view_angle = dot(view_dir, last_sort_dir);
		if(view_angle < 0.8f || pos_angle < 0.8f || !debug.lazy_sort) {
			do_sort = true;
			last_sort_pos = normalize(cyclopic_eye);
			last_sort_dir = view_dir;
		}

		// sort the segment indices
		if(debug.sort && do_sort && !debug.force_initial_order) {
			// measure sort time
			//render.sorter.begin_time_query();
			render.sorter.execute(ctx, render.render_sbo, *segment_idx_buffer_ptr, cyclopic_eye, view_dir, node_idx_buffer_ptr);
			//benchmark.sort_time_total += render.sorter.end_time_query();
			++benchmark.num_sorts;
		}

		tstr.set_cyclopic_eye(cyclopic_eye);
		tstr.set_view_dir(view_dir);
		tstr.set_viewport(vec4((float)viewport[0], (float)viewport[1], (float)viewport[2], (float)viewport[3]));
		tstr.set_render_style(render.style);
		tstr.enable_attribute_array_manager(ctx, render.aam);

		int count = static_cast<int>(render.data->indices.size() / 2);
		if(debug.limit_render_count) {
			count = static_cast<int>(debug.render_count);
		}

		render.render_sbo.bind(ctx, VBT_STORAGE, 0);
		render.arclen_sbo.bind(ctx, VBT_STORAGE, 1);
		//if (render.style.attrib_mode != textured_spline_tube_render_style::AM_ALL) {
			// for now we always bind the node indices buffer to enable smooth intra-segment t filtering
			node_idx_buffer_ptr->bind(ctx, VBT_STORAGE, 2);
			tstr.render(ctx, 0, count);
		/*}
		else
			tstr.render(ctx, 0, count);*/

		tstr.disable_attribute_array_manager(ctx, render.aam);

		// disable the drawing framebuffer
		fbc.disable(ctx);
	}
#ifdef RTX_SUPPORT
	else
	{
		// delegate to OptiX raytracing
		optix_draw_trajectories(ctx);

		// workaround for weird framework material behavior
		tstr.enable(ctx); tstr.disable(ctx);
	}
#endif
#ifdef RTX_SUPPORT
	if (   (!optix.enabled || !optix.initialized)
		|| (!optix.debug && optix.enabled && optix.initialized))
#endif
	{
		// perform the deferred shading pass and draw the image into the shading framebuffer when not using OptiX (for now)
		shader_program& prog = shaders.get("tube_shading");
		prog.enable(ctx);
		// set render parameters
		prog.set_uniform(ctx, "use_gamma", true);

		// set ambient occlusion parameters
		if(ao_style.enable) {
			//prog.set_uniform(ctx, "ambient_occlusion.enable", ao_style.enable);
			prog.set_uniform(ctx, "ambient_occlusion.sample_offset", ao_style.sample_offset);
			prog.set_uniform(ctx, "ambient_occlusion.sample_distance", ao_style.sample_distance);
			prog.set_uniform(ctx, "ambient_occlusion.strength_scale", ao_style.strength_scale);

			prog.set_uniform(ctx, "ambient_occlusion.tex_offset", ao_style.texture_offset);
			prog.set_uniform(ctx, "ambient_occlusion.tex_scaling", ao_style.texture_scaling);
			prog.set_uniform(ctx, "ambient_occlusion.texcoord_scaling", ao_style.texcoord_scaling);
			prog.set_uniform(ctx, "ambient_occlusion.texel_size", ao_style.texel_size);

			prog.set_uniform(ctx, "ambient_occlusion.cone_angle_factor", ao_style.angle_factor);
			prog.set_uniform_array(ctx, "ambient_occlusion.sample_directions", ao_style.sample_directions);
		}

		// set grid parameters
		prog.set_uniform(ctx, "grid_color", grid_color);
		prog.set_uniform(ctx, "normal_mapping_scale", normal_mapping_scale);
		for(size_t i = 0; i < grids.size(); ++i) {
			std::string base_name = "grids[" + std::to_string(i) + "].";
			prog.set_uniform(ctx, base_name + "scaling", grids[i].scaling);
			prog.set_uniform(ctx, base_name + "thickness", grids[i].thickness);
			prog.set_uniform(ctx, base_name + "blend_factor", grids[i].blend_factor);
		}

		// set attribute mapping parameters
		const auto &glyph_layers_config = render.visualizations.front().config;
		for(const auto& p : glyph_layers_config.constant_float_parameters)
			prog.set_uniform(ctx, p.first, *p.second);

		for(const auto& p : glyph_layers_config.constant_color_parameters)
			prog.set_uniform(ctx, p.first, *p.second);

		for(const auto& p : glyph_layers_config.mapping_parameters)
			prog.set_uniform(ctx, p.first, *p.second);

		// map global settings
		prog.set_uniform(
			ctx, "use_curvature_correction", (
				#if RTX_SUPPORT
					optix.enabled ||
				#endif
				render.style.is_tube()
			) && render.style.use_curvature_correction
		);

		prog.set_uniform(ctx, "length_scale", render.style.length_scale);
		prog.set_uniform(ctx, "antialias_radius", render.style.antialias_radius);

		const surface_render_style& srs = *static_cast<const surface_render_style*>(&render.style);

		prog.set_uniform(ctx, "map_color_to_material", int(srs.map_color_to_material));
		prog.set_uniform(ctx, "culling_mode", int(srs.culling_mode));
		prog.set_uniform(ctx, "illumination_mode", int(srs.illumination_mode));

		#ifdef RTX_SUPPORT
			prog.set_uniform(ctx, "holographic_raycast", optix.enabled && optix.initialized && optix.holographic);
		#else
			prog.set_uniform(ctx, "holographic_raycast", false);
		#endif
		prog.set_uniform(ctx, "viewport_width", (float)ctx.get_width());
		const auto fb_size = fbc.get_size();
		prog.set_uniform(ctx, "framebuf_width", (float)fb_size.x());

		fbc.enable_attachment(ctx, "albedo", 0);
		fbc.enable_attachment(ctx, "position", 1);
		fbc.enable_attachment(ctx, "normal", 2);
		fbc.enable_attachment(ctx, "tangent", 3);
		tex_depth.enable(ctx, 4);
		if(ao_style.enable)
			density_tex.enable(ctx, 5);
		color_map_mgr.ref_texture().enable(ctx, 6);

		// bind range attribute sbos of active glyph layers
		bool active_sbos[4] = { false, false, false, false };
		for(size_t i = 0; i < glyph_layers_config.layer_configs.size(); ++i) {
			if(glyph_layers_config.layer_configs[i].mapped_attributes.size() > 0) {
				const int attribs_handle = render.attribs_sbos[i].handle ? (const int&)render.attribs_sbos[i].handle - 1 : 0;
				const int aindex_handle = render.aindex_sbos[i].handle ? (const int&)render.aindex_sbos[i].handle - 1 : 0;
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2 * (GLuint)i + 0, attribs_handle);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2 * (GLuint)i + 1, aindex_handle);
			}
		}

		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		for(size_t i = 0; i < 4; ++i) {
			if(active_sbos[i]) {
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2 * (GLuint)i + 0, 0);
				glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2 * (GLuint)i + 1, 0);
			}
		}

		fbc.disable_attachment(ctx, "albedo");
		fbc.disable_attachment(ctx, "position");
		fbc.disable_attachment(ctx, "normal");
		fbc.disable_attachment(ctx, "tangent");
		tex_depth.disable(ctx);
		if(ao_style.enable)
			density_tex.disable(ctx);
		color_map_mgr.ref_texture().disable(ctx);

		prog.disable(ctx);

		if(playback.active)
			post_redraw();
	}
}

void on_tube_vis::draw_density_volume(context& ctx) {

	auto& vr = ref_volume_renderer(ctx);
	vr.set_render_style(vstyle);
	vr.set_volume_texture(&density_tex);
	vr.set_transfer_function_texture(&volume_tf.ref_texture());
	
	vr.set_bounding_box(density_volume.ref_voxel_grid().bounds);
	vr.transform_to_bounding_box(true);

	vr.render(ctx, 0, 0);
}

shader_define_map on_tube_vis::build_tube_shading_defines() {
	shader_define_map defines;

	// debug defines
	shader_code::set_define(defines, "DEBUG_SEGMENTS", debug.highlight_segments, false);

	// ambient occlusion defines
	shader_code::set_define(defines, "ENABLE_AMBIENT_OCCLUSION", ao_style.enable, true);

	// grid defines
	shader_code::set_define(defines, "GRID_MODE", grid_mode, GM_COLOR);
	unsigned gs = static_cast<unsigned>(grid_normal_settings);
	if(grid_normal_inwards) gs += 4u;
	if(grid_normal_variant) gs += 8u;
	shader_code::set_define(defines, "GRID_NORMAL_SETTINGS", gs, 0u);
	shader_code::set_define(defines, "ENABLE_FUZZY_GRID", enable_fuzzy_grid, false);

	// glyph layer defines
	const auto &glyph_layers_config = render.visualizations.front().config;
	shader_code::set_define(defines, "GLYPH_MAPPING_UNIFORMS", glyph_layers_config.uniforms_definition, std::string(""));

	shader_code::set_define(defines, "CONSTANT_FLOAT_UNIFORM_COUNT", glyph_layers_config.constant_float_parameters.size(), static_cast<size_t>(0));
	shader_code::set_define(defines, "CONSTANT_COLOR_UNIFORM_COUNT", glyph_layers_config.constant_color_parameters.size(), static_cast<size_t>(0));
	shader_code::set_define(defines, "MAPPING_PARAMETER_UNIFORM_COUNT", glyph_layers_config.mapping_parameters.size(), static_cast<size_t>(0));
	
	for(size_t i = 0; i < glyph_layers_config.layer_configs.size(); ++i) {
		const auto& lc = glyph_layers_config.layer_configs[i];
		shader_code::set_define(defines, "L" + std::to_string(i) + "_VISIBLE", lc.visible, true);
		shader_code::set_define(defines, "L" + std::to_string(i) + "_MAPPED_ATTRIB_COUNT", lc.mapped_attributes.size(), static_cast<size_t>(0));
		shader_code::set_define(defines, "L" + std::to_string(i) + "_GLYPH_DEFINITION", lc.glyph_definition, std::string(""));
	}

	return defines;
}


////
// Object registration

// plugins
cgv::base::object_registration<on_tube_vis> reg_tubes("");
cgv::base::registration_order_definition ro_def("stereo_view_interactor;on_tube_vis");

#ifdef CGV_FORCE_STATIC
	#include <OnTubeVis_shader_inc.h>
#endif

// Try to force the usage of the discrete Nvidia graphics card on Windows.
// This probably only works on single executable builds.
#if defined(_WIN32) || defined(WIN32)
	#include <windows.h>
	extern "C" {
		_declspec(dllexport) DWORD NvOptimusEnablement = true;
	}
#endif
