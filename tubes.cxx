#include "tubes.h"

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
#include <cgv/utils/xml.h>

// CGV framework graphics utility
#include <cgv_glutil/color_map_reader.h>
#include <cgv_glutil/color_map_writer.h>

// fltk_gl_view for controlling instant redraw
#include <plugins/cg_fltk/fltk_gl_view.h>

// stereo_view_interactor for controlling fix_view_up_dir
#include <plugins/crg_stereo_view/stereo_view_interactor.h>

// local includes
#include "arclen_helper.h"
#include "glyph_compiler.h"



// TODO: grid_mode enum does not get set properly thorugh config, because it is reflected as a boolean
// TODO: test sort order if primitives are behind camera and prevent drawing of invisible stuff? (probably irrelevant)
// TODO: star and line plot: the first mapped entry will always get mapped to the first overall color
	// Example: map only axis 2, so axis 0 and 1 are unmapped. Then color 0 will be taken for the mapped axis 2.

tubes::tubes() : application_plugin("Tubes")
{
	// adjust render style defaults
	//render.style.illumination_mode = IM_OFF;
	render.style.material.set_brdf_type(
		(cgv::media::illum::BrdfType)(cgv::media::illum::BrdfType::BT_STRAUSS_DIFFUSE
			| cgv::media::illum::BrdfType::BT_COOK_TORRANCE)
	);
	render.style.material.set_roughness(0.25);
	render.style.material.set_metalness(0.25);
	render.style.material.set_ambient_occlusion(0.75);
	render.style.material.set_emission({ 0.125f, 0.125f, 0.125f });
	render.style.material.set_specular_reflectance({ 0.05f, 0.05f, 0.05f });
	render.style.use_conservative_depth = true;
	
	bbox_style.culling_mode = cgv::render::CM_FRONTFACE;
	bbox_style.illumination_mode = cgv::render::IM_TWO_SIDED;
	bbox_style.surface_color = rgb(0.7f);

	vstyle.enable_depth_test = false;

#ifdef _DEBUG
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(32u);
#else
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(128u);
#endif

	shaders.add("tube_shading", "textured_spline_tube_shading.glpr");
	//shaders.add("taa", "taa.glpr");
	shaders.add("fxaa", "fxaa.glpr");
	shaders.add("taa_resolve", "taa_resolve.glpr");
	shaders.add("screen", "screen.glpr");

	// add framebuffer attachments needed for deferred rendering
	fbc.add_attachment("depth", "[D]");
	fbc.add_attachment("albedo", "flt32[R,G,B,A]");
	fbc.add_attachment("position", "flt32[R,G,B]");
	fbc.add_attachment("normal", "flt32[R,G,B]");
	fbc.add_attachment("tangent", "flt32[R,G,B]");

	const std::string color_format = "flt32[R,G,B,A]";
	//const std::string color_format = "uint8[R,G,B]";

	fbc_shading.add_attachment("color", color_format);

	fbc_post.add_attachment("color", color_format);

	fbc_hist.add_attachment("color", color_format, TF_LINEAR);

	fbc_final.add_attachment("color", color_format);

	cm_editor_ptr = register_overlay<cgv::glutil::color_map_editor>("Color Scales");
	cm_editor_ptr->set_visibility(false);
	cm_editor_ptr->gui_options.show_heading = false;

	tf_editor_ptr = register_overlay<cgv::glutil::color_map_editor>("Volume TF");
	tf_editor_ptr->set_visibility(false);
	tf_editor_ptr->gui_options.show_heading = false;
	tf_editor_ptr->set_opacity_support(true);

	navigator_ptr = register_overlay<cgv::glutil::navigator>("Navigator");
	navigator_ptr->set_visibility(false);
	navigator_ptr->gui_options.show_heading = false;
	navigator_ptr->gui_options.show_layout_options = false;
	navigator_ptr->set_overlay_alignment(cgv::glutil::overlay::AO_START, cgv::glutil::overlay::AO_END);
	
	cm_viewer_ptr = register_overlay<color_map_viewer>("Color Scale Viewer");
	cm_viewer_ptr->gui_options.show_heading = false;
	
	grids.resize(2);
	grids[0].scaling = vec2(1.0f, 1.0f);
	grids[0].thickness = 0.05f;
	grids[0].blend_factor = 0.5f;
	grids[1].scaling = vec2(4.0f);
	grids[1].thickness = 0.1f;
	grids[1].blend_factor = 0.333f;
	grid_color = rgba(0.25f, 0.25f, 0.25f, 0.75f);
	grid_mode = GM_COLOR_NORMAL;
	grid_normal_settings = (cgv::type::DummyEnum)1u;
	grid_normal_inwards = true;
	grid_normal_variant = true;
	normal_mapping_scale = 1.0f;
	enable_fuzzy_grid = false;

	//fh.file_name = QUOTE_SYMBOL_VALUE(INPUT_DIR);
	//fh.file_name += "/res/";
	fh.file_name = "";

	debug.segment_rs.rounded_caps = true;

	connect(cgv::gui::get_animation_trigger().shoot, this, &tubes::timer_event);
}

void tubes::handle_args (std::vector<std::string> &args)
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

void tubes::clear(cgv::render::context &ctx) {
	// decrease reference count of the renderers by one
	ref_textured_spline_tube_renderer(ctx, -1);
	ref_box_renderer(ctx, -1);
	ref_box_wire_renderer(ctx, -1);
	ref_cone_renderer(ctx, -1);
	ref_sphere_renderer(ctx, -1);
	ref_volume_renderer(ctx, -1);

	bbox_rd.destruct(ctx);
	bbox_wire_rd.destruct(ctx);

	srd.destruct(ctx);
	debug.node_rd.destruct(ctx);
	debug.segment_rd.destruct(ctx);

	shaders.clear(ctx);
	fbc.clear(ctx);
	fbc_shading.clear(ctx);
	fbc_post.clear(ctx);
	fbc_hist.clear(ctx);
	fbc_final.clear(ctx);

	color_map_mgr.destruct(ctx);

	delete render.sorter;
	render.sorter = nullptr;
}

bool tubes::self_reflect (cgv::reflect::reflection_handler &rh)
{
	return
		rh.reflect_member("datapath", datapath) &&
		rh.reflect_member("layer_config_file", fh.file_name) && // ToDo: figure out proper reflection name
		rh.reflect_member("show_hidden_glyphs", include_hidden_glyphs) && // ToDo: which of these two names is better?
		rh.reflect_member("render_style", render.style) &&
		rh.reflect_member("bounding_box_color", bbox_style.surface_color) &&
		rh.reflect_member("show_bounding_box", show_bbox) &&
		rh.reflect_member("show_wireframe_box", show_wireframe_bbox) &&
		rh.reflect_member("grid_mode", grid_mode) &&
		rh.reflect_member("grid_normal_settings", grid_normal_settings) &&
		rh.reflect_member("grid_normal_inwards", grid_normal_inwards) &&
		rh.reflect_member("grid_normal_variant", grid_normal_variant) &&
		rh.reflect_member("voxelize_gpu", voxelize_gpu) &&
		rh.reflect_member("instant_redraw_proxy", misc_cfg.instant_redraw_proxy) &&
		rh.reflect_member("vsync_proxy", misc_cfg.vsync_proxy) &&
		rh.reflect_member("fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy) &&
		rh.reflect_member("benchmark_mode", benchmark_mode);
}

void tubes::stream_help (std::ostream &os)
{
	os << "tubes: adapt <R>adius, toggle b<O>unds, <W>ire bounds" << std::endl;
}

#define SET_MEMBER(m, v) m = v; update_member(&m);

bool tubes::handle_event(cgv::gui::event &e) {

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
				SET_MEMBER(debug.force_initial_order, true);
				on_set(&debug.force_initial_order);
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
				SET_MEMBER(debug.force_initial_order, true);
				on_set(&debug.force_initial_order);
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
				SET_MEMBER(debug.force_initial_order, false);
				on_set(&debug.force_initial_order);
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
				SET_MEMBER(debug.force_initial_order, false);
				on_set(&debug.force_initial_order);
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
					std::cout << "Starting benchmark..." << std::endl;
					benchmark.requested = true;
				}
				handled = true;
				break;
			case 'G':
				grid_mode = static_cast<GridMode>((static_cast<int>(grid_mode) + 1) % 4);
				on_set(&grid_mode);
				handled = true;
				break;
			case 'N':
				if(navigator_ptr) {
					bool visible = navigator_ptr->is_visible();
					navigator_ptr->set_visibility(!visible);
					handled = true;
				}
				break;
			case 'M':
				if(cm_viewer_ptr) {
					bool visible = cm_viewer_ptr->is_visible();
					cm_viewer_ptr->set_visibility(!visible);
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
			case 'O':
				show_bbox = !show_bbox;
				on_set(&show_bbox);
				handled = true;
				break;
			case 'W':
				show_wireframe_bbox = !show_wireframe_bbox;
				on_set(&show_wireframe_bbox);
				handled = true;
				break;
			default:
				break;
			}
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
					fh.file_name = dnd.filenames[0];
					update_member(&fh.file_name);
					on_set(&fh.file_name);
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

void tubes::on_set(void *member_ptr) {
	// dataset settings
	bool data_set_changed = false;
	bool from_demo = false;
	// - configurable datapath
	if(member_ptr == &datapath && !datapath.empty()) {
		from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
		traj_mgr.clear();
		cgv::utils::stopwatch s(true);
		std::cout << "Reading data set from " << datapath << " ..." << std::endl;
		if(traj_mgr.load(datapath) != -1) {
			std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
			dataset.files.clear();
			dataset.files.emplace(datapath);

			data_set_changed = true;
		}
	}
	// - non-configurable dataset logic
	else if(member_ptr == &dataset) {
		from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
		// clear current dataset
		datapath.clear();
		traj_mgr.clear();

		// load new data
		bool loaded_something = false;
		for(const auto &file : dataset.files) {
			cgv::utils::stopwatch s(true);
			std::cout << "Reading data set from " << file << " ..." << std::endl;
			loaded_something = traj_mgr.load(file) != -1 || loaded_something;
			std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
		}
		update_member(&datapath);

		// update render state
		if(loaded_something)
			data_set_changed = true;
	}

	if(data_set_changed) {
		render.data = &(traj_mgr.get_render_data());
		if(from_demo) {
			ao_style = ao_style_bak;
			update_member(&ao_style);
		}
		update_attribute_bindings();
		update_grid_ratios();

		update_glyph_layer_manager();

		compile_glyph_attribs();
		ah_mgr.set_dataset(traj_mgr.dataset(0));

		context &ctx = *get_context();
		tube_shading_defines = build_tube_shading_defines();
		shaders.reload(ctx, "tube_shading", tube_shading_defines);

		// reset glyph layer configuration file
		fh.file_name = "";
		fh.has_unsaved_changes = false;
		update_member(&fh.file_name);
		on_set(&fh.has_unsaved_changes);

		post_recreate_gui();
	}

	// render settings
	if( member_ptr == &debug.highlight_segments ||
		member_ptr == &ao_style.enable ||
		member_ptr == &grid_mode ||
		member_ptr == &grid_normal_settings ||
		member_ptr == &grid_normal_inwards ||
		member_ptr == &grid_normal_variant ||
		member_ptr == &enable_fuzzy_grid) {
		shader_define_map defines = build_tube_shading_defines();
		if(defines != tube_shading_defines) {
			context& ctx = *get_context();
			tube_shading_defines = defines;
			shaders.reload(ctx, "tube_shading", tube_shading_defines);
		}
	}
	
	if(member_ptr == &enable_taa) {
		accumulate = false;
		post_redraw();
	}

	if(member_ptr == &taa_mix_factor) {
		taa_mix_factor = cgv::math::clamp(taa_mix_factor, 0.0f, 1.0f);
		accumulate = false;
		post_redraw();
	}

	// - debug render setting
	if(member_ptr == &debug.force_initial_order) {
		update_attribute_bindings();
	}

	if(member_ptr == &debug.render_percentage) {
		debug.render_count = static_cast<size_t>(debug.render_percentage * debug.segment_count);
		update_member(&debug.render_count);
	}

	if(member_ptr == &debug.render_count) {
		debug.render_percentage = static_cast<float>(debug.render_count) / static_cast<float>(debug.segment_count);
		update_member(&debug.render_percentage);
	}

	if(member_ptr == &debug.render_mode) {
		update_debug_attribute_bindings();
	}

	// voxelization settings
	if(member_ptr == &voxel_grid_resolution || member_ptr == &voxelize_gpu || member_ptr == &render.style.radius_scale) {
		context& ctx = *get_context();
		voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(cgv::math::clamp(static_cast<unsigned>(voxel_grid_resolution), 16u, 512u));
		create_density_volume(ctx, voxel_grid_resolution);

		if(member_ptr == &render.style.radius_scale) {
			update_grid_ratios();
			glyphs_out_of_date(true);
		}
	}

	// visualization settings
	if(member_ptr == &glyph_layer_mgr) {
		const auto action = glyph_layer_mgr.action_type();
		bool changes = false;
		if(action == AT_CONFIGURATION_CHANGE) {
			glyph_layers_config = glyph_layer_mgr.get_configuration();

			context& ctx = *get_context();
			tube_shading_defines = build_tube_shading_defines();
			shaders.reload(ctx, "tube_shading", tube_shading_defines);

			compile_glyph_attribs();

			post_recreate_gui();
			
			changes = true;
		} else if(action == AT_CONFIGURATION_VALUE_CHANGE) {
			glyph_layers_config = glyph_layer_mgr.get_configuration();
			glyphs_out_of_date(true);
			changes = true;
		} else if(action == AT_MAPPING_VALUE_CHANGE) {
			glyphs_out_of_date(true);
			changes = true;
		}

		if(changes) {
			fh.has_unsaved_changes = true;
			on_set(&fh.has_unsaved_changes);
		}
	}

	if(member_ptr == &color_map_mgr) {
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
			glyph_layer_mgr.set_color_map_names(color_map_mgr.get_names());

			context& ctx = *get_context();
			color_map_mgr.update_texture(ctx);
			if(cm_viewer_ptr) {
				cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
				cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
			}

			post_recreate_gui();
		}
			break;
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
		fh.has_unsaved_changes = true;
		on_set(&fh.has_unsaved_changes);
	}






	if(member_ptr == &fh.file_name) {
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

		std::string extension = cgv::utils::file::get_extension(fh.file_name);
		// only try to read the filename if it ends with an xml extension
		if(cgv::utils::to_upper(extension) == "XML") {
			if(read_layer_configuration(fh.file_name)) {
				fh.has_unsaved_changes = false;
				on_set(&fh.has_unsaved_changes);
			} else {
				std::cout << "Error: could not read glyph layer configuration from " << fh.file_name << std::endl;
			}
		}
	}

	if(member_ptr == &fh.save_file_name) {
		std::string extension = cgv::utils::file::get_extension(fh.save_file_name);

		if(extension == "") {
			extension = "xml";
			fh.save_file_name += "." + extension;
		}

		if(cgv::utils::to_upper(extension) == "XML") {
			if(save_layer_configuration(fh.save_file_name)) {
				fh.file_name = fh.save_file_name;
				update_member(&fh.file_name);
				fh.has_unsaved_changes = false;
				on_set(&fh.has_unsaved_changes);
			} else {
				std::cout << "Error: Could not write glyph_ layer configuration to file: " << fh.save_file_name << std::endl;
			}
		} else {
			std::cout << "Please specify a xml file name." << std::endl;
		}
	}

	if(member_ptr == &fh.has_unsaved_changes) {
		auto ctrl = find_control(fh.file_name);
		if(ctrl)
			ctrl->set("text_color", fh.has_unsaved_changes ? cgv::gui::theme_info::instance().warning_hex() : "");
	}

	if (member_ptr == &include_hidden_glyphs)
		compile_glyph_attribs();

	// misc settings
	// - instant redraw
	if(member_ptr == &misc_cfg.instant_redraw_proxy)
		// ToDo: handle the (virtually impossible) case that some other plugin than cg_fltk provides the gl_context
		dynamic_cast<fltk_gl_view*>(get_context())->set_void("instant_redraw", "bool", member_ptr);
	// - vsync
	if(member_ptr == &misc_cfg.vsync_proxy)
		// ToDo: handle the (virtually impossible) case that some other plugin than cg_fltk provides the gl_context
		dynamic_cast<fltk_gl_view*>(get_context())->set_void("enable_vsynch", "bool", member_ptr);
	// - fix view up dir
	else if(member_ptr == &misc_cfg.fix_view_up_dir_proxy)
		// ToDo: make stereo view interactors reflect this property, and handle the case that some other plugin that
		//       is not derived from stereo_view_interactor handles viewing
		//if (!misc_cfg.fix_view_up_dir_proxy)
		//	dynamic_cast<stereo_view_interactor*>(find_view_as_node())->set("fix_view_up_dir", false);
		//else
		if(misc_cfg.fix_view_up_dir_proxy)
			find_view_as_node()->set_view_up_dir(0, 1, 0);

	if(member_ptr == &test_dir[0] || member_ptr == &test_dir[1] || member_ptr == &test_dir[2]) {
		test_dir = normalize(test_dir);
		update_member(&test_dir[0]);
		update_member(&test_dir[1]);
		update_member(&test_dir[2]);
	}

	// default implementation for all members
	// - remaining logic
	update_member(member_ptr);
	post_redraw();
}

bool tubes::on_exit_request() {
	// TODO: does not seem to fire when window is maximized?
#ifndef _DEBUG
	if(fh.has_unsaved_changes) {
		return cgv::gui::question("The glyph layer configuration has unsaved changes. Are you sure you want to quit?");
	}
#endif
	return true;
}


void tubes::reload_shader() {

	shaders.reload(*get_context(), "tube_shading", tube_shading_defines);
	post_redraw();
}

bool tubes::save_layer_configuration(const std::string& file_name) {
	const auto& gams = glyph_layer_mgr.ref_glyph_attribute_mappings();

	auto to_col_uint8 = [](const float& val) {
		int ival = cgv::math::clamp(static_cast<int>(255.0f * val + 0.5f), 0, 255);
		return static_cast<unsigned char>(ival);
	};

	auto vec2_to_str = [](const vec2& v) {
		return std::to_string(v.x()) + ", " + std::to_string(v.y());
	};

	auto rgb_to_str = [](const rgb& v) {
		return std::to_string(v.R()) + ", " + std::to_string(v.G()) + ", " + std::to_string(v.B());
	};

	auto put = [](const std::string& n, const std::string& v) {
		return n + "=\"" + v + "\" ";
	};

	std::string content = "";
	content += "<GlyphConfiguration>\n";
	std::string tab = "  ";
	std::string t = tab;

	const auto& color_maps = color_map_mgr.ref_color_maps();

	content += t + "<ColorMaps>\n";
	t += tab;

	for(size_t i = 0; i < color_maps.size(); ++i) {
		const auto& cmc = color_maps[i];
		if(cmc.custom) {
			content += cgv::glutil::color_map_writer::to_xml(cmc.name, cmc.cm, false);
		}

		/*const auto& cmc = color_maps[i];
		content += t + "<ColorMap " + put("name", cmc.name) + put("custom", std::to_string(cmc.custom));
		if(cmc.custom) {
			content += ">\n";
			t += tab;

			const auto& cm = cmc.cm;
			for(size_t j = 0; j < cm.ref_color_points().size(); ++j) {
				const auto& p = cm.ref_color_points()[j];

				content += t + "<Point ";
				content += "x=\"" + std::to_string(p.first) + "\" ";
				content += "o=\"1\" ";
				content += "r=\"" + std::to_string(p.second.R()) + "\" ";
				content += "g=\"" + std::to_string(p.second.G()) + "\" ";
				content += "b=\"" + std::to_string(p.second.B()) + "\"";
				content += "/>\n";
			}

			t = tab + tab;
			content += t + "</ColorMap>\n";
		} else {
			content += "/>\n";
		}*/
	}

	t = tab;
	content += t + "</ColorMaps>\n";
	content += t + "<Layers>\n";
	t += tab;

	for(size_t i = 0; i < gams.size(); ++i) {
		const auto& gam = gams[i];
		const auto* shape_ptr = gam.get_shape_ptr();

		if(shape_ptr) {
			content += t + "<Layer " + put("glyph", shape_ptr->name());

			switch(gam.get_sampling_strategy()) {
			case ASS_AT_SAMPLES:
				content += put("sampling", "original");
				break;
			case ASS_UNIFORM:
				content += put("sampling", "uniform");
				content += put("sampling_step", std::to_string(gam.get_sampling_step()));
				break;
			case ASS_EQUIDIST:
				content += put("sampling", "equidist");
				content += put("sampling_step", std::to_string(gam.get_sampling_step()));
				break;
			}

			content += ">\n";
			t += tab;

			const auto& attribs = shape_ptr->supported_attributes();
			const auto attrib_indices = gam.get_attrib_indices();
			const auto color_indices = gam.get_color_map_indices();
			const auto& mapping_ranges = gam.ref_attrib_values();
			const auto& colors = gam.ref_attrib_colors();

			const auto& attrib_names = gam.ref_attribute_names();
			const auto& color_map_names = gam.ref_color_map_names();

			for(size_t j = 0; j < attribs.size(); ++j) {
				const auto& attrib = attribs[j];
				content += t + "<Property " + put("name", attrib.name);

				if(!(attrib.modifiers & GAM_GLOBAL)) {
					int a_idx = attrib_indices[j];
					if(a_idx > -1)
						content += put("attrib_name", attrib_names[a_idx]);
				}

				if(attrib.type == GAT_COLOR) {
					int c_idx = color_indices[j];
					if(c_idx > -1)
						content += put("color_map_name", color_map_names[c_idx]);
					else
						content += put("color", rgb_to_str(colors[j]));
				}

				vec4 mr = mapping_ranges[j];
				if(attrib.modifiers & GAM_GLOBAL) {
					if(attrib.type != GAT_COLOR)
						content += put("value", std::to_string(mr.w()));
				} else {
					content += put("in_range", vec2_to_str(vec2(mr.x(), mr.y())));
				}
				if(attrib.type != GAT_UNIT &&
					attrib.type != GAT_SIGNED_UNIT &&
					attrib.type != GAT_COLOR &&
					attrib.type != GAT_OUTLINE
					) {
					content += put("out_range", vec2_to_str(vec2(mr.z(), mr.w())));
				}
				content += "/>\n";
			}

			t = tab + tab;
			content += t + "</Layer>\n";
		}
	}
	t = tab;
	content += t + "</Layers>\n";
	content += "</GlyphConfiguration>\n";

	return cgv::utils::file::write(file_name, content, true);
}

bool tubes::read_layer_configuration(const std::string& file_name) {
	if(!cgv::utils::file::exists(file_name) || cgv::utils::to_upper(cgv::utils::file::get_extension(file_name)) != "XML")
		return false;

	bool read_color_maps = false;
	std::vector<std::string> color_map_lines;

	bool read_layers = false;
	std::vector<cgv::utils::xml_tag> layer_data;

	std::string content;
	cgv::utils::file::read(file_name, content, true);

	bool read = true;
	size_t nl_pos = content.find_first_of("\n");
	size_t line_offset = 0;

	while(read) {
		std::string line = "";

		if(nl_pos == std::string::npos) {
			read = false;
			line = content.substr(line_offset, std::string::npos);
		} else {
			size_t next_line_offset = nl_pos;
			line = content.substr(line_offset, next_line_offset - line_offset);
			line_offset = next_line_offset + 1;
			nl_pos = content.find_first_of('\n', line_offset);
		}

		cgv::utils::xml_tag tag = cgv::utils::xml_read_tag(line);
		if(tag.type == cgv::utils::XTT_UNDEF)
			continue;

		if(tag.name == "GlyphConfiguration") {
			// root node indicates that this is a glyph layer configuration file
			// do nothing
		} else if(tag.name == "ColorMaps") {
			if(tag.type == cgv::utils::XTT_OPEN) {
				read_color_maps = true;
			} else if(tag.type == cgv::utils::XTT_CLOSE) {
				read_color_maps = false;
			}
		} else if(tag.name == "Layers") {
			if(tag.type == cgv::utils::XTT_OPEN) {
				read_layers = true;
			} else if(tag.type == cgv::utils::XTT_CLOSE) {
				read_layers = false;
			}
		}

		if(read_color_maps) {
			color_map_lines.push_back(line);
		}

		if(read_layers) {
			layer_data.push_back(tag);
		}
	}

	cgv::glutil::color_map_reader::result color_maps;
	if(cgv::glutil::color_map_reader::read_from_xml(color_map_lines, color_maps)) {
		// clear previous custom color maps
		std::vector<std::string> current_names;
		const auto& current_color_maps = color_map_mgr.ref_color_maps();
		for(size_t i = 0; i < current_color_maps.size(); ++i) {
			if(current_color_maps[i].custom)
				current_names.push_back(current_color_maps[i].name);
		}

		for(size_t i = 0; i < current_names.size(); ++i)
			color_map_mgr.remove_color_map_by_name(current_names[i]);

		// add new custom color maps
		for(const auto& entry : color_maps)
			color_map_mgr.add_color_map(entry.first, entry.second, true);
	}
	
	// get a list of color map names
	std::vector<std::string> color_map_names = color_map_mgr.get_names();

	const auto index_of = [](const std::vector<std::string>& v, const std::string& elem) {
		int idx = -1;
		int c = 0;
		for(const auto& s : v) {
			if(s == elem) {
				idx = c;
				break;
			}
			++c;
		}
		return idx;
	};
	
	const auto read_vec2 = [](const std::string& str) {
		size_t space_pos = str.find_first_of(" ");
		size_t last_space_pos = 0;

		vec2 v(0.0f);
		std::string value_str = str.substr(last_space_pos, space_pos - last_space_pos);
		v[0] = std::strtof(value_str.c_str(), nullptr);

		last_space_pos = space_pos + 1;
		value_str = str.substr(space_pos + 1);
		v[1] = std::strtof(value_str.c_str(), nullptr);
		return v;
	};

	const auto read_vec3 = [](const std::string& str) {
		size_t space_pos = str.find_first_of(" ");
		size_t last_space_pos = 0;

		vec3 v(0.0f);
		std::string value_str = str.substr(last_space_pos, space_pos - last_space_pos);
		v[0] = std::strtof(value_str.c_str(), nullptr);

		last_space_pos = space_pos + 1;
		space_pos = str.find_first_of(" ", last_space_pos);
		value_str = str.substr(last_space_pos, space_pos - last_space_pos);
		v[1] = std::strtof(value_str.c_str(), nullptr);

		last_space_pos = space_pos + 1;
		value_str = str.substr(space_pos + 1);
		v[2] = std::strtof(value_str.c_str(), nullptr);
		return v;
	};


	glyph_layer_mgr.clear();

	const auto attrib_names = traj_mgr.dataset(0).get_attribute_names();

	bool read_layer = false;
	glyph_attribute_mapping gam;
	const glyph_shape* shape_ptr = nullptr;
	std::vector<std::string> shape_attribute_names;

	std::vector<std::pair<int, vec2>> input_ranges;

	for(size_t i = 0; i < layer_data.size(); ++i) {
		const cgv::utils::xml_tag& tag = layer_data[i];
		const auto end = tag.attributes.end();

		if(tag.name == "Layer") {
			if(tag.type == cgv::utils::XTT_OPEN) {
				read_layer = true;
				gam = glyph_attribute_mapping();
				shape_ptr = nullptr;
				shape_attribute_names.clear();
				input_ranges.clear();

				auto it = tag.attributes.find("glyph");
				if(it != end) {
					GlyphType glyph_type = glyph_type_registry::type((*it).second);
					gam.set_glyph_type(glyph_type);
					shape_ptr = gam.get_shape_ptr();

					for(const auto& a : shape_ptr->supported_attributes())
						shape_attribute_names.push_back(a.name);
				}

				it = tag.attributes.find("sampling");
				if(it != end) {
					std::string str = (*it).second;
					if(str == "original") {
						gam.set_sampling_strategy(ASS_AT_SAMPLES);
					} else if(str == "uniform") {
						gam.set_sampling_strategy(ASS_UNIFORM);
					} else if (str == "equidist") {
						gam.set_sampling_strategy(ASS_EQUIDIST);
					}
				}

				it = tag.attributes.find("sampling_step");
				if(it != end) {
					std::string str = (*it).second;
					float step = std::strtof(str.c_str(), nullptr);
					gam.set_sampling_step(step);
				}
			} else if(tag.type == cgv::utils::XTT_CLOSE) {
				if(read_layer) {
					glyph_layer_mgr.add_glyph_attribute_mapping(gam);

					auto& last_gam = const_cast<glyph_attribute_mapping&>(glyph_layer_mgr.ref_glyph_attribute_mappings().back());

					for(const auto& p : input_ranges)
						if(p.first > -1)
							last_gam.set_attrib_in_range(p.first, p.second);
				}
				read_layer = false;
			}
		} else if(tag.name == "Property") {
			if(read_layer && shape_ptr) {
				auto it = tag.attributes.find("name");
				if(it == end)
					// property has no name, skip
					continue;

				std::string name = (*it).second;
				int shape_attrib_idx = index_of(shape_attribute_names, name);
				if(shape_attrib_idx < 0)
					// shape does not have this attribute
					continue;

				const auto& attrib = shape_ptr->supported_attributes()[shape_attrib_idx];

				int attrib_idx = -1;
				it = tag.attributes.find("attrib_name");
				if(it != end) {
					// the name for a mapped attribute is given
					std::string attrib_name = (*it).second;
					// if not -1, then this attribute is also present in the loaded data set
					attrib_idx = index_of(attrib_names, attrib_name);
				}

				if(attrib_idx < 0) {
					// the shape attribute is not mapped
				} else {
					// a data set attribute is mapped to the shape attribute
					gam.set_attrib_source_index(static_cast<size_t>(shape_attrib_idx), attrib_idx);
				}

				if(attrib.modifiers && GAM_GLOBAL) {
					it = tag.attributes.find("value");
					if(it != end) {
						std::string str = (*it).second;
						vec2 r(0.0f);
						r.y() = std::strtof(str.c_str(), nullptr);
						gam.set_attrib_out_range(shape_attrib_idx, r);
					}
				}
				
				it = tag.attributes.find("in_range");
				if(it != end) {
					std::string str = (*it).second;
					vec2 r = read_vec2(str);
					gam.set_attrib_in_range(shape_attrib_idx, r);
					input_ranges.push_back({ shape_attrib_idx, r });
				} else {
					input_ranges.push_back({ -1, vec2(0.0f) });
				}

				if( attrib.type != GAT_UNIT &&
					attrib.type != GAT_SIGNED_UNIT &&
					attrib.type != GAT_COLOR &&
					attrib.type != GAT_OUTLINE
					) {
					it = tag.attributes.find("out_range");
					if(it != end) {
						std::string str = (*it).second;
						vec2 r = read_vec2(str);
						gam.set_attrib_out_range(shape_attrib_idx, r);
					}
				}

				if(shape_ptr->supported_attributes()[shape_attrib_idx].type == GAT_COLOR) {
					int color_map_idx = -1;
					it = tag.attributes.find("color_map_name");
					if(it != end) {
						// the name for a color map is given
						std::string color_map_name = (*it).second;
						// if not -1, then this attribute is also present in the loaded data set
						color_map_idx = index_of(color_map_names, color_map_name);
					}

					if(color_map_idx < 0) {
						// no color map selected
						it = tag.attributes.find("color");
						if(it != end) {
							std::string col_str = (*it).second;
							vec3 v = read_vec3(col_str);
							rgb col(v.x(), v.y(), v.z());
							gam.set_attrib_color(shape_attrib_idx, col);
						}
					} else {
						// a color map is selected
						gam.set_color_source_index(shape_attrib_idx, color_map_idx);
					}
				}
			}
		}
	}

	// update the dependant members
	color_map_mgr.update_texture(*get_context());
	if(cm_viewer_ptr) {
		cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
		cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	glyph_layer_mgr.set_color_map_names(color_map_names);
	glyph_layer_mgr.notify_configuration_change();
	
	return true;
}

void tubes::update_glyph_layer_manager() {
	if(!traj_mgr.has_data()) {
		std::cout << "Warning: update_glyph_layer_manager - trajectory manager has no data" << std::endl;
		return;
	}

	auto attrib_names = traj_mgr.dataset(0).get_attribute_names();
	std::vector<vec2> attrib_ranges;

	// collect value ranges for available attributes
	for(size_t i = 0; i < attrib_names.size(); ++i) {
		const auto& attrib = traj_mgr.dataset(0).attribute(attrib_names[i]);
		vec2 range(attrib.min(), attrib.max());
		attrib_ranges.push_back(range);
	}

	// clear old configuration of glyph layers and reset shader
	glyph_layer_mgr.clear();
	// set new information of available attributes and ranges
	glyph_layer_mgr.set_attribute_names(attrib_names);
	glyph_layer_mgr.set_attribute_ranges(attrib_ranges);
	glyph_layer_mgr.set_color_map_names(color_map_mgr.get_names());
}

void tubes::timer_event(double t, double dt)
{
	if (has_changed && t > change_time + recalc_delay)
		compile_glyph_attribs();
}

void tubes::glyphs_out_of_date(bool state) 
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

bool tubes::compile_glyph_attribs (void)
{
	bool success = false;
	if(glyph_layers_config.layer_configs.size() > 0) {
		cgv::utils::stopwatch s(true);
		std::cout << "Compiling glyph attributes... ";

		glyph_compiler gc;
		gc.length_scale = general_settings.length_scale;
		gc.include_hidden_glyphs = include_hidden_glyphs;

		const auto &dataset = traj_mgr.dataset(0);

		success = gc.compile_glyph_attributes(dataset, render.arclen_data, glyph_layers_config);

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

	if(success)
		post_redraw();

	return success;
}

bool tubes::init (cgv::render::context &ctx)
{
#ifdef CGV_FORCE_STATIC
	std::string app_path = cgv::base::ref_prog_path_prefix();
#else
	std::string app_path = QUOTE_SYMBOL_VALUE(INPUT_DIR);
	app_path += "/";
#endif

	// increase reference count of the renderers by one
	auto &tstr = ref_textured_spline_tube_renderer(ctx, 1);
	auto &vr = ref_volume_renderer(ctx, 1);
	bool success = tstr.ref_prog().is_linked() && vr.ref_prog().is_linked();

	// load all shaders in the library
	success &= shaders.load_shaders(ctx);

	color_map_mgr.init(ctx);
	glyph_layers_config = glyph_layer_mgr.get_configuration();

	tube_shading_defines = build_tube_shading_defines();
	shaders.reload(ctx, "tube_shading", tube_shading_defines);

	// init shared attribute array manager
	success &= render.aam.init(ctx);

	success &= density_volume.init(ctx, 0);

	render.sorter = new cgv::glutil::radix_sort_4way();
	render.sorter->set_data_type_override("vec4 pos_rad; vec4 color; vec4 tangent;");
	render.sorter->set_auxiliary_type_override("uint a_idx; uint b_idx;");

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

	render.sorter->set_key_definition_override(key_definition);

	ref_box_renderer(ctx, 1);
	ref_box_wire_renderer(ctx, 1);
	bbox_rd.init(ctx);
	bbox_wire_rd.init(ctx);

	// initialize debug renderer
	ref_sphere_renderer(ctx, 1);
	ref_cone_renderer(ctx, 1);
	srd.init(ctx);
	debug.node_rd.init(ctx);
	debug.segment_rd.init(ctx);

	// enable ambient occlusion
	ao_style.enable = true;

	// generate demo
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
	update_attribute_bindings();
	update_grid_ratios();

	// load sequential color maps from the resource directory
	std::string color_maps_path = app_path + "res/color_maps/sequential";
	if(std::filesystem::exists(color_maps_path)) {
		for(const auto &entry : std::filesystem::directory_iterator(color_maps_path)) {
			std::filesystem::path entry_path = entry.path();
			// only take xml files
			if(entry_path.extension() == ".xml") {
				cgv::glutil::color_map_reader::result color_maps;
				if(cgv::glutil::color_map_reader::read_from_xml(entry_path.string(), color_maps))
					for(const auto& entry : color_maps)
						color_map_mgr.add_color_map(entry.first, entry.second, false);
			}
		}
	}

	// load diverging color maps from the resource directory
	color_maps_path = app_path + "res/color_maps/diverging";
	if(std::filesystem::exists(color_maps_path)) {
		for(const auto &entry : std::filesystem::directory_iterator(color_maps_path)) {
			std::filesystem::path entry_path = entry.path();
			// only take xml files
			if(entry_path.extension() == ".xml") {
				cgv::glutil::color_map_reader::result color_maps;
				if(cgv::glutil::color_map_reader::read_from_xml(entry_path.string(), color_maps))
					for(const auto& entry : color_maps)
						color_map_mgr.add_color_map(entry.first, entry.second, false);
			}
		}
	}

	// load sequential RAINBOW color map
	if(std::filesystem::exists(app_path+"res/color_maps/RAINBOW.xml")) {
		cgv::glutil::color_map_reader::result color_maps;
		if(cgv::glutil::color_map_reader::read_from_xml(app_path + "res/color_maps/RAINBOW.xml", color_maps))
			for(const auto& entry : color_maps)
				color_map_mgr.add_color_map(entry.first, entry.second, false);
	}

	color_map_mgr.update_texture(ctx);
	if(cm_viewer_ptr) {
		cm_viewer_ptr->set_color_map_names(color_map_mgr.get_names());
		cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	update_glyph_layer_manager();

	compile_glyph_attribs();
	ah_mgr.set_dataset(traj_mgr.dataset(0));

	volume_tf.init(ctx);

	// use white background for paper screenshots
	//ctx.set_bg_color(1.0f, 1.0f, 1.0f, 1.0f);

	// done
	return success;
}

void tubes::init_frame (cgv::render::context &ctx)
{
	if(!view_ptr) {
		view_ptr = find_view_as_node();
		if(view_ptr) {
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
		}
	}

	if (misc_cfg.fix_view_up_dir_proxy && view_ptr)
		// ToDo: make stereo view interactors reflect this property
		/*dynamic_cast<stereo_view_interactor*>(find_view_as_node())->set(
			"fix_view_up_dir", misc_cfg.fix_view_up_dir_proxy
		);*/
		view_ptr->set_view_up_dir(0, 1, 0);

	// keep the framebuffer up to date with the viewport size
	// TODO: check for changes and reset accumulation
	bool updated = false;
	updated |= fbc.ensure(ctx);
	updated |= fbc_shading.ensure(ctx);
	updated |= fbc_post.ensure(ctx);
	updated |= fbc_hist.ensure(ctx);
	updated |= fbc_final.ensure(ctx);
	if(updated) {
		accumulate = false;

		float w = static_cast<float>(fbc.ref_frame_buffer().get_width());
		float h = static_cast<float>(fbc.ref_frame_buffer().get_height());
		vec2 view_size(w, h);

		jitter_offsets.clear();
		for(size_t i = 0; i < n_jitter_samples; ++i) {
			vec2 sample = sample_halton_2d(static_cast<unsigned>(i+1), 2, 3);
			vec2 offset = (2.0f * sample - 1.0f) / view_size;
			jitter_offsets.push_back(offset);
		}
	}

	// query the current viewport dimensions as this is needed for multiple draw methods
	glGetIntegerv(GL_VIEWPORT, viewport);

	/* Measurements:
	RTX 2080Ti
	>> HotRoom <<
	Average FPS(ms) over 10s long full horizontal rotation of data set
	(3 g-buffers) (quadratic tangents)
	Silhouette		| Full Ray Cast	| Geometry Only
	Box				| 292.49 (3.42)	| 719.43 (1.39)
	Approx. Billb.	| 462.27 (2.16)	| 985.90 (1.01)
	Exact Polygon	| 429.63 (2.33)	| 962.18 (1.04)
	Box Billboard	| 448.61 (2.23)	| 994.98 (1.01)
	*/

	if(benchmark.running) {
		++benchmark.total_frames;
		double benchmark_time = 10.0;

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

			std::cout << ss.str() << std::endl;
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
		}
	}

	if(cm_editor_ptr && cm_editor_ptr->was_updated()) {
		color_map_mgr.update_texture(ctx);
		if(cm_viewer_ptr)
			cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	if(tf_editor_ptr && tf_editor_ptr->was_updated()) {
		volume_tf.generate_texture(ctx);
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
		debug.far_extent_factor = 0.4;
		debug.near_extent_factor = 0.333333*debug.far_extent_factor;
		set_view();
		//grid_mode = GM_COLOR_NORMAL;
		//on_set(&grid_mode);
		benchmark_mode_setup = true;
	}
}

void tubes::draw (cgv::render::context &ctx)
{
	if(!view_ptr) return;

	//srd.render(ctx, ref_sphere_renderer(ctx), sphere_render_style());

	// draw dataset using selected renderer
	if(show_volume) {
		draw_density_volume(ctx);
	} else {
		if(traj_mgr.has_data()) {

			int debug_idx_count = static_cast<int>(render.data->indices.size());
			if(debug.limit_render_count)
				debug_idx_count = static_cast<int>(2 * debug.render_count);

			switch(debug.render_mode) {
			case DRM_NONE:
				draw_trajectories(ctx);
				break;
			case DRM_NODES:
				debug.node_rd.render(ctx, ref_sphere_renderer(ctx), debug.node_rs, 0, debug_idx_count);
				break;
			case DRM_SEGMENTS:
				debug.segment_rd.render(ctx, ref_cone_renderer(ctx), debug.segment_rs, 0, debug_idx_count);
				break;
			case DRM_NODES_SEGMENTS:
				debug.node_rd.render(ctx, ref_sphere_renderer(ctx), debug.node_rs, 0, debug_idx_count);
				debug.segment_rd.render(ctx, ref_cone_renderer(ctx), debug.segment_rs, 0, debug_idx_count);
				break;
			default: break;
			}

			if (show_wireframe_bbox)
				bbox_wire_rd.render(ctx, ref_box_wire_renderer(ctx), box_wire_render_style());

			if (show_bbox)
				bbox_rd.render(ctx, ref_box_renderer(ctx), bbox_style);
		}

		//srd.render(ctx, ref_sphere_renderer(ctx), sphere_render_style());
	}

	// display drag-n-drop information, if a dnd operation is in progress
	if(!dnd.text.empty())
		draw_dnd(ctx);
}

void tubes::create_gui(void) {
	// dataset settings
	add_decorator("Dataset", "heading", "level=1");
	//add_member_control(this, "data file/path", datapath);
	add_gui("Data Path", datapath, "file_name", "title='Open Trajectory Data';"
		"filter='Trajectory Files (bezdat, csv, sepia, obd, tgen):*.bezdat;*.csv;*.sepia;*.ipcdf;*.ppcdf;*.tgen|All Files:*.*';"
		"small_icon=true;w=168");

	// rendering settings
	add_decorator("Rendering", "heading", "level=1");
	if (begin_tree_node("Bounds", show_bbox, false)) {
		align("\a");
		add_member_control(this, "Color", bbox_style.surface_color);
		add_member_control(this, "Show", show_bbox, "check");
		add_member_control(this, "Show Wireframe", show_wireframe_bbox, "check");
		if (begin_tree_node("Bounds", bbox_style, false)) {
			align("\a");
			add_gui("box style", bbox_style);
			align("\b");
			end_tree_node(bbox_style);
		}
		align("\b");
		end_tree_node(show_bbox);
	}

	if(begin_tree_node("Tube Style", render.style, false)) {
		align("\a");
		add_gui("tube_style", render.style);
		align("\b");
		end_tree_node(render.style);
	}

	if(begin_tree_node("TAA", enable_taa, false)) {
		align("\a");
		add_member_control(this, "Enable", enable_taa, "toggle");
		add_member_control(this, "Mix Factor", taa_mix_factor, "value_slider", "min=0;max=1;step=0.001");
		add_member_control(this, "Jitter Scale", jitter_scale, "value_slider", "min=0;max=1;step=0.001");

		add_decorator("Resolve Settings", "heading", "level=3");
		add_member_control(this, "Use Velocity", settings.use_velocity, "check");
		add_member_control(this, "Use Closest Depth", settings.closest_depth, "check");
		add_member_control(this, "Clip Color", settings.clip_color, "check");
		add_member_control(this, "Static No-Clip", settings.static_no_clip, "check");

		add_member_control(this, "Enable FXAA", enable_fxaa, "toggle");
		add_member_control(this, "FXAA Mix Factor", fxaa_mix_factor, "value_slider", "min=0;max=1;step=0.0001");

		align("\b");
		end_tree_node(enable_taa);
	}

	if(begin_tree_node("AO Style", ao_style, false)) {
		align("\a");
		add_gui("ao_style", ao_style);
		add_member_control(this, "Voxel Grid Resolution", voxel_grid_resolution, "dropdown", "enums='16=16, 32=32, 64=64, 128=128, 256=256, 512=512'");
		add_member_control(this, "Voxelize using GPU", voxelize_gpu, "check");
		align("\b");
		end_tree_node(ao_style);
	}

	// attribute mapping settings
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

	if(begin_tree_node("General Settings", general_settings, false)) {
		align("\a");
		add_member_control(this, "Curvature Correction", general_settings.use_curvature_correction, "check");
		add_member_control(this, "Length Scale", general_settings.length_scale, "value_slider", "min=0.1;max=10;step=0.01;ticks=true;color=0xb51c1c");
		add_member_control(this, "Antialias Radius", general_settings.antialias_radius, "value_slider", "min=0;max=5;step=0.01;ticks=true");
		align("\b");
		end_tree_node(glyph_layer_mgr);
	}

	if(begin_tree_node("Attribute Mapping", glyph_layer_mgr, true)) {
		align("\a");
		add_decorator("Configuration File", "heading", "level=3");
		std::string filter = "XML Files (xml):*.xml|All Files:*.*";
		add_gui("File", fh.file_name, "file_name", "title='Open Transfer Function';filter='" + filter + "';save=false;w=136;small_icon=true;align_gui=' '" + (fh.has_unsaved_changes ? ";text_color=" + cgv::gui::theme_info::instance().warning_hex() : ""));
		add_gui("save_file_name", fh.save_file_name, "file_name", "title='Save Transfer Function';filter='" + filter + "';save=true;control=false;small_icon=true");
		add_decorator("", "separator", "level=3");
		connect_copy(add_button("Reload Shader")->click, cgv::signal::rebind(this, &tubes::reload_shader));
		connect_copy(add_button("Compile Attributes")->click, cgv::signal::rebind(this, &tubes::compile_glyph_attribs));
		add_member_control(this, "Max Count (Debug)", max_glyph_count, "value_slider", "min=1;max=100;step=1;ticks=true");
		add_member_control(this, "Show Hidden Glyphs", include_hidden_glyphs, "check");
		glyph_layer_mgr.create_gui(this, *this);
		align("\b");
		end_tree_node(glyph_layer_mgr);
	}

	if(begin_tree_node("Color Scales", cm_editor_ptr, false)) {
		align("\a");
		color_map_mgr.create_gui(this, *this);
		inline_object_gui(cm_editor_ptr);
		align("\b");
		end_tree_node(cm_editor_ptr);
	}

	if(begin_tree_node("Color Scale Viewer", cm_viewer_ptr, false)) {
		align("\a");
		inline_object_gui(cm_viewer_ptr);
		align("\b");
		end_tree_node(cm_viewer_ptr);
	}

	if(begin_tree_node("Navigator", navigator_ptr, false)) {
		align("\a");
		inline_object_gui(navigator_ptr);
		align("\b");
		end_tree_node(navigator_ptr);
	}

	// Misc settings contractable section
	add_decorator("Miscellaneous", "heading", "level=1");
	if(begin_tree_node("Volume Style", vstyle, false)) {
		align("\a");
		add_member_control(this, "Show Volume", show_volume, "check");
		add_gui("vstyle", vstyle);

		if(begin_tree_node("Transfer Function Editor", tf_editor_ptr, false)) {
			align("\a");
			inline_object_gui(tf_editor_ptr);
			align("\b");
			end_tree_node(tf_editor_ptr);
		}

		align("\b");
		end_tree_node(vstyle);
	}

	if(begin_tree_node("Tools (Persistent by Default)", misc_cfg, false)) {
		align("\a");
		add_member_control(
			this, "instant_redraw_proxy", misc_cfg.instant_redraw_proxy, "toggle",
			"tooltip='Controls the instant redraw state of the FLTK GL window.'"
		);
		add_member_control(
			this, "vsync_redraw_proxy", misc_cfg.vsync_proxy, "toggle",
			"tooltip='Controls the vsync state of the FLTK GL window.'"
		);
		add_member_control(
			this, "fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy, "toggle",
			"tooltip='Controls the \"fix_view_up_dir\" state of the view interactor.'"
		);
		align("\b");
		end_tree_node(misc_cfg);
	}

	if(begin_tree_node("(Debug)", benchmark, false)) {
		align("\a");
		add_member_control(this, "Render Mode", debug.render_mode, "dropdown", "enums='Default, Nodes, Segments, Nodes + Segments'");
		add_member_control(this, "Show Segments", debug.highlight_segments, "check");

		add_member_control(this, "Sort by Distance", debug.sort, "check");
		add_member_control(this, "Force Initial Order", debug.force_initial_order, "check");

		add_member_control(this, "Limit Render Count", debug.limit_render_count, "check");
		add_member_control(this, "Render Percentage", debug.render_percentage, "value_slider", "min=0.0;step=0.001;max=1.0;ticks=true");
		add_member_control(this, "Render Count", debug.render_count, "value", "w=70;min=0;step=1;max=" + std::to_string(debug.segment_count), " ");
		add_member_control(this, "", debug.render_count, "wheel", "w=120;min=0;step=1;max=" + std::to_string(debug.segment_count));
		
		add_member_control(this, "Start Benchmark", benchmark.requested, "toggle", "");
		create_vec3_gui("Eye Pos", test_eye, -10.0f, 10.0f);
		create_vec3_gui("Eye Dir", test_dir, -1.0f, 1.0f);
		align("\b");
		end_tree_node(benchmark);
	}
}

void tubes::create_vec3_gui(const std::string& name, vec3& value, float min, float max) {

	std::string value_config = "w=55;min=" + std::to_string(min) + ";max=" + std::to_string(max);
	std::string slider_config = "w=55;min=" + std::to_string(min) + ";max=" + std::to_string(max) + ";step=0.0001;ticks=true";

	add_member_control(this, name, value[0], "value", value_config, " ");
	add_member_control(this, "", value[1], "value", value_config, " ");
	add_member_control(this, "", value[2], "value", value_config);
	add_member_control(this, "", value[0], "slider", slider_config, " ");
	add_member_control(this, "", value[1], "slider", slider_config, " ");
	add_member_control(this, "", value[2], "slider", slider_config);
}

void tubes::set_view(void)
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
}

void tubes::update_grid_ratios(void) {
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
		grids[0].scaling.x() = general_settings.length_scale = 1.f / float(mean_rad);
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
		update_member(&general_settings.length_scale);
	}
}

void tubes::update_attribute_bindings(void) {
	auto &ctx = *get_context();

	if (traj_mgr.has_data())
	{
		calculate_bounding_box(); 
		set_view();

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
				/* .tangent */  render.data->tangents[i]  // <- does already contain radius deriv. in w-component
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
		auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);
		tstr.enable_attribute_array_manager(ctx, render.aam);
		tstr.set_node_id_array(ctx, reinterpret_cast<const uvec2*>(render.data->indices.data()), segment_count, sizeof(uvec2));
		tstr.set_indices(ctx, segment_indices);
		tstr.disable_attribute_array_manager(ctx, render.aam);

		if(!render.sorter->init(ctx, render.data->indices.size() / 2))
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
}

void tubes::update_debug_attribute_bindings() {
	auto &ctx = *get_context();

	auto& nodes = debug.node_rd;
	auto& segments = debug.segment_rd;

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

				nodes.add(render.data->positions[i], render.data->radii[i], col);
				segments.adds(render.data->positions[i]);
				segments.adds(radius_scale * render.data->radii[i]);
				segments.adds(col);
			}
			// also use index buffer for nodes even though this means we render most nodes twice
			// this is necessary for correct usage of the render count limit
			nodes.ref_idx() = render.data->indices;
			segments.ref_idx() = render.data->indices;
		}
	}
}

void tubes::calculate_bounding_box(void) {

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

void tubes::create_density_volume(context& ctx, unsigned resolution) {

	density_volume.initialize_voxel_grid(bbox, resolution);
	ivec3 res = density_volume.ref_voxel_grid().resolution;
	std::cout << "Generating density volume with resolution (" << res.x() << ", " << res.y() << ", " << res.z() << ")... ";

	if(density_tex.is_created()) {
		if(res.x() != density_tex.get_width() || res.y() != density_tex.get_height() || res.z() != density_tex.get_depth()) {
			density_tex.destruct(ctx);
		}
	}

	if(!density_tex.is_created()) {
		std::vector<float> density_data(res.x()*res.y()*res.z(), 0.5f);

		cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res.x(), res.y(), res.z(), TI_FLT32, cgv::data::CF_R), density_data.data());
		density_tex = texture("flt32[R,G,B]", TF_LINEAR, TF_LINEAR_MIPMAP_LINEAR, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER);
		density_tex.create(ctx, dv, 0);
		density_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);
		density_tex.generate_mipmaps(ctx);
	}

	cgv::utils::stopwatch s(true);

	if(voxelize_gpu) {
		// prepare SSBO and index buffer handles
		auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);
		const int node_idx_handle = tstr.get_vbo_handle(ctx, render.aam, "node_ids");
		const int data_handle = render.render_sbo.handle ? (const int&)render.render_sbo.handle - 1 : 0;
		if(node_idx_handle > 0 && data_handle > 0)
			density_volume.compute_density_volume_gpu(ctx, render.data, render.style.radius_scale, node_idx_handle, data_handle, density_tex);
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

tubes::vec2 tubes::sample_halton_2d(unsigned k, int base1, int base2) {

	return vec2(van_der_corput(k, base1), van_der_corput(k, base2));
}
float tubes::van_der_corput(int n, int base) {

	float vdc = 0.0f;
	int denominator = 1;

	while(n > 0) {
		denominator *= base;
		int remainder = n % base;
		n /= base;
		vdc += remainder / static_cast<float>(denominator);
	}

	return vdc;
}

void tubes::draw_dnd(context& ctx) {
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

void tubes::draw_trajectories(context& ctx) {

	vec3 eye_pos = view_ptr->get_eye();
	const vec3& view_dir = view_ptr->get_view_dir();
	vec2 viewport_size(
		static_cast<float>(fbc.ref_frame_buffer().get_width()),
		static_cast<float>(fbc.ref_frame_buffer().get_height())
	);

	// enable drawing framebuffer
	fbc.enable(ctx);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// shear the projection matrix to jitter vertex positions
	dmat4 m;
	m.identity();

	vec2 jitter_offset = jitter_scale * jitter_offsets[accumulate_count];
	m(0, 2) = jitter_offset.x();
	m(1, 2) = jitter_offset.y();

	ctx.push_projection_matrix();
	ctx.mul_projection_matrix(m);

	// render tubes
	auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);

	// prepare SSBO handles
	const int data_handle = render.render_sbo.handle ? (const int&)render.render_sbo.handle - 1 : 0,
			  arclen_handle = render.arclen_sbo.handle ? (const int&)render.arclen_sbo.handle - 1 : 0;
	if (!data_handle || !arclen_handle) return;

	// sort the sgment indices
	int segment_idx_handle = tstr.get_index_buffer_handle(render.aam);
	int node_idx_handle = tstr.get_vbo_handle(ctx, render.aam, "node_ids");	
	if(data_handle > 0 && segment_idx_handle > 0 && node_idx_handle > 0 && debug.sort & !debug.force_initial_order)
		//render.sorter->sort(ctx, data_handle, segment_idx_handle, test_eye, test_dir, node_idx_handle);
		render.sorter->sort(ctx, data_handle, segment_idx_handle, eye_pos, view_dir, node_idx_handle);

	tstr.set_eye_pos(eye_pos);
	tstr.set_view_dir(view_dir);
	tstr.set_viewport(vec4((float)viewport[0], (float)viewport[1], (float)viewport[2], (float)viewport[3]));
	tstr.set_render_style(render.style);
	tstr.enable_attribute_array_manager(ctx, render.aam);

	int count = static_cast<int>(render.data->indices.size() / 2);
	if(debug.limit_render_count) {
		//count = static_cast<int>(render.percentage * count);
		count = static_cast<int>(debug.render_count);
	}
	
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, data_handle);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, arclen_handle);
	if (render.style.attrib_mode != textured_spline_tube_render_style::AM_ALL) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, node_idx_handle);
		tstr.render(ctx, 0, count);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
	}
	else
		tstr.render(ctx, 0, count);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);

	tstr.disable_attribute_array_manager(ctx, render.aam);

	// store the current matrices to use as the previous for the next frame
	mat4 curr_modelview_matrix = ctx.get_modelview_matrix();

	ctx.pop_projection_matrix();

	// need to store the projection matrix without jitter
	mat4 curr_projection_matrix = ctx.get_projection_matrix();
	
	// disable the drawing framebuffer
	fbc.disable(ctx);

	



	// perform the deferred shading pass and draw the image into the shading framebuffer
	fbc_shading.enable(ctx);
	glDepthFunc(GL_ALWAYS);
	
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
	for(const auto& p : glyph_layers_config.constant_float_parameters)
		prog.set_uniform(ctx, p.first, *p.second);

	for(const auto& p : glyph_layers_config.constant_color_parameters)
		prog.set_uniform(ctx, p.first, *p.second);

	for(const auto& p : glyph_layers_config.mapping_parameters)
		prog.set_uniform(ctx, p.first, *p.second);

	// map global settings
	prog.set_uniform(ctx, "general_settings.use_curvature_correction", general_settings.use_curvature_correction);
	prog.set_uniform(ctx, "general_settings.length_scale", general_settings.length_scale);
	prog.set_uniform(ctx, "general_settings.antialias_radius", general_settings.antialias_radius);

	const surface_render_style& srs = *static_cast<const surface_render_style*>(&render.style);
	
	prog.set_uniform(ctx, "map_color_to_material", int(srs.map_color_to_material));
	prog.set_uniform(ctx, "culling_mode", int(srs.culling_mode));
	prog.set_uniform(ctx, "illumination_mode", int(srs.illumination_mode));

	fbc.enable_attachment(ctx, "albedo", 0);
	fbc.enable_attachment(ctx, "position", 1);
	fbc.enable_attachment(ctx, "normal", 2);
	fbc.enable_attachment(ctx, "tangent", 3);
	fbc.enable_attachment(ctx, "depth", 4);
	if(ao_style.enable)
		density_tex.enable(ctx, 5);
	color_map_mgr.ref_texture().enable(ctx, 6);

	// bind range attribute sbos of active glyph layers
	bool active_sbos[4] = { false, false, false, false };
	for(size_t i = 0; i < glyph_layers_config.layer_configs.size(); ++i) {
		if(glyph_layers_config.layer_configs[i].mapped_attributes.size() > 0) {
			const int attribs_handle = render.attribs_sbos[i].handle ? (const int&)render.attribs_sbos[i].handle - 1 : 0;
			const int aindex_handle = render.aindex_sbos[i].handle ? (const int&)render.aindex_sbos[i].handle - 1 : 0;
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2*(GLuint)i + 0, attribs_handle);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2*(GLuint)i + 1, aindex_handle);
		}
	}

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	for(size_t i = 0; i < 4; ++i) {
		if(active_sbos[i]) {
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2*(GLuint)i + 0, 0);
			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2*(GLuint)i + 1, 0);
		}
	}

	fbc.disable_attachment(ctx, "albedo");
	fbc.disable_attachment(ctx, "position");
	fbc.disable_attachment(ctx, "normal");
	fbc.disable_attachment(ctx, "tangent");
	fbc.disable_attachment(ctx, "depth");
	if(ao_style.enable)
		density_tex.disable(ctx);
	color_map_mgr.ref_texture().disable(ctx);

	prog.disable(ctx);

	// disable the shading framebuffer
	fbc_shading.disable(ctx);





	if(enable_fxaa) {
		fbc_post.enable(ctx);

		auto& fxaa_prog = shaders.get("fxaa");
		fxaa_prog.enable(ctx);
		fxaa_prog.set_uniform(ctx, "inverse_viewport_size", vec2(1.0f) / viewport_size);
		fxaa_prog.set_uniform(ctx, "mix_factor", fxaa_mix_factor);

		fbc_shading.enable_attachment(ctx, "color", 0);

		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		fbc_shading.disable_attachment(ctx, "color");

		fxaa_prog.disable(ctx);

		fbc_post.disable(ctx);
	}









	

	// TODO: comment
	

	bool first = !accumulate;
	if(accumulate) {
		// enable the final framebuffer to draw the resolved image into
		fbc_final.enable(ctx);

		auto& resolve_prog = shaders.get("taa_resolve");
		resolve_prog.enable(ctx);
		resolve_prog.set_uniform(ctx, "alpha", taa_mix_factor);

		++accumulate_count;
		if(accumulate_count > n_jitter_samples - 1)
			accumulate_count = 0;

		bool clip_enabled = !settings.static_no_clip;
		if(prev_eye_pos != eye_pos || prev_view_dir != view_dir)
			clip_enabled = true;

		resolve_prog.set_uniform(ctx, "curr_projection_matrix", curr_projection_matrix);
		resolve_prog.set_uniform(ctx, "curr_eye_to_prev_clip_matrix", prev_projection_matrix * prev_modelview_matrix * inv(curr_modelview_matrix));
		resolve_prog.set_uniform(ctx, "settings.use_velocity", settings.use_velocity);
		resolve_prog.set_uniform(ctx, "settings.closest_depth", settings.closest_depth);
		resolve_prog.set_uniform(ctx, "settings.clip_color", settings.clip_color && clip_enabled);

		auto& color_src_fbc = enable_fxaa ? fbc_post : fbc_shading;

		color_src_fbc.enable_attachment(ctx, "color", 0);
		fbc_hist.enable_attachment(ctx, "color", 1);
		fbc.enable_attachment(ctx, "position", 2);
		fbc.enable_attachment(ctx, "depth", 3);

		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		color_src_fbc.disable_attachment(ctx, "color");
		fbc_hist.disable_attachment(ctx, "color");
		fbc.disable_attachment(ctx, "position");
		fbc.disable_attachment(ctx, "depth");

		resolve_prog.disable(ctx);

		// disable the final framebuffer
		fbc_final.disable(ctx);
	} else {
		accumulate = enable_taa;
		accumulate_count = 0;
	}

	auto& color_src_fbc = first ? (enable_fxaa ? fbc_post : fbc_shading) : fbc_final;

	auto& screen_prog = shaders.get("screen");
	screen_prog.enable(ctx);
	screen_prog.set_uniform(ctx, "test", false);

	color_src_fbc.enable_attachment(ctx, "color", 0);
	fbc.enable_attachment(ctx, "depth", 1);

	fbc_hist.enable(ctx);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	fbc_hist.disable(ctx);

	screen_prog.set_uniform(ctx, "test", false);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	color_src_fbc.disable_attachment(ctx, "color");
	fbc.disable_attachment(ctx, "depth");

	screen_prog.disable(ctx);
	
	glDepthFunc(GL_LESS);

	prev_projection_matrix = curr_projection_matrix;
	prev_modelview_matrix = curr_modelview_matrix;
	prev_eye_pos = eye_pos;
	prev_view_dir = view_dir;
}

void tubes::draw_density_volume(context& ctx) {

	auto& vr = ref_volume_renderer(ctx);
	vr.set_render_style(vstyle);
	vr.set_volume_texture(&density_tex);
	vr.set_transfer_function_texture(&volume_tf.ref_texture());
	
	vr.set_bounding_box(density_volume.ref_voxel_grid().bounds);
	vr.transform_to_bounding_box(true);

	vr.render(ctx, 0, 0);
}

shader_define_map tubes::build_tube_shading_defines() {
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

cgv::base::object_registration<tubes> reg_tubes("");
cgv::base::registration_order_definition ro_def("stereo_view_interactor;tubes");

#ifdef CGV_FORCE_STATIC
#include <tubes_shader_inc.h>

#define REGISTER_SHADER_FILES
#include <cgv_glutil/shader_inc.h>
#endif

// Force the usage of the discrete Nvidia graphics card.
// This is needed for systems with Nvidia Optimus technology.
#define NOMINMAX
#include <windows.h>
extern "C" {
	_declspec(dllexport) DWORD NvOptimusEnablement = true;
}
