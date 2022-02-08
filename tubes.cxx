#include "tubes.h"

#include <filesystem>

// CGV framework core
#include <cgv/defines/quote.h>
#include <cgv/math/ftransform.h>
#include <cgv/media/image/image_reader.h>
#include <cgv/utils/advanced_scan.h>

// CGV framework graphics utility
#include <cgv_glutil/color_map_reader.h>

// fltk_gl_view for controlling instant redraw
#include <plugins/cg_fltk/fltk_gl_view.h>

// stereo_view_interactor for controlling fix_view_up_dir
#include <plugins/crg_stereo_view/stereo_view_interactor.h>

// local includes
#include "arclen_helper.h"



// TODO: test sort order if primitives are behind camera and prevent drawing of invisible stuff

tubes::tubes() : application_plugin("tubes_instance")
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
	
	vstyle.enable_depth_test = false;

#ifdef _DEBUG
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(32u);
#else
	voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(128u);
#endif

	shaders.add("tube_shading", "textured_spline_tube_shading.glpr");

	// add frame buffer attachments needed for deferred rendering
	fbc.add_attachment("depth", "[D]");
	fbc.add_attachment("albedo", "flt32[R,G,B,A]");
	fbc.add_attachment("position", "flt32[R,G,B]");
	fbc.add_attachment("normal", "flt32[R,G,B]");
	fbc.add_attachment("tangent", "flt32[R,G,B]");
	fbc.add_attachment("info", "uint32[R,G,B,A]");

	cm_editor_ptr = register_overlay<cgv::glutil::color_map_editor>("Color Scales");
	cm_editor_ptr->set_visibility(false);
	cm_editor_ptr->gui_options.show_heading = false;

	tf_editor_ptr = register_overlay<cgv::glutil::transfer_function_editor>("Volume TF");
	tf_editor_ptr->set_visibility(false);
	tf_editor_ptr->gui_options.show_heading = false;

	navigator_ptr = register_overlay<cgv::glutil::navigator>("Navigator");
	navigator_ptr->set_visibility(false);
	navigator_ptr->gui_options.show_heading = false;
	navigator_ptr->gui_options.show_layout_options = false;
	
	cm_viewer_ptr = register_overlay<color_map_viewer>("Color Scale Viewer");
	cm_viewer_ptr->gui_options.show_heading = false;
	
	grids.resize(2);
	grids[0].scaling = vec2(1.0f, 1.0f);
	grids[0].thickness = 0.05f;
	grids[0].blend_factor = 0.5f;
	grids[1].scaling = vec2(10.0f, 10.0f);
	grids[1].thickness = 0.1f;
	grids[1].blend_factor = 0.333f;
	grid_color = rgba(0.25f, 0.25f, 0.25f, 0.75f);
	grid_mode = GM_COLOR_NORMAL;
	grid_normal_settings = (cgv::type::DummyEnum)1u;
	grid_normal_inwards = true;
	grid_normal_variant = true;
	normal_mapping_scale = 1.0f;
	enable_fuzzy_grid = false;

	show_demo = true;

	do_benchmark = false;
	benchmark_running = false;
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
	ref_sphere_renderer(ctx, -1);
	ref_volume_renderer(ctx, -1);

	srd.destruct(ctx);

	shaders.clear(ctx);
	fbc.clear(ctx);

	delete render.sorter;
	render.sorter = nullptr;
}

bool tubes::self_reflect (cgv::reflect::reflection_handler &rh)
{
	return
		rh.reflect_member("datapath", datapath) &&
		rh.reflect_member("show_demo", show_demo) &&
		rh.reflect_member("render_style", render.style) &&
		rh.reflect_member("grid_mode", grid_mode) &&
		rh.reflect_member("grid_normal_settings", grid_normal_settings) &&
		rh.reflect_member("grid_normal_inwards", grid_normal_inwards) &&
		rh.reflect_member("grid_normal_variant", grid_normal_variant) &&
		rh.reflect_member("instant_redraw_proxy", misc_cfg.instant_redraw_proxy) &&
		rh.reflect_member("vsync_proxy", misc_cfg.vsync_proxy) &&
		rh.reflect_member("fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy);
}

void tubes::stream_help (std::ostream &os)
{
	os << "tubes" << std::endl;
}

bool tubes::handle_event(cgv::gui::event &e) {

	if(e.get_kind() == cgv::gui::EID_KEY) {
		// do nothing for now
	}

	if(e.get_kind() == cgv::gui::EID_MOUSE) {
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
			dnd.pos = ivec2(me.get_x(), me.get_y());
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
			dataset.files.clear();
			for(const std::string &filename : dnd.filenames)
				dataset.files.emplace(filename);
			dnd.filenames.clear();
			dnd.text.clear();
			on_set(&dataset);
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
	// - configurable datapath
	if(member_ptr == &datapath && !datapath.empty()) {
		const bool from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
		traj_mgr.clear();
		cgv::utils::stopwatch s(true);
		std::cout << "Reading data set from " << datapath << " ..." << std::endl;
		if(traj_mgr.load(datapath) != -1) {
			std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
			dataset.files.clear();
			dataset.files.emplace(datapath);
			render.data = &(traj_mgr.get_render_data());
			if (from_demo) {
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

			post_recreate_gui();
		}
	}
	// - non-configurable dataset logic
	else if(member_ptr == &dataset) {
		// clear current dataset
		const bool from_demo = traj_mgr.has_data() && traj_mgr.dataset(0).data_source() == "DEMO";
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
		if(loaded_something) {
			render.data = &(traj_mgr.get_render_data());
			if (from_demo) {
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

			post_recreate_gui();
		}
	}

	// render settings
	if( member_ptr == &grid_mode ||
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

	// voxelization settings
	if(member_ptr == &voxel_grid_resolution) {
		context& ctx = *get_context();
		voxel_grid_resolution = static_cast<cgv::type::DummyEnum>(cgv::math::clamp(static_cast<unsigned>(voxel_grid_resolution), 16u, 512u));
		create_density_volume(ctx, voxel_grid_resolution);
	}

	// visualization settings
	if(member_ptr == &glyph_layer_mgr) {
		if(glyph_layer_mgr.action_type() == AT_CONFIGURATION_CHANGE) {
			glyph_layers_config = glyph_layer_mgr.get_configuration();

			context& ctx = *get_context();
			tube_shading_defines = build_tube_shading_defines();
			shaders.reload(ctx, "tube_shading", tube_shading_defines);

			post_recreate_gui();
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
			if(cm_viewer_ptr)
				cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());

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

	}

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

bool tubes::compile_glyph_attribs (void)
{
	bool success = false;
	if(glyph_layers_config.mapped_attributes.size() > 0)
		success = compile_glyph_attribs_new();
	else
		success = compile_glyph_attribs_old();

	if(success)
		post_redraw();

	return success;
}

bool tubes::compile_glyph_attribs_new(void) {
	cgv::utils::stopwatch s(true);
	std::cout << "Compiling glyph attributes... ";

	// ToDo: replace with actual timestamps on trajectory position samples
	struct segment_time {
		float t0, t1;
		inline static segment_time get(const traj_attribute<float> &attrib, const range &traj, unsigned segment_index) {
			const float *ts = attrib.get_timestamps();
			const unsigned startid = traj.i0 + segment_index;
			return { ts[startid], ts[startid + 1] };
		}
	};

	// clamp value v to range [r.x,r.y] and remap to [r.z,r.w]
	auto clamp_remap = [](float v, vec4 r) {
		v = cgv::math::clamp(v, r.x(), r.y());
		float t = 0.0f;
		if(abs(r.x() - r.y()) > std::numeric_limits<float>::epsilon())
			t = (v - r.x()) / (r.y() - r.x());
		return cgv::math::lerp(r.z(), r.w(), t);
	};

	// get context
	const auto &ctx = *get_context();

	// Compile attribute data for GPU upload - example for one scalar attribute (which one is hardcoded above)
	// - CPU-side staging
	
	// Only consider first data set for now
	const auto &dataset = traj_mgr.dataset(0);
	// convenience shorthands
	const auto &P = dataset.positions().attrib;
	const auto &tube_trajs = dataset.trajectories(P);

	auto attrib_names = dataset.get_attribute_names();

	std::vector<const traj_attribute<float>*> mapped_attribs;
	std::vector<const std::vector<range>*> attribs_trajs;
	std::vector<vec2> attrib_data_ranges;

	// when mapping radius, magnitude_at produces a vector subscript out of range error
	// calling magnitude_at or signed_magnitude_at on attributes without timestamps causes a crash
	// TODO: add has_timestamps method to trajectory_attrib class and only inlcude attribs that have timestamps

	for(size_t i = 0; i < glyph_layers_config.mapped_attributes.size(); ++i) {
		int attrib_idx = glyph_layers_config.mapped_attributes[i];
		if(attrib_idx < 0 || attrib_idx >= attrib_names.size())
			std::cout << "Error: tubes::compile_glyph_attribs - attribute index out of range" << std::endl;

		// from the docs: returns an explicitly invalid attribute interface that acts "empty" on all relevant queries
		// if no attribute of the given name exists in the dataset
		const traj_attribute<float>& attrib = dataset.attribute(attrib_names[attrib_idx]);
		mapped_attribs.push_back(&attrib);

		// from the docs: returns an explicitly invalid range that indicates zero samples in the trajectory if the dataset has
		// no trajectory information for the attribute
		attribs_trajs.push_back(&dataset.trajectories(attrib));

		vec2 range(attrib.min(), attrib.max());
		attrib_data_ranges.push_back(range);

		std::cout << attrib_names[attrib_idx] << std::endl;
	}

	size_t attrib_count = mapped_attribs.size();

	struct {
		size_t count = 0;
		std::vector<float> data;

		bool empty() const { return size() == 0; }

		size_t size() const { return data.size(); }

		size_t glyph_count() const {
			return size() / (2 + count);
		}

		void add(const float& x) {
			data.push_back(x);
		}

		float& operator [](int idx) {
			return data[idx];
		}

		float operator [](int idx) const {
			return data[idx];
		}

		float last_glyph_s() const {
			if(size() > 0)
				return data[size() - 1 - 1 - count];
			else
				return 0.0f;
		}
	} attribs;

	attribs.count = attrib_count;

	struct irange { int i0, n; };
	//std::vector<float> attribs; // buffer of attribute values
	std::vector<irange> ranges; // buffer of index ranges per segment (indexes into 'attribs')

	// reserve memory
	// might not make a huge or even noticeable difference
	//attribs.reserve(attribs.size() + ...);
	// reserve the maximum amount of possible segments; actual segment count may be less if some nodes are used multiple times
	ranges.reserve(ranges.size() + P.num() - tube_trajs.size());
	// - compile data

	// TODO: support multiple data sets? Out of scope for current paper/implementation.
	// TODO: enable usage of multiple glpyh layers

	const glyph_shape* current_shape = glyph_layers_config.shapes[0];

	unsigned traj_offset = 0;
	for(unsigned trj = 0; trj < (unsigned)tube_trajs.size(); trj++) {
		const auto &tube_traj = tube_trajs[trj];
		const auto &attrib_traj = attribs_trajs[0]->at(trj);
		const auto *alen = render.arclen_data.data();
		const unsigned num_segments = tube_traj.n - 1;
		const unsigned attribs_traj_offset = (unsigned)attribs.glyph_count();

		// make sure there is exactly one 'range' entry per segment
		ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

		float prev_glyph_size = 0.0f;
		float last_commited_s = 0.0f;

		// index for the current segment
		unsigned seg = 0;

		// TODO: move this outside of the loop and re-init every iteration if necessary
		unsigned attrib_count = mapped_attribs.size();
		// create an index for each attribute
		std::vector<unsigned> attrib_indices(attrib_count, 0);
		// create storage for attribute and glyph parameter values
		std::vector<traj_attribute<float>::datapoint_mag> data_points(attrib_count);
		std::vector<float> attrib_values(attrib_count);
		std::vector<bool> has_sample(attrib_count);
		std::vector<float> glyph_params(current_shape->num_size_attribs());
		
		// stores the minimum t over all current attribute sample points in each iteration
		float min_t;

		bool run = true;
		for(size_t i = 0; i < attrib_indices.size(); ++i) {
			if(attrib_indices[i] >= (unsigned)attribs_trajs[i]->at(trj).n)
				run &= false;
		}
		run &= seg < num_segments;
		
		

		// following variables only needed for debugging
		unsigned min_a_idx = 0;
		


		while(run) {
			// TODO: do we need to test for monotone increasing vaues of t?
			//if(i > 0) // enforce monotonicity
			//	// TODO: this fails when using the debug-size dataset
			//	assert(a.t >= mapped_attribs[0]->signed_magnitude_at(i - 1).t);

			min_t = std::numeric_limits<float>::max();

			for(size_t i = 0; i < attrib_count; ++i) {
				auto a = mapped_attribs[i]->signed_magnitude_at(attrib_indices[i]);
				data_points[i] = a;
				if(a.t < min_t) {
					min_a_idx = (unsigned)i;
					min_t = a.t;
				}
			}

			// advance segment pointer
			auto segtime = segment_time::get(P, tube_traj, seg);
			while(min_t >= segtime.t1) {
				if(seg >= num_segments - 1)
					break;
				segtime = segment_time::get(P, tube_traj, ++seg);

				// handle overlap from previous segment
				const unsigned global_seg = traj_offset + seg;
				if(ranges[global_seg - 1].n > 0) {
					// using half size of previous glyph
					if(alen[global_seg][0] < attribs.last_glyph_s() + 0.5f*prev_glyph_size) {
						ranges[global_seg].i0 = (int)attribs.glyph_count() - 1;
						ranges[global_seg].n = 1;
					}
				}
			}
			const unsigned global_seg = traj_offset + seg;

			// commit the attribute if it falls into the current segment
			if(min_t >= segtime.t0 && min_t < segtime.t1) {
				// compute segment-relative t and arclength
				const float t_seg = (min_t - segtime.t0) / (segtime.t1 - segtime.t0),
					s = arclen::eval(alen[global_seg], t_seg);

				// store the number of interpolated attributes for this glyph (debug only)
				unsigned num_interpolated = 0;
				for(size_t i = 0; i < attrib_count; ++i) {
					unsigned attrib_idx = attrib_indices[i];

					const auto& a_curr = data_points[i];
					float val = a_curr.val;

					// TODO: make epsilon adjustable
					bool found_sample = abs(min_t - a_curr.t) < 0.001f;
					has_sample[i] = found_sample;
					
					if(!found_sample && attrib_idx > 0) {
						// get interpolated value
						auto a_prev = mapped_attribs[i]->signed_magnitude_at(attrib_idx - 1);
						float t = (min_t - a_prev.t) / (a_curr.t - a_prev.t);
						val = cgv::math::lerp(a_prev.val, val, t);
						++num_interpolated;
					}

					attrib_values[i] = val;
				}

				// setup parameters of potential glyph
				for(size_t i = 0; i < glyph_layers_config.glyph_mapping_parameters.size(); ++i) {
					const auto& triple = glyph_layers_config.glyph_mapping_parameters[i];
					if(triple.type == 0) {
						// constant attribute
						glyph_params[i] = (*triple.v)[3];
					} else {
						// mapped attribute
						const vec4& ranges = *(triple.v);
						// use windowing and remapping to get the value of the glyph parameter
						glyph_params[i] = clamp_remap(attrib_values[triple.idx], ranges);
					}
				}

				float new_glyph_size = current_shape->get_size(glyph_params);
				new_glyph_size /= general_settings.length_scale;

				// infer potential glyph extents
				const float min_dist = attribs.size() > 0 ?
					std::max(new_glyph_size, prev_glyph_size) :
					new_glyph_size;

				// only include samples that are far enough away from last sample to not cause (too much) overlap
				/*if(attribs.glyph_count() == attribs_traj_offset || s >= attribs.last_glyph_s() + min_dist) {
					auto &cur_range = ranges[global_seg];
					if(cur_range.n < 1) {
						// first free attribute that falls into this segment
						cur_range.i0 = (unsigned)attribs.glyph_count();
						cur_range.n = 1;
						// handle overlap to previous segment
						if(seg > 0 && alen[global_seg - 1][15] > s - 0.5f*new_glyph_size)
							ranges[global_seg - 1].n++;
					} else {
						// one more free attribute that falls into this segment
						cur_range.n++;
					}
					// store the new glyph
					attribs.add(s);
					attribs.add(1.0f);
					attribs.add(v);
					//store the size when this glyph is actually placed
					prev_glyph_size = new_glyph_size;
				}*/

				//bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= attribs.last_glyph_s() + min_dist;
				bool include_glyph = attribs.glyph_count() == attribs_traj_offset || s >= last_commited_s + min_dist;

				auto &cur_range = ranges[global_seg];
				if(cur_range.n < 1) {
					// first free attribute that falls into this segment
					cur_range.i0 = (unsigned)attribs.glyph_count();
					cur_range.n = 1;
					// handle overlap to previous segment
					if(seg > 0 && alen[global_seg - 1][15] > s - 0.5f*new_glyph_size)
						ranges[global_seg - 1].n++;
				} else {
					// one more free attribute that falls into this segment
					cur_range.n++;
				}
				// store the new glyph
				attribs.add(s);
				int debug_info = 0;

				debug_info |= num_interpolated;
				debug_info |= min_a_idx << 2;

				if(include_glyph)
					debug_info |= 0x1000;

				attribs.add(*reinterpret_cast<float*>(&debug_info));

				std::copy(attrib_values.begin(), attrib_values.end(), std::back_inserter(attribs.data));

				//for(size_t i = 0; i < attrib_indices.size(); ++i)
				//	attribs.add(attrib_values[i]);

				//store the size when this glyph is actually placed
				if(include_glyph) {
					prev_glyph_size = new_glyph_size;
					last_commited_s = s;
				}
			} else if(seg > (unsigned)tube_traj.n - 2) {
				// we went beyond the last segment
				break;
			}

			// increment indices and check whether the indices of all attributes have reached the end
			for(size_t i = 0; i < attrib_count; ++i) {
				// only increment indices of attributes that have a sample at the current location (min_a.t)
				if(has_sample[i])
					attrib_indices[i] = std::min((unsigned)attribs_trajs[i]->at(trj).n, ++attrib_indices[i]);
				if(attrib_indices[i] >= (unsigned)attribs_trajs[i]->at(trj).n)
					run &= false;
			}

			run &= seg < num_segments;
		}

		// update auxiliary indices
		traj_offset += num_segments;
	}

	std::cout << "ATTRIBS SIZE: " << attribs.size() << std::endl;

	// - sanity check
	{
		const float num_ranges = (float)ranges.size(), num_segs = float(render.data->indices.size()) / 2;
		assert(num_ranges == num_segs);
	}
	// - upload
	// ...attrib nodes
	if(attribs.empty()) {
		// fill with one entry of empty glyph attribs
		attribs.add(0.0f);
		attribs.add(0.0f);
		for(size_t i = 0; i < attrib_count; ++i)
			attribs.add(0.0f);
	}
	render.attribs_sbo.destruct(ctx);
	if(!render.attribs_sbo.create(ctx, attribs.data))
		std::cerr << "!!! unable to create glyph attribute Storage Buffer Object !!!" << std::endl << std::endl;
	// ...index ranges
	render.aindex_sbo.destruct(ctx);
	if(!render.aindex_sbo.create(ctx, ranges))
		std::cerr << "!!! unable to create glyph index ranges Storage Buffer Object !!!" << std::endl << std::endl;

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

	// done!
	return true;
}

bool tubes::compile_glyph_attribs_old(void) {
	cgv::utils::stopwatch s(true);
	std::cout << "Compiling glyph attributes... ";

	// for copying default values into new glyphs
	/*const static attribute_mapping_parameters glyph_defaults;
	const static glyph_attribs empty_glyph = {
		0.f, glyph_defaults.radius0, glyph_defaults.radius1, glyph_defaults.angle0, glyph_defaults.angle1
	};*/
	const static glyph_attribs empty_glyph = { 0.0f, 0.0f, 0.0f, 0.0f };

	// ToDo: replace with actual timestamps on trajectory position samples
	struct segment_time {
		float t0, t1;
		inline static segment_time get(const traj_attribute<float> &attrib, const range &traj, unsigned segment_index) {
			const float *ts = attrib.get_timestamps();
			const unsigned startid = traj.i0 + segment_index;
			return { ts[startid], ts[startid + 1] };
		}
	};

	// ToDo: generalize to arbitrary free attributes
	#define FREE_ATTRIBUTE_SERIES "scalar"

	// get context
	const auto &ctx = *get_context();

	struct irange { int i0, n; };
	std::vector<glyph_attribs> attribs; // buffer of attribute values
	std::vector<irange> ranges;         // buffer of index ranges per segment (indexes into 'attribs')
	for(unsigned ds = 0; ds < std::max(traj_mgr.num_datasets(), (unsigned)1); ds++) {
		// convenience shorthands
		const auto &dataset = traj_mgr.dataset(ds);
		const auto &P = dataset.positions().attrib;
		const auto &tube_trajs = dataset.trajectories(P);

		// from the docs: returns an explicitly invalid attribute interface that acts "empty" on all relevant queries
		// if no attribute of the given name exists in the dataset
		const auto &free_attrib = dataset.attribute(FREE_ATTRIBUTE_SERIES);

		// from the docs: returns an explicitly invalid range that indicates zero samples in the trajectory if the dataset has
		// no trajectory information for the attribute
		const auto &attrib_trajs = dataset.trajectories(free_attrib);

		// determine min, nax value (ToDo: observe glyph config)
		const float vmin = free_attrib.min(), vmax = free_attrib.max();

		// reserve memory
		attribs.reserve(attribs.size() + free_attrib.num());
		ranges.reserve(ranges.size() + /* estimated_num_segments = */ P.num() - tube_trajs.size());
		// - compile data
		unsigned traj_offset = 0;
		for(unsigned trj = 0; trj < (unsigned)tube_trajs.size(); trj++) {
			const auto &tube_traj = tube_trajs[trj];
			const auto &attrib_traj = attrib_trajs[trj];
			const auto *alen = render.arclen_data.data();
			const unsigned num_segments = tube_traj.n - 1;
			const unsigned attribs_traj_offset = (unsigned)attribs.size();

			// make sure there is exactly one 'range' entry per segment
			ranges.resize(traj_offset + num_segments); // takes care of zero-initializing each entry

			// - primary loop is through free attributes, segment assignment based on timestamps
			for(unsigned i = 0, seg = 0; i < (unsigned)attrib_traj.n && seg < num_segments; i++) {
				const auto a = free_attrib.magnitude_at(i);
				//if(i > 0) // enforce monotonicity
				//	// TODO: this fails when using the debug-size dataset
				//	assert(a.t >= free_attrib.magnitude_at(i - 1).t);

				// advance segment pointer
				auto segtime = segment_time::get(P, tube_traj, seg);
				while(a.t >= segtime.t1) {
					if(seg >= num_segments - 1)
						break;
					segtime = segment_time::get(P, tube_traj, ++seg);
					// handle overlap from previous segment
					const unsigned global_seg = traj_offset + seg;
					if(ranges[global_seg - 1].n > 0) {
						const float prev_rad = glyph_attribs::calc_radius(attribs.back(), general_settings.length_scale).rad;
						if(alen[global_seg][0] < attribs.back().s + prev_rad) {
							ranges[global_seg].i0 = (int)attribs.size() - 1;
							ranges[global_seg].n = 1;
						}
					}
				}
				const unsigned global_seg = traj_offset + seg;

				// commit the attribute if it falls into the current segment
				if(a.t >= segtime.t0 && a.t < segtime.t1) {
					// compute segment-relative t and arclength
					const float t_seg = (a.t - segtime.t0) / (segtime.t1 - segtime.t0),
						s = arclen::eval(alen[global_seg], t_seg);

					// normalize attribute value
					const float v = (a.val - vmin) / (vmax - vmin);

					// setup potential glyph
					glyph_attribs new_glyph;
					new_glyph.s = s;
					new_glyph.radius0 = v;
					new_glyph.radius1 = new_glyph.radius0 / 3;
					new_glyph.angle0 = 0.0f;
					new_glyph.angle1 = 0.0f;

					// infer potential glyph extents
					const auto new_glyph_ext = glyph_attribs::calc_radius(new_glyph, general_settings.length_scale);
					const float min_dist = attribs.size() > 0 ? std::max(
						new_glyph_ext.diam,
						glyph_attribs::calc_radius(attribs.back(), general_settings.length_scale).diam
					) : new_glyph_ext.diam;

					// only include samples that are far enough away from last sample to not cause (too much) overlap
					if(attribs.size() == attribs_traj_offset || s >= attribs.back().s + min_dist) {
						auto &cur_range = ranges[global_seg];
						if(cur_range.n < 1) {
							// first free attribute that falls into this segment
							cur_range.i0 = (unsigned)attribs.size();
							cur_range.n = 1;
							// handle overlap to previous segment
							if(seg > 0 && alen[global_seg - 1][15] > s - new_glyph_ext.rad)
								ranges[global_seg - 1].n++;
						} else
							// one more free attribute that falls into this segment
							cur_range.n++;
						attribs.emplace_back(std::move(new_glyph));
					}
				} else if(seg > (unsigned)tube_traj.n - 2)
					// we went beyond the last segment
					break;
			}

			// update auxiliary indices
			traj_offset += num_segments;
		}
	}
	// - sanity check
	{ const float num_ranges = (float)ranges.size(), num_segs = float(render.data->indices.size()) / 2;
	assert(num_ranges == num_segs); }
	// - upload
	// ...attrib nodes
	if(attribs.empty())
		attribs.emplace_back(empty_glyph);
	render.attribs_sbo.destruct(ctx);
	if(!render.attribs_sbo.create(ctx, attribs))
		std::cerr << "!!! unable to create glyph attribute Storage Buffer Object !!!" << std::endl << std::endl;
	// ...index ranges
	render.aindex_sbo.destruct(ctx);
	if(!render.aindex_sbo.create(ctx, ranges))
		std::cerr << "!!! unable to create glyph index ranges Storage Buffer Object !!!" << std::endl << std::endl;

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

	// ToDo: REMOVE
	/*// test nearest attribute binary search
	for (unsigned seg = 0; seg<ranges.size(); seg++) for (float t = 0; t <= 1; t += 1.f / 16)
	{
		if (ranges[seg].n < 1)
			continue;
		const float s = arclen::eval(render.arclen_data[seg], t);
		const auto &rng_orig = ranges[seg];
			  auto  rng = rng_orig;
		while (rng.n > 1)
		{
			const int mid_n = rng.n/2, mid = rng.i0+mid_n;
			const auto &attrib_mid = attribs[mid];
			if (attrib_mid.s > s)
				// left node
				rng.n = mid_n;
			else
			{
				// right node
				rng.i0 = rng.i0 + mid_n;
				rng.n  = rng.n  - mid_n;
			}
		}
		const int gidn = rng.i0 < rng_orig.i0+rng_orig.n-1 ? rng.i0+1 : rng.i0;
		const float distc = std::abs(attribs[rng.i0].s-s), distn = std::abs(attribs[gidn].s-s);
		const bool c_is_closer = distc <= distn;
		//const float dist = c_is_closer ? distc : distn;
		const int gid = c_is_closer ? rng.i0 : gidn;
		assert(gid >= rng_orig.i0 && gid < rng_orig.i0+rng_orig.n); // sanity check
		//std::cout << "seg"<<seg<<", t="<<t<<" -> attrib "<<gid<<" (dist="<<dist<<")" << std::endl;
	}*/

	// done!
	return true;
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

	success &= load_transfer_function(ctx);

	ref_sphere_renderer(ctx, 1);
	// TODO: remove sphere renderer
	srd.init(ctx);

	// generate demo
	// - demo AO settings
	ao_style.enable = true;
	ao_style.cone_angle = 60.f;
	ao_style.sample_offset = 0.08f;
	ao_style.sample_distance = 0.5f;
	ao_style.strength_scale = 7.5f;
	ao_style.generate_sample_directions();
	ao_style_bak = ao_style;
	update_member(&ao_style);
	// - demo geometry
	constexpr unsigned seed = 11;
#ifdef _DEBUG
	constexpr unsigned num_trajectories = 1;
	constexpr unsigned num_segments = 16;
#else
	constexpr unsigned num_trajectories = 256;
	constexpr unsigned num_segments = 256;
#endif
	for (unsigned i=0; i<num_trajectories; i++)
		dataset.demo_trajs.emplace_back(demo::gen_trajectory(num_segments, seed+i));
	traj_mgr.add_dataset(
		demo::compile_dataset(dataset.demo_trajs)
	);
	update_attribute_bindings();
	update_grid_ratios();

	
	



	// load diverging color maps from the resource directory
	std::string color_maps_path = app_path + "res/color_maps/diverging";
	if(std::filesystem::exists(color_maps_path)) {
		for(const auto &entry : std::filesystem::directory_iterator(color_maps_path)) {
			std::filesystem::path entry_path = entry.path();
			// only take xml files
			if(entry_path.extension() == ".xml") {
				std::string name = "";
				cgv::glutil::color_map cm;
				if(cgv::glutil::color_map_reader::read_from_xml(entry_path.string(), cm, name))
					color_map_mgr.add_color_map(name, cm);
			}
		}
		color_map_mgr.update_texture(ctx);
		if(cm_viewer_ptr)
			cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	update_glyph_layer_manager();

	compile_glyph_attribs();
	ah_mgr.set_dataset(traj_mgr.dataset(0));







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
		}
	}

	if (misc_cfg.fix_view_up_dir_proxy)
		// ToDo: make stereo view interactors reflect this property
		/*dynamic_cast<stereo_view_interactor*>(find_view_as_node())->set(
			"fix_view_up_dir", misc_cfg.fix_view_up_dir_proxy
		);*/
		find_view_as_node()->set_view_up_dir(0, 1, 0);

	// keep the frame buffer up to date with the viewport size
	fbc.ensure(ctx);

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

	if(benchmark_running) {
		++total_frames;
		double benchmark_time = 10.0;

		double seconds_since_start = benchmark_timer.get_elapsed_time();
		double alpha = (seconds_since_start - last_seconds_since_start) / benchmark_time;
		last_seconds_since_start = seconds_since_start;

		double depth = cgv::math::length(view_ptr->get_eye() - view_ptr->get_focus());

		view_ptr->rotate(0.0, cgv::math::deg2rad(360.0 * alpha), depth);

		if(seconds_since_start >= benchmark_time) {
			benchmark_running = false;

			double avg_fps = (double)total_frames / seconds_since_start;

			std::stringstream ss;
			ss.precision(2);
			ss << std::fixed;
			ss << "Average FPS: " << avg_fps << " | " << (1000.0f / avg_fps) << "ms";

			std::cout << ss.str() << std::endl;
		}
	}

	if(do_benchmark) {
		do_benchmark = false;
		update_member(&do_benchmark);

		misc_cfg.instant_redraw_proxy = true;
		misc_cfg.vsync_proxy = false;
		on_set(&misc_cfg.instant_redraw_proxy);
		on_set(&misc_cfg.vsync_proxy);

		if(!benchmark_running) {
			benchmark_running = true;
			benchmark_timer.restart();

			total_frames = 0u;
			initial_eye_pos = view_ptr->get_eye();
			initial_focus = view_ptr->get_focus();
			last_seconds_since_start = 0.0;
		}
	}

	if(cm_editor_ptr && cm_editor_ptr->was_updated()) {
		color_map_mgr.update_texture(ctx);
		if(cm_viewer_ptr)
			cm_viewer_ptr->set_color_map_texture(&color_map_mgr.ref_texture());
	}

	srd.clear();
	srd.add(vec3(0.0f), 1.0f, rgb(1, 0, 0));
}

void tubes::draw (cgv::render::context &ctx)
{
	if(!view_ptr) return;

	//srd.render(ctx, ref_sphere_renderer(ctx), sphere_render_style());

	// draw dataset using selected renderer
	if(show_volume) {
		draw_density_volume(ctx);
	} else {
		if(traj_mgr.has_data())
			draw_trajectories(ctx);
	}

	//auto& sr = ref_sphere_renderer(ctx);
	//srd.render(ctx, sr, sphere_render_style());

	// display drag-n-drop information, if a dnd operation is in progress
	if(!dnd.text.empty())
		draw_dnd(ctx);
}

void tubes::create_gui (void)
{
	// dataset settings
	add_decorator("Dataset", "heading", "level=1");
	//add_member_control(this, "data file/path", datapath);
	add_gui("Data Path", datapath, "file_name", "title='Open Trajectory Data';"
		"filter='Trajectory Files (bezdat, csv):*.bezdat;*.csv|All Files:*.*';"
		"small_icon=true");

	// rendering settings
	add_decorator("Rendering", "heading", "level=1");
	if (begin_tree_node("Tube Style", render.style, false))
	{
		align("\a");
		add_gui("tube_style", render.style);
		// render percentage exists for debug reasons only and can be removed at some point
		add_member_control(this, "Render Percentage", render.percentage, "value_slider", "min=0.0;step=0.001;max=1.0;ticks=true");
		add_member_control(this, "Sort by Distance", render.sort, "check");
		align("\b");
		end_tree_node(render.style);
	}

	if(begin_tree_node("AO Style", ao_style, false)) {
		align("\a");
		add_member_control(this, "Voxel Grid Resolution", voxel_grid_resolution, "dropdown", "enums='16=16, 32=32, 64=64, 128=128, 256=256, 512=512'");
		add_gui("ao_style", ao_style);
		align("\b");
		end_tree_node(ao_style);
	}

	// attribute mapping settings
	//add_decorator("Attribute Mapping", "heading", "level=1");
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

	if(begin_tree_node("General Settings", general_settings, true)) {
		align("\a");
		add_member_control(this, "Curvature Correction", general_settings.use_curvature_correction, "check");
		add_member_control(this, "Length Scale", general_settings.length_scale, "value_slider", "min=0.1;max=10;step=0.01;ticks=true;color=0xff0000");
		add_member_control(this, "Antialias Radius", general_settings.antialias_radius, "value_slider", "min=0;max=5;step=0.01;ticks=true");
		align("\b");
		end_tree_node(glyph_layer_mgr);
	}

	if(begin_tree_node("Attribute Mapping", glyph_layer_mgr, true)) {
		align("\a");
		connect_copy(add_button("Compile Attributes")->click, cgv::signal::rebind(this, &tubes::compile_glyph_attribs));
		glyph_layer_mgr.create_gui(this, *this);
		align("\b");
		end_tree_node(glyph_layer_mgr);
	}

	if(begin_tree_node("Color Scales", cm_editor_ptr, false)) {
		align("\a");
		color_map_mgr.create_gui(this, *this);
		//cs_editor_ptr->create_gui(*this);
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
		//navigator_ptr->create_gui(*this);
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
			//tf_editor_ptr->create_gui(*this);
			inline_object_gui(tf_editor_ptr);
			align("\b");
			end_tree_node(tf_editor_ptr);
		}

		align("\b");
		end_tree_node(vstyle);
	}

	if (begin_tree_node("Tools (Persistent by Default)", misc_cfg, false))
	{
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

	add_member_control(this, "Start Benchmark", do_benchmark, "toggle", "");

	create_vec3_gui("Eye Pos", test_eye, -10.0f, 10.0f);
	create_vec3_gui("Eye Dir", test_dir, -1.0f, 1.0f);
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
	if(!view_ptr || !traj_mgr.has_data()) return;

	view_ptr->set_focus(bbox.get_center());
	double extent_factor = 0.8;
	view_ptr->set_y_extent_at_focus(extent_factor * (double)length(bbox.get_extent()));
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
		// we base everything on the mean of all trajectory median radii
		grids[0].scaling.x() = general_settings.length_scale = 1.f / float(mean_rad);
		grids[0].scaling.y() = grids[0].scaling.x()/4;
		grids[1].scaling.x() = grids[0].scaling.x()*4;
		grids[1].scaling.y() = grids[0].scaling.x();
		for (unsigned i=0; i<2; i++)
		{
			update_member(&(grids[i].scaling[0]));
			update_member(&(grids[i].scaling[1]));
			update_member(&general_settings.length_scale);
		}
	}
}

void tubes::update_attribute_bindings(void) {
	auto &ctx = *get_context();

	if (traj_mgr.has_data())
	{
		calculate_bounding_box(); 
		set_view();

		create_density_volume(ctx, voxel_grid_resolution);

		// Recompute arclength parametrization
		cgv::utils::stopwatch s(true);
		std::cout << "Computing arclength parametrization... ";

		render.arclen_sbo.destruct(ctx);
		render.arclen_data = arclen::compile_renderdata(traj_mgr);
		render.arclen_sbo = arclen::upload_renderdata(ctx, render.arclen_data);
		
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

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
}

void tubes::create_density_volume(context& ctx, unsigned resolution) {

	cgv::utils::stopwatch s(true);

	density_volume.initialize_voxel_grid(bbox, resolution);

	ivec3 res = density_volume.ref_voxel_grid().resolution;
	std::cout << "Generating density volume with resolution (" << res.x() << ", " << res.y() << ", " << res.z() << ")... ";

	density_volume.compute_density_volume(render.data);

	if(density_tex.is_created())
		density_tex.destruct(ctx);

	std::vector<float>& density_data = density_volume.ref_voxel_grid().data;

	cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(res.x(), res.y(), res.z(), TI_FLT32, cgv::data::CF_R), density_data.data());
	density_tex = texture("flt32[R,G,B]", TF_LINEAR, TF_LINEAR_MIPMAP_LINEAR, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER, TW_CLAMP_TO_BORDER);
	density_tex.create(ctx, dv, 0);
	density_tex.set_border_color(0.0f, 0.0f, 0.0f, 0.0f);
	density_tex.generate_mipmaps(ctx);

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

	// create histogram
	// TODO: remove later. We dont need this or the transfer function editor for the paper.
	unsigned n_bins = 256;
	std::vector<unsigned> hist(n_bins, 0u);

	float x = 0.0f;
	for(int i = 0; i < density_data.size(); ++i) {
		// don't count 0
		unsigned bin = static_cast<unsigned>(density_data[i] * (n_bins - 1));
		if(bin != 0)
			hist[bin] += 1;
	}

	tf_editor_ptr->set_histogram(hist);

	ao_style.derive_voxel_grid_parameters(density_volume.ref_voxel_grid());
}

void tubes::draw_dnd(context& ctx) {

	static const rgb dnd_col(1, 0.5f, 0.5f);
	// compile the text we're going to draw and gather its approximate dimensions at the same time
	float w = 0, s = ctx.get_current_font_size();
	std::stringstream dnd_drawtext;
	dnd_drawtext << "Load dataset:" << std::endl;
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

	fbc.enable(ctx);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);

	// prepare SSBO handles
	const int data_handle = render.render_sbo.handle ? (const int&)render.render_sbo.handle-1 : 0,
	          arclen_handle = render.arclen_sbo.handle ? (const int&)render.arclen_sbo.handle-1 : 0,
	          attribs_handle = render.attribs_sbo.handle ? (const int&)render.attribs_sbo.handle-1 : 0,
	          aindex_handle = render.aindex_sbo.handle ? (const int&)render.aindex_sbo.handle-1 : 0;
	if (!data_handle || !arclen_handle /*|| !attribs_handle || !aindex_handle*/) return;

	// sort the sgment indices
	int segment_idx_handle = tstr.get_index_buffer_handle(render.aam);
	int node_idx_handle = tstr.get_vbo_handle(ctx, render.aam, "node_ids");	
	if(data_handle > 0 && segment_idx_handle > 0 && node_idx_handle > 0 && render.sort)
		//render.sorter->sort(ctx, data_handle, segment_idx_handle, test_eye, test_dir, node_idx_handle);
		render.sorter->sort(ctx, data_handle, segment_idx_handle, eye_pos, view_dir, node_idx_handle);

	tstr.set_eye_pos(eye_pos);
	tstr.set_view_dir(view_dir);
	tstr.set_viewport(vec4((float)viewport[0], (float)viewport[1], (float)viewport[2], (float)viewport[3]));
	tstr.set_render_style(render.style);
	tstr.enable_attribute_array_manager(ctx, render.aam);

	int count = int(render.percentage * (render.data->indices.size()/2));
	
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, data_handle);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, arclen_handle);
	tstr.render(ctx, 0, count);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);

	tstr.disable_attribute_array_manager(ctx, render.aam);

	fbc.disable(ctx);

	shader_program& prog = shaders.get("tube_shading");
	prog.enable(ctx);
	// set render parameters
	prog.set_uniform(ctx, "use_gamma", true);
	
	// set ambient occlusion parameters
	prog.set_uniform(ctx, "ambient_occlusion.enable", ao_style.enable);
	prog.set_uniform(ctx, "ambient_occlusion.sample_offset", ao_style.sample_offset);
	prog.set_uniform(ctx, "ambient_occlusion.sample_distance", ao_style.sample_distance);
	prog.set_uniform(ctx, "ambient_occlusion.strength_scale", ao_style.strength_scale);

	prog.set_uniform(ctx, "ambient_occlusion.tex_offset", ao_style.texture_offset);
	prog.set_uniform(ctx, "ambient_occlusion.tex_scaling", ao_style.texture_scaling);
	prog.set_uniform(ctx, "ambient_occlusion.texcoord_scaling", ao_style.texcoord_scaling);
	prog.set_uniform(ctx, "ambient_occlusion.texel_size", ao_style.texel_size);

	prog.set_uniform(ctx, "ambient_occlusion.cone_angle_factor", ao_style.angle_factor);
	prog.set_uniform_array(ctx, "ambient_occlusion.sample_directions", ao_style.sample_directions);
	
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

	//glDepthFunc(GL_ALWAYS);

	fbc.enable_attachment(ctx, "albedo", 0);
	fbc.enable_attachment(ctx, "position", 1);
	fbc.enable_attachment(ctx, "normal", 2);
	fbc.enable_attachment(ctx, "tangent", 3);
	fbc.enable_attachment(ctx, "info", 4);
	fbc.enable_attachment(ctx, "depth", 5);
	density_tex.enable(ctx, 6);
	color_map_mgr.ref_texture().enable(ctx, 7);
	//cm_editor_ptr->ref_tex().enable(ctx, 7);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, attribs_handle);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, aindex_handle);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0); 

	fbc.disable_attachment(ctx, "albedo");
	fbc.disable_attachment(ctx, "position");
	fbc.disable_attachment(ctx, "normal");
	fbc.disable_attachment(ctx, "tangent");
	fbc.disable_attachment(ctx, "info");
	fbc.disable_attachment(ctx, "depth");
	density_tex.disable(ctx);
	color_map_mgr.ref_texture().disable(ctx);
	//cm_editor_ptr->ref_tex().disable(ctx);

	//glDepthFunc(GL_LESS);

	prog.disable(ctx);
}

void tubes::draw_density_volume(context& ctx) {

	auto& vr = ref_volume_renderer(ctx);
	vr.set_render_style(vstyle);
	vr.set_volume_texture(&density_tex);
	//vr.set_transfer_function_texture(&tf_tex);
	vr.set_transfer_function_texture(&tf_editor_ptr->ref_tex());
	
	vr.set_bounding_box(density_volume.ref_voxel_grid().bounds);
	vr.transform_to_bounding_box(true);

	vr.render(ctx, 0, 0);
}

bool tubes::load_transfer_function(context& ctx)
{
	// attempt to load a transfer function from an image file
	cgv::data::data_format format;
	cgv::media::image::image_reader image(format);
	cgv::data::data_view image_data;

	bool success = false;

	std::string file_name = "res://inferno.bmp";
	if(!image.read_image(file_name, image_data)) {
		std::cout << "Error: Could not read image file " << file_name << std::endl;
	} else {
		// the image shall be given with a resolution of (width x 1)
		// yet this still results in a 2d data view
		// reduce the data to a 1d view and add an alpha channel

		// TODO: make data_view reduce operations change format or make texture create get dims from view and not format (ask Stefan)
		cgv::data::data_view tex_data_1d = image_data(0);

		unsigned w = tex_data_1d.get_format()->get_width();
		std::vector<unsigned char> data(4 * w, 0u);

		unsigned char* src_ptr = tex_data_1d.get_ptr<unsigned char>();
		
		for(unsigned i = 0; i < w; ++i) {
			float alpha = static_cast<float>(i / 4) / static_cast<float>(w);
			data[4 * i + 0] = src_ptr[3 * i + 0];
			data[4 * i + 1] = src_ptr[3 * i + 1];
			data[4 * i + 2] = src_ptr[3 * i + 2];
			data[4 * i + 3] = static_cast<unsigned char>(255.0f * alpha);
		}

		tex_data_1d.set_format(new cgv::data::data_format(w, cgv::type::info::TypeId::TI_UINT8, cgv::data::ComponentFormat::CF_RGBA));
		tex_data_1d.set_ptr((void*)data.data());

		tf_tex.create(ctx, tex_data_1d, 0);
		tf_tex.set_min_filter(cgv::render::TextureFilter::TF_LINEAR);
		tf_tex.set_mag_filter(cgv::render::TextureFilter::TF_LINEAR);
		tf_tex.set_wrap_s(cgv::render::TextureWrap::TW_CLAMP_TO_EDGE);
		
		success = true;
	}

	image.close();
	return success;
}

shader_define_map tubes::build_tube_shading_defines() {
	shader_define_map defines;
	shader_code::set_define(defines, "GRID_MODE", grid_mode, GM_COLOR);
	unsigned gs = static_cast<unsigned>(grid_normal_settings);
	if(grid_normal_inwards) gs += 4u;
	if(grid_normal_variant) gs += 8u;
	shader_code::set_define(defines, "GRID_NORMAL_SETTINGS", gs, 0u);
	shader_code::set_define(defines, "ENABLE_FUZZY_GRID", enable_fuzzy_grid, false);

	shader_code::set_define(defines, "MAPPED_ATTRIBUTE_COUNT", glyph_layers_config.shader_config.mapped_attrib_count, 1u);
	shader_code::set_define(defines, "GLYPH_MAPPING_UNIFORMS", glyph_layers_config.shader_config.uniforms_definition, std::string("vec4 glyph_m_param[MAPPED_ATTRIBUTE_COUNT];"));
	shader_code::set_define(defines, "GLYPH_ATTRIBUTES_DEFINITION", glyph_layers_config.shader_config.attribute_buffer_definition, std::string("float v[1];"));
	shader_code::set_define(defines, "GLYPH_LAYER_DEFINITION", glyph_layers_config.shader_config.glyph_layers_definition, std::string(""));
	return defines;
}

////
// Object registration

cgv::base::object_registration<tubes> reg_tubes("");

//#ifdef CGV_FORCE_STATIC
	cgv::base::registration_order_definition ro_def("stereo_view_interactor;tubes");
//#endif
