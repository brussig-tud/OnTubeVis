#include "tubes.h"

// fltk_gl_view for controlling instant redraw
#include <plugins/cg_fltk/fltk_gl_view.h>

// stereo_view_interactor for controlling fix_view_up_dir
#include <plugins/crg_stereo_view/stereo_view_interactor.h>

// local includes
#include "arclen_helper.h"



tubes::tubes() : node("tubes_instance")
{
	// adjust render style defaults
	render.style.material.set_brdf_type(
		(cgv::media::illum::BrdfType)(cgv::media::illum::BrdfType::BT_STRAUSS_DIFFUSE
			| cgv::media::illum::BrdfType::BT_COOK_TORRANCE)
	);
	render.style.material.set_roughness(0.25);
	render.style.material.set_metalness(0.25);
	render.style.material.set_ambient_occlusion(0.75);
	render.style.material.set_emission({ 0.125f, 0.125f, 0.125f });
	render.style.material.set_specular_reflectance({ 0.05f, 0.05f, 0.05f });

	shaders.add("screen", "screen_quad.glpr");

	// add frame buffer attachments needed for deferred rendering
	fbc.add_attachment("depth", "[D]");
	fbc.add_attachment("albedo", "flt32[R,G,B]");
	fbc.add_attachment("position", "flt32[R,G,B]");
	fbc.add_attachment("normal", "flt32[R,G,B]");
	fbc.add_attachment("texcoord", "flt32[R,G]");
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
	cgv::render::ref_textured_spline_tube_renderer(ctx, -1);
	cgv::render::ref_spline_tube_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);

	shaders.clear(ctx);
	fbc.clear(ctx);

	delete render.sorter;
	render.sorter = nullptr;
}

bool tubes::self_reflect (cgv::reflect::reflection_handler &rh)
{
	return
		rh.reflect_member("datapath", datapath) &&
		rh.reflect_member("render_style", render.style) &&
		rh.reflect_member("instant_redraw_proxy", misc_cfg.instant_redraw_proxy) &&
		rh.reflect_member("fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy);
}

void tubes::stream_help (std::ostream &os)
{
	os << "tubes" << std::endl;
}

bool tubes::handle(cgv::gui::event &e) {
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
		traj_mgr.clear();
		if(traj_mgr.load(datapath) != -1) {
			dataset.files.clear();
			dataset.files.emplace(datapath);
			render.data = &(traj_mgr.get_render_data());
			update_attribute_bindings();
		}
	}
	// - non-configurable dataset logic
	else if(member_ptr == &dataset) {
		// clear current dataset
		datapath.clear();
		traj_mgr.clear();

		// load new data
		bool loaded_something = false;
		for(const auto &file : dataset.files)
			loaded_something = traj_mgr.load(file) != -1 || loaded_something;
		update_member(&datapath);

		// update render state
		if(loaded_something) {
			render.data = &(traj_mgr.get_render_data());
			update_attribute_bindings();
		}
	}

	// misc settings
	// - instant redraw
	if(member_ptr == &misc_cfg.instant_redraw_proxy)
		// ToDo: handle the (virtually impossible) case that some other plugin than cg_fltk provides the gl_context
		dynamic_cast<fltk_gl_view*>(get_context())->set_void("instant_redraw", "bool", member_ptr);
	// - fix view up dir
	else if(member_ptr == &misc_cfg.fix_view_up_dir_proxy)
		// ToDo: make stereo view interactors reflect this property, and handle the case that some other plugin that
		//       is not derived from stereo_view_interactor handles viewing
		//if (!misc_cfg.fix_view_up_dir_proxy)
		//	dynamic_cast<stereo_view_interactor*>(find_view_as_node())->set("fix_view_up_dir", false);
		//else
		if(misc_cfg.fix_view_up_dir_proxy)
			find_view_as_node()->set_view_up_dir(0, 1, 0);

	// default implementation for all members
	// - dirty hack to catch GUI changes to render_cfg.render_style
	//*dynamic_cast<cgv::render::surface_render_style*>(&render.rounded_cone_rstyle) = render_cfg.render_style;
	//*dynamic_cast<cgv::render::surface_render_style*>(&render.spline_tube_rstyle) = render_cfg.render_style;
	//*dynamic_cast<cgv::render::surface_render_style*>(&render.textured_spline_tube_rstyle) = render_cfg.render_style;
	// - remaining logic
	update_member(member_ptr);
	post_redraw();
}

bool tubes::init (cgv::render::context &ctx)
{
	// increase reference count of the renderers by one
	auto &rcr = cgv::render::ref_rounded_cone_renderer(ctx, 1);
	auto &str = cgv::render::ref_spline_tube_renderer(ctx, 1);
	auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx, 1);
	bool success = rcr.ref_prog().is_linked() && str.ref_prog().is_linked();

	// load all shaders in the library
	success &= shaders.load_shaders(ctx);

	// init shared attribute array manager
	success = success && render.aam.init(ctx);

	// TODO: test some stuff
	/*if(traj_mgr.has_data()) {
		const traj_dataset<float>& ds = traj_mgr.dataset(0);
		const std::vector<traj_dataset<float>::trajectory>& trajs = ds.trajectories();
	}*/

	render.sorter = new cgv::glutil::radix_sort_4way();
	render.sorter->set_value_format(TI_UINT32, 2);
	render.sorter->initialize_values_on_sort(false);
	render.sorter->set_data_type_override("float x, y, z;");
	
	std::string key_definition =
		R"(uvec2 indices = values[idx]; \
		data_type a = data[indices.x]; \
		data_type b = data[indices.y]; \
		vec3 pa = vec3(a.x, a.y, a.z); \
		vec3 pb = vec3(b.x, b.y, b.z); \
		\
		vec3 x = 0.5*(pa + pb); \
		vec3 eye_to_pos = x - eye_pos; \
		float key = dot(eye_to_pos, eye_to_pos);)";

	render.sorter->set_key_definition_override(key_definition);

	// done
	return success;
}

void tubes::init_frame (cgv::render::context &ctx)
{
	if(!view_ptr) {
		view_ptr = find_view_as_node();
		if(view_ptr) {
			// do one-time initialization that needs the view if necessary
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
}

void tubes::draw (cgv::render::context &ctx)
{
	if(!view_ptr) return;

	// display drag-n-drop information, if a dnd operation is in progress
	if (!dnd.text.empty())
		draw_dnd(ctx);

	// draw dataset using selected renderer
	if(traj_mgr.has_data())
		draw_trajectories(ctx);
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
		add_gui("style", render.style);
		align("\b");
		end_tree_node(render.style);
	}

	add_member_control(this, "Conservative Depth Test", render.use_conservative_depth, "check");
	add_member_control(this, "Render Percentage", render.percentage, "value_slider", "min=0.0;step=0.001;max=1.0;ticks=true");
	add_member_control(this, "Sort by Distance", render.sort, "check");

	// Misc settings contractable section
	add_decorator("Miscellaneous", "heading", "level=1");
	if (begin_tree_node("Tools (Persistent by Default)", misc_cfg, false))
	{
		align("\a");
		add_member_control(
			this, "instant_redraw_proxy", misc_cfg.instant_redraw_proxy, "toggle",
			"tooltip='Controls the instant redraw state of the FLTK GL window.'"
		);
		add_member_control(
			this, "fix_view_up_dir_proxy", misc_cfg.fix_view_up_dir_proxy, "toggle",
			"tooltip='Controls the \"fix_view_up_dir\" state of the view interactor.'"
		);
		align("\b");
		end_tree_node(misc_cfg);
	}
}

void tubes::update_attribute_bindings (void)
{
	auto &ctx = *get_context();
	if (traj_mgr.has_data())
	{
		render.arclen_sbo.destruct(ctx);
		render.arclen_data = arclen::compile_renderdata(traj_mgr);
		render.arclen_sbo = arclen::upload_renderdata(ctx, render.arclen_data);

		auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);
		tstr.enable_attribute_array_manager(ctx, render.aam);
		tstr.set_position_array(ctx, render.data->positions);
		tstr.set_tangent_array(ctx, render.data->tangents);
		tstr.set_radius_array(ctx, render.data->radii);
		tstr.set_color_array(ctx, render.data->colors);
		tstr.set_indices(ctx, render.data->indices);
		tstr.disable_attribute_array_manager(ctx, render.aam);

		if(!render.sorter->init(ctx, render.data->indices.size() / 2))
			std::cout << "Could not initialize gpu sorter" << std::endl;
	}
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
	vec3 view_dir = view_ptr->get_view_dir();


	fbc.enable(ctx);
	ctx.push_modelview_matrix();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	

	auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);

	// sort the indices
	int pos_handle = tstr.get_vbo_handle(ctx, render.aam, "position");
	int idx_handle = tstr.get_index_buffer_handle(render.aam);

	if(pos_handle > 0 && idx_handle > 0 && render.sort)
		render.sorter->sort(ctx, pos_handle, idx_handle, eye_pos);

	tstr.set_eye_pos(eye_pos);
	tstr.set_view_dir(view_dir);
	tstr.set_viewport(vec4(2.0f) / vec4(viewport[0], viewport[1], viewport[2], viewport[3]));
	tstr.enable_conservative_depth(render.use_conservative_depth);
	tstr.set_render_style(render.style);
	tstr.enable_attribute_array_manager(ctx, render.aam);
	//tstr.render(ctx, 0, render.data->indices.size());

	int count = render.percentage * render.data->indices.size();
	if(count & 1) count += 1;
	count = std::min(count, (int)render.data->indices.size());
	tstr.render(ctx, 0, count);

	tstr.disable_attribute_array_manager(ctx, render.aam);


	ctx.pop_modelview_matrix();
	fbc.disable(ctx);





	shader_program& prog = shaders.get("screen");
	prog.enable(ctx);

	fbc.enable_attachment(ctx, "albedo", 0);
	fbc.enable_attachment(ctx, "position", 1);
	fbc.enable_attachment(ctx, "normal", 2);
	fbc.enable_attachment(ctx, "texcoord", 3);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	fbc.disable_attachment(ctx, "albedo");
	fbc.disable_attachment(ctx, "position");

	prog.disable(ctx);
}

////
// Object registration

cgv::base::object_registration<tubes> reg_tubes("");

#ifdef CGV_FORCE_STATIC
	cgv::base::registration_order_definition ro_def("stereo_view_interactor;tubes");
#endif
