#include "tubes.h"

// CGV framework core
#include <cgv/math/ftransform.h>
#include <cgv/media/image/image_reader.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/stopwatch.h>

// fltk_gl_view for controlling instant redraw
#include <plugins/cg_fltk/fltk_gl_view.h>

// stereo_view_interactor for controlling fix_view_up_dir
#include <plugins/crg_stereo_view/stereo_view_interactor.h>

// local includes
#include "arclen_helper.h"



tubes::tubes() : application_plugin("tubes_instance")
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
	render.style.use_conservative_depth = true;

	vstyle.enable_depth_test = false;

	ao_style.sample_distance = 0.5f;
	ao_style.cone_angle = 40.0f;

	shaders.add("screen", "screen_quad.glpr");

	// add frame buffer attachments needed for deferred rendering
	fbc.add_attachment("depth", "[D]");
	fbc.add_attachment("albedo", "flt32[R,G,B]");
	fbc.add_attachment("position", "flt32[R,G,B]");
	fbc.add_attachment("normal", "flt32[R,G,B]");
	fbc.add_attachment("texcoord", "flt32[R,G]");

	tf_editor_ptr = register_overlay<cgv::glutil::transfer_function_editor>("Volume TF");
	tf_editor_ptr->set_visibility(false);

	grids.resize(2);
	grids[0].scaling = vec2(5.0, 1.0);
	grids[0].thickness = 0.05;
	grids[0].blend_factor = 0.75;
	grids[1].scaling = vec2(50.0, 10.0);
	grids[1].thickness = 0.1;
	grids[1].blend_factor = 0.5;
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
	// - remaining logic
	update_member(member_ptr);
	post_redraw();
}

bool tubes::init (cgv::render::context &ctx)
{
	// increase reference count of the renderers by one
	auto &tstr = ref_textured_spline_tube_renderer(ctx, 1);
	auto &vr = ref_volume_renderer(ctx, 1);
	bool success = tstr.ref_prog().is_linked() && vr.ref_prog().is_linked();

	// load all shaders in the library
	success &= shaders.load_shaders(ctx);

	// init shared attribute array manager
	success &= render.aam.init(ctx);

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

	cgv::data::data_format tex_format;
	cgv::media::image::image_reader image(tex_format);
	cgv::data::data_view tex_data;

	std::string file_name = "res://plus.png";
	if(!image.read_image(file_name, tex_data)) {
		std::cout << "Error: Could not read image file " << file_name << std::endl;
		return false;
	} else {
		tex.create(ctx, tex_data, 0);
		tex.set_min_filter(cgv::render::TextureFilter::TF_LINEAR);
		tex.set_mag_filter(cgv::render::TextureFilter::TF_LINEAR);
		tex.set_wrap_s(cgv::render::TextureWrap::TW_REPEAT);
		tex.set_wrap_t(cgv::render::TextureWrap::TW_REPEAT);
	}
	image.close();

	success &= load_transfer_function(ctx);

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
}

void tubes::draw (cgv::render::context &ctx)
{
	if(!view_ptr) return;

	// display drag-n-drop information, if a dnd operation is in progress
	if (!dnd.text.empty())
		draw_dnd(ctx);

	// draw dataset using selected renderer
	if(show_volume) {
		draw_density_volume(ctx);
	} else {
		if(traj_mgr.has_data())
			draw_trajectories(ctx);
	}
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
		add_gui("ao_style", ao_style);
		align("\b");
		end_tree_node(ao_style);
	}

	if(begin_tree_node("Volume Style", vstyle, false)) {
		align("\a");
		add_member_control(this, "Show Volume", show_volume, "check");
		add_gui("vstyle", vstyle);
		align("\b");
		end_tree_node(vstyle);
	}

	// attribute mapping settings
	add_decorator("Attribute Mapping", "heading", "level=1");
	if(begin_tree_node("Grid", grids, false)) {
		align("\a");
		add_member_control(this, "Mode", grid_mode, "dropdown", "enums='Color, Bump, Color + Bump'");
		add_member_control(this, "Bump Scale", bump_scale, "value_slider", "min=0;max=0.2;step=0.001;log=true;ticks=true");
		for(size_t i = 0; i < grids.size(); ++i) {
			add_decorator("Grid " + std::to_string(i), "heading", "level=3");
			add_member_control(this, "Scaling U", grids[i].scaling[0], "value_slider", "min=1;max=50;step=0.5;ticks=true");
			add_member_control(this, "Scaling V", grids[i].scaling[1], "value_slider", "min=1;max=50;step=0.5;ticks=true");
			add_member_control(this, "Thickness", grids[i].thickness, "value_slider", "min=0;max=1;step=0.01;ticks=true");
			add_member_control(this, "Blend Factor", grids[i].blend_factor, "value_slider", "min=0;max=1;step=0.01;ticks=true");
		}
		align("\b");
		end_tree_node(am_parameters);
	}

	if(begin_tree_node("Parameters", am_parameters, true)) {
		align("\a");
		add_member_control(this, "Gylph Type", am_parameters.glyph_type, "dropdown", "enums='Circle,Ring,Wedge,Arc,Triangle'");
		add_member_control(this, "Curvature Correction", am_parameters.curvature_correction, "check");
		add_member_control(this, "Radius 0", am_parameters.radius0, "value_slider", "min=0;max=1;step=0.01;ticks=true");
		add_member_control(this, "Radius 1", am_parameters.radius1, "value_slider", "min=0;max=1;step=0.01;ticks=true");
		add_member_control(this, "Angle 0", am_parameters.angle0, "value_slider", "min=0;max=360;step=0.01;ticks=true");
		add_member_control(this, "Angle 1", am_parameters.angle1, "value_slider", "min=0;max=360;step=0.01;ticks=true");
		// only for teting purposes
		add_member_control(this, "Length Scale", am_parameters.length_scale, "value_slider", "min=0.1;max=10;step=0.01;ticks=true;color=0xff0000");
		align("\b");
		end_tree_node(am_parameters);
	}
	
	if(begin_tree_node("Transfer Function Editor", tf_editor_ptr, false)) {
		align("\a");
		tf_editor_ptr->create_gui(*this);
		align("\b");
		end_tree_node(tf_editor_ptr);
	}

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

void tubes::set_view(void)
{
	if(!view_ptr || !traj_mgr.has_data()) return;

	// TODO: is an accurate bounding box necessary?
	// get a crude approximation of the bounding box
	auto& positions = render.data->positions;

	box3 bbox;

	for(unsigned i = 0; i < positions.size(); ++i) {
		bbox.add_point(positions[i]);
	}

	view_ptr->set_focus(bbox.get_center());
	double extent_factor = 0.8;
	view_ptr->set_y_extent_at_focus(extent_factor * (double)length(bbox.get_extent()));
}

void tubes::update_attribute_bindings (void)
{
	auto &ctx = *get_context();

	set_view();
	calculate_bounding_box();
	
	create_density_volume(ctx, 128);

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

void tubes::calculate_bounding_box(void) {

	cgv::utils::stopwatch s(true);
	std::cout << "Calculating bounding box... ";

	bbox.invalidate();
	
	if(traj_mgr.has_data()) {
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

			bbox.add_axis_aligned_box(hermite_spline_tube::calculate_bounding_box(
				p0, p1,
				vec3(t0), vec3(t1),
				r0, r1,
				t0.w(), t1.w(),
				true
			));
		}
	}

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;
}

/** 3D Quadratic Bezier SDF
	https://www.shadertoy.com/view/ldj3Wh
*/
tubes::vec2 tubes::sd_quadratic_bezier(const vec3& A, const vec3& B, const vec3& C, const vec3& pos) {

	vec3 a = B - A;
	vec3 b = A - 2.0f*B + C;
	vec3 c = a * 2.0f;
	vec3 d = A - pos;

	float kk = 1.0 / dot(b, b);
	float kx = kk * dot(a, b);
	float ky = kk * (2.0f*dot(a, a) + dot(d, b)) / 3.0f;
	float kz = kk * dot(d, a);

	vec2 res;

	float p = ky - kx * kx;
	float p3 = p * p*p;
	float q = kx * (2.0f*kx*kx - 3.0f*ky) + kz;
	float h = q * q + 4.0f*p3;

	if(h >= 0.0) {
		h = sqrt(h);
		vec2 x = (vec2(h, -h) - q) / 2.0;
		vec2 uv = sign(x)*cgv::math::pow(abs(x), vec2(1.0f / 3.0f));
		float t = cgv::math::clamp(uv.x() + uv.y() - kx, 0.0f, 1.0f);

		// 1 root
		res = vec2(cgv::math::sqr_length(d + (c + b * t)*t), t);
	} else {
		float z = sqrt(-p);
		float v = acos(q / (p*z*2.0f)) / 3.0f;
		float m = cos(v);
		float n = sin(v)*1.732050808f;
		vec3 t = cgv::math::clamp(vec3(m + m, -n - m, n - m) * z - kx, 0.0f, 1.0f);

		// 3 roots, but only need two
		float dis = cgv::math::sqr_length(d + (c + b * t.x())*t.x());
		res = vec2(dis, t.x());

		dis = cgv::math::sqr_length(d + (c + b * t.y())*t.y());
		if(dis < res.x()) res = vec2(dis, t.y());
	}

	res.x() = sqrt(res.x());
	return res;
}

int tubes::sample_voxel(const ivec3& vidx, const hermite_spline_tube::q_tube& qt) {

	const unsigned num_samples_per_dim = 3;
	const float vs = density_volume.voxel_size / static_cast<float>(num_samples_per_dim);
	const vec3 vo(0.5f * vs);

	const vec3 sample_position_offsets[27] = {
		vo + vec3(1,1,1) * vs,
		
		vo + vec3(0,0,0) * vs,
		vo + vec3(0,0,1) * vs,
		vo + vec3(0,0,2) * vs,
		vo + vec3(0,1,0) * vs,
		vo + vec3(0,1,1) * vs,
		vo + vec3(0,1,2) * vs,
		vo + vec3(0,2,0) * vs,
		vo + vec3(0,2,1) * vs,
		vo + vec3(0,2,2) * vs,
		vo + vec3(1,0,0) * vs,
		vo + vec3(1,0,1) * vs,
		vo + vec3(1,0,2) * vs,
		vo + vec3(1,1,0) * vs,

		vo + vec3(1,1,2) * vs,
		vo + vec3(1,2,0) * vs,
		vo + vec3(1,2,1) * vs,
		vo + vec3(1,2,2) * vs,
		vo + vec3(2,0,0) * vs,
		vo + vec3(2,0,1) * vs,
		vo + vec3(2,0,2) * vs,
		vo + vec3(2,1,0) * vs,
		vo + vec3(2,1,1) * vs,
		vo + vec3(2,1,2) * vs,
		vo + vec3(2,2,0) * vs,
		vo + vec3(2,2,1) * vs,
		vo + vec3(2,2,2) * vs
	};

	float half_voxel_diag = 0.5f * sqrt(3.0f) * density_volume.voxel_size;

	vec3 voxel_min = density_volume.bounds.ref_min_pnt() + vec3(vidx) * density_volume.voxel_size;

	int count = 1;

	vec3 spos = voxel_min + sample_position_offsets[0];
	vec2 dist = sd_quadratic_bezier(qt.s.pos, qt.h.pos, qt.e.pos, spos);
	if(dist.x() - qt.s.rad > half_voxel_diag)
		return 0;

	for(unsigned k = 1; k < 27; ++k) {
		//vec3 idx = sample_position_offsets[k];// (i, j, k);
		vec3 spos = voxel_min + sample_position_offsets[k];// vo + idx * vs;

		// TODO: move to own quadratic tube class
		vec2 dist = sd_quadratic_bezier(qt.s.pos, qt.h.pos, qt.e.pos, spos);

		// TODO: currently a constant radius per segment is assumed
		if(dist.x() <= qt.s.rad)
			++count;
	}
	
	return count;
}

void tubes::voxelize_q_tube(const hermite_spline_tube::q_tube& qt) {

	box3 box = hermite_spline_tube::q_spline_exact_bbox(qt);

	ivec3 sidx((box.get_min_pnt() - density_volume.bounds.ref_min_pnt()) / density_volume.voxel_size);
	ivec3 eidx((box.get_max_pnt() - density_volume.bounds.ref_min_pnt()) / density_volume.voxel_size);
	sidx = cgv::math::clamp(sidx, ivec3(0), density_volume.resolution - 1);
	eidx = cgv::math::clamp(eidx, ivec3(0), density_volume.resolution - 1);

	ivec3 res = density_volume.resolution;

	for(unsigned z = sidx.z(); z <= eidx.z(); ++z) {
		for(unsigned y = sidx.y(); y <= eidx.y(); ++y) {
			for(unsigned x = sidx.x(); x <= eidx.x(); ++x) {
				int count = sample_voxel(ivec3(x, y, z), qt);
				float occupancy = static_cast<float>(count) * 1.0f / 27.0f;

				int idx = x + res.x() * y + res.x() * res.y() * z;

#pragma omp atomic
				density_volume.data[idx] += occupancy;
			}
		}
	}
}

void tubes::create_density_volume(context& ctx, unsigned resolution) {

	cgv::utils::stopwatch s(true);

	density_volume.data.clear();
	density_volume.compute_bounding_box(bbox, resolution);

	density_volume.data.resize(density_volume.resolution.x() * density_volume.resolution.y() * density_volume.resolution.z(), 0.0f);

	std::cout << "Generating density volume with resolution (" << density_volume.resolution.x() << ", " << density_volume.resolution.y() << ", " << density_volume.resolution.z() << ")... ";
	
	auto& positions = render.data->positions;
	auto& tangents = render.data->tangents;
	auto& radii = render.data->radii;
	auto& indices = render.data->indices;

#pragma omp parallel for
	for(int i = 0; i < indices.size(); i += 2) {
		unsigned idx_a = indices[i + 0];
		unsigned idx_b = indices[i + 1];

		vec3 p0 = positions[idx_a];
		vec3 p1 = positions[idx_b];
		float r0 = radii[idx_a];
		float r1 = radii[idx_b];
		vec4 t0 = tangents[idx_a];
		vec4 t1 = tangents[idx_b];

		hermite_spline_tube::q_tube qt0, qt1;
		hermite_spline_tube::split_to_qtubes(
			p0, p1,
			vec3(t0), vec3(t1),
			r0, r1,
			t0.w(), t1.w(),
			qt0, qt1
		);

		voxelize_q_tube(qt0);
		voxelize_q_tube(qt1);
	}

	for(unsigned i = 0; i < density_volume.data.size(); ++i)
		density_volume.data[i] = cgv::math::clamp(density_volume.data[i], 0.0f, 1.0f);

	if(density_tex.is_created())
		density_tex.destruct(ctx);

	cgv::data::data_view dv = cgv::data::data_view(new cgv::data::data_format(density_volume.resolution.x(), density_volume.resolution.y(), density_volume.resolution.z(), TI_FLT32, cgv::data::CF_R), density_volume.data.data());
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
	for(int i = 0; i < density_volume.data.size(); ++i) {
		// don't count 0
		unsigned bin = static_cast<unsigned>(density_volume.data[i] * (n_bins - 1));
		if(bin != 0)
			hist[bin] += 1;
	}

	tf_editor_ptr->set_histogram(hist);



	ao_style.derive_voxel_grid_parameters(density_volume);
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

	// sort the indices
	int pos_handle = tstr.get_vbo_handle(ctx, render.aam, "position");
	int idx_handle = tstr.get_index_buffer_handle(render.aam);

	if(pos_handle > 0 && idx_handle > 0 && render.sort)
		render.sorter->sort(ctx, pos_handle, idx_handle, eye_pos);

	tstr.set_eye_pos(eye_pos);
	tstr.set_view_dir(view_dir);
	tstr.set_viewport(vec4(2.0f) / vec4(viewport[0], viewport[1], viewport[2], viewport[3]));
	tstr.set_render_style(render.style);
	tstr.enable_attribute_array_manager(ctx, render.aam);
	//tstr.render(ctx, 0, render.data->indices.size());

	int count = render.percentage * render.data->indices.size();
	if(count & 1) count += 1;
	count = std::min(count, (int)render.data->indices.size());
	tstr.render(ctx, 0, count);

	tstr.disable_attribute_array_manager(ctx, render.aam);

	fbc.disable(ctx);

	shader_program& prog = shaders.get("screen");
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
	prog.set_uniform(ctx, "grid_mode", (int)grid_mode);
	prog.set_uniform(ctx, "bump_scale", bump_scale);
	for(size_t i = 0; i < grids.size(); ++i) {
		std::string base_name = "grids[" + std::to_string(i) + "].";
		prog.set_uniform(ctx, base_name + "scaling", grids[i].scaling);
		prog.set_uniform(ctx, base_name + "thickness", grids[i].thickness);
		prog.set_uniform(ctx, base_name + "blend_factor", grids[i].blend_factor);
	}

	// set attribute mapping parameters
	prog.set_uniform(ctx, "attribute_mapping.glyph_type", (int)am_parameters.glyph_type);
	prog.set_uniform(ctx, "attribute_mapping.curvature_correction", am_parameters.curvature_correction);
	prog.set_uniform(ctx, "attribute_mapping.radius0", am_parameters.radius0);
	prog.set_uniform(ctx, "attribute_mapping.radius1", am_parameters.radius1);
	prog.set_uniform(ctx, "attribute_mapping.angle0", am_parameters.angle0);
	prog.set_uniform(ctx, "attribute_mapping.angle1", am_parameters.angle1);

	prog.set_uniform(ctx, "attribute_mapping.length_scale", am_parameters.length_scale);

	const surface_render_style& srs = *static_cast<const surface_render_style*>(&render.style);
	
	prog.set_uniform(ctx, "map_color_to_material", int(srs.map_color_to_material));
	prog.set_uniform(ctx, "culling_mode", int(srs.culling_mode));
	prog.set_uniform(ctx, "illumination_mode", int(srs.illumination_mode));


	fbc.enable_attachment(ctx, "albedo", 0);
	fbc.enable_attachment(ctx, "position", 1);
	fbc.enable_attachment(ctx, "normal", 2);
	fbc.enable_attachment(ctx, "texcoord", 3);
	fbc.enable_attachment(ctx, "depth", 4);
	tex.enable(ctx, 5);
	density_tex.enable(ctx, 6);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	fbc.disable_attachment(ctx, "albedo");
	fbc.disable_attachment(ctx, "position");
	fbc.disable_attachment(ctx, "normal");
	fbc.disable_attachment(ctx, "texcoord");
	fbc.disable_attachment(ctx, "depth");
	tex.disable(ctx);
	density_tex.disable(ctx);

	prog.disable(ctx);
}

void tubes::draw_density_volume(context& ctx) {

	auto& vr = ref_volume_renderer(ctx);
	vr.set_render_style(vstyle);
	vr.set_volume_texture(&density_tex);
	//vr.set_transfer_function_texture(&tf_tex);
	vr.set_transfer_function_texture(&tf_editor_ptr->ref_tex());
	
	vr.set_bounding_box(density_volume.bounds);
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
		
		for(int i = 0; i < w; ++i) {
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

////
// Object registration

cgv::base::object_registration<tubes> reg_tubes("");

#ifdef CGV_FORCE_STATIC
	cgv::base::registration_order_definition ro_def("stereo_view_interactor;tubes");
#endif



#include <cgv/gui/provider.h>

namespace cgv {
	namespace gui {

	/// define a gui creator for the ambient occlusion style struct
	struct ambient_occlusion_style_gui_creator : public gui_creator {
		/// attempt to create a gui and return whether this was successful
		bool create(provider* p, const std::string& label, void* value_ptr, const std::string& value_type, const std::string& gui_type, const std::string& options, bool*)
		{
			if(value_type != cgv::type::info::type_name<tubes::ambient_occlusion_style>::get_name())
				return false;

			tubes::ambient_occlusion_style* s_ptr = reinterpret_cast<tubes::ambient_occlusion_style*>(value_ptr);
			cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);

			p->add_member_control(b, "Enable", s_ptr->enable, "check");
			p->add_member_control(b, "Sample Offset", s_ptr->sample_offset, "value_slider", "min=0.0;step=0.0001;max=0.2;log=true;ticks=true");
			p->add_member_control(b, "Sample Distance", s_ptr->sample_distance, "value_slider", "min=0.0;step=0.0001;max=1.0;log=true;ticks=true");
			p->add_member_control(b, "Strength Scale", s_ptr->strength_scale, "value_slider", "min=0.0;step=0.0001;max=10.0;log=true;ticks=true");
			
			connect_copy(
				p->add_member_control(b, "Cone Angle", s_ptr->cone_angle, "value_slider", "min=10.0;step=0.0001;max=90.0;ticks=true")->value_change,
				cgv::signal::rebind(s_ptr, &tubes::ambient_occlusion_style::generate_sample_dirctions));

			return true;
		}
	};

#include <cgv_gl/gl/lib_begin.h>

	cgv::gui::gui_creator_registration<ambient_occlusion_style_gui_creator> ambient_occlusion_s_gc_reg("ambient_occlusion_style_gui_creator");
	}
}
