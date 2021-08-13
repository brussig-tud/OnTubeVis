#include "tubes.h"

// CGV framework core
//#include <cgv/media/image/image.h>
#include <cgv/media/image/image_reader.h>
#include <cgv/utils/advanced_scan.h>
#include <cgv/utils/stopwatch.h>

// CGV OpenGL lib
#include <cgv_gl/box_renderer.h>

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

	brd = cgv::glutil::box_render_data<>(true);
	srd = cgv::glutil::sphere_render_data<>(true);
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
	//ref_spline_tube_renderer(ctx, -1);
	//ref_rounded_cone_renderer(ctx, -1);
	ref_box_renderer(ctx, -1);
	ref_sphere_renderer(ctx, -1);

	shaders.clear(ctx);
	fbc.clear(ctx);

	brd.destruct(ctx);
	srd.destruct(ctx);

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
	//auto &rcr = cgv::render::ref_rounded_cone_renderer(ctx, 1);
	//auto &str = cgv::render::ref_spline_tube_renderer(ctx, 1);
	auto &tstr = ref_textured_spline_tube_renderer(ctx, 1);
	auto &br = ref_box_renderer(ctx, 1);
	bool success = tstr.ref_prog().is_linked() && br.ref_prog().is_linked();

	// load all shaders in the library
	success &= shaders.load_shaders(ctx);

	// init shared attribute array manager
	success &= render.aam.init(ctx);

	ref_sphere_renderer(ctx, 1);

	brd.init(ctx);
	srd.init(ctx);

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

	/*
	Is this of interest?
	https://www.shadertoy.com/view/XtdyDn
	*/

	/*
	Calculation of the v texture coordinate (around the tube) is taken from:
	Works for varying radii.
	Produces twisting under certein circumstances.
	Problem with bulge data set.
	https://www.shadertoy.com/view/XssGWl

	// geometrymatrix
	geometrymatrix[0] = vec4(endpoint0,0.);
	geometrymatrix[1] = vec4(endpoint1,0.);
	geometrymatrix[2] = vec4(tangent0,0.);
	geometrymatrix[3] = vec4(tangent1,0.);

	// MG
	mat4 Mh;
	Mh[0] = vec4( 2, -2,  1,  1);
	Mh[1] = vec4(-3,  3, -2, -1);
	Mh[2] = vec4( 0,  0,  1,  0);
	Mh[3] = vec4( 1,  0,  0,  0);
	sp.MG = geometrymatrix * Mh;



	// ORTHO
	vec3 Spline_EvaluateTangent(mat4 MG, float t )
{
	vec4 tvec = vec4(3.*t*t, 2.*t, 1, 0.);
	vec3 p = (MG*tvec).xyz;
	return p;
}

vec3 Spline_EvaluateBinormal(mat4 MG, float t )
{
	vec4 tvec = vec4(6.*t, 2., 0., 0.);
	vec3 p = (MG*tvec).xyz;
	return p;
}

vec3 SplineOrtho(float t)
{
//	return normalize( Spline_EvaluateBinormal(spline.MG,t));
	return normalize(cross(Spline_EvaluateTangent(spline.MG,t),Spline_EvaluateBinormal(spline.MG,t)));
}


	// TEXTURING
	vec3 n = normalize(X-C);
			//	vec3 n = C;
				//c = vec3(n*0.5+0.5);
				c = 0.5*n+0.5;//vec3(max(dot(n,L),0.));
#if 1
				vec3 b = SplineOrtho(t_close);
				float v = acos(dot(b,n));
				//c *= 0.1 + pow(texture(iChannel1,vec2(t_close,v/16.)*4.).xyz,vec3(2.2));

	*/




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

	/*auto& br = ref_box_renderer(ctx);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	brd.render(ctx, br, box_render_style());
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);*/

	//auto& sr = ref_sphere_renderer(ctx);
	//srd.render(ctx, sr, sphere_render_style());

	/*std::vector<box3> boxes = { bbox };
	auto& br = ref_box_renderer(ctx);
	br.set_box_array(ctx, boxes);

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	br.render(ctx, 0, 1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);*/
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

void tubes::set_view(void)
{
	if(!view_ptr || !traj_mgr.has_data()) return;

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
	
	
	/*
	3D Quadratic Bezier SDF
	https://www.shadertoy.com/view/ldj3Wh

	float dot2( in vec3 v ) { return dot(v,v); }
	vec2 sdBezier(vec3 pos, vec3 A, vec3 B, vec3 C)
	{
		vec3 a = B - A;
		vec3 b = A - 2.0*B + C;
		vec3 c = a * 2.0;
		vec3 d = A - pos;

		float kk = 1.0 / dot(b,b);
		float kx = kk * dot(a,b);
		float ky = kk * (2.0*dot(a,a)+dot(d,b)) / 3.0;
		float kz = kk * dot(d,a);

		vec2 res;

		float p = ky - kx*kx;
		float p3 = p*p*p;
		float q = kx*(2.0*kx*kx - 3.0*ky) + kz;
		float h = q*q + 4.0*p3;

		if(h >= 0.0)
		{
			h = sqrt(h);
			vec2 x = (vec2(h, -h) - q) / 2.0;
			vec2 uv = sign(x)*pow(abs(x), vec2(1.0/3.0));
			float t = clamp(uv.x+uv.y-kx, 0.0, 1.0);

			// 1 root
			res = vec2(dot2(d+(c+b*t)*t),t);
		}
		else
		{
			float z = sqrt(-p);
			float v = acos( q/(p*z*2.0) ) / 3.0;
			float m = cos(v);
			float n = sin(v)*1.732050808;
			vec3 t = clamp( vec3(m+m,-n-m,n-m)*z-kx, 0.0, 1.0);

			// 3 roots, but only need two
			float dis = dot2(d+(c+b*t.x)*t.x);
			res = vec2(dis,t.x);

			dis = dot2(d+(c+b*t.y)*t.y);
			if( dis<res.x ) res = vec2(dis,t.y );
		}

		res.x = sqrt(res.x);
		return res;
	}
	*/
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
		res = vec2(dot2(d + (c + b * t)*t), t);
	} else {
		float z = sqrt(-p);
		float v = acos(q / (p*z*2.0f)) / 3.0f;
		float m = cos(v);
		float n = sin(v)*1.732050808f;
		vec3 t = cgv::math::clamp(vec3(m + m, -n - m, n - m) * z - kx, 0.0f, 1.0f);

		// 3 roots, but only need two
		float dis = dot2(d + (c + b * t.x())*t.x());
		res = vec2(dis, t.x());

		dis = dot2(d + (c + b * t.y())*t.y());
		if(dis < res.x()) res = vec2(dis, t.y());
	}

	res.x() = sqrt(res.x());
	return res;
}

std::vector<std::pair<int, float>> tubes::traverse_line(vec3& a, vec3& b, vec3& vbox_min, float vsize, ivec3& res) {
	
	std::vector<std::pair<int, float>> intervals;

	// Amanatides Woo line traversal algorithm
	vec3 dir = normalize(vec3(b - a));
	vec3 dt;
	ivec3 step;
	vec3 orig_grid = a - vbox_min;
	vec3 dest_grid = b - vbox_min;
	vec3 t(0.0f);
	float ct = 0.0f;

	for(unsigned i = 0; i < 3; ++i) {
		float delta = vsize / dir[i];
		if(dir[i] < 0.0f) {
			dt[i] = -delta;
			t[i] = (floor(orig_grid[i] / vsize) * vsize - orig_grid[i]) / dir[i];
			step[i] = -1;
		} else {
			dt[i] = delta;
			t[i] = ((floor(orig_grid[i] / vsize) + 1) * vsize - orig_grid[i]) / dir[i];
			step[i] = 1;
		}
	}

	ivec3 cell_idx(
		(int)(floor(orig_grid.x() / vsize)),
		(int)(floor(orig_grid.y() / vsize)),
		(int)(floor(orig_grid.z() / vsize))
	);

	ivec3 end_idx(
		(int)(floor(dest_grid.x() / vsize)),
		(int)(floor(dest_grid.y() / vsize)),
		(int)(floor(dest_grid.z() / vsize))
	);

	intervals.push_back(std::make_pair<int, float>(cell_idx[0] + res[0] * cell_idx[1] + res[0] * res[1] * cell_idx[2], 0.0f));

	vec3 p = orig_grid;
	size_t idx = 0;

	while(cell_idx != end_idx) {
		unsigned mi = cgv::math::min_index(t);

		cell_idx[mi] += step[mi];
		if(cell_idx[mi] < 0 || cell_idx[mi] >= res[mi])
			break;
		p = orig_grid + t[mi] * dir;
		t[mi] += dt[mi];

		float l = (orig_grid - p).length() - ct;
		ct += l;
		intervals[idx].second = l;

		intervals.push_back(std::make_pair<int, float>(cell_idx[0] + res[0] * cell_idx[1] + res[0] * res[1] * cell_idx[2], 0.0f));
		++idx;
	}

	float l = (p - dest_grid).length();
	intervals[idx].second = l;

	return intervals;
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
	if(dist.x() - qt.s.rad > half_voxel_diag) {
		return 0;
	}

	for(unsigned k = 1; k < 27; ++k) {
	//for(unsigned k = 0; k < num_samples_per_dim; ++k) {
	//	for(unsigned j = 0; j < num_samples_per_dim; ++j) {
	//		for(unsigned i = 0; i < num_samples_per_dim; ++i) {
				//vec3 idx = sample_position_offsets[k];// (i, j, k);
		vec3 spos = voxel_min + sample_position_offsets[k];// vo + idx * vs;

				// TODO: move to own quadratic tube class
				vec2 dist = sd_quadratic_bezier(qt.s.pos, qt.h.pos, qt.e.pos, spos);

				// TODO: currentyl a constant radius per segment is assumed
				if(dist.x() <= qt.s.rad) {
					++count;
					//srd.add(spos);
					//srd.add(0.05f * density_volume.voxel_size);
					//srd.add(rgb(1.0f, 0.0f, 0.0f));
				}// else {
				//	srd.add(spos);
				//	srd.add(0.01f * density_volume.voxel_size);
				//	srd.add(rgb(0.5f, 0.5f, 0.5f));
				//}
			}
	//	}
	//}

	return count;
}

void tubes::voxelize_q_tube(const hermite_spline_tube::q_tube& qt) {

	box3 box = hermite_spline_tube::q_spline_exact_bbox(qt);

	// TODO: look at particle voxelizer from master thesis for correct formulas
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

//#pragma omp parallel for
	/*for(int i = 0; i < indices.size(); i += 2) {
		unsigned idx_a = indices[i + 0];
		unsigned idx_b = indices[i + 1];

		vec3 p0 = positions[idx_a];
		vec3 p1 = positions[idx_b];
		float r0 = radii[idx_a];
		float r1 = radii[idx_b];
		vec4 t0 = tangents[idx_a];
		vec4 t1 = tangents[idx_b];

		std::vector<std::pair<int, float>> intervals = traverse_line(p0, p1, vbox_min, vsize, ivec3(vres));

		float total_length = (p1 - p0).length();
		float accum_length = 0.0f;

		for(size_t k = 0; k < intervals.size(); ++k) {
			float length = intervals[k].second;

			float alpha0 = accum_length / total_length;
			float alpha1 = (accum_length + length) / total_length;

			float radius0 = (1.0f - alpha0) * r0 + alpha0 * r1;
			float radius1 = (1.0f - alpha1) * r0 + alpha1 * r1;

			float vol = (3.1415f / 3.0f) * (r0*r0 + r0 * r1 + r1 * r1) * length;
			float vol_rel = vol / vvol;

			accum_length += length;
			voxels[intervals[k].first] += vol_rel;
		}
	}*/


	srd.clear();

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

		/*float total_length = (p1 - p0).length();
		float accum_length = 0.0f;

		for(size_t k = 0; k < intervals.size(); ++k) {
			float length = intervals[k].second;

			float alpha0 = accum_length / total_length;
			float alpha1 = (accum_length + length) / total_length;

			float radius0 = (1.0f - alpha0) * r0 + alpha0 * r1;
			float radius1 = (1.0f - alpha1) * r0 + alpha1 * r1;

			float vol = (3.1415f / 3.0f) * (r0 * r0 + r0 * r1 + r1 * r1) * length;
			float vol_rel = vol / vvol;

			accum_length += length;
			voxels[intervals[k].first] += vol_rel;
		}*/
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

	// Set style attributes of volume rendering
	//mat4 vol_transformation = cgv::math::translate4(vbox_min) * cgv::math::scale4(vbox_ext);
	//vstyle.transformation_matrix = vol_transformation;

	std::cout << "done (" << s.get_elapsed_time() << "s)" << std::endl;

	set_ao_uniforms(ctx);

	brd.clear();
	//srd.clear();

	brd.add(density_volume.bounds.get_center(), density_volume.bounds.get_extent());
	brd.add(rgb(1.0f));

	std::vector<rgb> cols = {
		rgb(0.5f),
		rgb(0.0f, 0.0f, 1.0f),
		rgb(0.0f, 0.5f, 0.5f),
		rgb(0.0f, 1.0f, 0.0f),
		rgb(0.5f, 0.5f, 0.0f),
		rgb(1.0f, 0.0f, 0.0f)
	};

	const ivec3& vres = density_volume.resolution;
	const float vsize = density_volume.voxel_size;
	const vec3& vbox_min = density_volume.bounds.ref_min_pnt();

	for(unsigned z = 0; z < vres.z(); ++z) {
		for(unsigned y = 0; y < vres.y(); ++y) {
			for(unsigned x = 0; x < vres.x(); ++x) {
				vec3 voxel_min = vbox_min + vec3(x, y, z)*vsize;
				brd.add(voxel_min + 0.5f * vsize, vec3(vsize));
				brd.add(rgb(0.5f, 1.0f, 0.0f));

				/*for(unsigned k = 0; k < samples; ++k) {
					for(unsigned j = 0; j < samples; ++j) {
						for(unsigned i = 0; i < samples; ++i) {
							vec3 idx(i, j, k);
							vec3 spos = voxel_min + vo + idx * vs;
							srd.add(spos);

							int inside = 0;
							for(unsigned idx = 0; idx < indices.size(); idx += 2) {
								unsigned idx_a = indices[idx + 0];
								unsigned idx_b = indices[idx + 1];

								vec3 p0 = positions[idx_a];
								vec3 p1 = positions[idx_b];
								float r0 = radii[idx_a];
								float r1 = radii[idx_b];
								vec4 t0 = tangents[idx_a];
								vec4 t1 = tangents[idx_b];

								std::vector<vec4> qtn;
								hermite_spline_tube::split_to_qtubes(
									p0, p1,
									vec3(t0), vec3(t1),
									r0, r1,
									t0.w(), t1.w(),
									qtn
								);

								vec2 dist0 = sd_quadratic_bezier(qtn[0], qtn[1], qtn[2], spos);
								vec2 dist1 = sd_quadratic_bezier(qtn[3], qtn[4], qtn[5], spos);
								
								if(dist0.x() <= r0)
									++inside;;
								if(dist1.x() <= r0)
									++inside;
							}

							srd.add(cols[inside]);

							if(inside == 0) {
								srd.add(0.01f * vsize);
							} else {
								srd.add(0.05f * vsize);
							}
						}
					}
				}*/
			}
		}
	}

	//srd.fill(0.05f*vsize);

	brd.set_out_of_date();
	srd.set_out_of_date();
}

void tubes::set_ao_uniforms(context& ctx) {
	
	const box3& volume_bbox = density_volume.bounds;
	const ivec3& volume_resolution = density_volume.resolution;

	auto& prog = shaders.get("screen");
	prog.enable(ctx);
	prog.set_uniform(ctx, "ambient_occlusion.enable", false);
	prog.set_uniform(ctx, "ambient_occlusion.sample_offset", 0.04f);
	prog.set_uniform(ctx, "ambient_occlusion.distance", 0.8f);
	//prog.set_uniform(ctx, "ambient_occlusion.strength_scale", 10.0f);
	prog.set_uniform(ctx, "ambient_occlusion.strength_scale", 1.0f);

	unsigned max_extent_axis = cgv::math::max_index(volume_bbox.get_extent());

	prog.set_uniform(ctx, "ambient_occlusion.tex_offset", volume_bbox.get_min_pnt());
	prog.set_uniform(ctx, "ambient_occlusion.tex_scaling", vec3(1.0f) / volume_bbox.get_extent());
	prog.set_uniform(ctx, "ambient_occlusion.texcoord_scaling", vec3(volume_resolution[max_extent_axis]) / vec3(volume_resolution));
	prog.set_uniform(ctx, "ambient_occlusion.texel_size", 1.0f / volume_resolution[max_extent_axis]);

	// generate 3 cone sample directions
	std::vector<vec3> sample_dirs(3);
	float cone_angle = 50.0f;

	float alpha2 = cgv::math::deg2rad(cone_angle / 2.0f);
	float beta = cgv::math::deg2rad(90.0f - (cone_angle / 2.0f));

	float a = sinf(alpha2);
	float dh = tanf(cgv::math::deg2rad(30.0f)) * a;

	float c = length(vec2(a, dh));

	float b = sqrtf(1 - c * c);

	sample_dirs[0] = vec3(0.0f, b, c);
	sample_dirs[1] = vec3(a, b, -dh);
	sample_dirs[2] = vec3(-a, b, -dh);

	prog.set_uniform(ctx, "ambient_occlusion.cone_angle_factor", 2.0f * sinf(alpha2) / sinf(beta));
	prog.set_uniform_array(ctx, "ambient_occlusion.sample_directions", sample_dirs);
	prog.disable(ctx);
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
	//ctx.push_modelview_matrix();

	

	auto &tstr = cgv::render::ref_textured_spline_tube_renderer(ctx);

	// sort the indices
	int pos_handle = tstr.get_vbo_handle(ctx, render.aam, "position");
	int idx_handle = tstr.get_index_buffer_handle(render.aam);

	if(pos_handle > 0 && idx_handle > 0 && render.sort)
		render.sorter->sort(ctx, pos_handle, idx_handle, eye_pos);

	tstr.set_eye_pos(eye_pos);
	tstr.set_view_dir(view_dir);
	tstr.set_viewport(vec4(2.0f) / vec4(viewport[0], viewport[1], viewport[2], viewport[3]));
	//tstr.enable_conservative_depth(render.use_conservative_depth);
	tstr.set_render_style(render.style);
	tstr.enable_attribute_array_manager(ctx, render.aam);
	//tstr.render(ctx, 0, render.data->indices.size());

	int count = render.percentage * render.data->indices.size();
	if(count & 1) count += 1;
	count = std::min(count, (int)render.data->indices.size());
	tstr.render(ctx, 0, count);

	tstr.disable_attribute_array_manager(ctx, render.aam);


	//ctx.pop_modelview_matrix();
	fbc.disable(ctx);



	/*
	fragment_file:fragment.glfs
	fragment_file:side.glsl
	fragment_file:lights.glsl
	fragment_file:brdf.glsl
	fragment_file:bump_map.glfs
	fragment_file:surface.glsl
	*/

	
	shader_program& prog = shaders.get("screen");
	prog.enable(ctx);
	prog.set_uniform(ctx, "use_gamma", true);

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

////
// Object registration

cgv::base::object_registration<tubes> reg_tubes("");

#ifdef CGV_FORCE_STATIC
	cgv::base::registration_order_definition ro_def("stereo_view_interactor;tubes");
#endif
