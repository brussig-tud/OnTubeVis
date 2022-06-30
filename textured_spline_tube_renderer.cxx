#include "textured_spline_tube_renderer.h"
#include <cgv_gl/gl/gl.h>
#include <cgv_gl/gl/gl_tools.h>

namespace cgv {
	namespace render {
		textured_spline_tube_renderer& ref_textured_spline_tube_renderer(context& ctx, int ref_count_change)
		{
			static int ref_count = 0;
			static textured_spline_tube_renderer r;
			r.manage_singleton(ctx, "textured_spline_tube_renderer", ref_count, ref_count_change);
			return r;
		}

		render_style* textured_spline_tube_renderer::create_render_style() const
		{
			return new textured_spline_tube_render_style();
		}

		textured_spline_tube_render_style::textured_spline_tube_render_style()
		{
			radius_scale = 1.0f;
			radius = 1.0f;
			fragment_mode = FM_RAY_CAST;
			bounding_geometry = BG_ALIGNED_BOX_BILLBOARD;
			attrib_mode = AM_ALL;
			use_conservative_depth = false;
			use_cubic_tangents = true;
			use_view_space_position = true;
			cap_clip_distance = 20.0f;
		}

		textured_spline_tube_renderer::textured_spline_tube_renderer()
		{
			has_node_ids = false;
			has_radii = false;
			has_tangents = false;
		}
		/// call this before setting attribute arrays to manage attribute array in given manager
		void textured_spline_tube_renderer::enable_attribute_array_manager(const context& ctx, attribute_array_manager& aam)
		{
			surface_renderer::enable_attribute_array_manager(ctx, aam);
			if (has_attribute(ctx, "radius"))
				has_radii = true;
			if (has_attribute(ctx, "tangent"))
				has_tangents = true;
		}
		/// call this after last render/draw call to ensure that no other users of renderer change attribute arrays of given manager
		void textured_spline_tube_renderer::disable_attribute_array_manager(const context& ctx, attribute_array_manager& aam)
		{
			surface_renderer::disable_attribute_array_manager(ctx, aam);
			has_radii = false;
			has_tangents = false;
		}
		bool textured_spline_tube_renderer::validate_attributes(const context& ctx) const
		{
			// validate set attributes
			//bool res = surface_renderer::validate_attributes(ctx);
			//return res;

			if(!has_node_ids) {
				ctx.error("renderer::enable() node id attribute not set");
				return false;
			}
			return true;
		}
		void textured_spline_tube_renderer::set_additional_defines(shader_define_map& defines) {
			additional_defines = defines;
		}
		void textured_spline_tube_renderer::update_defines(shader_define_map& defines)
		{
			const textured_spline_tube_render_style& rs = get_style<textured_spline_tube_render_style>();

			defines.clear();

			shader_code::set_define(defines, "USE_CONSERVATIVE_DEPTH", rs.use_conservative_depth, false);
			shader_code::set_define(defines, "USE_CUBIC_TANGENTS", rs.use_cubic_tangents, true);
			shader_code::set_define(defines, "USE_VIEW_SPACE_POSITION", rs.use_view_space_position, true);
			shader_code::set_define(defines, "ATTRIB_MODE", rs.attrib_mode, textured_spline_tube_render_style::AM_ALL);
			shader_code::set_define(defines, "BOUNDING_GEOMETRY_TYPE", rs.bounding_geometry, textured_spline_tube_render_style::BG_ALIGNED_BOX_BILLBOARD);
			shader_code::set_define(defines, "MODE", rs.fragment_mode, textured_spline_tube_render_style::FM_RAY_CAST);

			for(const auto& define : additional_defines)
				defines.insert(define);
		}
		bool textured_spline_tube_renderer::build_shader_program(context& ctx, shader_program& prog, const shader_define_map& defines)
		{
			return prog.build_program(ctx, "textured_spline_tube.glpr", true, defines);
		}
		bool textured_spline_tube_renderer::enable(context& ctx)
		{
			const textured_spline_tube_render_style& rs = get_style<textured_spline_tube_render_style>();

			if (!surface_renderer::enable(ctx))
				return false;
			
			if(!ref_prog().is_linked())
				return false;

			ref_prog().set_uniform(ctx, "radius_scale", rs.radius_scale);
			ref_prog().set_uniform(ctx, "eye_pos", eye_pos);
			ref_prog().set_uniform(ctx, "view_dir", view_dir);
			ref_prog().set_uniform(ctx, "viewport", viewport);
			ref_prog().set_uniform(ctx, "cap_clip_distance", rs.cap_clip_distance);

			return true;
		}
		///
		bool textured_spline_tube_renderer::disable(context& ctx)
		{
			if (!attributes_persist()) {
				has_radii = false;
				has_tangents = false;
			}

			return surface_renderer::disable(ctx);
		}

		void textured_spline_tube_renderer::draw(context& ctx, size_t start, size_t count, bool use_strips, bool use_adjacency, uint32_t strip_restart_index)
		{
			glDisable(GL_CULL_FACE);
			draw_impl(ctx, PT_POINTS, start, count, use_strips, use_adjacency, strip_restart_index);
			glEnable(GL_CULL_FACE);
		}

		bool textured_spline_tube_render_style_reflect::self_reflect(cgv::reflect::reflection_handler& rh)
		{
			return
				rh.reflect_base(*static_cast<surface_render_style*>(this)) &&
				rh.reflect_member("radius", radius) &&
				rh.reflect_member("radius_scale", radius_scale);
		}

		cgv::reflect::extern_reflection_traits<textured_spline_tube_render_style, textured_spline_tube_render_style_reflect> get_reflection_traits(const textured_spline_tube_render_style&)
		{
			return cgv::reflect::extern_reflection_traits<textured_spline_tube_render_style, textured_spline_tube_render_style_reflect>();
		}
	}
}

#include <cgv/gui/provider.h>

namespace cgv {
	namespace gui {

	struct textured_spline_tube_render_style_gui_creator : public gui_creator {
		/// attempt to create a gui and return whether this was successful
		bool create(provider* p, const std::string& label,
			void* value_ptr, const std::string& value_type,
			const std::string& gui_type, const std::string& options, bool*) {
			if(value_type != cgv::type::info::type_name<cgv::render::textured_spline_tube_render_style>::get_name())
				return false;
			cgv::render::textured_spline_tube_render_style* rs_ptr = reinterpret_cast<cgv::render::textured_spline_tube_render_style*>(value_ptr);
			cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);

			p->add_member_control(b, "Default Radius", rs_ptr->radius, "value_slider", "min=0.001;step=0.0001;max=10.0;log=true;ticks=true");
			p->add_member_control(b, "Radius Scale", rs_ptr->radius_scale, "value_slider", "min=0.01;step=0.0001;max=100.0;log=true;ticks=true");

			p->add_member_control(b, "Conservative Depth", rs_ptr->use_conservative_depth, "check");
			p->add_member_control(b, "Cubic Tangents", rs_ptr->use_cubic_tangents, "check");
			p->add_member_control(b, "View Space Position", rs_ptr->use_view_space_position, "check");

			p->add_member_control(b, "Cap Clip Distance", rs_ptr->cap_clip_distance, "value_slider", "min=0.0;max=100.0;step=0.01;ticks=true");
			p->add_member_control(b, "Attribute-Less Mode", rs_ptr->attrib_mode, "dropdown", "enums='Off,No curve data,No node color,Attribute-less'");

			p->add_gui("surface_render_style", *static_cast<cgv::render::surface_render_style*>(rs_ptr));

			if(p->begin_tree_node("Debug Options", rs_ptr->fragment_mode)) {
				p->align("\a");
				p->add_member_control(b, "Bounding Geometry", rs_ptr->bounding_geometry, "dropdown", "enums='Oriented box (use attribute-less mode),Approximate billboard,Exact oriented box flat polygon,Oriented box billboard,View-aligned box billboard,Oriented box simulated split (1 triangle strip),Oriented box simulated split (2 triangle strips),VABB simulated split (1 triangle strip),VABB simulated split (2 triangle strips)'");
				p->add_member_control(b, "Fragment Mode", rs_ptr->fragment_mode, "dropdown", "enums='No-Op, Rasterize Debug, Ray Cast Debug, Ray Cast'");
				p->align("\b");
				p->end_tree_node(rs_ptr->fragment_mode);
			}
			return true;
		}
	};

#include <cgv_gl/gl/lib_begin.h>

		cgv::gui::gui_creator_registration<textured_spline_tube_render_style_gui_creator> textured_spline_tube_rs_gc_reg("textured_spline_tube_render_style_gui_creator");
	}
}
