#pragma once

#include "cgv_gl/surface_renderer.h"
#include <cgv_reflect_types/media/color.h>

#include "cgv_gl/gl/lib_begin.h"

namespace cgv { // @<
	namespace render { // @<
		class textured_spline_tube_renderer;

		//! reference to a singleton textured spline tube renderer that is shared among drawables
		/*! the second parameter is used for reference counting. Use +1 in your init method,
			-1 in your clear method and default 0 argument otherwise. If internal reference
			counter decreases to 0, singelton renderer is destructed. */
		extern textured_spline_tube_renderer& ref_textured_spline_tube_renderer(context& ctx, int ref_count_change = 0);

		/*!	Style to control the look of textured spline tubes.
		*/
		struct textured_spline_tube_render_style : public surface_render_style
		{
			/// multiplied to the tube radius, initialized to 1
			float radius_scale;
			/// default tube radius, initialized to 1
			float radius;
			/// specifies the calculation routinbe and output of the fragment shader for debug purposes
			enum FragmentMode {
				FM_NO_OP = 0, // discards all fragments
				FM_RASTERIZE_DEBUG = 1, // show rasterized fragments but don't perform ray cast
				FM_RAY_CAST_DEBUG = 2, // do ray cast but don't discard fragments with no hit
				FM_RAY_CAST = 3 // full ray cast only keeping the intersected fragments
			} fragment_mode;
			/// specifies the type of bounding geometry to use for ray casting
			enum BoundingGeometry {
				BG_BOX = 0, // not implemented yet
				BG_APPROXIMATE_BILLBOARD = 1, // quadrilateral billboard built from encapsulating the bounding box in a rounded cone and using the rounded cone billboard
				BG_EXACT_POLYGON = 2, // a single polygon with 4 or 6 corners covering exactly the bounding box silhouette in screen space
				BG_BOX_BILLBOARD = 3, // a single quadrilateral covering the full bounding box
				BG_ALIGNED_BOX_BILLBOARD = 4 // a single quadrilateral calculated as the front side of a view-aligned bounding box
			} bounding_geometry;
			/// whether to use conservative depth extension to re-enable early depth testing
			bool use_conservative_depth;
			/// whether to calculate tangents from the cubic hermite definition or from the two quadratic bezier segments
			bool use_cubic_tangents;
			/// whether to use the vertex position in view space or the projected position for ray casting (reduces per-vertex output when turned off)
			bool use_view_space_position;
			/// the minimum distance from the camera from which segment end caps are clipped to prevent z-fighting artifacts
			float cap_clip_distance;
			/// construct with default values
			textured_spline_tube_render_style();
		};

		/// renderer that supports textured cubic hermite spline tubes
		class textured_spline_tube_renderer : public surface_renderer
		{
		protected:
			/// whether node ids are specified
			bool has_node_ids;
			/// whether radii are specified
			bool has_radii;
			/// whether tangents are specified
			bool has_tangents;
			/// position of the eye point
			vec3 eye_pos;
			/// camera view direction
			vec3 view_dir;
			/// viewport rectangle (offset and size)
			vec4 viewport;
			/// additional defines not dependant on the style and set from outside the renderer
			shader_define_map additional_defines;
			/// overload to allow instantiation of box_renderer
			render_style* create_render_style() const;
			/// update shader defines based on render style
			void update_defines(shader_define_map& defines);
			/// build rounded cone program
			bool build_shader_program(context& ctx, shader_program& prog, const shader_define_map& defines);
		public:
			/// initializes position_is_center to true 
			textured_spline_tube_renderer();
			/// call this before setting attribute arrays to manage attribute array in given manager
			void enable_attribute_array_manager(const context& ctx, attribute_array_manager& aam);
			/// call this after last render/draw call to ensure that no other users of renderer change attribute arrays of given manager
			void disable_attribute_array_manager(const context& ctx, attribute_array_manager& aam);
			///
			void set_eye_pos(const vec3& eye_pos) { this->eye_pos = eye_pos; }
			///
			void set_view_dir(const vec3& view_dir) { this->view_dir = view_dir; }
			///
			void set_viewport(const vec4& viewport) { this->viewport = viewport; }
			/// set additional defines that do not depend on the style
			void set_additional_defines(shader_define_map& defines);
			///
			template <typename T = float>
			void set_node_id_array(const context& ctx, const std::vector<T>& node_ids) { has_node_ids = true; set_attribute_array(ctx, "node_ids", node_ids); }
			/// 
			template <typename T = float>
			void set_node_id_array(const context& ctx, const T* node_ids, size_t nr_elements, unsigned stride_in_bytes = 0) { has_node_ids = true; set_attribute_array(ctx, "node_ids", node_ids, nr_elements, stride_in_bytes); }
			///
			template <typename T = float>
			void set_radius_array(const context& ctx, const std::vector<T>& radii) { has_radii = true; set_attribute_array(ctx, "radius", radii); }
			/// 
			template <typename T = float>
			void set_radius_array(const context& ctx, const T* radii, size_t nr_elements, unsigned stride_in_bytes = 0) { has_radii = true; set_attribute_array(ctx, "radius", radii, nr_elements, stride_in_bytes); }
			///
			template <typename T = float>
			void set_tangent_array(const context& ctx, const std::vector<T>& tangents) { has_tangents = true; set_attribute_array(ctx, "tangent", tangents); }
			/// 
			template <typename T = float>
			void set_tangent_array(const context& ctx, const T* tangents, size_t nr_elements, unsigned stride_in_bytes = 0) { has_tangents = true; set_attribute_array(ctx, "tangent", tangents, nr_elements, stride_in_bytes); }
			///
			bool validate_attributes(const context& ctx) const;
			/// 
			bool enable(context& ctx);
			///
			bool disable(context& ctx);
			///
			void draw(context& ctx, size_t sunt, size_t count,
				bool use_strips = false, bool use_adjacency = false, uint32_t strip_resunt_index = -1);
		};

		struct textured_spline_tube_render_style_reflect : public textured_spline_tube_render_style
		{
			bool self_reflect(cgv::reflect::reflection_handler& rh);
		};
		extern cgv::reflect::extern_reflection_traits<textured_spline_tube_render_style, textured_spline_tube_render_style_reflect> get_reflection_traits(const textured_spline_tube_render_style&);
	}
}

#include <cgv/config/lib_end.h>
