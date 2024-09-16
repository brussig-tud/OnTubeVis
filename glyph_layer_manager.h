#pragma once

// make sure this is the first thing the compiler sees, while preventing warnings if
// it happened to already be defined by something else including this header
#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES 1
#endif
#include <list>
#include <vector>

#include <cgv/base/base.h>
#include <cgv/media/color.h>
#include <cgv/gui/provider.h>

#include "gui_util.h"
#include "glyph_attribute_mapping.h"



class glyph_layer_manager : public cgv::base::base
{
public:
	using vec4 = cgv::vec4;
	using rgb = cgv::rgb;

	typedef std::pair<bool, float> parameter_pair;
	typedef std::vector<parameter_pair> parameter_list;

	struct configuration {
		static const std::string constant_float_parameter_name_prefix;
		static const std::string constant_color_parameter_name_prefix;
		static const std::string mapped_parameter_name_prefix;

		struct glyph_mapping_triple {
			int type; // 0 = constant, 1 = mapped
			size_t idx; // index into attribute values
			const vec4* v; // pointer to
		};

		struct layer_configuration {
			bool visible;
			AttributeSamplingStrategy sampling_strategy;
			float sampling_step;
			const glyph_shape* shape_ptr;
			std::vector<int> mapped_attributes;
			/// Information required to obtain the attribute values that determine the size of a
			/// given glyph.
			/// All other attributes are ignored, no parameters are stored for them.
			std::vector<glyph_mapping_triple> glyph_mapping_parameters;
			std::string glyph_definition = "";
			/// Scratch buffer used by `glyph_length` to hold, for a given glyph, all attribute
			/// values that impact its size.
			/// Stored as a member to avoid frequent allocation and deallocation.
			/// Entries correspond to `glyph_mapping_parameters`.
			std::unique_ptr<float[]> _size_attrib_values;

			[[nodiscard]] layer_configuration() = default;

			// Explicit copy constructor required since `size_attrib_values` cannot be implicitly
			// copied.
			[[nodiscard]] layer_configuration(const layer_configuration &src)
				: visible                  {src.visible}
				, sampling_strategy        {src.sampling_strategy}
				, sampling_step            {src.sampling_step}
				, shape_ptr                {src.shape_ptr}
				, mapped_attributes        {src.mapped_attributes}
				, glyph_mapping_parameters {src.glyph_mapping_parameters}
				, glyph_definition         {src.glyph_definition}
				// This is a scratch buffer, we don't care about its value outside `glyph_length`.
				, _size_attrib_values {
						std::make_unique<float[]>(glyph_mapping_parameters.size())}
			{}

			// Explicit copy assignment required since `size_attrib_values` cannot be implicitly
			// copied.
			layer_configuration &operator= (const layer_configuration &src)
			{
				visible                  = src.visible;
				sampling_strategy        = src.sampling_strategy;
				sampling_step            = src.sampling_step;
				shape_ptr                = src.shape_ptr;
				mapped_attributes        = src.mapped_attributes;
				glyph_mapping_parameters = src.glyph_mapping_parameters;
				glyph_definition         = src.glyph_definition;
				// This is a scratch buffer, we don't care about its value outside `glyph_length`.
				_size_attrib_values =
						std::make_unique<float[]>(glyph_mapping_parameters.size());
				return *this;
			}

			/// Calculate the extent of a glyph instance along the trajectory.
			/// `glyph_data` must point to an array of attributes as they are stored in the render
			/// buffer, including attributes that do not affect the size of the glyph, but excluding
			/// arc length and debug info.
			/// NOTE: To get the correct length, the same scaling factor as used by the shader must
			/// be applied, usually `textured_spline_tube_render_style::length_scale`.
			[[nodiscard]] float glyph_length (const float *glyph_data) const noexcept;
		};

		struct shader_configuration {
			std::string uniforms_definition = "";
			std::vector<std::string> glyph_layer_definitions;

			void clear() {
				uniforms_definition = "";
				glyph_layer_definitions.clear();
			}
		};

		//
		std::vector<std::pair<std::string, const float*>> constant_float_parameters;
		std::vector<std::pair<std::string, const rgb*>> constant_color_parameters;
		std::vector<std::pair<std::string, const vec4*>> mapping_parameters;
		std::vector<layer_configuration> layer_configs;
		std::string uniforms_definition = "";

		void clear()
		{
			layer_configs.clear();
			constant_float_parameters.clear();
			constant_color_parameters.clear();
			mapping_parameters.clear();

			uniforms_definition = "";
		}

		void create_uniforms_definition(bool verbose = false)
		{
			std::string uniforms_str = "";

			if(constant_float_parameters.size() > 0)
				uniforms_str += "uniform float " + constant_float_parameter_name_prefix + "[" + std::to_string(constant_float_parameters.size()) + "];";

			if(constant_color_parameters.size() > 0)
				uniforms_str += "uniform vec3 " + constant_color_parameter_name_prefix + "[" + std::to_string(constant_color_parameters.size()) + "];";

			if(mapping_parameters.size() > 0)
				uniforms_str += "uniform vec4 " + mapped_parameter_name_prefix + "[" + std::to_string(mapping_parameters.size()) + "];";

			uniforms_definition = uniforms_str;
		}
	};

protected:
	//
	cgv::base::base_ptr base_ptr;
	//
	ActionType last_action_type = AT_NONE;

	std::vector<glyph_attribute_mapping> glyph_attribute_mappings;

	std::shared_ptr<const visualization_variables_info> visualization_variables;

	configuration config;

	std::string new_attribute_mapping_name = "";

	void on_set(void* member_ptr);

	void create_glyph_attribute_mapping();

	void remove_glyph_attribute_mapping(const size_t index);

	void move_glyph_attribute_mapping(const size_t index, int offset);

public:
	glyph_layer_manager(cgv::base::base_ptr base_ptr) : base_ptr(base_ptr) {}

	[[nodiscard]] glyph_layer_manager(glyph_layer_manager &&) noexcept = default;
	glyph_layer_manager &operator= (glyph_layer_manager &&) noexcept = default;

	~glyph_layer_manager() {}

	void clear();

	const std::vector<glyph_attribute_mapping>& ref_glyph_attribute_mappings() const {
		return glyph_attribute_mappings;
	}

	void set_visualization_variables(std::shared_ptr<const visualization_variables_info> variables);

	const configuration& get_configuration();

	ActionType action_type();

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);

	void notify_configuration_change();

	void add_glyph_attribute_mapping(const glyph_attribute_mapping& attribute_mapping);
};
