#pragma once

// make sure this is the first thing the compiler sees, while preventing warnings if
// it happened to already be defined by something else including this header
#ifndef _USE_MATH_DEFINES
	#define _USE_MATH_DEFINES 1
#endif
#include <memory>

#include <cgv/base/base.h>
#include <cgv/data/ref_ptr.h>
#include <cgv/gui/control.h>
#include <cgv/gui/provider.h>
#include <cgv/math/interval.h>
#include <cgv/media/color.h>
#include <cgv/type/bool32_t.h>

#include "gui_util.h"
#include "glyph_shapes.h"
#include "visualization_variables_info.h"



enum AttributeSamplingStrategy {
	ASS_UNIFORM,
	ASS_EQUIDIST,
	ASS_AT_SAMPLES,
};

struct scalar_mapping {
	// input values should be clampled and normalized to this range
	cgv::vec2 input_range; 
	// normalized values should be mapped to this range.
	cgv::vec2 output_range;
};

class glyph_attribute_mapping {
protected:
	ActionType last_action_type = AT_NONE;

	std::string name = "";
	bool active = true;

	std::shared_ptr<const visualization_variables_info> visualization_variables;

	AttributeSamplingStrategy sampling_strategy = ASS_AT_SAMPLES;
	float sampling_step = 1.0f;
	GlyphType type = GT_CIRCLE;
	glyph_shape* shape_ptr = nullptr;

	std::vector<int> attrib_source_indices;
	std::vector<int> color_source_indices;
	std::vector<scalar_mapping> attrib_mapping_values;
	// Use bool class type to prevent specialization of std::vector.
	std::vector<cgv::type::bool32_t> reverse_colors;
	std::vector<cgv::rgb> attrib_colors;
	
	void on_set(void* member_ptr, cgv::base::base* base_ptr);

	void update_name(cgv::base::base* base_ptr);

	void create_glyph_shape();

	template <typename T>
	cgv::data::ref_ptr<cgv::gui::control<T>> add_local_member_control(cgv::gui::provider& p, cgv::base::base* base_ptr, const std::string& label, T& value, const std::string& gui_type = "", const std::string& options = "", const std::string& align = "\n") {
		cgv::data::ref_ptr<cgv::gui::control<T>> cp = p.add_control(label, value, gui_type, options, align);
		if(cp)
			connect_copy(cp->value_change, cgv::signal::rebind(this, &glyph_attribute_mapping::on_set, &value, cgv::signal::_c<cgv::base::base*>(base_ptr)));
		return cp;
	}

	std::string to_display_str(const std::string& name) const;

	void create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i, const glyph_attribute& attrib, const bool global_block);

public:
	// the index used to indicate unmapped attributes
	static const int k_unmapped_index = -1;

	glyph_attribute_mapping();

	glyph_attribute_mapping(const glyph_attribute_mapping& other);

	glyph_attribute_mapping(glyph_attribute_mapping&& other) noexcept;

	glyph_attribute_mapping& operator=(glyph_attribute_mapping other);

	~glyph_attribute_mapping();

	friend void swap(glyph_attribute_mapping& first, glyph_attribute_mapping& second) {
		using std::swap;

		swap(first.name, second.name);
		swap(first.active, second.active);
		swap(first.sampling_strategy, second.sampling_strategy);
		swap(first.sampling_step, second.sampling_step);
		swap(first.type, second.type);
		swap(first.attrib_source_indices, second.attrib_source_indices);
		swap(first.color_source_indices, second.color_source_indices);
		swap(first.attrib_mapping_values, second.attrib_mapping_values);
		swap(first.reverse_colors, second.reverse_colors);
		swap(first.attrib_colors, second.attrib_colors);
		swap(first.visualization_variables, second.visualization_variables);

		swap(first.shape_ptr, second.shape_ptr);
	}

	ActionType action_type();

	const std::string& get_name() const { return name; }

	void set_name(const std::string& name) { this->name = name; }

	bool& ref_active() { return active; }

	bool get_active() const { return active; }

	void set_active(bool flag) { active = flag; }

	const AttributeSamplingStrategy get_sampling_strategy() const { return sampling_strategy; }
	
	const float get_sampling_step() const { return sampling_step; }

	void set_sampling_strategy(AttributeSamplingStrategy strategy) { sampling_strategy = strategy; }

	void set_sampling_step(float step) { sampling_step = step; }

	const glyph_shape* get_shape_ptr() const { return shape_ptr; }

	void set_glyph_type(GlyphType type);

	const std::vector<int> get_attrib_indices() const { return attrib_source_indices; }

	const std::vector<int> get_color_map_indices() const { return color_source_indices; }

	const std::vector<scalar_mapping> &ref_attrib_mapping_values() const { return attrib_mapping_values; }

	const std::vector<cgv::rgb>& ref_attrib_colors() const { return attrib_colors; }

	const std::shared_ptr<const visualization_variables_info> get_visualization_variables() const { return visualization_variables; }

	void set_visualization_variables(std::shared_ptr<const visualization_variables_info> variables);

	void set_attrib_source_index(size_t attrib_idx, int source_idx);

	void set_color_source_index(size_t color_idx, int source_idx);

	void set_attrib_in_range(size_t idx, const cgv::vec2& range);

	void set_attrib_out_range(size_t idx, const cgv::vec2& range);

	void set_attrib_color(size_t idx, const cgv::rgb& color);

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);
};
