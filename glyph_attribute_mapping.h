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

#include "gui_util.h"
#include "glyph_shapes.h"
#include "visualization_variables_info.h"



enum AttributeSamplingStrategy {
	ASS_UNIFORM,
	ASS_EQUIDIST,
	ASS_AT_SAMPLES,
};

class glyph_attribute_mapping : public cgv::render::render_types {
protected:
	ActionType last_action_type = AT_NONE;

	std::string name = "";
	bool active = true;

	std::shared_ptr<const visualization_variables_info> visualization_variables;

	AttributeSamplingStrategy sampling_strategy = ASS_AT_SAMPLES;
	float sampling_step = 1.0f;
	GlyphType type = GT_CIRCLE;
	glyph_shape* shape_ptr = nullptr;

	std::vector<cgv::type::DummyEnum> attrib_source_indices;
	std::vector<cgv::type::DummyEnum> color_source_indices;
	std::vector<vec4> attrib_mapping_values;
	std::vector<rgb> attrib_colors;
	
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

	int dummy_enum_to_int(cgv::type::DummyEnum index) const;

	cgv::type::DummyEnum int_to_dummy_enum(int index) const;

	std::string to_display_str(const std::string& name) const;

	void create_attribute_gui(cgv::base::base* bp, cgv::gui::provider& p, const size_t i, const glyph_attribute& attrib, const bool global_block);

public:
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

	const std::vector<int> get_attrib_indices() const;

	const std::vector<int> get_color_map_indices() const;

	const std::vector<vec4>& ref_attrib_mapping_values() const;

	const std::vector<vec4> &
	ref_attrib_values() const
	{
		return attrib_mapping_values;
	}

	const std::vector<rgb>& ref_attrib_colors() const { return attrib_colors; }

	const std::shared_ptr<const visualization_variables_info> get_visualization_variables() const { return visualization_variables; }

	void set_visualization_variables(std::shared_ptr<const visualization_variables_info> variables) {
		
		visualization_variables = variables;
	
		for(size_t i = 0; i < attrib_mapping_values.size(); ++i) {
			int attrib_idx = dummy_enum_to_int(attrib_source_indices[i]);
			if(attrib_idx > -1) {
				const vec2& range = visualization_variables->ref_attribute_ranges()[attrib_idx];
				attrib_mapping_values[i].x() = range.x();
				attrib_mapping_values[i].y() = range.y();
			}
		}
	}

	//const std::vector<std::string>& ref_attribute_names() const { return visualization_variables->ref_attribute_names(); }
	//
	//const std::vector<std::string>& ref_color_map_names() const { return visualization_variables->ref_color_map_names(); }

	void create_gui(cgv::base::base* bp, cgv::gui::provider& p);

	void set_attrib_source_index(size_t attrib_idx, int source_idx) {
		if(attrib_idx < attrib_source_indices.size())
			attrib_source_indices[attrib_idx] = int_to_dummy_enum(source_idx);
	}

	void set_color_source_index(size_t color_idx, int source_idx) {
		if(color_idx < color_source_indices.size())
			color_source_indices[color_idx] = int_to_dummy_enum(source_idx);
	}

	void set_attrib_in_range(size_t idx, const vec2& range) {
		if(idx < attrib_mapping_values.size()) {
			attrib_mapping_values[idx].x() = range.x();
			attrib_mapping_values[idx].y() = range.y();
		}
	}

	void set_attrib_out_range(size_t idx, const vec2& range) {
		if(idx < attrib_mapping_values.size()) {
			attrib_mapping_values[idx].z() = range.x();
			attrib_mapping_values[idx].w() = range.y();
		}
	}

	void set_attrib_color(size_t idx, const rgb& color) {
		if(idx < attrib_colors.size())
			attrib_colors[idx] = color;
	}
};
