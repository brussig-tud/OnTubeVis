#pragma once

#include <cgv/utils/scan.h>


class visualization_variables_info {
private:
	std::vector<std::string> attribute_names;
	std::vector<cgv::vec2> attribute_ranges;
	std::vector<std::string> color_map_names;

	std::string attribute_names_list = "";
	std::string color_map_names_list = "";

public:

	void set_attribute_names(const std::vector<std::string>& names) {

		attribute_names = names;
		attribute_names_list = cgv::utils::join(attribute_names, ",");
	}

	void set_attribute_ranges(const std::vector<cgv::vec2>& ranges) {

		attribute_ranges = ranges;
	}

	void set_color_map_names(const std::vector<std::string>& names) {

		color_map_names = names;
		color_map_names_list = cgv::utils::join(color_map_names, ",");
	}

	const std::vector<std::string>& ref_attribute_names() const {

		return attribute_names;
	}

	const std::vector<cgv::vec2>& ref_attribute_ranges() const {

		return attribute_ranges;
	}

	const std::vector<std::string>& ref_color_map_names() const {

		return color_map_names;
	}

	const std::string& get_attribute_names_list() const {

		return attribute_names_list;
	}

	const std::string& get_color_map_names_list() const {

		return color_map_names_list;
	}
};
