#pragma once

// CGV framework core
#include <cgv/render/render_types.h>
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>

// CGV framework application utility
#include <cgv_app/color_map_reader.h>
#include <cgv_app/color_map_writer.h>

// CGV framework 3rd party libraries
#include <3rd/xml/tinyxml2/tinyxml2.h>
#include <3rd/xml/cgv_xml/print.h>
#include <3rd/xml/cgv_xml/query.h>

// local includes
#include "glyph_layer_manager.h"
#include "color_map_manager.h"

class layer_configuration_io {
private:
	static int index_of(const std::vector<std::string>& v, const std::string& elem) {

		for(size_t i = 0; i < v.size(); ++i) {
			if(v[i] == elem)
				return static_cast<int>(i);
		}
		return -1;
	}

	static void write_color_maps(tinyxml2::XMLPrinter& printer, const color_map_manager& color_map_mgr) {

		printer.OpenElement("ColorMaps");

		const auto& color_maps = color_map_mgr.ref_color_maps();
		for(const auto& cmc : color_maps) {
			if(cmc.custom)
				cgv::app::color_map_writer::to_xml_printer(printer, cmc.name, cmc.cm, false);
		}

		printer.CloseElement();
	}

	static void write_layer(
		tinyxml2::XMLPrinter& printer,
		std::shared_ptr<const visualization_variables_info> visualization_variables,
		const glyph_attribute_mapping& gam) {

		const auto* shape_ptr = gam.get_shape_ptr();

		if(shape_ptr) {
			printer.OpenElement("Layer");

			cgv::xml::PushAttribute(printer, "name", gam.get_name());
			cgv::xml::PushAttribute(printer, "active", gam.get_active());
			cgv::xml::PushAttribute(printer, "glyph", shape_ptr->name());

			std::string sampling_type = "";

			switch(gam.get_sampling_strategy()) {
			case ASS_AT_SAMPLES: sampling_type = "original"; break;
			case ASS_UNIFORM: sampling_type = "uniform"; break;
			case ASS_EQUIDIST: sampling_type = "equidist"; break;
			default: break;
			}

			if(sampling_type != "") {
				cgv::xml::PushAttribute(printer, "sampling", sampling_type);
				if(sampling_type != "original")
					cgv::xml::PushAttribute(printer, "sampling_step", std::to_string(gam.get_sampling_step()));
			}

			write_layer_properties(printer, visualization_variables, gam, shape_ptr->supported_attributes());

			printer.CloseElement();
		}
	}

	static void write_layer_properties(
		tinyxml2::XMLPrinter& printer,
		std::shared_ptr<const visualization_variables_info> visualization_variables,
		const glyph_attribute_mapping& gam,
		const glyph_shape::attribute_list& shape_attributes) {

		const auto attribute_indices = gam.get_attrib_indices();
		const auto color_indices = gam.get_color_map_indices();

		const auto &mapping_ranges = gam.ref_attrib_mapping_values();
		const auto& colors = gam.ref_attrib_colors();

		const auto& attribute_names = visualization_variables->ref_attribute_names();
		const auto& color_map_names = visualization_variables->ref_color_map_names();

		auto vec2_to_str = [](const cgv::render::vec2& v) {
			return std::to_string(v.x()) + ", " + std::to_string(v.y());
		};

		auto rgb_to_str = [](const cgv::render::rgb& v) {
			return std::to_string(v.R()) + ", " + std::to_string(v.G()) + ", " + std::to_string(v.B());
		};

		for(size_t i = 0; i < shape_attributes.size(); ++i) {
			const auto& attribute = shape_attributes[i];

			printer.OpenElement("Property");
			cgv::xml::PushAttribute(printer, "name", attribute.name);

			if(!(attribute.modifiers & GAM_GLOBAL)) {
				int attribute_index = attribute_indices[i];
				if(attribute_index > -1)
					cgv::xml::PushAttribute(printer, "attrib_name", attribute_names[attribute_index]);
			}

			if(attribute.type == GAT_COLOR) {
				int color_index = color_indices[i];
				if(color_index > -1)
					cgv::xml::PushAttribute(printer, "color_map_name", color_map_names[color_index]);
				else
					cgv::xml::PushAttribute(printer, "color", colors[i]);
			}

			cgv::render::vec4 mr = mapping_ranges[i];
			if(attribute.modifiers & GAM_GLOBAL) {
				if(attribute.type != GAT_COLOR)
					cgv::xml::PushAttribute(printer, "value", std::to_string(mr.w()));
			} else {
				cgv::xml::PushAttribute(printer, "in_range", cgv::render::vec2(mr.x(), mr.y()));
			}
			if(attribute.type != GAT_UNIT &&
			   attribute.type != GAT_SIGNED_UNIT &&
			   //attribute.type != GAT_COLOR &&
			   attribute.type != GAT_OUTLINE
			   ) {
				cgv::xml::PushAttribute(printer, "out_range", cgv::render::vec2(mr.z(), mr.w()));
			}

			printer.CloseElement();
		}
	}

	static void extract_color_maps(const tinyxml2::XMLElement& elem, color_map_manager& color_map_mgr) {

		cgv::app::color_map_reader::result color_maps;
		cgv::app::color_map_reader::read_from_xml(elem, color_maps);

		// clear previous custom color maps
		std::vector<std::string> current_names;
		const auto& current_color_maps = color_map_mgr.ref_color_maps();
		for(size_t i = 0; i < current_color_maps.size(); ++i) {
			if(current_color_maps[i].custom)
				current_names.push_back(current_color_maps[i].name);
		}

		for(size_t i = 0; i < current_names.size(); ++i)
			color_map_mgr.remove_color_map_by_name(current_names[i]);

		// add new custom color maps
		for(const auto& entry : color_maps)
			color_map_mgr.add_color_map(entry.first, entry.second, true);
	}

	static void extract_layer(const tinyxml2::XMLElement& elem,
							  glyph_layer_manager& glyph_layer_mgr,
							  const std::vector<std::string>& attribute_names,
							  const std::vector<std::string>& color_map_names) {

		glyph_attribute_mapping gam;
		const glyph_shape* shape_ptr = nullptr;
		std::vector<std::string> shape_attribute_names;
		std::vector<std::pair<int, cgv::render::vec2>> input_ranges;

		extract_layer_attributes(elem, gam, shape_ptr, shape_attribute_names);

		if(shape_ptr) {
			auto property_elem = elem.FirstChildElement();

			while(property_elem) {
				extract_layer_property(*property_elem, attribute_names, color_map_names, gam, shape_ptr, shape_attribute_names, input_ranges);
				property_elem = property_elem->NextSiblingElement();
			}

			glyph_layer_mgr.add_glyph_attribute_mapping(gam);

			auto& last_gam = const_cast<glyph_attribute_mapping&>(glyph_layer_mgr.ref_glyph_attribute_mappings().back());

			for(const auto& p : input_ranges)
				if(p.first > -1)
					last_gam.set_attrib_in_range(p.first, p.second);
		}
	}

	static void extract_layer_attributes(const tinyxml2::XMLElement& elem,
										 glyph_attribute_mapping& gam,
										 const glyph_shape*& shape_ptr,
										 std::vector<std::string>& shape_attribute_names) {

		std::string layer_name = "";
		if(cgv::xml::QueryStringAttribute(elem, "name", layer_name) == tinyxml2::XML_SUCCESS)
			gam.set_name(layer_name);

		bool layer_active = true;
		if(cgv::xml::QueryBoolAttribute(elem, "active", layer_active) == tinyxml2::XML_SUCCESS)
			gam.set_active(layer_active);

		std::string glyph_name = "";
		if(cgv::xml::QueryStringAttribute(elem, "glyph", glyph_name) == tinyxml2::XML_SUCCESS) {
			GlyphType glyph_type = glyph_type_registry::type(glyph_name);
			gam.set_glyph_type(glyph_type);
			shape_ptr = gam.get_shape_ptr();

			for(const auto& a : shape_ptr->supported_attributes())
				shape_attribute_names.push_back(a.name);
		}

		std::string sampling = "";
		if(cgv::xml::QueryStringAttribute(elem, "sampling", sampling) == tinyxml2::XML_SUCCESS) {
			if(sampling == "original") {
				gam.set_sampling_strategy(ASS_AT_SAMPLES);
			} else if(sampling == "uniform") {
				gam.set_sampling_strategy(ASS_UNIFORM);
			} else if(sampling == "equidist") {
				gam.set_sampling_strategy(ASS_EQUIDIST);
			}
		}

		float sampling_step = 1.0f;
		if(elem.QueryFloatAttribute("sampling_step", &sampling_step) == tinyxml2::XML_SUCCESS)
			gam.set_sampling_step(sampling_step);
	}

	static void extract_layer_property(const tinyxml2::XMLElement& elem,
									   const std::vector<std::string>& attrib_names,
									   const std::vector<std::string>& color_map_names,
									   glyph_attribute_mapping& gam,
									   const glyph_shape* shape_ptr,
									   std::vector<std::string>& shape_attribute_names,
									   std::vector<std::pair<int, cgv::render::vec2>>& input_ranges) {

		std::string name = "";
		if(cgv::xml::QueryStringAttribute(elem, "name", name) != tinyxml2::XML_SUCCESS)
			// property has no name, skip
			return;

		int shape_attrib_idx = index_of(shape_attribute_names, name);

		if(shape_attrib_idx < 0)
			// shape does not have this attribute
			return;

		const auto& attrib = shape_ptr->supported_attributes()[shape_attrib_idx];

		std::string attrib_name = "";
		int attrib_idx = -1;
		if(cgv::xml::QueryStringAttribute(elem, "attrib_name", attrib_name) == tinyxml2::XML_SUCCESS) {
			// the name for a mapped attribute is given
			// if not -1, then this attribute is also present in the loaded data set
			attrib_idx = index_of(attrib_names, attrib_name);
		}

		if(attrib_idx < 0) {
			// the shape attribute is not mapped
		} else {
			// a data set attribute is mapped to the shape attribute
			gam.set_attrib_source_index(static_cast<size_t>(shape_attrib_idx), attrib_idx);
		}

		if(attrib.modifiers & GAM_GLOBAL) {
			float value = 1.0f;
			if(elem.QueryFloatAttribute("value", &value) == tinyxml2::XML_SUCCESS)
				gam.set_attrib_out_range(shape_attrib_idx, cgv::render::vec2(0.0f, value));
		}

		cgv::render::vec2 in_range(0.0f, 1.0f);
		if(cgv::xml::QueryVecAttribute(elem, "in_range", in_range) == tinyxml2::XML_SUCCESS) {
			gam.set_attrib_in_range(shape_attrib_idx, in_range);
			input_ranges.push_back({ shape_attrib_idx, in_range });
		} else {
			input_ranges.push_back({ -1, in_range });
		}

		if(attrib.type != GAT_UNIT &&
		   attrib.type != GAT_SIGNED_UNIT &&
		   //attrib.type != GAT_COLOR &&
		   attrib.type != GAT_OUTLINE
		   ) {
			cgv::render::vec2 out_range(0.0f, 1.0f);
			if(cgv::xml::QueryVecAttribute(elem, "out_range", out_range) == tinyxml2::XML_SUCCESS)
				gam.set_attrib_out_range(shape_attrib_idx, out_range);
		}

		if(shape_ptr->supported_attributes()[shape_attrib_idx].type == GAT_COLOR) {
			int color_map_idx = -1;
			std::string color_map_name = "";
			if(cgv::xml::QueryStringAttribute(elem, "color_map_name", color_map_name) == tinyxml2::XML_SUCCESS) {
				// the name for a color map is given
				// if not -1, then this attribute is also present in the loaded data set
				color_map_idx = index_of(color_map_names, color_map_name);
			}

			if(color_map_idx < 0) {
				// no color map selected
				cgv::render::rgb color(0.0f);
				if(cgv::xml::QueryRGBAttribute(elem, "color", color) == tinyxml2::XML_SUCCESS)
					gam.set_attrib_color(shape_attrib_idx, color);
			} else {
				// a color map is selected
				gam.set_color_source_index(shape_attrib_idx, color_map_idx);
			}
		}
	};

public:
	layer_configuration_io() = delete;
	~layer_configuration_io() = delete;

	static bool write_layer_configuration(
		const std::string& file_name,
		std::shared_ptr<const visualization_variables_info> visualization_variables,
		const glyph_layer_manager& glyph_layer_mgr,
		const color_map_manager& color_map_mgr) {

		tinyxml2::XMLPrinter printer;
		printer.OpenElement("GlyphConfiguration");

		write_color_maps(printer, color_map_mgr);

		printer.OpenElement("Layers");

		const auto& glyph_attribute_mappings = glyph_layer_mgr.ref_glyph_attribute_mappings();

		for(const auto& gam : glyph_attribute_mappings)
			write_layer(printer, visualization_variables, gam);

		printer.CloseElement();

		printer.CloseElement();

		std::string xml = printer.CStr();
		return cgv::utils::file::write(file_name, xml, true);
	}

	static bool read_layer_configuration(
		const std::string& file_name,
		std::shared_ptr<visualization_variables_info> visualization_variables,
		glyph_layer_manager& glyph_layer_mgr,
		color_map_manager& color_map_mgr) {

		glyph_layer_mgr.clear();

		if(!cgv::utils::file::exists(file_name) || cgv::utils::to_upper(cgv::utils::file::get_extension(file_name)) != "XML")
			return false;

		tinyxml2::XMLDocument doc;
		if(doc.LoadFile(file_name.c_str()) != tinyxml2::XML_SUCCESS)
			return false;

		cgv::xml::FindElementByNameVisitor findElementByName("ColorMaps");
		doc.Accept(&findElementByName);

		if(auto color_maps_elem = findElementByName.Result())
			extract_color_maps(*color_maps_elem, color_map_mgr);

		// get a list of color map names
		std::vector<std::string> color_map_names = color_map_mgr.get_names();
		visualization_variables->set_color_map_names(color_map_names);
		
		findElementByName.SetQueryName("Layers");
		doc.Accept(&findElementByName);

		if(auto layers_elem = findElementByName.Result()) {
			auto layer_elem = layers_elem->FirstChildElement();

			while(layer_elem) {
				extract_layer(*layer_elem, glyph_layer_mgr, visualization_variables->ref_attribute_names(), color_map_names);
				layer_elem = layer_elem->NextSiblingElement();
			}
		}

		return true;
	}

};
