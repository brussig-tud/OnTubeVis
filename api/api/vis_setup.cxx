
//////
//
// Includes
//

// C++ STL
#include <iostream>

// Public interface
#include <OnTubeVis/OnTubeVis.h>

// Private interface
#include <util/cppstream.h>
#include <state/core.h>



//////
//
// Functions
//

OTV_API OTV_VisSetupHandle otv__create_VisSetup (const char *const name) {
	auto new_vs = new VisSetup(name);
	std::clog << "otv__create_VisSetup: created setup "<<hex(new_vs)<<std::dec<<" named '"<<new_vs->name<<"'."
	          << std::endl;
	return (OTV_VisSetupHandle)new_vs;
}

OTV_API void otv__free_VisSetup (OTV_VisSetupHandle vis_setup) {
	std::clog << "otv__free_VisSetup: freeing setup "<<hex(vis_setup)<<std::dec<<"." << std::endl;
	delete (VisSetup*)vis_setup;
}

OTV_API uint32_t otv__add_trajectory (OTV_VisSetupHandle vis_setup, const float radius)
{
	// Retrieve actual vis setup object
	auto &vs = *(VisSetup*)vis_setup;

	// Add it
	const auto &traj = vs.trajs.emplace_back(
		trajectory_setup{vs.counter++, radius}
	);
	std::clog << "otv__add_trajectory: added trajectory #"<<traj.id << std::endl
	          << " - setup for '"+vs.name+"' now has "<<vs.trajs.size()<<" trajectorie(s)." << std::endl;

	return traj.id;
}

OTV_API bool otv__add_layer (OTV_VisSetupHandle vis_setup, const OTV_LayerConfig *config)
{
	// Retrieve actual vis setup object
	auto &vs = *(VisSetup*)vis_setup;

	// Validity checks
	if (vs.layers.size()>3) {
		std::cerr << "otv__add_layer: unable to add layer!" << std::endl
		          << " - setup for '"+vs.name+"' already has 4 layers." << std::endl;
		return false;
	}

	// Add the new layer
	auto &new_layer = vs.layers.emplace_back(*config);
	vs.layer_sources.emplace_back();
	std::clog << "otv__add_layer: added layer of type '"<<otv__string_from_GlyphType(new_layer.type)<<"'" << std::endl
	          << " - setup for '"+vs.name+"' now has "<<vs.layers.size()<<" layer(s)." << std::endl;
	return true;
}

OTV_API void otv__layer_color_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const OTV_Interval in_range, const char *const desc
){
	((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Color] = {
		{in_range.min, in_range.max}, {}, desc
	};
}

OTV_API void otv__layer_height_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const OTV_Interval in_range, const OTV_Interval out_range,
	const char *const desc
){
	((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Height] = {
		{in_range.min, in_range.max}, {out_range.min, out_range.max}, desc
	};
}

OTV_API void otv__layer_orientation_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const OTV_Interval in_range, const OTV_Interval out_range,
	const char *const desc
){
	((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Orientation] = {
		{in_range.min, in_range.max}, {out_range.min, out_range.max}, desc
	};
}

OTV_API void otv__layer_radius_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const OTV_Interval in_range, const OTV_Interval out_range,
	const char *const desc
){
	((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Radius] = {
		{in_range.min, in_range.max}, {out_range.min, out_range.max}, desc
	};
}

OTV_API void otv__layer_value_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const unsigned value_id, const OTV_Interval in_range,
	const char *const desc
){
	switch (value_id)
	{
		case 0:
			((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Value0] = {
				{in_range.min, in_range.max}, {}, desc
			};
			return;
		case 1:
			((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Value1] = {
				{in_range.min, in_range.max}, {}, desc
			};
			return;
		case 2:
			((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Value2] = {
				{in_range.min, in_range.max}, {}, desc
			};
			return;
		case 3:
			((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Value3] = {
				{in_range.min, in_range.max}, {}, desc
			};
			return;

		default:
			/* do_nothing() */; // will crash below
	}
	throw std::runtime_error("FATAL ERROR: value ids must be between [0..3], got "+std::to_string(value_id)+"!");
}

OTV_API void otv__layer_width_source (
	OTV_VisSetupHandle vis_setup, const unsigned layer, const OTV_Interval in_range, const OTV_Interval out_range,
	const char *const desc
){
	((VisSetup*)vis_setup)->layer_sources[layer][VAttrib::Width] = {
		{in_range.min, in_range.max}, {out_range.min, out_range.max}, desc
	};
}

OTV_API void otv__geo_reference (OTV_VisSetupHandle vis_setup, const double latitude, const double longitude) {
	((VisSetup*)vis_setup)->georef.emplace(latitude, longitude);
}

OTV_API void otv__extrapolation_length (OTV_VisSetupHandle vis_setup, const uint32_t num_segments) {
	((VisSetup*)vis_setup)->num_extrapol_segments = num_segments;
}

OTV_API void otv__extrapol_progression (OTV_VisSetupHandle vis_setup, const OTV_ExtrapolProgression progression)
{
	switch (progression)
	{
		case OTV_ExtrapolProgression::Instant:
			((VisSetup*)vis_setup)->use_natural_progression = false;
			return;

		case OTV_ExtrapolProgression::Natural:
			((VisSetup*)vis_setup)->use_natural_progression = true;
			return;
	}

	// This should never happen
	std::cerr << std::endl << "FATAL ERROR: unknown extrapolation progression mode!" << std::endl << std::endl;
	*(size_t*)size_t(-1) = 0; // <-- crash and burn
}
