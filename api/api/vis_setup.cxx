
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
	std::clog << "otv__add_layer: added layer of type '"<<otv__string_from_GlyphType(new_layer.type)<<"'" << std::endl
	          << " - setup for '"+vs.name+"' now has "<<vs.layers.size()<<" layer(s)." << std::endl;
	return true;
}
