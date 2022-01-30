
// C++ STL
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <utility>
#include <limits>

// local includes
#include "regulargrid.h"

// implemented header
#include "attrib_handle_manager.h"


////
// Local types and variables

// anonymous namespace begin
namespace {

// some stuff
/* int nothing = 0*/;

// Anonymous namespace end
}


////
// Class implementation

template <class flt_type>
void attrib_handle_manager<flt_type>::clear (void)
{
	this->dataset = nullptr;
}

template <class flt_type>
void attrib_handle_manager<flt_type>::set_dataset (const traj_dataset<real> &dataset)
{
	// clean up first if managed dataset changes
	if (this->dataset != &dataset)
		clear();

	// build state
	this->dataset = &dataset;

	////
	// various exploratory test code...

	// determine start and end time according to the position attribute (which defines what we can see anyway)
	const auto &pos = dataset.positions();
	real tmin=std::numeric_limits<real>::infinity(), tmax=-tmin;
	for (const auto &traj : dataset.trajectories(pos.attrib))
	{
		tmin = std::min(tmin, pos.data.timestamps[traj.i0]);
		tmax = std::max(tmax, pos.data.timestamps[traj.i0+traj.n-1]);
	}
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class attrib_handle_manager<float>;
template class attrib_handle_manager<double>;
