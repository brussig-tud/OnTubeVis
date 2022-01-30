
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
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class attrib_handle_manager<float>;
template class attrib_handle_manager<double>;
