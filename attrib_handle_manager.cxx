
// C++ STL
#include <vector>
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
struct attrib_handle_manager<flt_type>::Impl
{
	// basic types
	typedef typename attrib_handle_manager::real real;
	typedef typename attrib_handle_manager::vec2 vec2;
	typedef typename attrib_handle_manager::vec3 vec3;
	typedef typename attrib_handle_manager::vec4 vec4;
	typedef typename attrib_handle_manager::color color;

	// helper structs
	struct attrib_ref
	{
		unsigned sample_id;
		unsigned traj_id;
	};

	// the handled dataset
	const traj_dataset<real>* dataset = nullptr;

	// the spatial database, one grid per attribute
	std::unordered_map<unsigned, grid3D<real>> db;

	// the reference database, one array per attribute
	std::unordered_map<unsigned, std::vector<attrib_ref>> refs;
};

template <class flt_type>
attrib_handle_manager<flt_type>::attrib_handle_manager() : pimpl(nullptr)
{
	pimpl = new Impl;
}

template <class flt_type>
attrib_handle_manager<flt_type>::~attrib_handle_manager()
{
	if (pimpl)
	{
		clear();
		delete pimpl;
	}
	pimpl = nullptr;
}

template <class flt_type>
void attrib_handle_manager<flt_type>::clear (void)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// reset everything
	impl.dataset = nullptr;
	impl.db.clear();
	impl.refs.clear();
}

template <class flt_type>
void attrib_handle_manager<flt_type>::set_dataset (const traj_dataset<real> &dataset)
{
	// shortcut for saving one indirection
	auto &impl = *pimpl;

	// clean up first if managed dataset changes
	if (impl.dataset != &dataset)
		clear();

	// build state
	impl.dataset = &dataset;


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

	// per attribute, commit to grid and reference databases
	const real cellwidth = dataset.avg_segment_length();
	for (const auto &attrib_name : dataset.get_attribute_names())
	{
		// get attribute info
		const auto &attrib = dataset.attribute(attrib_name);
		const auto &trajs = dataset.trajectories(attrib);
		// prepare databse
		auto &attrib_grid = impl.db[attrib.id()];
		auto &attrib_refs = impl.refs[attrib.id()];
		attrib_grid.reset(cellwidth,
			[&attrib_refs] (vec3 *pnt, size_t id) -> bool {
				if (id < attrib_refs.size())
				{
					// ToDo: placeholder, needs compile_glyph_attribs like functionality
					*pnt = vec3(-1, 0, 1); 
					return true;
				}
				else
					return false;
			}
		);
		// preallocate memory
		attrib_refs.reserve(attrib.num());
		for (unsigned tid=0; tid<trajs.size(); tid++)
		{
			const auto &traj = trajs[tid];
			const unsigned sid_end = traj.i0 + traj.n;
			for (unsigned sid=traj.i0; sid<sid_end; sid++)
			{
				attrib_refs.emplace_back(Impl::attrib_ref{sid, tid});
				attrib_grid.insert(attrib_refs.size()-1);
			}
		}
	}
}


////
// Explicit template instantiations

// Only float and double variants are intended
template class attrib_handle_manager<float>;
template class attrib_handle_manager<double>;
