#pragma once

// C++ STL
#include <cstdint>

// forward declarations
namespace cgv::gui {
	class provider;
}


namespace otv {

/// Determines how `traj_grid` influences trajectory shading.
struct traj_grid_shading {
	enum class color_fn : uint8_t {
		none,               // Don't use the trajectory grid.
		relation,           // Calculate a relation between trajectories.
		dbg_seg_t,          // Color by segment-local curve parameter.
		dbg_index_xyz,      // Color by spatial grid index.
		dbg_index_t,        // Color by temporal grid index.
		dbg_signature,      // Color by index signature hash.
		dbg_bucket_load,    // Color by hash bucket load factor.
		dbg_local_interval, // Color by curve parameter relative to local trajectory interval.
	}
	/// Function used to choose trajectories' base color using the trajectory grid.
	color_fn {};

	/// Generate GUI elements to change shading parameters.
	void build_gui (cgv::gui::provider&);
};

} // namespace otv
