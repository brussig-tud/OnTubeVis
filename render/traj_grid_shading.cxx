// CGV framework
#include <cgv/gui/provider.h>

// implemented header
#include "render/traj_grid_shading.h"


namespace otv {

void traj_grid_shading::build_gui (cgv::gui::provider& p)
{
	auto const b = dynamic_cast<cgv::base::base*>(&p);
	p.add_member_control(b, "Traj. Grid", color_fn, "dropdown", "enums='"
		"None,"
		"Trajectory Relation,"
		"[debug] Curve Parameter,"
		"[debug] Spatial Index,"
		"[debug] Temporal Index,"
		"[debug] Index Hash,"
		"[debug] Bucket Load,"
		"[debug] Number of Interval'"
	);
}

} // namespace otv
