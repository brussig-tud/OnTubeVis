#pragma once

// local includes
#include <userstudies/view_interaction.h>

namespace userstudies {

struct trial {
	virtual ~trial() = default;

	virtual bool on_view_interaction (const view_interaction&) { return false; }
};

}
