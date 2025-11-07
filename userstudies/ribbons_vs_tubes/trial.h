#pragma once

// local includes
#include <userstudies/trial.h>
#include <userstudies/view_interaction.h>


namespace userstudies {
namespace RvT {

struct trial : public userstudies::trial
{
	view_interaction_accumulator view_interactions;

	bool on_view_interaction (const view_interaction &interaction) override
	{
		switch (interaction.kind)
		{
			case view_interaction::Kind::Orbit:
				std::clog << "orbited "<<view_interactions.log_interaction(interaction)<<"°" << std::endl;
				return true;
			case view_interaction::Kind::Pan:
				std::clog << "panned "<<view_interactions.log_interaction(interaction)<<"%" << std::endl;
				return true;
			case view_interaction::Kind::Roll:
				std::clog << "rolled "<<view_interactions.log_interaction(interaction)<<"°" << std::endl;
				return true;
			case view_interaction::Kind::Zoom:
				std::clog << "zoomed "<<view_interactions.log_interaction(interaction)<<'%' << std::endl;
				return true;
			case view_interaction::Kind::FocusChange:
				std::clog << "focus changed "<<view_interactions.log_interaction(interaction)<<" units" << std::endl;
				return true;
			case view_interaction::Kind::FocusChangeFromZoom:
				std::clog << "focus changed "<<interaction.amount<<" units (via zoom action)" << std::endl;
				return false;
		}
	}
};

}}
