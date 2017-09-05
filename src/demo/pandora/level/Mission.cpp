#include "Mission.h"

#include <sstream>
#include "../ontology/Goal.h"

int Mission::global_mission_id_ = 0;

Mission::Mission(MissionSite& mission_site)
	: mission_site_(&mission_site)
{
	std::stringstream ss;
	ss << "Mission" << global_mission_id_;
	id_ = ss.str();
	global_mission_id_++;
}

void Mission::addGoal(Goal& goal)
{
	goals_.push_back(&goal);
}

void Mission::getStrageticPoints(std::vector<glm::vec3>& points) const
{
	for (std::vector<Goal*>::const_iterator ci = goals_.begin(); ci != goals_.end(); ++ci)
	{
		(*ci)->getStrategicPoints(points);
	}
}
