#include "MissionSite.h"

#include <stdlib.h>

#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/texture/FreeImageLoader.h"
#include "dpengine/texture/TargaTexture.h"

#include "../Waypoint.h"
#include "../shaders/CausticShader.h"
#include "../structures/Structure.h"
#include "Mission.h"

int MissionSite::global_mission_site_id_ = 0;

MissionSite::MissionSite(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const glm::vec3& start_point, OntologyInterface& ontology)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, "Mission Site"), can_recharge_(false), ontology_(&ontology)
{
	std::stringstream ss;
	ss << "mission_site_" << global_mission_site_id_;
	id_ = ss.str();
	
	ss.str(std::string());
	ss << "mission_site_start_point_" << global_mission_site_id_;
	start_waypoint_ = new Waypoint(ss.str(), getLocalLocation() + start_point);
	++global_mission_site_id_;
}

void MissionSite::addStructure(Structure& structure)
{
	std::cout << "MissionSite::addStructure" << std::endl;
	structures_.push_back(&structure);
}

bool MissionSite::canRecharge() const
{
	for (std::vector<Structure*>::const_iterator ci = structures_.begin(); ci != structures_.end(); ++ci)
	{
		if ((*ci)->canRecharge())
		{
			return true;
		}
	}
	return false;
}

void MissionSite::removeStructure(const Structure& structure)
{
	for (std::vector<Structure*>::iterator ci = structures_.begin(); ci != structures_.end(); ++ci)
	{
		if (*ci == &structure)
		{
			std::cout << "Found the structure, getting rid of it!" << std::endl;
			structures_.erase(ci);
			break;
		}
	}
	
	for (std::vector<Mission*>::reverse_iterator ri = missions_.rbegin(); ri != missions_.rend(); ++ri)
	{
		Mission* mission = *ri;
		bool remove_mission = false;
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			if (&(*ci)->getStructure() == &structure)
			{
				remove_mission = true;
				break;
			}
		}
		
		if (remove_mission)
		{
			missions_.erase((ri + 1).base());
		}
	}
}
