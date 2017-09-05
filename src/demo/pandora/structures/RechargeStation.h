#ifndef PANDORA_STRUCTURES_RECHARGE_STATION_H
#define PANDORA_STRUCTURES_RECHARGE_STATION_H

#include <glm/glm.hpp>

#include "../level/MissionSite.h"

class SceneManager;
class SceneNode;
class Waypoint;

class RechargeStation : public MissionSite
{
public:
	RechargeStation(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation);
};

#endif
