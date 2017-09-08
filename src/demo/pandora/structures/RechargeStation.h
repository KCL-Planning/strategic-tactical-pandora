#ifndef PANDORA_STRUCTURES_RECHARGE_STATION_H
#define PANDORA_STRUCTURES_RECHARGE_STATION_H

#include <glm/glm.hpp>

#include "../level/MissionSite.h"

namespace DreadedPE
{
	class SceneManager;
	class SceneNode;
};
class Waypoint;

class RechargeStation : public MissionSite
{
public:
	RechargeStation(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation);
};

#endif
