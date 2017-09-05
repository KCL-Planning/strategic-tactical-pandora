#ifndef DEMO_PANDORA_STRUCTURES_SMALL_MANIFOLD_H
#define DEMO_PANDORA_STRUCTURES_SMALL_MANIFOLD_H

#include <glm/glm.hpp>
#include <vector>

#include "../../../core/entities/Entity.h"
#include "Structure.h"

class InspectionPoint;
class SceneManager;
class SceneNode;
class MissionSite;

class SmallManifold : public Entity, public Structure
{
public:
	SmallManifold(SceneManager& scene_manager, SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation);
	
private:
	
};

#endif
