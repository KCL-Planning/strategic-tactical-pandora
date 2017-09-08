#ifndef DEMO_PANDORA_STRUCTURES_SMALL_MANIFOLD_H
#define DEMO_PANDORA_STRUCTURES_SMALL_MANIFOLD_H

#include <glm/glm.hpp>
#include <vector>

#include "dpengine/entities/Entity.h"
#include "Structure.h"

class InspectionPoint;
namespace DreadedPE
{
	class SceneManager;
	class SceneNode;
};
class MissionSite;

class SmallManifold : public DreadedPE::Entity, public Structure
{
public:
	SmallManifold(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation);
	
private:
	
};

#endif
