#ifndef DEMO_PANDORA_STRUCTURES_CHAIN_LINK_H
#define DEMO_PANDORA_STRUCTURES_CHAIN_LINK_H

#include <glm/glm.hpp>
#include "../../../core/entities/Entity.h"

class Sphere;
class SceneManager;
class SceneNode;
class SceneLeafModel;

class ChainLink : public Entity
{
public:
	ChainLink(SceneManager& scene_manager, SceneNode& parent, const glm::mat4& local_transformation);
	
	bool activate(Entity& activator);
	
	void setIsFound(bool is_found) { been_found_ = is_found; }
private:
	bool been_found_;
	Sphere* sphere_;
	SceneLeafModel* status_node_;
};

#endif
