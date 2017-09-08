#ifndef DEMO_PANDORA_STRUCTURES_CHAIN_LINK_H
#define DEMO_PANDORA_STRUCTURES_CHAIN_LINK_H

#include <glm/glm.hpp>
#include "dpengine/entities/Entity.h"

namespace DreadedPE
{
	class Sphere;
	class SceneManager;
	class SceneNode;
	class SceneLeafModel;
};

class ChainLink : public DreadedPE::Entity
{
public:
	ChainLink(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode& parent, const glm::mat4& local_transformation);
	
	bool activate(DreadedPE::Entity& activator);
	
	void setIsFound(bool is_found) { been_found_ = is_found; }
private:
	bool been_found_;
	DreadedPE::Sphere* sphere_;
	DreadedPE::SceneLeafModel* status_node_;
};

#endif
