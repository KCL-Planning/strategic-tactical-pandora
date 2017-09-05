#ifndef CORE_ENTITIES_LEVER_H
#define CORE_ENTITIES_LEVER_H

#include <glm/glm.hpp>

#include "Entity.h"

class SceneNode;
class SceneManager;

class Lever : public Entity
{
public:
	Lever(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type);

	virtual void prepare(float dt);

	virtual bool activate(Entity& activator);

	bool isLeverOn() const { return lever_on_; }

private:
	bool lever_is_activated_;
	bool lever_on_;
};

#endif
