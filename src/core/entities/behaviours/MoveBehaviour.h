#ifndef CORE_ENTITIES_BEHAVIOURS_MOVE_BEHAVIOUR_H
#define CORE_ENTITIES_BEHAVIOURS_MOVE_BEHAVIOUR_H

#include "Behaviour.h"

class MoveBehaviour : public Behaviour
{
public:

	MoveBehaviour(SceneNode& entity, const glm::vec3& velocity);

	virtual void prepare(float dt);
	virtual void activate(const Entity& activator);
private:
	glm::vec3 velocity_;
};

#endif
