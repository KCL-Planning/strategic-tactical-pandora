#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "MoveBehaviour.h"
#include "../Entity.h"

MoveBehaviour::MoveBehaviour(SceneNode& entity, const glm::vec3& velocity)
	: Behaviour(entity), velocity_(velocity)
{

}

void MoveBehaviour::prepare(float dt)
{
	entity_->setTransformation(glm::translate(entity_->getLocalTransformation(), velocity_ * dt));
}

void MoveBehaviour::activate(const Entity& activator)
{

}
