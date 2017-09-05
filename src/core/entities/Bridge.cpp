#include <glm/gtc/matrix_transform.hpp>

#include "Bridge.h"
#include "Lever.h"
#include "../scene/SceneNode.h"

Bridge::Bridge(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, Lever& lever, ENTITY_TYPE type, const glm::vec3& begin_position, const glm::vec3& end_position)
	: Entity(scene_manager, parent, transformation, type, "bridge"), lever_(&lever), begin_position_(begin_position), end_position_(end_position), target_position_(begin_position)
{
//	lever.addActivateListener(*this);
}

void Bridge::prepare(float dt)
{
	// Move the bridge.
	glm::vec3 direction = target_position_ - getGlobalLocation();
	if (glm::length(direction) > 0.1f)
	{
		glm::vec3 actual_direction = glm::normalize(direction) * dt;
		if (glm::length(direction) < glm::length(actual_direction))
		{
			actual_direction = direction;
		}
	
		setTransformation(glm::translate(getLocalTransformation(), actual_direction));
	}
	Entity::prepare(dt);
}

void Bridge::onActivate(const Entity& activated_entity)
{
	if (lever_->isLeverOn())
	{
		target_position_ = end_position_;
	}
	else
	{
		target_position_ = begin_position_;
	}
}
