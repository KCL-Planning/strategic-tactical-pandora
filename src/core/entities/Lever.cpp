#include <glm/gtc/matrix_transform.hpp>

//#include <windows.h>

#include "Lever.h"
#include "../scene/SceneNode.h"

Lever::Lever(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type)
	: Entity(scene_manager, parent, transformation, type, "lever"), lever_is_activated_(false), lever_on_(false)
{
	
}

void Lever::prepare(float dt)
{
	// Check if we need to move the lever.
	if (lever_is_activated_)
	{
		const glm::mat4& local_transformation = getLocalTransformation();
		glm::fquat rotation = glm::quat_cast(local_transformation);
		float pitch = glm::pitch(rotation);

		// Check which direction the lever needs to go.
		float modifier = 10.0f;
		if (lever_on_)
		{
			if (pitch > 60)
			{
				modifier = 0.0f;
				lever_is_activated_ = false;

				// Do the prescribed behaviour.
			}
		}
		else
		{
			modifier = -10.0f;
			if (pitch < -60)
			{
				modifier = 0.0f;
				lever_is_activated_ = false;

				// Do the prescribed behaviour.
			}
		}
		glm::mat4 rotated_transformation = glm::translate(local_transformation, glm::vec3(0.0f, -0.5f, 0.0f));
		rotated_transformation = glm::rotate(rotated_transformation, modifier * dt, glm::vec3(1.0f, 0.0f, 0.0f));
		rotated_transformation = glm::translate(rotated_transformation, glm::vec3(0.0f, 0.5f, 0.0f));
		setTransformation(rotated_transformation);
	}
	Entity::prepare(dt);
	/*
	const glm::mat4& parent_transform = scene_node_->getCompleteTransformation();
	global_location_ = glm::vec3(parent_transform[3][0], parent_transform[3][1], parent_transform[3][2]);
	global_rotation_ = glm::quat_cast(parent_transform);
	*/
}

bool Lever::activate(Entity& activator)
{
	// Check if the lever was not activated just a moment ago.
	if (lever_is_activated_)
	{
		return false;
	}

	lever_on_ = !lever_on_;
	lever_is_activated_ = true;
	return Entity::activate(activator);
}
