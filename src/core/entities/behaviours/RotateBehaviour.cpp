#include <iostream>

#include "RotateBehaviour.h"
#include <glm/glm.hpp>
#include "../Entity.h"

RotateBehaviour::RotateBehaviour(SceneNode& entity, const glm::vec3& rotation_point, const glm::vec3& rotation_axis, float min_rotation, float max_rotation, float current_rotation, float rotation_speed)
	: Behaviour(entity), activated_(false), rotation_point_(rotation_point), rotation_axis_(rotation_axis), min_rotation_(min_rotation), max_rotation_(max_rotation), current_rotation_(current_rotation), rotation_speed_(rotation_speed)
{
	
}

void RotateBehaviour::prepare(float dt)
{
	if (activated_)
	{
		const glm::mat4& local_transformation = entity_->getLocalTransformation();

		// Check which direction the entity needs to rotate.
		float rotation_dt = rotation_speed_ * dt;
		if (rotate_to_max_)
		{
			if (current_rotation_ + rotation_dt > max_rotation_)
			{
				rotation_dt = max_rotation_ - current_rotation_;
				activated_ = false;
			}
		}
		else
		{
			rotation_dt = -rotation_dt;
			if (current_rotation_ + rotation_dt < min_rotation_)
			{
				rotation_dt = min_rotation_ - current_rotation_;
				activated_ = false;
			}
		}

		current_rotation_ += rotation_dt;
		//std::cout << "current rotation: " << current_rotation_ << "(" << rotation_dt << ")[" << rotation_speed_ << "]{" << dt << "}" << "=" << glm::yaw(entity_->getGlobalRotation()) << ", " << glm::pitch(entity_->getGlobalRotation()) << ", " << glm::roll(entity_->getGlobalRotation()) << std::endl;
		glm::mat4 rotated_transformation = glm::translate(local_transformation, -rotation_point_);
		rotated_transformation = glm::rotate(rotated_transformation, rotation_dt, rotation_axis_);
		rotated_transformation = glm::translate(rotated_transformation, rotation_point_);
		entity_->setTransformation(rotated_transformation);
	}
}

void RotateBehaviour::activate(const Entity& activator)
{
	// Check if the lever was not activated just a moment ago.
	if (!activated_)
	{
		rotate_to_max_ = !rotate_to_max_;
		activated_ = true;
	}
}
