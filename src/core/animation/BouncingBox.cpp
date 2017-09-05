#include <stdlib.h>
#include <time.h>

#include "BouncingBox.h"
#include "../scene/SceneNode.h"

BouncingBox::BouncingBox(SceneManager& scene_manager, SceneNode* scene_node, const glm::mat4& translation, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, const glm::vec3& direction, const glm::vec3& location)
	: Entity(scene_manager, scene_node, translation, OBSTACLE, "bouncing box"), min_x_(min_x), min_y_(min_y), min_z_(min_z), max_x_(max_x), max_y_(max_y), max_z_(max_z), current_direction_(direction), current_location_(location)
{
	current_yaw_ = 0.0f;
	current_pitch_ = 0.0f;
	current_roll_ = 0.0f;
	yaw_ = 0.0f;
	pitch_ = 0.0f;
	roll_ = 0.0f;
}

void BouncingBox::prepare(float dt)
{
	current_location_ += current_direction_ * dt;

	if (current_yaw_ > 360)
	{
		current_yaw_ -= 360;
	}
	if (current_pitch_ > 360)
	{
		current_pitch_ -= 360;
	}
	if (current_roll_ > 360)
	{
		current_roll_ -= 360;
	}

	// Check if we violate any boundaries.
	bool violates_constraint = true;
	while (violates_constraint)
	{
		violates_constraint = false;
		if (current_location_.x < min_x_)
		{
			current_location_.x += min_x_ - current_location_.x;
			violates_constraint = true;
			current_direction_.x *= -1;
		}
		else if (current_location_.x > max_x_)
		{
			current_location_.x -= current_location_.x - max_x_;
			violates_constraint = true;
			current_direction_.x *= -1;
		}
		if (current_location_.y < min_y_)
		{
			current_location_.y += min_y_ - current_location_.y;
			violates_constraint = true;
			current_direction_.y *= -1;
		}
		else if (current_location_.y > max_y_)
		{
			current_location_.y -= current_location_.y - max_y_;
			violates_constraint = true;
			current_direction_.y *= -1;
		}
		if (current_location_.z < min_z_)
		{
			current_location_.z += min_z_ - current_location_.z;
			violates_constraint = true;
			current_direction_.z *= -1;
		}
		else if (current_location_.z > max_z_)
		{
			current_location_.z -= current_location_.z - max_z_;
			violates_constraint = true;
			current_direction_.z *= -1;
		}
	}

	// Update the node.
	local_transformation_ = glm::translate(glm::mat4(1.0), current_location_);
	local_transformation_ = glm::rotate(local_transformation_, current_pitch_, glm::vec3(1.0f, 0.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, current_yaw_, glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, current_roll_, glm::vec3(0.0f, 0.0f, 1.0f));
	Entity::prepare(dt);
	//scene_node_->setTransformation(translation);
}
