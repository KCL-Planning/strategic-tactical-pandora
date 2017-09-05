#include "LinearAnimation.h"

LinearAnimation::LinearAnimation(SceneManager& scene_manager, SceneNode* parent, const glm::vec3& transform, const float pitch, const float yaw, const float roll)
	: Entity(scene_manager, parent, glm::mat4(1.0), OBSTACLE, "linear animiation"), total_time_(0.0f), transform_(transform), pitch_(pitch), yaw_(yaw), roll_(roll)
{

}

void LinearAnimation::prepare(float dt)
{
	if (dt > 0)
	{
		total_time_ += dt;

		local_transformation_ = glm::translate(glm::mat4(1.0), transform_ * total_time_);
		local_transformation_ = glm::rotate(local_transformation_, pitch_ * total_time_, glm::vec3(1, 0, 0));
		local_transformation_ = glm::rotate(local_transformation_, yaw_ * total_time_, glm::vec3(0, 1, 0));
		local_transformation_ = glm::rotate(local_transformation_, roll_ * total_time_, glm::vec3(0, 0, 1));
	}
	SceneNode::prepare(dt);
}

