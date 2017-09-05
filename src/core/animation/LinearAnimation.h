#ifndef CORE_ANIMATION_LINEAR_ANIMATION_H
#define CORE_ANIMATION_LINEAR_ANIMATION_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 

#include "../entities/Entity.h"

class LinearAnimation : public Entity
{
public:
	LinearAnimation(SceneManager& scene_manager, SceneNode* parent, const glm::vec3& transform, const float yaw, const float pitch, const float roll);

	virtual void prepare(float dt);
private:
	float total_time_;

	glm::vec3 transform_;
	float pitch_;
	float yaw_;
	float roll_;
};

#endif
