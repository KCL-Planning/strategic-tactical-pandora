#ifndef CORE_ENTITIES_BEHAVIOURS_ROTATE_BEHAVIOUR_H
#define CORE_ENTITIES_BEHAVIOURS_ROTATE_BEHAVIOUR_H

#include <glm/gtc/matrix_transform.hpp>

#include "Behaviour.h"

class RotateBehaviour : public Behaviour
{
public:
	RotateBehaviour(SceneNode& entity, const glm::vec3& rotation_point, const glm::vec3& rotation_axis, float min_rotation, float max_rotation, float current_rotation, float rotation_speed);

	void prepare(float dt);
	void activate(const Entity& activator);

private:
	bool activated_;      // True if the bahaviour is currently active.
	bool rotate_to_max_;  // True if we rotate to the max rotation.
	glm::vec3 rotation_point_;
	glm::vec3 rotation_axis_;
	float min_rotation_, max_rotation_, current_rotation_, rotation_speed_;
};

#endif
