#ifndef CORE_ANIMATION_BOUNCING_BOX_H
#define CORE_ANIMATION_BOUNCING_BOX_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 

#include "../entities/Entity.h"

class Camera;
class Renderer;
class SceneNode;
class SceneManager;

/**
 * Animation that bounces an object withtin a given box.
 */
class BouncingBox : public Entity
{
public:
	BouncingBox(SceneManager& scene_manager, SceneNode* scene_node, const glm::mat4& translation, float min_x, float min_y, float min_z, float max_x, float max_y, float max_z, const glm::vec3& direction, const glm::vec3& location);

	void prepare(float dt);
private:
	float min_x_, min_y_, min_z_, max_x_, max_y_, max_z_;
	float yaw_, pitch_, roll_;
	float current_yaw_, current_pitch_, current_roll_;

	glm::vec3 current_direction_;
	glm::vec3 current_location_;
};

#endif
