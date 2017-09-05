#include "Propeller.h"

#include <iostream>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

Propeller::Propeller(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation)
	: SceneNode(scene_manager, parent, transformation)
{
	
}

void Propeller::prepare(float dt)
{
	local_transformation_ = glm::rotate(local_transformation_, rotation_speed_ * dt, glm::vec3(0.0f, 1.0f, 0.0f));
	SceneNode::prepare(dt);
}