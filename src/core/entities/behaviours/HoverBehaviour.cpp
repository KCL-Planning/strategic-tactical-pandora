#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "HoverBehaviour.h"

#include "../Entity.h"

HoverBehaviour::HoverBehaviour(SceneNode& entity)
	: Behaviour(entity)
{
	time_elapsed_ = 1;
}

void HoverBehaviour::prepare(float dt)
{
	time_elapsed_ += dt;
	glm::vec3 hover(sin(time_elapsed_) * 0.02f, cos(time_elapsed_ * 3) * 0.012f, 0.0f);
	//if (time_elapsed_ > 2.0f) time_elapsed_ = -2.0f;
	//glm::vec3 hover(time_elapsed_, 0.0f, 0.0f);
	entity_->setTransformation(glm::translate(glm::mat4(1.0f), hover));
}

void HoverBehaviour::activate(const Entity& activator)
{

}

