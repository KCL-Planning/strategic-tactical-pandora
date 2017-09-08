#include "dpengine/entities/camera/FreeMovingCamera.h"
//#define _USE_MATH_DEFINES
//#include <cmath>

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

FreeMovingCamera::FreeMovingCamera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane)
	: Camera(scene_manager, parent, transformation, fov, width, height, near_plane, far_plane), yaw_(0), pitch_(0), roll_(0)
{

}

void FreeMovingCamera::prepare(float dt)
{
	Window* window = Window::getActiveWindow();

	int width, height;
	window->getSize(width, height);

	double mouseX, mouseY;
	window->getMouseCursor(mouseX, mouseY);

	glm::vec3 current_direction(0.0f, 0.0f, 0.0f);
	if (window->isKeyPressed('W'))
	{
		current_direction.z = -1.0f;
	}
	else if (window->isKeyPressed('S'))
	{
		current_direction.z = 1.0f;
	}
	if (window->isKeyPressed('A'))
	{
		current_direction.x = -1.0f;
	}
	else if (window->isKeyPressed('D'))
	{
		current_direction.x = 1.0f;
	}

	if ((height / 2.0f) - mouseY >= 1.0f || (height / 2.0f) - mouseY <= -1.0f)
	{
		pitch_ += ((height / 2.0f) - mouseY) * 0.5f;
	}
	if ((width / 2.0f) - mouseX >= 1.0f || (width / 2.0f) - mouseX <= -1.0f)
	{
		yaw_ += ((width / 2.0f) - mouseX) * 0.27f;
	}
	Camera::checkLimits(pitch_, yaw_, roll_);
	
	float speed = 3.0f;

	if (window->isKeyPressed(GLFW_KEY_LEFT_SHIFT) || window->isKeyPressed(GLFW_KEY_RIGHT_SHIFT))
	{
		speed = 8.0f;
	}

	glm::vec3 direction(0, 0, 0);
	if (current_direction.x != 0.0f || current_direction.z != 0.0f)
	{
		direction = glm::normalize(current_direction) * dt * speed;
		direction = glm::rotate(direction, glm::radians(pitch_), glm::vec3(1, 0, 0));
		direction = glm::rotate(direction, glm::radians(yaw_), glm::vec3(0, 1, 0));
		direction = glm::rotate(direction, glm::radians(roll_), glm::vec3(0, 0, 1));
	}
	
	local_transformation_ = glm::translate(glm::mat4(1.0f), getLocalLocation() + direction);
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(roll_), glm::vec3(0, 0, 1));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(yaw_), glm::vec3(0, 1, 0));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(pitch_), glm::vec3(1, 0, 0));
	updateTransformations();

	window->setMouseCursor(width / 2, height / 2);
	Camera::prepare(dt);
}

};
