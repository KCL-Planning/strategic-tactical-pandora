#include "FreeMovingCamera.h"
//#define _USE_MATH_DEFINES
//#include <cmath>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <GL/glew.h>
#include <GL/glfw.h>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

FreeMovingCamera::FreeMovingCamera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane)
	: Camera(scene_manager, parent, transformation, fov, width, height, near_plane, far_plane), yaw_(0), pitch_(0), roll_(0)
{

}

void FreeMovingCamera::prepare(float dt)
{
	int width, height;
	glfwGetWindowSize(&width, &height);

	// Handle input.
	int mouseX, mouseY;
	glfwGetMousePos(&mouseX, &mouseY);

	glm::vec3 current_direction(0.0f, 0.0f, 0.0f);
	if (glfwGetKey('W') == GLFW_PRESS)
	{
		current_direction.z = -1.0f;
	}
	else if (glfwGetKey('S') == GLFW_PRESS)
	{
		current_direction.z = 1.0f;
	}
	if (glfwGetKey('A') == GLFW_PRESS)
	{
		current_direction.x = -1.0f;
	}
	else if (glfwGetKey('D') == GLFW_PRESS)
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

	if (glfwGetKey(GLFW_KEY_LSHIFT) == GLFW_PRESS || glfwGetKey(GLFW_KEY_RSHIFT) == GLFW_PRESS)
	{
		speed = 8.0f;
	}

	glm::vec3 direction(0, 0, 0);
	if (current_direction.x != 0.0f || current_direction.z != 0.0f)
	{
		direction = glm::normalize(current_direction) * dt * speed;
		direction = glm::rotate(direction, pitch_, glm::vec3(1, 0, 0));
		direction = glm::rotate(direction, yaw_, glm::vec3(0, 1, 0));
		direction = glm::rotate(direction, roll_, glm::vec3(0, 0, 1));
	}
	
	local_transformation_ = glm::translate(glm::mat4(1.0f), getLocalLocation() + direction);
	local_transformation_ = glm::rotate(local_transformation_, roll_, glm::vec3(0, 0, 1));
	local_transformation_ = glm::rotate(local_transformation_, yaw_, glm::vec3(0, 1, 0));
	local_transformation_ = glm::rotate(local_transformation_, pitch_, glm::vec3(1, 0, 0));
	updateTransformations();
}
