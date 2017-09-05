#include "DynamicCamera.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include <iostream>

#include "GL/glfw.h"

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "Camera.h"
#include "../../math/Math.h"

DynamicCamera::DynamicCamera(SceneNode& to_follow, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane)
	: Camera(scene_manager, parent, transformation, fov, width, height, near_plane, far_plane), to_follow_(&to_follow), prefered_transformation_(glm::vec3(transformation[3][0], transformation[3][1], transformation[3][2])), moved_camera_last_frame_(false)
{
	//camera_ = new Camera();
	roll_ = 0;
	yaw_ = 0;
	pitch_ = 0;
}

void DynamicCamera::mouseScrollCallBack(double xoffset, double yoffset)
{

}

DynamicCamera::~DynamicCamera()
{
	//delete camera_;
}

void DynamicCamera::prepare(float dt)
{
	// Capture input to control the mouse.
	//if (glfwGetKey(GLFW_KEY_LALT) != GLFW_PRESS)
	if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	{
		int width, height;
		glfwGetWindowSize(&width, &height);
		
		int mouseX, mouseY;
		glfwGetMousePos(&mouseX, &mouseY);
		
		// Reset mouse pointer.
		glfwSetMousePos(width / 2, height / 2);
		
		// Wait for the mouse to be reset before moving the camera. Otherwise we get very 
		// sporatic and sudden movements.
		if (!moved_camera_last_frame_)
		{
			//if (abs(mouseX - width / 2) < 10 && abs(mouseY - height / 2) < 10)
			{
				moved_camera_last_frame_ = true;
			}
			SceneNode::prepare(dt);
			return;
		}
		
		// Move the mouse sideways relative to the target we are currently following.
		float mouse_x_delta = (width / 2.0f) - mouseX;
		float mouse_y_delta = (height / 2.0f) - mouseY;
		if (mouse_x_delta > 1 || mouse_x_delta < -1 || mouse_y_delta > 1 || mouse_y_delta < -1)
		{
			yaw_ += mouse_x_delta * 0.27f;
			pitch_ += mouse_y_delta * 0.5f;
		}
	}
	else
	{
		moved_camera_last_frame_ = false;
		SceneNode::prepare(dt);
		return;
	}

	local_transformation_ = glm::rotate(glm::mat4(1.0f), roll_, glm::vec3(0.0f, 0.0f, 1.0f));
	local_transformation_ = glm::rotate(local_transformation_, yaw_, glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, pitch_, glm::vec3(1.0f, 0.0f, 0.0f));
	local_transformation_ = glm::translate(local_transformation_, glm::vec3(0.0f, 0.0f, std::max(4.0f, (glfwGetMouseWheel() + 8) * 0.5f)));
	
	glm::vec3 local_location(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]);

	// Make sure that the camera is looking at the target.
	updateTransformations();
	glm::vec3 cam_loc = getGlobalLocation();
	glm::vec3 target_loc = to_follow_->getGlobalLocation();
	glm::vec3 look_vector = glm::normalize(target_loc - cam_loc);

	float yaw = (180.0f / M_PI) * Math::signedAngle(glm::vec2(look_vector.x, look_vector.z), glm::vec2(0.0f, -1.0f));
	float pitch = (180.0f / M_PI) * Math::signedAngle(glm::vec2(0.0f, -1.0f), glm::vec2(look_vector.y, sqrt(look_vector.z * look_vector.z + look_vector.x * look_vector.x)));
	//float yaw = (180.0f / glm::pi<float>()) * Math::signedAngle(glm::vec2(look_vector.x, look_vector.z), glm::vec2(0.0f, -1.0f));
	//float pitch = (180.0f / glm::pi<float>()) * Math::signedAngle(glm::vec2(0.0f, -1.0f), glm::vec2(look_vector.y, sqrt(look_vector.z * look_vector.z + look_vector.x * look_vector.x)));
	float roll = 0;

	// Counter the rotation of the parent.
	if (parent_ != NULL)
	{
		glm::fquat parent_rotation = parent_->getGlobalRotation();
		yaw -= glm::yaw(parent_rotation);
		pitch -= glm::pitch(parent_rotation);
		roll -= glm::roll(parent_rotation);
	}

	if (pitch > 90)
	{
		pitch -= 180;
		pitch = -pitch;
	}
	else if (pitch < -90)
		{
		pitch += 180;
		pitch = -pitch;
	}
	
	Camera::checkLimits(pitch, yaw, roll);

	// Construct local translation.
	local_transformation_ = glm::translate(glm::mat4(1.0f), local_location);
	local_transformation_ = glm::rotate(local_transformation_, roll, glm::vec3(0.0f, 0.0f, 1.0f));
	local_transformation_ = glm::rotate(local_transformation_, yaw, glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, pitch, glm::vec3(1.0f, 0.0f, 0.0f));

	/*
	local_transformation_ = glm::rotate(glm::mat4(1.0f), roll_, glm::vec3(0.0f, 0.0f, 1.0f));
	local_transformation_ = glm::rotate(local_transformation_, yaw_, glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, pitch_, glm::vec3(1.0f, 0.0f, 0.0f));
	local_transformation_ = glm::translate(local_transformation_, glm::vec3(0.0f, 0.0f, std::max(4.0f, (glfwGetMouseWheel() + 8) * 0.5f)));
	*/

	//camera_->setRotation(pitch, yaw, roll);

	SceneNode::prepare(dt);
	//Camera::prepare(dt);
	
	//std::cout << "(" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
	//std::cout << "yaw: " << glm::yaw(getGlobalRotation()) << ", " << glm::pitch(getGlobalRotation()) << ", " << glm::roll(getGlobalRotation()) << std::endl;
	// Move the camera to the prefered position -- if possible.
	//camera_->setPosition(getGlobalLocation().x, getGlobalLocation().y, getGlobalLocation().z);
}
