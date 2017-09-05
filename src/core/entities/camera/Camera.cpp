#include "Camera.h"

#include "GL/glew.h"
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

Camera::Camera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane)
	: Entity(scene_manager, parent, transformation, PLAYER, "Camera"), fov_(fov), width_(width), height_(height), near_plane_(near_plane), far_plane_(far_plane)
{
	//perspective_matrix_ = glm::perspective(90.0f, float(1024) / float(768), 0.1f, 300.0f);
	perspective_matrix_ = glm::perspective(fov, width / height, near_plane, far_plane);
}

void Camera::checkLimits(float& pitch, float& yaw, float& roll)
{
	if ((int)(roll) == 180)
	{
		pitch = -pitch;

		if (yaw > 0) yaw -= 180 - yaw;
		else yaw -= -180 - yaw;
		roll = 0;
		return;
	}
		
	
	if (pitch > 90)
	{
		pitch = 90;
	}
	else if (pitch < -90)
	{
		pitch = -90;
	}
	
	if (yaw < 0)
	{
		yaw += 360;
	}
	else if (yaw > 360)
	{
		yaw -= 360;
	}
}

glm::mat4 Camera::getViewMatrix() const
{
	glm::fquat rot = getGlobalRotation();
	glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -glm::pitch(rot), glm::vec3(1.0f, 0.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::yaw(rot), glm::vec3(0.0f, 1.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::roll(rot), glm::vec3(0.0f, 0.0f, 1.0f));
	view_matrix = glm::translate(view_matrix, -getGlobalLocation());
	return view_matrix;
}

glm::vec3 Camera::getLocation() const
{
	return getGlobalLocation();
}

glm::mat4 Camera::getPerspectiveMatrix() const
{
	return perspective_matrix_;
}

void Camera::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	perspective_matrix_ = glm::perspective(fov_, width_ / height_, near_plane_, far_plane_);
}
