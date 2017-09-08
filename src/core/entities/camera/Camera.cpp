#include "dpengine/entities/camera/Camera.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/collision/CollisionInfo.h"

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace DreadedPE
{

Camera::Camera(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, float fov, float width, float height, float near_plane, float far_plane)
	: Entity(scene_manager, parent, transformation, PLAYER, "Camera"), fov_(fov), width_(width), height_(height), near_plane_(near_plane), far_plane_(far_plane)
{
	perspective_matrix_ = glm::perspective(glm::radians(fov), width / height, near_plane, far_plane);
}

Entity* Camera::pickEntity(float mouse_x, float mouse_y, glm::vec3& intersection)
{
	// Transform the viewport coordinates to the normalised devise coordinates.
	float ndc_mouse_x = (2 * mouse_x) / getWidth() - 1.0f;
	float ndc_mouse_y = 1.0f - (2 * mouse_y) / getHeight();

	// Transform the normalised device coordinates into homogeneous clip coordinates.
	glm::vec4 hcc_ray(ndc_mouse_x, ndc_mouse_y, -1.0f, 1.0f);

	// Transform these into eye coordinates.
	glm::vec4 eye_ray = glm::inverse(getPerspectiveMatrix()) * hcc_ray;
	eye_ray.z = -1.0f;
	eye_ray.w = 0.0f;

	// Transform these into world coordinates.
	glm::vec3 world_coordinates = glm::vec3(glm::inverse(getViewMatrix()) * eye_ray);
	glm::vec3 direction = glm::normalize(world_coordinates);

	// Get the collisions.
	std::vector<CollisionInfo> collisions;
	if (!scene_manager_->getRoot().getCollisions(*this, getGlobalLocation(), getGlobalLocation() + glm::vec3(direction) * getFarPlane(), collisions))
	{
		return NULL;
	}

	Entity* closest_entity = NULL;
	float min_distance = std::numeric_limits<float>::max();
	for (std::vector<CollisionInfo>::const_iterator ci = collisions.begin(); ci != collisions.end(); ++ci)
	{
		const CollisionInfo& collision_info = *ci;
		Entity* e = collision_info.colliding_entity_ == NULL ? collision_info.other_colliding_entity_ : collision_info.colliding_entity_;
		for (std::vector<CollisionPoint>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
		{
			const glm::vec3& l = (*ci).intersection_point_;
			float d = glm::distance(l, getGlobalLocation());
			if (d < min_distance)
			{
				min_distance = d;
				closest_entity = e;
				intersection = (*ci).intersection_point_;
			}
		}
	}
	return closest_entity;
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
	glm::fquat rot = getInterpolatedRotation();
	glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -glm::pitch(rot), glm::vec3(1.0f, 0.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::yaw(rot), glm::vec3(0.0f, 1.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::roll(rot), glm::vec3(0.0f, 0.0f, 1.0f));
	view_matrix = glm::translate(view_matrix, -getLocation());
	return view_matrix;
}

glm::vec3 Camera::getLocation() const
{
	return glm::vec3(getInterpolatedMatrix()[3][0], getInterpolatedMatrix()[3][1], getInterpolatedMatrix()[3][2]);
}

glm::mat4 Camera::getPerspectiveMatrix() const
{
	return perspective_matrix_;
}

void Camera::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	perspective_matrix_ = glm::perspective(glm::radians(fov_), width_ / height_, near_plane_, far_plane_);
}

};
