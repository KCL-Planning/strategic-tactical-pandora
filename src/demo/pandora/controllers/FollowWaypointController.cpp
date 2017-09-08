#include "FollowWaypointController.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../AUV.h"
#include "../RRT.h"
#include "dpengine/math/Math.h"
#include "dpengine/scene/SceneManager.h"

#include "../gui/AUVStatusIcon.h"


FollowWaypointController::FollowWaypointController(DreadedPE::SceneManager& scene_manager, AUV& auv, RRT& rrt)
	: scene_manager_(&scene_manager), auv_(&auv), rrt_(&rrt), move_controlled_(false), pitch_(0), yaw_(0), time_(0), collision_distance_(1.0f)
{
	//rrt_->addListener(*this);
}

void FollowWaypointController::followWaypoint(const glm::vec3& goal, bool move_controlled, float pitch, float yaw, float collision_distance)
{
	time_ = 0;
	goal_ = goal;
	collision_distance_ = collision_distance;
	
	move_controlled_ = move_controlled;
	pitch_ = pitch;
	yaw_ = yaw;
	
	// Update the pitch such that it falls between [-90, 90].
	if (pitch_ < -90)
	{
		pitch_ = 180 + pitch_;
	}
	else if (pitch > 90)
	{
		pitch_ = 180 - pitch_;
	}
	
	std::cout << "Go to the path: (" << goal.x << ", " << goal.y << ", " << goal.z << ")" << std::endl;
	
	if (move_controlled)
	{
		std::cout << "Controlled movement, face: " << pitch_ << " " << yaw_ << std::endl;
	}
	
	if (move_controlled_)
	{
		auv_->setDesiredOrientation(yaw_, pitch_);
	}
	else
	{
		auv_->unsetDesiredOrientation();
	}
	
	// Change the AUV's heading.
	// Check if we are (roughly) pointing in the direction of the next waypoint.
	glm::vec3 prefered_direction = goal_ - auv_->getGlobalLocation();
	if (glm::length(prefered_direction) > 0)
	{
		prefered_direction = glm::normalize(prefered_direction);
	}
	else
	{
		prefered_direction = glm::vec3(0, 0, -1);
	}
	
	auv_->setDirection(prefered_direction);
}

void FollowWaypointController::update(float dt)
{
	time_ += dt;
	// Set the direction at which the AUV is pointing.
	if (move_controlled_)
	{
		auv_->setDesiredOrientation(yaw_, pitch_);
	}
	else
	{
		auv_->unsetDesiredOrientation();
	}
	
	// Before we start, make sure we haven't all ready accomplished our task!
	float distance = glm::distance(auv_->getGlobalLocation(), goal_);
	if (distance < 0.1f)
	{
		//std::cout << "AUV: Direction: Move on to the next waypoint!" << std::endl;
		auv_->setVelocity(0);
		return;
	}
	
	glm::vec3 direction = goal_ - auv_->getGlobalLocation();
	float length = glm::length(direction);
	glm::vec3 end_point = goal_;
		
	// Check if the AUV is about to collide, if this is the case then we do an emergency stop.
	if (scene_manager_->getRoot().doesCollide(auv_, auv_->getGlobalLocation(), end_point, collision_distance_))
	{
		auv_->setVelocity(0);
	}
	
	// Check if we are (roughly) pointing in the direction of the next waypoint.
	glm::vec3 prefered_direction = goal_ - auv_->getGlobalLocation();
	prefered_direction = glm::normalize(prefered_direction);
	
	// Change the AUV's heading.
	auv_->setDirection(prefered_direction);
	
	// Move the AUV forward. The speed we set is relative to the distance to the point.
	//auv_->setVelocity(std::min(0.5f, distance / 2.0f));
	if (move_controlled_)
	{
		auv_->setVelocity(1.0f);
	}
	else
	{
		auv_->setVelocity(2.0f);
	}
}

PlannerAction::PLANNER_ACTION_STATUS FollowWaypointController::getStatus()
{
	float distance = glm::distance(auv_->getGlobalLocation(), goal_);

	float d_yaw = auv_->getDesiredYaw() - auv_->getYaw();

	if (d_yaw > 180) d_yaw -= 360;
	if (d_yaw < -180) d_yaw += 360;
	
	if (distance < 0.1f && (!move_controlled_ || d_yaw < 0.1f))
	{
		auv_->setVelocity(0.0f);
		return SUCCEEDED;
	}
	
	glm::vec3 direction = goal_ - auv_->getGlobalLocation();
	float length = glm::length(direction);
	glm::vec3 end_point = goal_;
	
	if (length > 5.0f)
	{
		end_point = auv_->getGlobalLocation() + glm::normalize(direction) * 5.0f;
	}
	
	// Check if the AUV is about to collide, if this is the case then we do an emergency stop.
	if (scene_manager_->getRoot().doesCollide(auv_, auv_->getGlobalLocation(), end_point, collision_distance_))
	{
		auv_->setVelocity(0.0f);
		std::cout << "ACTION FAILED!!!!!!!!!!" << std::endl;
		
		AUVStatusIcon& auv_status_icon = AUVStatusIcon::getInstance();
		const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getCollisionIcon();
		/*
		std::vector<glm::vec2> uv_mapping;
		uv_mapping.push_back(glm::vec2(0.5f, 1.0f));
		uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
		uv_mapping.push_back(glm::vec2(0.5f, 0.75f));
		uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
		*/
		auv_->setBillBoardUVs(uv_mapping);
		return FAILED;
	}
	
	return EXECUTING;
}
