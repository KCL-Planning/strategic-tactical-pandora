#ifndef DEMO_PANDORA_CONTROLLERS_FOLLOW_WAYPOINT_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_FOLLOW_WAYPOINT_CONTROLLER_H

#include <vector>

#include <glm/glm.hpp>

#include "PlannerAction.h"
#include "../RRT.h"

class AUV;
class RRT;
class SceneManager;

class FollowWaypointController : public PlannerAction
{
public:
	FollowWaypointController(SceneManager& scene_manager, AUV& auv, RRT& rrt);
	
	void followWaypoint(const glm::vec3& goal, bool move_controlled, float pitch, float yaw, float collision_distance = 1.0f);
	
	void update(float dt);
	
	PlannerAction::PLANNER_ACTION_STATUS getStatus();
	//bool isFinished() { return path_.size() == current_waypoint_; }
private:
	SceneManager* scene_manager_;
	AUV* auv_;
	RRT* rrt_;
	glm::vec3 goal_;
	
	bool move_controlled_;
	float pitch_;
	float yaw_;
	float time_;
	float collision_distance_;
};

#endif
