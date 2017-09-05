#ifndef DEMO_PANDORA_CONTROLLERS_VALVE_TURN_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_VALVE_TURN_CONTROLLER_H

#include <ros/ros.h>

#include "PlannerAction.h"

class AUV;
class Line;
class SceneNode;
class SceneLeafModel;
class SceneManager;
class Material;
class OntologyInterface;
class Valve;
class FollowWaypointController;
class RRT;

class ValveTurnController : public PlannerAction
{
public:
	ValveTurnController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub, FollowWaypointController& follow_waypoint_controller);
	
	void setValve(Valve& valve, float desired_increment, int valve_deadline);
	
	/**
	 * Create a cool visualisation of the AUV inspecting the given point.
	 * @param inspection_point The point that must be inspected.
	 */
	//void observe(const glm::vec3& inspection_point);
	
	/**
	 * Return true if we are done observing.
	 */
	PlannerAction::PLANNER_ACTION_STATUS getStatus();
	
	/**
	 * Update the observe visual.
	 */
	void update(float dt);
	
	/**
	 * Update the feedback with reasons why the valve panel has failed (if it has).
	 */
	void amendFeedback(planning_msgs::ActionFeedback& feedback, PlannerAction::PLANNER_ACTION_STATUS status);
	
private:
	AUV* auv_;
	
	//glm::vec3 inspection_point_; // The location of the point we want to inspect.
	//Line* line_; // We visualise the inspection with a few lines (for now, it will look crap, but who cares :).
	float time_; // Time that we have been observing.
	
	OntologyInterface* ontology_; // The ontology that stores all the things we care about.
	
	Valve* valve_; // Valve.
	float desired_increment_;
	int valve_deadline_;
	
	bool valve_blocked_; // Safe whether the valve blocked this time or not.
	bool valve_is_turning_;
	
	ros::Publisher* action_feedback_pub_;
	
	FollowWaypointController* follow_waypoint_controller_;
};

#endif
