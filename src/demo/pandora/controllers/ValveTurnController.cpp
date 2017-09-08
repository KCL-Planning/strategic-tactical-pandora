#include "ValveTurnController.h"
#include "FollowWaypointController.h"

#include <cstdlib>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include <planning_msgs/ActionFeedback.h>

#include "../AUV.h"
#include "../ontology/OntologyInterface.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/shaders/ShadowShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shapes/Line.h"
#include "dpengine/shapes/FrustumShape.h"
#include "../structures/Structure.h"
#include "../structures/Valve.h"
#include "../models/RobotHand.h"

ValveTurnController::ValveTurnController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub, FollowWaypointController& follow_waypoint_controller)
	: auv_(&auv), time_(0), ontology_(&ontology), valve_(NULL), desired_increment_(0), valve_deadline_(0), action_feedback_pub_(&action_feedback_pub), follow_waypoint_controller_(&follow_waypoint_controller)
{
	
}

void ValveTurnController::setValve(Valve& valve, float desired_increment, int valve_deadline)
{
	valve_ = &valve;
	desired_increment_ = desired_increment;
	valve_deadline_ = valve_deadline;
	
	//std::cout << "Desired rotation:" << desired_increment << std::endl;
	
	valve.getStructure().setExamined(false);
	valve_is_turning_ = false;
	
	glm::vec4 valve_access_panel(0, 0, 2, 1);
	valve_access_panel = valve_->getCompleteTransformation() * valve_access_panel;
	
	follow_waypoint_controller_->followWaypoint(glm::vec3(valve_access_panel) + glm::vec3(0, 0.0f, -0.1f), true, 0, glm::yaw(glm::quat_cast(valve_->getCompleteTransformation())) + 180);
	
	//auv_->getRobotHand().unfoldArm(true);
	
	// Set a chance that the valve is blocked.
	valve_blocked_ = true;
	if (valve_->getTimesBlocked() < 2)
	{
/*
		float p = (float)rand() / (float)RAND_MAX;
		if (p < 0.5f)
		{
			valve_->setBlocked();
			std::cout << "A great disaster has struck your tribe, the valve [ID=" << valve_->getId() << "] is blocked! The gods are most pleased..." << std::endl;
		}
		else
*/
		{
			valve_blocked_ = false;
			//valve.setRotation(desired_increment);
		}
	}
}


PlannerAction::PLANNER_ACTION_STATUS ValveTurnController::getStatus()
{
	if (valve_->getTimesBlocked() > 1 || valve_blocked_)
	{
		return FAILED;
	}
	if (valve_->doneRotating() && valve_is_turning_)
	{
		auv_->getRobotHand().rotateHand(0);
		auv_->getRobotHand().unfoldArm(false);
		valve_->getStructure().setExamined(false);
		
		return SUCCEEDED;
	}
	return EXECUTING;
}

void ValveTurnController::amendFeedback(planning_msgs::ActionFeedback& feedback, PlannerAction::PLANNER_ACTION_STATUS status)
{
	if (status == FAILED && valve_blocked_)
	{
		diagnostic_msgs::KeyValue key_value;
		std::stringstream ss;
		ss << "valve_" << valve_->getName() << "_state";
		key_value.key = ss.str();
		key_value.value = "valve_blocked";
		feedback.information.push_back(key_value);
	}
}

void ValveTurnController::update(float dt)
{
	follow_waypoint_controller_->update(dt);
	
	// Check if the AUV is near the valve.
	if (follow_waypoint_controller_->getStatus() == SUCCEEDED)
	{
		auv_->getRobotHand().unfoldArm(true);
		if (auv_->getRobotHand().isArmUnfolded() && !valve_is_turning_)
		{
			valve_->setRotation(desired_increment_);
			valve_is_turning_ = true;
			auv_->getRobotHand().rotateHand(desired_increment_);
		}
	}
	
	time_ += dt;
}
