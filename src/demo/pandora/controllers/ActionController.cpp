#include "ActionController.h"

#include <sstream>
#include <vector>

#include <diagnostic_msgs/KeyValue.h>

#include "../AUV.h"
#include "FollowWaypointController.h"
#include "ObserveController.h"
#include "ExaminePanelController.h"
#include "ValveTurnController.h"
#include "IlluminateController.h"
#include "ChainFollowController.h"

#include "ActionExecutionListener.h"
#include "PlannerAction.h"
#include "RechargeController.h"
#include "../ontology/OntologyInterface.h"
#include "../ontology/Ontology.h"
#include "../structures/Structure.h"
#include "../structures/Valve.h"
#include "../structures/Chain.h"
#include "../level/MissionSite.h"
#include "../gui/ActionLabel.h"

#include "../models/HeightMap.h"

const std::string ActionController::GOTO_ACTION_NAME = "goto";
const std::string ActionController::GOTO_STRUCTURE_ACTION_NAME = "goto_structure";
const std::string ActionController::OBSERVE_ACTION_NAME = "observe";
const std::string ActionController::EXAMINE_PANEL_ACTION_NAME = "examine_panel";
const std::string ActionController::TURN_VALVE_ACTION_NAME = "turn_valve";
const std::string ActionController::CANCEL_ACTION_NAME = "cancel_action";
const std::string ActionController::ILLUMINATE_PILLAR_NAME = "illuminate_pillar";
const std::string ActionController::OBSERVE_PILLAR_NAME = "observe_pillar";
const std::string ActionController::FOLLOW_CHAIN_ACTION_NAME = "enable_chain_follow";
const std::string ActionController::DOCK_AUV_ACTION_NAME = "dock_auv";
const std::string ActionController::UNDOCK_AUV_ACTION_NAME = "undock_auv";
const std::string ActionController::RECHARGE_ACTION_NAME = "recharge";

#define USE_MULTIPLE_AUVS

ActionController::ActionController(DreadedPE::SceneManager& scene_manager, ros::NodeHandle& node_handle, AUV& auv, HeightMap& height_map, FollowWaypointController& follow_waypoint_controller, OntologyInterface& ontology, ActionLabel* label)
	: node_handle_(&node_handle), auv_(&auv), height_map_(&height_map), follow_waypoint_controller_(&follow_waypoint_controller), ontology_(&ontology), current_action_(NULL), label_(label)
{
	std::stringstream ss_dispatch;
#ifdef USE_MULTIPLE_AUVS
	ss_dispatch << "/planning_system/" << auv.getName() << "/action_dispatch";
#else
	ss_dispatch << "/planning_system/action_dispatch";
#endif
	
	std::stringstream ss_feedback;
#ifdef USE_MULTIPLE_AUVS
	ss_feedback << "/planning_system/" << auv.getName() << "/action_feedback";
#else
	ss_feedback << "/planning_system/action_feedback";
#endif
	
	action_dispatch_sub_ = node_handle.subscribe(ss_dispatch.str(), 1000, &ActionController::actionDispatch, this);
	action_feedback_pub_ = node_handle.advertise<planning_msgs::ActionFeedback>(ss_feedback.str(), 1000);
	
	notification_sub_ = node_handle.subscribe("/knowledge/ontology/notification", 1000, &ActionController::notificationReceived, this);
	ROS_INFO("Action dispatch listener active!");
	
	observe_controller_ = new ObserveController(scene_manager, auv, ontology);
	examine_panel_controller_ = new ExaminePanelController(scene_manager, auv, ontology, action_feedback_pub_);
	valve_turn_controller_ = new ValveTurnController(scene_manager, auv, ontology, action_feedback_pub_, follow_waypoint_controller);
	illuminate_controller_ = new IlluminateController(scene_manager, auv, ontology);
	follow_chain_controller_ = new ChainFollowController(scene_manager, auv, ontology, action_feedback_pub_, *height_map_);
	recharge_controller_ = new RechargeController(scene_manager, auv, ontology);
}

void ActionController::addListener(ActionExecutionListener& listener)
{
	listeners_.push_back(&listener);
}

void ActionController::removeListener(ActionExecutionListener& listener)
{
	for (std::vector<ActionExecutionListener*>::iterator i = listeners_.begin(); i != listeners_.end(); ++i)
	{
		if (*i == &listener)
		{
			listeners_.erase(i);
		}
	}
}

void ActionController::update(float dt)
{
	if (current_action_ == NULL)
	{
		return;
	}
	
	if (current_action_->getStatus() == PlannerAction::EXECUTING)
	{
		current_action_->update(dt);
	}
	
	// Stop execution and report a failed action to the planner.
	else if (current_action_->getStatus() == PlannerAction::FAILED)
	{
		for (std::vector<ActionExecutionListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
		{
			(*ci)->actionExecutionFailed(*current_action_);
		}
		//ROS_INFO("Action failed!");
		planning_msgs::ActionFeedback feedback;
		feedback.action_id = current_action_->getActionMsg().action_id;
		feedback.status = "action failed";
		
		if (label_ != NULL)
			label_->setLabel("REPLANNING");
		
		current_action_->amendFeedback(feedback, PlannerAction::FAILED);
		
		action_feedback_pub_.publish(feedback);

		current_action_ = NULL;
		return;
	}

	if (current_action_->getStatus() == PlannerAction::SUCCEEDED)
	{
		for (std::vector<ActionExecutionListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
		{
			(*ci)->actionExecutionSucceeded(*current_action_);
		}
		//ROS_INFO("Action completed!");
		planning_msgs::ActionFeedback feedback;
		feedback.action_id = current_action_->getActionMsg().action_id;
		feedback.status = "action achieved";
		
		current_action_->amendFeedback(feedback, PlannerAction::SUCCEEDED);
		
		action_feedback_pub_.publish(feedback);
		
		current_action_ = NULL;
		return;
	}
}

void ActionController::actionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg)
{
	if (label_ != NULL)
	{
		std::stringstream ss2;
		ss2 << auv_->getName() << " " << msg->name;
		label_->setLabel(ss2.str());
	}
	auv_->setVelocity(0);
	/*
	std::stringstream ss;
	ss << "Action: (d=" << msg->duration << ")" << msg->name << " [" << msg->action_id << "]" << std::endl;
	for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
	{
		ss << " - " << (*ci).key << " - " << (*ci).value << std::endl;
	}
	ROS_INFO(ss.str().c_str());
	*/
	//current_action_ = *msg;
	auv_->setLightOn(false);
	
	// Make the AUV move to the given location.
	if (GOTO_STRUCTURE_ACTION_NAME == msg->name)
	{
		glm::vec3 goal;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
		{
			if ("structure" == (*ci).key)
			{
				const std::string& structure_id = (*ci).value;
				for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
				{
					MissionSite* mission_site = *ci;
					if (mission_site->getStartWaypoint().id_ == structure_id)
					{
						goal = mission_site->getStartWaypoint().position_;
					}
				}
			}
		}
		
		follow_waypoint_controller_->followWaypoint(goal, false, 0, 0);
		
		follow_waypoint_controller_->setActionMsg(*msg);
		current_action_ = follow_waypoint_controller_;
	}
	else if (GOTO_ACTION_NAME == msg->name)
	{
		glm::vec3 goal;
		float pitch = 0;
		float yaw = 0;
		bool move_controlled = false;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
		{
			if ("north" == (*ci).key)
			{
				goal.z = ::atof((*ci).value.c_str());
			}
			else if ("east" == (*ci).key)
			{
				goal.x = ::atof((*ci).value.c_str());
			}
			else if ("depth" == (*ci).key)
			{
				goal.y = ::atof((*ci).value.c_str());
			}
			
			// Check if we have to moved controlled, if so we need to point the ship in a different direction.
			else if ("movement_type" == (*ci).key && "controlled" == (*ci).value)
			{
				move_controlled = true;
			}
			else if ("yaw" == (*ci).key)
			{
				yaw = ::atof((*ci).value.c_str()) * 180.0f / M_PI;
			}
			else if ("pitch" == (*ci).key)
			{
				pitch = ::atof((*ci).value.c_str()) * 180.0f / M_PI;
			}
		}
		
		follow_waypoint_controller_->followWaypoint(goal, move_controlled, pitch, yaw);
		
		follow_waypoint_controller_->setActionMsg(*msg);
		current_action_ = follow_waypoint_controller_;
	}
	else if (OBSERVE_ACTION_NAME == msg->name)
	{
		glm::vec3 goal;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
		{
			if ("ip_north" == (*ci).key)
			{
				goal.z = ::atof((*ci).value.c_str());
			}
			else if ("ip_east" == (*ci).key)
			{
				goal.x = ::atof((*ci).value.c_str());
			}
			else if ("ip_depth" == (*ci).key)
			{
				goal.y = ::atof((*ci).value.c_str());
			}
		}
		observe_controller_->observe(goal);
		observe_controller_->setActionMsg(*msg);
		current_action_ = observe_controller_;
	}
	else if (EXAMINE_PANEL_ACTION_NAME == msg->name)
	{
		std::string panel_id;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
		{
			if ("panel_id" == (*ci).key)
			{
				panel_id = (*ci).value;
			}
		}
		
		Structure* valve_panel = NULL;
		for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* vp = *ci;
				std::cout << "compare " << panel_id << " = " << vp->getName() << std::endl;
				if (vp->getName() == panel_id)
				{
					valve_panel = vp;
					break;
				}
			}
		}
		
		if (valve_panel == NULL)
		{
			std::cerr << "ERROR: Asked for a valve panel that does not exist!" << std::endl;
		}
		else
		{
			examine_panel_controller_->setValvePanel(*valve_panel);
			examine_panel_controller_->setActionMsg(*msg);
			current_action_ = examine_panel_controller_;
			//"valve_n_in_panel" : "[int]"
		}
	}
	else if (TURN_VALVE_ACTION_NAME == msg->name)
	{
		std::string valve_id;
		float desired_increment = 0;
		int valve_deadline = 0;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
		{
			if ("valve_id" == (*ci).key)
			{
				valve_id = (*ci).value;
			}
			else if ("desired_increment" == (*ci).key)
			{
				//desired_increment = ::atof((*ci).value.c_str()) * 180.0f / M_PI;
				desired_increment = ::atof((*ci).value.c_str());
			}
			else if ("valve_deadline" == (*ci).key)
			{
				valve_deadline = ::atof((*ci).value.c_str());
			}
		}
		
		Valve* valve = NULL;
		for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* vp = *ci;
				for (std::vector<Valve*>::const_iterator ci = vp->getValves().begin(); ci != vp->getValves().end(); ++ci)
				{
					Valve* v = *ci;
					std::cout << "compare " << valve_id << " = " << v->getName() << std::endl;
					if (v->getName() == valve_id)
					{
						valve = v;
						break;
					}
				}
			}
		}
		
		if (valve == NULL)
		{
			std::cerr << "ERROR: Asked for a valve panel that does not exist!" << std::endl;
		}
		else
		{
			valve_turn_controller_->setValve(*valve, desired_increment, valve_deadline);
			valve_turn_controller_->setActionMsg(*msg);
			current_action_ = valve_turn_controller_;
		}
	}
	else if (OBSERVE_PILLAR_NAME == msg->name)
	{
		glm::vec3 goal;
		observe_controller_->observe(goal);
		observe_controller_->setActionMsg(*msg);
		current_action_ = observe_controller_;
	}
	else if (ILLUMINATE_PILLAR_NAME == msg->name)
	{
		auv_->setLightOn(true);
		float duration = 0;
		illuminate_controller_->setDuration(msg->duration);
		illuminate_controller_->setActionMsg(*msg);
		current_action_ = illuminate_controller_;
	}
	else if (CANCEL_ACTION_NAME == msg->name)
	{
		planning_msgs::ActionFeedback feedback;
		feedback.action_id = msg->action_id;
		feedback.status = "action enabled";
		action_feedback_pub_.publish(feedback);
		
		ROS_INFO("Pretend the action is completed!");
		feedback.action_id = msg->action_id;
		feedback.status = "action failed";
		action_feedback_pub_.publish(feedback);
		
		current_action_ = NULL;
	}
	// TODO: Figure out which chain to follow.
	else if (FOLLOW_CHAIN_ACTION_NAME == msg->name)
	{
		Chain* matching_chain = NULL;
		for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
			{
				Chain* chain = *ci;
				matching_chain = chain;
			}
		}
		
		if (matching_chain == NULL)
		{
			std::cerr << "ERROR: Asked for a chain that does not exist!" << std::endl;
		}
		else
		{
			std::cout << "Found a chain to follow!" << std::endl;
			follow_chain_controller_->followChain(*matching_chain, msg->duration);
			current_action_ = follow_chain_controller_;
		}
	}
	else if (DOCK_AUV_ACTION_NAME == msg->name)
	{
		glm::vec3 goal = auv_->getGlobalLocation() + glm::vec3(0, 2.0f, 0);
		follow_waypoint_controller_->followWaypoint(goal, true, 0, 90.0f, 0.1f);
		follow_waypoint_controller_->setActionMsg(*msg);
		current_action_ = follow_waypoint_controller_;
	}
	else if (UNDOCK_AUV_ACTION_NAME == msg->name)
	{
		glm::vec3 goal = auv_->getGlobalLocation() + glm::vec3(0, -2.0f, 0);
		follow_waypoint_controller_->followWaypoint(goal, true, 0, 90.0f, 0.1f);
		follow_waypoint_controller_->setActionMsg(*msg);
		current_action_ = follow_waypoint_controller_;
	}
	else if (RECHARGE_ACTION_NAME == msg->name)
	{
		std::cout << "START RECHARGING!" << std::endl;
		recharge_controller_->setDuration(msg->duration);
		recharge_controller_->setActionMsg(*msg);
		current_action_ = recharge_controller_;
	}
	else
	{
		std::cerr << "Unknown action dispatched: " << msg->name << std::endl;
		planning_msgs::ActionFeedback feedback;
		feedback.action_id = msg->action_id;
		feedback.status = "action enabled";
		action_feedback_pub_.publish(feedback);
		
		current_action_ = observe_controller_;
		current_action_->setActionMsg(*msg);
		for (std::vector<ActionExecutionListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
		{
			(*ci)->actionExecutionStarted(*current_action_);
		}
		
		ROS_INFO("Pretend the action is completed!");
		feedback.action_id = msg->action_id;
		feedback.status = "action achieved";
		action_feedback_pub_.publish(feedback);
		
		// Temp.
		current_action_ = observe_controller_;
		current_action_->setActionMsg(*msg);
		for (std::vector<ActionExecutionListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
		{
			(*ci)->actionExecutionSucceeded(*current_action_);
		}
		
		current_action_ = NULL;
	}
	
	if (current_action_ != NULL)
	{
		for (std::vector<ActionExecutionListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
		{
			(*ci)->actionExecutionStarted(*current_action_);
		}
	}
	
	// Let the planner know that we have received the message.
	planning_msgs::ActionFeedback feedback;
	feedback.action_id = msg->action_id;
	feedback.status = "action enabled";
	action_feedback_pub_.publish(feedback);
}

void ActionController::notificationReceived(const knowledge_msgs::Notification::ConstPtr& msg)
{
	if (label_ != NULL)
		label_->setLabel("REPLANNING");
	/*
	if (current_action_ == NULL)
	{
		return;
	}
	
	planning_msgs::ActionFeedback feedback;
	
	feedback.action_id = current_action_->getActionMsg().action_id;
	feedback.status = "action failed";
	action_feedback_pub_.publish(feedback);
	
	current_action_ = NULL;
	*/
}
