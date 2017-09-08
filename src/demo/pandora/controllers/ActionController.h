#ifndef DEMO_PANDORA_CONTROLLERS_ACTION_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_ACTION_CONTROLLER_H

#include <ros/ros.h>

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/ActionFeedback.h>
#include <knowledge_msgs/Notification.h>

class ActionLabel;
class ObserveController;
class FollowWaypointController;
class AUV;
class PlannerAction;

namespace DreadedPE
{
	class SceneManager;
	class HeightMap;
};

class OntologyInterface;
class ActionExecutionListener;
class ExaminePanelController;
class ValveTurnController;
class IlluminateController;
class ActionLabel;
class ChainFollowController;
class RechargeController;

/**
 * Class that listens to the actions being send by the planner and executes them.
 */
class ActionController
{
public:
	/**
	 * Constructor.
	 */
	ActionController(DreadedPE::SceneManager& scene_manager, ros::NodeHandle& node_handle, AUV& auv, DreadedPE::HeightMap& height_map, FollowWaypointController& follow_waypoint_controller, OntologyInterface& ontology, ActionLabel* label = NULL);
	
	/**
	 * Check the progress of the current action. Then either execute the action or report a success or 
	 * failed state to the planner.
	 * @param dt The amount of time that has passed since the last frame.
	 */
	void update(float dt);
	
	/**
	 * Add an execution listener.
	 * @param listener The listener.
	 */
	void addListener(ActionExecutionListener& listener);
	
	/**
	 * Remove an execution listener.
	 * @param listener The listener.
	 */
	void removeListener(ActionExecutionListener& listener);
	
private:
	
	/**
	 * This function is called everytime an action is dispatched.
	 */
	void actionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg);
	
	/**
	 * Everytime a triggered in the ontology fires, we get notified. The planner currently replans 
	 * for every notification received and does not let us know that it is replanning. So we do 
	 * this manually now. NOTE needs to change later on.
	 * @param msg The message containing the notification of which filter triggered.
	 */
	void notificationReceived(const knowledge_msgs::Notification::ConstPtr& msg);
	
	ros::NodeHandle* node_handle_; /// The ROS node handle.
	AUV* auv_;
	DreadedPE::HeightMap* height_map_;
	FollowWaypointController* follow_waypoint_controller_;
	ObserveController* observe_controller_;
	ExaminePanelController* examine_panel_controller_;
	ValveTurnController* valve_turn_controller_;
	IlluminateController* illuminate_controller_;
	ChainFollowController* follow_chain_controller_;
	RechargeController* recharge_controller_;
	
	OntologyInterface* ontology_;
	
	static const std::string GOTO_ACTION_NAME;          /// The action name for goto actions.
	static const std::string GOTO_STRUCTURE_ACTION_NAME;/// The action name for moving between mission sites.
	static const std::string OBSERVE_ACTION_NAME;       /// The action name for observation actions.
	static const std::string EXAMINE_PANEL_ACTION_NAME; /// The action name for examine panel actions.
	static const std::string TURN_VALVE_ACTION_NAME;    /// The action name for turning valve actions.
	static const std::string CANCEL_ACTION_NAME;        /// The action name for when the dispatcher times out.
	static const std::string ILLUMINATE_PILLAR_NAME;    /// The action name for illuminating a pillar.
	static const std::string OBSERVE_PILLAR_NAME;       /// The action name for observing a pillar.
	static const std::string FOLLOW_CHAIN_ACTION_NAME;  /// The action name for following a chain.
	static const std::string DOCK_AUV_ACTION_NAME;      /// The action name for docking the AUV.
	static const std::string UNDOCK_AUV_ACTION_NAME;    /// The action name for undocking the AUV.
	static const std::string RECHARGE_ACTION_NAME;      /// The action name for recharging the AUV.
	
	ros::Subscriber action_dispatch_sub_; /// The subscriber that listens to action dispatches.
	
	// Possible feedbacks:
	// "action enabled"
	// "action achieved"
	//
	// Specific:
	// * goto:
	//  "action failed"
	// * observe
	//  ?
	// * valve_state
	//  ?
	// * turn_valve
	//  ?
	ros::Publisher action_feedback_pub_; /// The publisher that publishes feedback back to the planner.
	ros::Subscriber notification_sub_; /// Listen to the notifications of filters being triggered in the ontology. 
	
	PlannerAction* current_action_;
	ActionLabel* label_;
	
	std::vector<ActionExecutionListener*> listeners_; /// List if listeners that get informed of the execution process.
};

#endif
