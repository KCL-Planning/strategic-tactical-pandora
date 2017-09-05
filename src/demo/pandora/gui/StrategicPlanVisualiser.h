#ifndef DEMO_PANDORA_GUI_STRATEGIC_PLAN_VISUALISER_H
#define DEMO_PANDORA_GUI_STRATEGIC_PLAN_VISUALISER_H

#include <ros/ros.h>

#include <planning_msgs/CompletePlan.h>

#include "../../../core/scene/SceneNode.h"
#include "../controllers/ActionExecutionListener.h"

class AUV;
class Line;
class OntologyInterface;
class SceneLeafModel;
class BillBoard;
class Theme;
class Font;
class Camera;
class PlannerAction;

class StrategicPlanVisualiser : public SceneNode, public ActionExecutionListener
{
public:
	StrategicPlanVisualiser(ros::NodeHandle& ros_node, AUV& auv, OntologyInterface& ontology, SceneNode& parent, SceneManager& scene_manager, Theme& theme, Font& font, Camera& camera);
	
#ifndef _WIN32
	void actionExecutionStarted(const PlannerAction& action);
	void actionExecutionFailed(const PlannerAction& action);
	void actionExecutionSucceeded(const PlannerAction& action);
#endif
	
private:
	
	/**
	 * Every time a new plan is generated, this method is called.
	 * @param plan The new action sequence of the current plan.
	 */
	void setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg);
	
	
	AUV* auv_;
	OntologyInterface* ontology_;
	SceneLeafModel* path_;
	Line* line_;
	
	std::map<int, BillBoard*> active_bill_boards_;
	Theme* theme_;
	Font* font_;
	Camera* camera_;
	
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
};

#endif
