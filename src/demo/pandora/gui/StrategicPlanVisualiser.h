#ifndef DEMO_PANDORA_GUI_STRATEGIC_PLAN_VISUALISER_H
#define DEMO_PANDORA_GUI_STRATEGIC_PLAN_VISUALISER_H

#include <memory>
#include <ros/ros.h>

#include <planning_msgs/CompletePlan.h>

#include <dpengine/scene/SceneNode.h>
#include "../controllers/ActionExecutionListener.h"

namespace DreadedPE
{
	class SceneLeafModel;
	class SceneManager;
	class Theme;
	class Font;
	class Camera;
	class Line;
};

class AUV;
class OntologyInterface;
class BillBoard;
class PlannerAction;

class StrategicPlanVisualiser : public DreadedPE::SceneNode, public ActionExecutionListener
{
public:
	StrategicPlanVisualiser(ros::NodeHandle& ros_node, AUV& auv, OntologyInterface& ontology, DreadedPE::SceneNode& parent, DreadedPE::SceneManager& scene_manager, DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::Camera& camera);
	
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
	DreadedPE::SceneLeafModel* path_;
	std::shared_ptr<DreadedPE::Line> line_;
	
	std::map<int, BillBoard*> active_bill_boards_;
	DreadedPE::Theme* theme_;
	DreadedPE::Font* font_;
	DreadedPE::Camera* camera_;
	
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
};

#endif
