#ifndef DEMO_PANDORA_GUI_PLAN_VISUALISER_H
#define DEMO_PANDORA_GUI_PLAN_VISUALISER_H

#include <memory>
#include <ros/ros.h>

#include <planning_msgs/CompletePlan.h>

#include <dpengine/scene/SceneNode.h>
#include "../controllers/ActionExecutionListener.h"

namespace DreadedPE
{
	class Line;
	class SceneLeafModel;
	class Theme;
	class Font;
	class Camera;
};

class AUV;
class OntologyInterface;
class BillBoard;
class PlannerAction;

class PlanVisualiser : public DreadedPE::SceneNode, public ActionExecutionListener
{
public:
	PlanVisualiser(ros::NodeHandle& ros_node, AUV& auv, OntologyInterface& ontology, DreadedPE::SceneNode& parent, DreadedPE::SceneManager& scene_manager, DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::Camera& camera);
	
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
	
	std::shared_ptr<DreadedPE::Line> bad_line_;
	DreadedPE::SceneLeafModel* bad_path_;
	
	std::shared_ptr<DreadedPE::Line> filter_line_;
	DreadedPE::SceneLeafModel* filter_path_;
	
	//std::vector<BillBoard*> active_bill_boards_;
	std::map<int, BillBoard*> active_bill_boards_;
	DreadedPE::Theme* theme_;
	DreadedPE::Font* font_;
	DreadedPE::Camera* camera_;
	
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
	
	ros::Subscriber complete_strategic_plan_listener_; /// Listen to messages that contain the complete strategic plan.
	
	ros::Publisher rviz_plan_publisher_; /// Publisher that publishes a path that is visible in rviz.
};

#endif
