#ifndef PANDORA_GUI_STRATEGIC_PLAN_GUI_ELEMENT_H
#define PANDORA_GUI_STRATEGIC_PLAN_GUI_ELEMENT_H

#include <ros/ros.h>
#include <glm/glm.hpp>

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/CompletePlan.h>

#include "../../../core/gui/Container.h"

#include "../controllers/PlannerAction.h"

class AUV;
class Theme;
class Font;
class Line;
class PlanGraph;
class Scrollbar;
class PlanLine;

class StrategicPlanGUIElement : public Container
{
public:
	StrategicPlanGUIElement(ros::NodeHandle& ros_node, Theme& theme, Font& font, float x, float y, float size_x, float size_y, float pixels_per_seconds);
	void update(float dt);
	
	void setPixelsPerSecond(float pixels_per_second) { pixels_per_second_ = pixels_per_second; }
	
	void draw(const glm::mat4& perspective_matrix, int level) const;
	
	void actionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg);
	
	/**
	 * Make sure that the elements are in the right position after the screen gets updated.
	 */
	void onResize(float width ,float height);
	
private:
	ros::NodeHandle* ros_node_;
	
	Line* time_markers_;
	Line* current_time_line_;
	Font* font_;
	float plan_line_heights_;
	
	PlanGraph* plan_graph_;
	Scrollbar* scrollbar_;
	
	PlanLine* plan_line_;
	
	float pixels_per_second_;
	float start_time_;
	float running_time_;
	float total_planning_time_;
	
	void setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg);
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
	ros::Subscriber strategic_plan_dispatcher_;
	
	PlannerAction previous_planner_action_;
};

#endif
