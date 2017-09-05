#ifndef DEMO_PANDORA_GUI_PLANNING_GUI_H
#define DEMO_PANDORA_GUI_PLANNING_GUI_H

#include <ros/ros.h>
#include <vector>

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/CompletePlan.h>

#include "../../../core/gui/Container.h"
#include "../controllers/ActionExecutionListener.h"

class AUV;
class Font;
class Label;
class Line;
class PlanLine;
class PlanGraph;

/**
 * This GUI covers the entire screen and handles all planning related GUI elements.
 */
class PlanningGUI : public Container, public ActionExecutionListener
{
public:
	PlanningGUI(ros::NodeHandle& ros_node, Theme& theme, Font& font, float x, float y, float size_x, float size_y, float pixels_per_seconds);
	//virtual ~PlanningGUI();
	void update(float dt);
	
	void setPixelsPerSecond(float pixels_per_second) { pixels_per_second_ = pixels_per_second; }
	
	void draw(const glm::mat4& perspective_matrix, int level) const;
	
	/**
	 * Add a plan line that displays the plan for a specific AUV.
	 * @param auv The AUV we want to visualise the plan for.
	 */
	void addPlanLine(AUV& auv);
	
	/**
	 * Make sure that the elements are in the right position after the screen gets updated.
	 */
	void onResize(float width ,float height);
	
	/**
	 * Handle button presses.
	 */
	//void buttonPressed(const Button& source);
	
	/**
	 * Callback functions for the Action Execution Listener.
	 */
	void actionExecutionStarted(const PlannerAction& action);
	void actionExecutionFailed(const PlannerAction& action);
	void actionExecutionSucceeded(const PlannerAction& action);
	
private:
	ros::NodeHandle* ros_node_;
	
	Line* time_markers_;
	Line* current_time_line_;
	Font* font_;
	float plan_line_heights_;
	
	PlanGraph* plan_graph_;
	//Scrollbar* scrollbar_;
	
	std::vector<PlanLine*> planlines_;
	
	float pixels_per_second_;
	float start_time_;
	float running_time_;
	float total_planning_time_;
	
	bool hide_till_first_action_;
	
	//Button* increase_button_;
	//Button* decrease_button_;
	
	void strategicActionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg);
	void setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg);
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
	ros::Subscriber strategic_dispatch_listener_;
};

#endif
