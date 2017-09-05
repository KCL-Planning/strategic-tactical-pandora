#ifndef DEMO_PANDORA_GUI_PLAN_LINE_H
#define DEMO_PANDORA_GUI_PLAN_LINE_H

#ifndef _WIN32
#include <ros/ros.h>

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/CompletePlan.h>
#endif

#include <map>

#include "../../../core/gui/Frame.h"
#include "../../../core/gui/Container.h"
#include "../../../core/gui/events/ButtonPressedListener.h"

#ifndef _WIN32
#include "../controllers/ActionExecutionListener.h"
#include "../controllers/PlannerAction.h"
#endif

class AUV;
class ActionLabel;
class Font;
class Button;
class Texture;

/**
 * Draws a plan line.
 */
#ifndef _WIN32
class PlanLine : public Container, public ActionExecutionListener, public ButtonPressedListener
#else
class PlanLine : public Frame
#endif
{
public:
#ifndef _WIN32
	PlanLine(ros::NodeHandle& ros_node, AUV* auv, float pixels_per_second, const Theme& theme, Font& font, float x, float y, float size_x, float size_y);
#else
	PlanLine(Theme& theme, Font& font, float x, float y, float size_x, float size_y);
#endif

#ifndef _WIN32
	void actionExecutionStarted(const PlannerAction& action);
	void actionExecutionFailed(const PlannerAction& action);
	void actionExecutionSucceeded(const PlannerAction& action);
#endif
	void update(float dt);
	
	/**
	 * Update the location and size of the plan line to fill the bottom row.
	 */
	void onResize(float width, float height);
	
	/**
	 * Handle button presses.
	 */
	void buttonPressed(const Button& source);
	
	/**
	 * Change the pixels per seconds and update the graph to reflect this change.
	 */
	void setPixelsPerSecond(float pixels_per_second);

	/**
	 * Set the current plan that is being executed.
	 */
	void setCurrentPlan(const planning_msgs::CompletePlan& msg);
	
private:
	AUV* auv_;
	Font* font_;
	float total_time_;
	float start_time_current_action_;
	float pixels_per_second_;
	
	//std::map<std::string, float> plan_line_time_;
	
#ifndef _WIN32
	
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
	planning_msgs::ActionDispatch current_action_;
#endif	
	std::vector<ActionLabel*> action_labels_;
	
	Texture* icon_texture_;
	
	static int INVALID_ACTION_ID_;
	
};
#endif
