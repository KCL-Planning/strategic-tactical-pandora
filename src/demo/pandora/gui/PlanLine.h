#ifndef DEMO_PANDORA_GUI_PLAN_LINE_H
#define DEMO_PANDORA_GUI_PLAN_LINE_H

#ifndef _WIN32
#include <ros/ros.h>

#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/CompletePlan.h>
#endif

#include <map>

#include "dpengine/gui/Frame.h"
#include "dpengine/gui/Container.h"
#include "dpengine/gui/events/ButtonPressedListener.h"

#include "../controllers/ActionExecutionListener.h"
#include "../controllers/PlannerAction.h"

class AUV;
class ActionLabel;

namespace DreadedPE
{
	class Font;
	class Button;
	class Texture;
};

/**
 * Draws a plan line.
 */
class PlanLine : public DreadedPE::Container, public ActionExecutionListener, public DreadedPE::ButtonPressedListener
{
public:
	PlanLine(ros::NodeHandle& ros_node, AUV* auv, float pixels_per_second, const DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, float size_x, float size_y);

	void actionExecutionStarted(const PlannerAction& action);
	void actionExecutionFailed(const PlannerAction& action);
	void actionExecutionSucceeded(const PlannerAction& action);
	void update(float dt);
	
	/**
	 * Update the location and size of the plan line to fill the bottom row.
	 */
	void onResize(float width, float height);
	
	/**
	 * Handle button presses.
	 */
	void buttonPressed(const DreadedPE::Button& source);
	
	/**
	 * Change the pixels per seconds and update the graph to reflect this change.
	 */
	void setPixelsPerSecond(float pixels_per_second);

	/**
	 * Set the current plan that is being executed.
	 */
	void setCurrentPlan(const planning_msgs::CompletePlan& msg);
	
	/**
	 * Get all the action labels.
	 */
	const std::vector<ActionLabel*>& getActionLabels() const { return action_labels_; }
	
private:
	AUV* auv_;
	DreadedPE::Font* font_;
	float total_time_;
	float start_time_current_action_;
	float pixels_per_second_;
	
	//std::map<std::string, float> plan_line_time_;
	
	ros::Subscriber complete_plan_listener_; /// Listen to messages that contain the complete current plan.
	planning_msgs::ActionDispatch current_action_;

	std::vector<ActionLabel*> action_labels_;
	
	DreadedPE::Texture* icon_texture_;
	
	static int INVALID_ACTION_ID_;
	
};
#endif
