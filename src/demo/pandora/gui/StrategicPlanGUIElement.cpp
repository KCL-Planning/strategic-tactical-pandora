#include "StrategicPlanGUIElement.h"

#include <GL/glfw.h>
#include <sstream>
#include <math.h>

#include "../../../shapes/Line.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../core/gui/Label.h"
#include "../../../core/gui/Button.h"
#include "../../../core/gui/Scrollbar.h"
#include "../../../core/gui/fonts/TexturedFont.h"

#include "PlanLine.h"
#include "PlanGraph.h"

StrategicPlanGUIElement::StrategicPlanGUIElement(ros::NodeHandle& ros_node, Theme& theme, Font& font, float x, float y, float size_x, float size_y, float pixels_per_second)
	: Container(theme, font, x, y, size_x, size_y, true), ros_node_(&ros_node), font_(&font), plan_line_heights_(30), pixels_per_second_(pixels_per_second), start_time_(0), running_time_(0), total_planning_time_(0)
{
	std::cout << pixels_per_second_ << std::endl;
	int width, height;
	glfwGetWindowSize(&width, &height);
	
	plan_graph_ = new PlanGraph(theme, font.clone(), 0, 0, size_x - 20, 60, pixels_per_second_);
	addElement(*plan_graph_, 0, -size_y + 60);
	scrollbar_ = new Scrollbar(theme, font.clone(), 0, 0, size_x, 10, *plan_graph_, false);
	addElement(*scrollbar_, 10, -size_y + 10);
	
	//setPosition(0, -height + size_y);
	setPosition(0, -50);
	
	complete_plan_listener_ = ros_node.subscribe("/planning_system/current_strategic_plan", 1, &StrategicPlanGUIElement::setCurrentPlan, this);
	strategic_plan_dispatcher_ = ros_node.subscribe("/planning_system/strategic_action_dispatch", 1, &StrategicPlanGUIElement::actionDispatch, this);
	
	plan_line_ = new PlanLine(*ros_node_, NULL, pixels_per_second_, *theme_, font_->clone(), 0, 0, size_x_, plan_line_heights_);
	plan_graph_->addElement(*plan_line_, 0, -plan_graph_->getContentSizeY() + plan_line_heights_);
	
	planning_msgs::ActionDispatch ad;
	ad.action_id = -1;
	previous_planner_action_.setActionMsg(ad);
}

void StrategicPlanGUIElement::draw(const glm::mat4& perspective_matrix, int level) const
{
	Container::draw(perspective_matrix, level);
}

void StrategicPlanGUIElement::onResize(float width, float height)
{
	std::cout << "StrategicPlanGUIElement::onResize!(" << width << ", " << height << ")" << std::endl;
	size_x_ = width;
	size_y_ = 60;
	
	scrollbar_->setDimensions(width, 10);
	
	plan_line_->onResize(width, height);
	
	plan_graph_->setDimensions(width, plan_line_heights_);
	plan_graph_->onResize(width, height);
	updateTransformations();
	Container::onResize(width, height);
}

void StrategicPlanGUIElement::actionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg)
{
	if (previous_planner_action_.getActionMsg().action_id != -1)
	{
		plan_line_->actionExecutionSucceeded(previous_planner_action_);
		std::cout << "FINISHED The action: " << previous_planner_action_.getActionMsg().action_id << std::endl;
	}
	
	planning_msgs::ActionDispatch ad = *msg;
	ad.action_id = previous_planner_action_.getActionMsg().action_id + 1;
	
	PlannerAction pa;
	pa.setActionMsg(ad);
	plan_line_->actionExecutionStarted(pa);
	
	previous_planner_action_ = pa;
	
	std::cout << "STARTED The action: " << previous_planner_action_.getActionMsg().action_id << std::endl;
}

void StrategicPlanGUIElement::update(float dt)
{
	running_time_ += dt;
	// Move the plan graph such that the time line is in the centre.
	//float time_from_begining = running_time_ - start_time_;
	float time_from_begining = running_time_;
	
	// Check if need to move the timeline to the beginning.
	float min_time = (size_x_ / 2.0f) / pixels_per_second_;
	float max_time = total_planning_time_ - size_x_ / pixels_per_second_;
	
	if (time_from_begining < min_time)
	{
		plan_graph_->setPosition(0, plan_graph_->getLocalY());
	}/*
	else if (time_from_begining > max_time)
	{
		plan_graph_->setPosition(-max_time * pixels_per_second_, plan_graph_->getLocalY());
	}*/
	else
	{
		float time_in_pixels_in_middle = time_from_begining * pixels_per_second_ - size_x_ / 2.0f;
		plan_graph_->setPosition(-time_in_pixels_in_middle, plan_graph_->getLocalY());
	}
	Container::update(dt);
}

void StrategicPlanGUIElement::setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg)
{
	start_time_ = 0;
	running_time_ = 0;
	total_planning_time_ = 0;
	
	planning_msgs::ActionDispatch ad;
	ad.action_id = -1;
	previous_planner_action_.setActionMsg(ad);
	
	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg->actions.begin(); ci != msg->actions.end(); ++ci)
	{
		planning_msgs::ActionDispatch ad = *ci;
		if (total_planning_time_ < ad.dispatch_time + ad.duration)
		{
			total_planning_time_ = ad.dispatch_time + ad.duration;
		}
	}
	
	planning_msgs::CompletePlan complete_plan = *msg;
	unsigned int action_id = 0;
	for (std::vector<planning_msgs::ActionDispatch>::iterator ci = complete_plan.actions.begin(); ci != complete_plan.actions.end(); ++ci)
	{
		planning_msgs::ActionDispatch& ad = *ci;
		ad.action_id = action_id;
		++action_id;
		std::cout << "[" << ad.action_id << "] " << ad.name << " " << ad.dispatch_time << " d=" << ad.duration << std::endl;
	}
	plan_graph_->resetTimeLine();
	plan_line_->setCurrentPlan(complete_plan);
}
