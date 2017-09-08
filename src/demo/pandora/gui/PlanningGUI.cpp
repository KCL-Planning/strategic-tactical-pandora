#include "PlanningGUI.h"

#include <sstream>
#include <math.h>

#include "dpengine/shapes/Line.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/gui/Label.h"
#include "dpengine/gui/Button.h"
#include "dpengine/gui/Scrollbar.h"
#include "dpengine/gui/fonts/TexturedFont.h"
#include <dpengine/renderer/Window.h>

#include "PlanLine.h"
#include "PlanGraph.h"

PlanningGUI::PlanningGUI(ros::NodeHandle& ros_node, DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, float size_x, float size_y, float pixels_per_second)
	: DreadedPE::Container(theme, font, x, y, size_x, size_y, true), ros_node_(&ros_node), font_(&font), plan_line_heights_(30), pixels_per_second_(pixels_per_second), start_time_(0), running_time_(0), total_planning_time_(0), hide_till_first_action_(false)
{
	std::cout << pixels_per_second_ << std::endl;
	int width, height;
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->getSize(width, height);
	
	
	// Add buttons to reduce / increase the pixels / second.
	//increase_button_ = new Button(theme, 10, 10, "P", 12);
	//decrease_button_ = new Button(theme, 10, 10, "M", 12);
	
	//addElement(*increase_button_, size_x - 15, -size_y_ + 20);
	//addElement(*decrease_button_, size_x - 15, -size_y_ + 40);
	
	plan_graph_ = new PlanGraph(theme, font.clone(), 0, 0, size_x - 20, 60, pixels_per_second_);
	addElement(*plan_graph_, 0, -size_y + 60);
	//scrollbar_ = new Scrollbar(theme, font.clone(), 0, 0, size_x, 10, *plan_graph_, false);
	//addElement(*scrollbar_, 10, -size_y + 10);
	
	//increase_button_->addListener(*this);
	//decrease_button_->addListener(*this);
	
	setPosition(0, -height + size_y);
	
	complete_plan_listener_ = ros_node.subscribe("/planning_system/current_plan", 1, &PlanningGUI::setCurrentPlan, this);
	strategic_dispatch_listener_ = ros_node.subscribe("/planning_system/strategic_action_dispatch", 1, &PlanningGUI::strategicActionDispatch, this);
}

void PlanningGUI::addPlanLine(AUV& auv)
{
 	PlanLine* plan_line = new PlanLine(*ros_node_, &auv, pixels_per_second_, *theme_, font_->clone(), 0, 0, size_x_, plan_line_heights_);
	plan_graph_->addElement(*plan_line, 0, -plan_graph_->getContentSizeY() + plan_line_heights_ * (planlines_.size() + 1));
	planlines_.push_back(plan_line);
}

void PlanningGUI::draw(const glm::mat4& perspective_matrix, int level) const
{
	DreadedPE::Container::draw(perspective_matrix, level);
}

void PlanningGUI::onResize(float width, float height)
{
	std::cout << "PlanningGUI::onResize!(" << width << ", " << height << ")" << std::endl;
	size_x_ = width;
	size_y_ = 60;
	
	setPosition(0, -height + size_y_);

	//scrollbar_->setPosition(0, -size_y_ + 10);
	//scrollbar_->setDimensions(width, 10);
	
	plan_graph_->setPosition(0, (1 + planlines_.size()) * plan_line_heights_ - size_y_);
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		(*ci)->onResize(width, height);
	}
	plan_graph_->setDimensions(width, (1 + planlines_.size()) * plan_line_heights_);
	plan_graph_->onResize(width, height);
	
	updateTransformations();
	DreadedPE::Container::onResize(width, height);
	plan_graph_->setVisible(false);
}
/*
void PlanningGUI::buttonPressed(const Button& source)
{
	if (&source == decrease_button_ && pixels_per_second_ > 1)
	{
		pixels_per_second_ -= 1;
	}
	else if (&source == increase_button_)
	{
		pixels_per_second_ += 1;
	}
	
	// Update the labels to reflect this change.
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		PlanLine* plan_line = *ci;
		plan_line->setPixelsPerSecond(pixels_per_second_);
	}
	plan_graph_->setPixelsPerSecond(pixels_per_second_);
}
*/
void PlanningGUI::actionExecutionStarted(const PlannerAction& action)
{
	if (hide_till_first_action_)
	{
		plan_graph_->setVisible(true);
		hide_till_first_action_ = false;
	}
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		PlanLine* plan_line = *ci;
		plan_line->actionExecutionStarted(action);
	}
}

void PlanningGUI::actionExecutionFailed(const PlannerAction& action)
{
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		PlanLine* plan_line = *ci;
		plan_line->actionExecutionFailed(action);
	}
}

void PlanningGUI::actionExecutionSucceeded(const PlannerAction& action)
{
	std::cout << "An action has been succeeded! " << plan_graph_->getTime() << std::endl;
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		PlanLine* plan_line = *ci;
		plan_line->actionExecutionSucceeded(action);
	}
}

void PlanningGUI::update(float dt)
{
	running_time_ += dt;
	// Move the plan graph such that the time line is in the centre.
	//float time_from_begining = running_time_ - start_time_;
	float time_from_begining = running_time_;
	
	// Check if need to move the timeline to the beginning.
	float min_time = (size_x_ / 2.0f) / pixels_per_second_;
	float max_time = total_planning_time_ - size_x_ / pixels_per_second_;
	
	//std::cout << "[" << running_time_ << "] Time from beginning: " << time_from_begining << " total plan time: " << total_planning_time_ << " (" << min_time << ", " << max_time << ")" << std::endl;
	
	if (time_from_begining < min_time)
	{
		plan_graph_->setPosition(0, plan_graph_->getLocalY());
		plan_graph_->markForUpdate();
		plan_graph_->updateBuffers();
		
		for (PlanLine* plan_line : planlines_)
		{
			plan_line->setPosition(0, plan_line->getLocalY());
			plan_line->markForUpdate();
			plan_line->updateBuffers();
		}
		//std::cout << "Set position to 0." << std::endl;
	}/*
	else if (time_from_begining > max_time)
	{
		plan_graph_->setPosition(-max_time * pixels_per_second_, plan_graph_->getLocalY());
		//std::cout << "Set position to " << max_time * pixels_per_second_ << "." << std::endl;
	}*/
	else
	{
		float time_in_pixels_in_middle = time_from_begining * pixels_per_second_ - size_x_ / 2.0f;
		plan_graph_->setPosition(-time_in_pixels_in_middle, plan_graph_->getLocalY());
		plan_graph_->markForUpdate();
		plan_graph_->updateBuffers();
		
		for (PlanLine* plan_line : planlines_)
		{
			plan_line->setPosition(-time_in_pixels_in_middle, plan_line->getLocalY());
			plan_line->markForUpdate();
			plan_line->updateBuffers();
		}
		//std::cout << "Set position to " << time_in_pixels_in_middle << "." << std::endl;
	}
	
	Container::update(dt);
	markForUpdate();
	updateBuffers();
}

void PlanningGUI::strategicActionDispatch(const planning_msgs::ActionDispatch::ConstPtr& msg)
{
	if ("complete_mission" != msg->name)
	{
		plan_graph_->setVisible(false);
		hide_till_first_action_ = false;
	}
	else
	{
		hide_till_first_action_ = true;
	}
}

void PlanningGUI::setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg)
{
	start_time_ = 0;
	running_time_ = 0;
	total_planning_time_ = 0;
	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg->actions.begin(); ci != msg->actions.end(); ++ci)
	{
		planning_msgs::ActionDispatch ad = *ci;
		if (total_planning_time_ < ad.dispatch_time + ad.duration)
		{
			total_planning_time_ = ad.dispatch_time + ad.duration;
		}
	}
	
	for (std::vector<PlanLine*>::const_iterator ci = planlines_.begin(); ci != planlines_.end(); ++ci)
	{
		PlanLine* plan_line = *ci;
		plan_line->setCurrentPlan(*msg);
	}
	
	plan_graph_->resetTimeLine();
}
