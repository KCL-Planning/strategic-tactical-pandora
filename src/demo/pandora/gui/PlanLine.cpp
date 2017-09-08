#include "PlanLine.h"
#include <sstream>
#include <map>

#include <stdlib.h> 

#include "dpengine/gui/GUIManager.h"
#include "dpengine/gui/Frame.h"
#include "dpengine/gui/Container.h"
#include "ActionLabel.h"
#include "dpengine/gui/Button.h"
#include "dpengine/gui/Label.h"
#include "dpengine/gui/Scrollbar.h"
#ifndef _WIN32
#include "../controllers/PlannerAction.h"
#endif
#include "dpengine/gui/fonts/Font.h"
#include "dpengine/gui/Canvas.h"
#include "dpengine/texture/TargaTexture.h"
#include "../AUV.h"
#include "PlanGraph.h"
#include "AUVStatusIcon.h"

int PlanLine::INVALID_ACTION_ID_ = -123456;

/**
 * Visualiser the abstract plan and the expanded plan. The user is able to click though logs and see previous executions
 * of plans and eximine their contents. We also visualise the 'free time' built up during execution.
 */
PlanLine::PlanLine(ros::NodeHandle& ros_node, AUV* auv, float pixels_per_second, const DreadedPE::Theme& theme, DreadedPE::Font& font, float x, float y, float size_x, float size_y)
	: DreadedPE::Container(theme, font, x, y, size_x, size_y, false), auv_(auv), font_(&font), total_time_(0.0f), start_time_current_action_(0.0f), pixels_per_second_(pixels_per_second)
{
	//GUIManager& gui_manager = GUIManager::getInstance();
	//gui_manager.addFrame(*this);
//	return;
#ifndef _WIN32	
	//complete_plan_listener_ = ros_node.subscribe("/planning_system/current_plan", 1, &PlanLine::setCurrentPlan, this);
	//complete_plan_listener_ = ros_node.subscribe("/planning_system/current_strategic_plan", 1, &PlanLine::setCurrentPlan, this);
#endif
	current_action_.action_id = INVALID_ACTION_ID_;
	/*
	planning_msgs::ActionDispatch action_dispatch;
	ActionLabel* label = new ActionLabel(*theme_, font_->clone(), glm::vec4(1, 1, 1, 1), 20 * pixels_per_second_ - 2, size_y_, "TEST", action_dispatch);
	
	label->setStartTime(total_time_ + 10);
	label->setDuration(100);
	addElement(*label, (total_time_ + 10) * pixels_per_second_, -2.5f);
	action_labels_.push_back(label);
	*/
	icon_texture_ = DreadedPE::TargaTexture::loadTexture("data/textures/icons.tga");
}

#ifndef _WIN32
void PlanLine::actionExecutionStarted(const PlannerAction& action)
{
	start_time_current_action_ = total_time_;
	std::cout << "Started plan #" << action.getActionMsg().action_id << " duration=" << action.getActionMsg().duration << std::endl;
	current_action_ = action.getActionMsg();

	// Move all the labels such that the begin point of the action corresponds to 'now'.
	bool post_label = false;
	float time = total_time_;
	
	//for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	for (unsigned int i = 0; i < action_labels_.size(); ++i)
	{
		ActionLabel* action_label = action_labels_[i];
		if (action_label->getAction().action_id == action.getActionMsg().action_id)
		{
			std::cout << "Set the start of the execution of this action at: " << time << std::endl;
			action_label->setPosition(time * pixels_per_second_, -2.5f);
			action_label->updateBuffers();
			post_label = true;
		}
		
		if (post_label)
		{
			// Calculate the "gap" between this action and the next action.
			float delta = 0;
			
			if (i > 0)
			{
				delta = action_label->getAction().dispatch_time - (action_labels_[i - 1]->getAction().dispatch_time + action_labels_[i - 1]->getAction().duration);
			}
			
			if (action_label->getAction().action_id != action.getActionMsg().action_id &&
			    ("turn_valve" == action_label->getAction().name ||
				  "undock_auv" == action_label->getAction().name))
			{
				time = std::max(action_label->getAction().dispatch_time, time);
				delta = 0;
			}
			
			action_label->setPosition(time * pixels_per_second_, -2.5f);
			action_label->updateBuffers();
			time += action_label->getAction().duration + delta;
			action_label->setStartTime(time);
		}
	}
}

void PlanLine::actionExecutionFailed(const PlannerAction& action)
{
	bool post_failed_action = false;
	for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	{
		ActionLabel* action_label = *ci;
		
		if (post_failed_action)
		{
			action_label->setVisible(false);
		}
		
		if (action_label->getAction().action_id == action.getActionMsg().action_id)
		{
			//action_label->setDimensions(size_x_ / 2.0f - 2.5f, (total_time_ - start_time_current_action_) * pixels_per_second_);
			action_label->setDimensions((total_time_ - start_time_current_action_) * pixels_per_second_, size_y_);
			action_label->setDuration(total_time_ - start_time_current_action_);
			action_label->updateBuffers();
			post_failed_action = true;
		}
	}
	current_action_.action_id = INVALID_ACTION_ID_;
}

void PlanLine::actionExecutionSucceeded(const PlannerAction& action)
{
	bool post_label = false;
	float time = total_time_;
	std::cout << "Time now is: " << time << std::endl;
	for (unsigned int i = 0; i < action_labels_.size(); ++i)
	{
		ActionLabel* action_label = action_labels_[i];
		
		if (post_label)
		{
			// Calculate the "gap" between this action and the next action.
			float delta = 0;
			
			if (i > 0)
			{
				delta = action_label->getAction().dispatch_time - (action_labels_[i - 1]->getAction().dispatch_time + action_labels_[i - 1]->getAction().duration);
			}
			//action_label->setPosition(2.5f, -time * pixels_per_second_);
			action_label->setPosition(time * pixels_per_second_, -2.5f);
			action_label->setStartTime(time);
			action_label->updateBuffers();
			action_label->markForUpdate();
			time += action_label->getAction().duration + delta;
		}
		
		if (action_label->getAction().action_id == action.getActionMsg().action_id)
		{
			std::cout << "Found the action that has been completed." << std::endl;
			//action_label->setDimensions(size_x_ / 2.0f - 2.5f, (total_time_ - start_time_current_action_) * pixels_per_second_);
			action_label->setDimensions((total_time_ - start_time_current_action_) * pixels_per_second_, size_y_);
			action_label->setDuration(total_time_ - start_time_current_action_);
			action_label->updateBuffers();
			action_label->markForUpdate();
			post_label = true;
			break;
		}
	}
	current_action_.action_id = INVALID_ACTION_ID_;
}
#endif
void PlanLine::update(float dt)
{
	total_time_ += dt;
	// Check if the current action takes longer than expected.
	if (current_action_.action_id != INVALID_ACTION_ID_ && current_action_.duration < total_time_ - start_time_current_action_)
	{
		bool post_label = false;
		float time = total_time_;
		
		for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
		{
			ActionLabel* action_label = *ci;
			if (post_label)
			{
				if ("turn_valve" == action_label->getAction().name ||
				    "undock_auv" == action_label->getAction().name)
				{
					time = std::max(action_label->getAction().dispatch_time, time);
				}
				
				action_label->setPosition(time * pixels_per_second_, -2.5f);
				//action_label->updateBuffers();
				time += action_label->getAction().duration;
			}
			
			if (action_label->getAction().action_id == current_action_.action_id)
			{
				//action_label->setDimensions(size_x_ / 2.0f - 2.5f, (total_time_ - start_time_current_action_) * pixels_per_second_);
				action_label->setDimensions((total_time_ - start_time_current_action_) * pixels_per_second_, size_y_);
				action_label->setDuration(total_time_ - start_time_current_action_);
				//action_label->updateBuffers();
				post_label = true;
			}
			action_label->updateBuffers();
			action_label->markForUpdate();
		}
	}
	DreadedPE::Container::update(dt);
	markForUpdate();
	//std::cout << getGlobalY() << "-" << getLocalY() << " >=>> " << std::endl;
}

void PlanLine::onResize(float width, float height)
{
	size_x_ = width;
	//local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(0, -height + size_y_, 0));
	
	for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	{
		ActionLabel* action_label = *ci;
		action_label->onResize(width, height);
	}
	
	updateTransformations();
	DreadedPE::Container::onResize(width, height);
}

void PlanLine::buttonPressed(const DreadedPE::Button& source)
{
	/*
	if (&source == decrease_button_ && pixels_per_second_ > 1)
	{
		pixels_per_second_ -= 1;
	}
	else if (&source == increase_button_)
	{
		pixels_per_second_ += 1;
	}
	
	// Update the labels to reflect this change.
	for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	{
		ActionLabel* action_label = *ci;
		action_label->setPosition(action_label->getStartTime() * pixels_per_second_, -2.5f);
		action_label->setDimensions(action_label->getDuration() * pixels_per_second_, size_y_ / 2.0f - 2.5f);
		
		std::cout << "Action: " << action_label->getAction().name << ": " << action_label->getStartTime() << "(d=" << action_label->getDuration() << ")" << std::endl;
	}
	*/
}

void PlanLine::setPixelsPerSecond(float pixels_per_second)
{
	pixels_per_second_ = pixels_per_second;
	
	// Update the labels to reflect this change.
	for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	{
		ActionLabel* action_label = *ci;
		action_label->setPosition(action_label->getStartTime() * pixels_per_second_, -2.5f);
		action_label->setDimensions(action_label->getDuration() * pixels_per_second_, size_y_);
		
		std::cout << "Action: " << action_label->getAction().name << ": " << action_label->getStartTime() << "(d=" << action_label->getDuration() << ")" << std::endl;
	}
}

#ifndef _WIN32
void PlanLine::setCurrentPlan(const planning_msgs::CompletePlan& msg)
{
	std::cout << "NEW PLAN" << std::endl;
	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg.actions.begin(); ci != msg.actions.end(); ++ci)
	{
		std::cout << (*ci).name << " (d = " << (*ci).duration << ")" << std::endl;
	}
	
	
	total_time_ = 0;
	for (std::vector<ActionLabel*>::const_iterator ci = action_labels_.begin(); ci != action_labels_.end(); ++ci)
	{
		removeElement(**ci);
		delete *ci;
	}
	action_labels_.clear();
	
	int i = 0;
	//float plan_duration = 0;
	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg.actions.begin(); ci != msg.actions.end(); ++ci, ++i)
	{
		//glm::vec4 colour((float)rand() / (float)RAND_MAX,(float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX, 1.0f); 
		const planning_msgs::ActionDispatch& action = *ci;
		/*
		if (auv_ != NULL && action.auv != auv_->getName())
		{
			continue;
		}
		*/
		
		std::vector<glm::vec2> uv_mapping;
		AUVStatusIcon& auv_status_icon = AUVStatusIcon::getInstance();
		
		glm::vec4 colour(1, 0, 0, 1);
		if ("goto" == action.name || "goto_structure" == action.name)
		{
			colour = glm::vec4(0.4f, 0.4f, 1.0f, 1.0f);
			uv_mapping = auv_status_icon.getGotoIcon();
		}
		else if ("observe" == action.name || "examine_panel" == action.name)
		{
			colour = glm::vec4(0.2f, 1.0f, 0.2f, 1);
			uv_mapping = auv_status_icon.getObserveIcon();
		}
		else if ("turn_valve" == action.name)
		{
			colour = glm::vec4(1.0f, 1.0f, 0.0f, 1.0f);
			uv_mapping = auv_status_icon.getTurnValveIcon();
		}
		else if ("recharge" == action.name)
		{
			colour = glm::vec4(1.0f, 0.5f, 0.0f, 1.0f);
			uv_mapping = auv_status_icon.getRechargeIcon();
		}
		else if ("undock" == action.name)
		{
			colour = glm::vec4(1.0f, 0.5f, 1.0f, 1.0f);
			uv_mapping = auv_status_icon.getUndockIcon();
		}
		else if ("dock" == action.name)
		{
			colour = glm::vec4(0.5f, 1.0f, 0.5f, 1.0f);
			uv_mapping = auv_status_icon.getDockIcon();
		}
		else
		{
			colour = glm::vec4((float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX, (float)rand() / (float)RAND_MAX, 1.0f);
			uv_mapping = auv_status_icon.getEmptyIcon();
		}
		
		std::stringstream ss;
		ss << "Name: " << action.name << std::endl;
		ss << "Duration: " << action.duration << std::endl;
		ss << "ID: " << action.action_id;
		
		ActionLabel* label = new ActionLabel(*theme_, font_->clone(), colour, action.duration * pixels_per_second_ - 2, size_y_, action.name, action);

		label->setStartTime(action.dispatch_time);
		label->setDuration(action.duration);
		std::cout << "L"<<pixels_per_second_<<std::endl;
		std::cout << "New action: " << label->getAction().name << ": " << label->getStartTime() << "(d=" << label->getDuration() << ")" << std::endl;
		
		DreadedPE::Canvas* icon_gui = new DreadedPE::Canvas(*theme_, font_->clone(), 0, 0, 50, label->getHeight(), *icon_texture_);
		icon_gui->setTextureUVMapping(uv_mapping);
		
		label->addElement(*icon_gui, 0, 0);
		
		std::cout << "Add the label: " << label->getAction().name << " starting at: " << action.dispatch_time << "(pixels=" << action.dispatch_time * pixels_per_second_ << ")" << std::endl;
		addElement(*label, action.dispatch_time * pixels_per_second_, -2.5f);
		action_labels_.push_back(label);
	}
}
#endif
