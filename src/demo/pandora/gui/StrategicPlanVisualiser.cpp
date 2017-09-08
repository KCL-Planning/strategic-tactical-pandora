#include <nav_msgs/Path.h>

#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/scene/Material.h"
#include "dpengine/gui/themes/MyGUITheme.h"
#include "dpengine/gui/GUIManager.h"

#include "dpengine/shapes/Line.h"
#include "../controllers/PlannerAction.h"

#include "StrategicPlanVisualiser.h"
#include "../ontology/OntologyInterface.h"
#include "../ontology/filter/Filter.h"
#include "../AUV.h"
#include "BillBoard.h"
#include "../level/MissionSite.h"
#include "../level/Mission.h"
#include "../Waypoint.h"

#define USE_MULTILE_AUVS

StrategicPlanVisualiser::StrategicPlanVisualiser(ros::NodeHandle& ros_node, AUV& auv, OntologyInterface& ontology, DreadedPE::SceneNode& parent, DreadedPE::SceneManager& scene_manager, DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::Camera& camera)
	: SceneNode(scene_manager, &parent, glm::mat4(1.0f)), auv_(&auv), ontology_(&ontology), theme_(&theme), font_(&font), camera_(&camera)
{
	line_ = std::make_shared<DreadedPE::Line>(false);
	
	DreadedPE::MaterialLightProperty ambient(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty diffuse(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty specular(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty emmisive(1, 1, 0, 0.8f);
	std::shared_ptr<DreadedPE::Material> material(std::make_shared<DreadedPE::Material>(ambient, diffuse, specular, emmisive));
	
	path_ = new DreadedPE::SceneLeafModel(*this, NULL, line_, material, DreadedPE::LineShader::getShader(), true, true);
	
	complete_plan_listener_ = ros_node.subscribe("/planning_system/strategic_plan", 1, &StrategicPlanVisualiser::setCurrentPlan, this);
}

void StrategicPlanVisualiser::actionExecutionStarted(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->enableBlinking(true);
	}
}

void StrategicPlanVisualiser::actionExecutionFailed(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->setVisible(false);
	}
}

void StrategicPlanVisualiser::actionExecutionSucceeded(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->setVisible(false);
	}
}

void StrategicPlanVisualiser::setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg)
{
	std::cout << "StrategicPlanVisualiser::setCurrentPlan" << std::endl;
	
	std::vector<glm::vec3> line_segments;
	glm::vec3 previous_point = auv_->getGlobalLocation();
	
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";
	
	
	// HACK: Reset the missions.
	for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Mission*>::const_iterator ci = mission_site->getMissions().begin(); ci != mission_site->getMissions().end(); ++ci)
		{
			Mission* mission = *ci;
			for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
			{
				Goal* goal = *ci;
				goal->reset();
			}
		}
	}
	
	// Remove the old bill boards.
	DreadedPE::GUIManager& gui_manager = DreadedPE::GUIManager::getInstance();
	for (std::map<int, BillBoard*>::const_iterator ci = active_bill_boards_.begin(); ci != active_bill_boards_.end(); ++ci)
	{
		//gui_manager.deleteFrame(*((*ci).second));
		(*ci).second->setVisible(false);
	}

	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg->actions.begin(); ci != msg->actions.end(); ++ci)
	{
		const planning_msgs::ActionDispatch& action = *ci;

		std::cout << "StrategicPlanVisualiser::setCurrentPlan: " << action.name << std::endl;

		if (action.name == "goto_structure")
		{
			glm::vec3 point;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = action.parameters.begin(); ci != action.parameters.end(); ++ci)
			{
				if ("structure" == (*ci).key)
				{
					MissionSite* mission_site = ontology_->getMissionSite((*ci).value);
					if (mission_site != NULL)
					{
						point = mission_site->getStartWaypoint().position_;
					}
				}
			}
			
			if (point != previous_point)
			{
				SceneNode* dummy_node = new SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), (point + previous_point) / 2.0f));
				std::vector<glm::vec2> uv_mapping;
				uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
				uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
				uv_mapping.push_back(glm::vec2(0.75f, 0.5f));
				uv_mapping.push_back(glm::vec2(1.0f, 0.5f));
				BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
				gui_manager.addFrame(*bb);
				active_bill_boards_[action.action_id] = bb;
			}
			
			line_segments.push_back(previous_point + glm::vec3(0.0f, 0.01f, 0.0f));
			line_segments.push_back(point + glm::vec3(0.0f, 0.01f, 0.0f));
			
			line_segments.push_back(previous_point + glm::vec3(0.0f, -0.01f, 0.0f));
			line_segments.push_back(point + glm::vec3(0.0f, -0.01f, 0.0f));
			
			line_segments.push_back(previous_point + glm::vec3(-0.01f, 0.0f, 0.0f));
			line_segments.push_back(point + glm::vec3(-0.01f, 0.0f, 0.0f));
			
			line_segments.push_back(previous_point + glm::vec3(0.01f, 0.0f, 0.0f));
			line_segments.push_back(point + glm::vec3(0.01f, 0.0f, 0.0f));
			
			line_segments.push_back(previous_point + glm::vec3(0.00f, 0.0f, 0.01f));
			line_segments.push_back(point + glm::vec3(0.00f, 0.0f, 0.01f));
			
			line_segments.push_back(previous_point + glm::vec3(0.00f, 0.0f, -0.01f));
			line_segments.push_back(point + glm::vec3(0.00f, 0.0f, -0.01f));
			previous_point = point;
		}
		else if (action.name == "complete_mission")
		{
			DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), previous_point));
			std::vector<glm::vec2> uv_mapping;
			uv_mapping.push_back(glm::vec2(0.25f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.5f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.25f, 0.75));
			uv_mapping.push_back(glm::vec2(0.5f, 0.75f));
			BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
			gui_manager.addFrame(*bb);
			active_bill_boards_[action.action_id] = bb;
		}
	}
	
	line_->setVertexBuffer(line_segments);
}
