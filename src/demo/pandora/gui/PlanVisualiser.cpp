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

#include "PlanVisualiser.h"
#include "../ontology/OntologyInterface.h"
#include "../ontology/filter/Filter.h"
#include "../AUV.h"
#include "BillBoard.h"
#include "AUVStatusIcon.h"

#define USE_MULTILE_AUVS

PlanVisualiser::PlanVisualiser(ros::NodeHandle& ros_node, AUV& auv, OntologyInterface& ontology, DreadedPE::SceneNode& parent, DreadedPE::SceneManager& scene_manager, DreadedPE::Theme& theme, DreadedPE::Font& font, DreadedPE::Camera& camera)
	: SceneNode(scene_manager, &parent, glm::mat4(1.0f)), auv_(&auv), ontology_(&ontology), theme_(&theme), font_(&font), camera_(&camera)
{
	line_ = std::make_shared<DreadedPE::Line>(false);
	
	DreadedPE::MaterialLightProperty ambient(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty diffuse(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty specular(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty emmisive(1, 1, 0, 0.8f);
	std::shared_ptr<DreadedPE::Material> material(std::make_shared<DreadedPE::Material>(ambient, diffuse, specular, emmisive));
	
	path_ = new DreadedPE::SceneLeafModel(*this, NULL, line_, material, DreadedPE::LineShader::getShader(), true, true);
	
	bad_line_ = std::make_shared<DreadedPE::Line>(false);
	DreadedPE::MaterialLightProperty bad_ambient(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty bad_diffuse(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty bad_specular(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty bad_emmisive(1, 0, 0, 0.8f);
	std::shared_ptr<DreadedPE::Material> bad_material(std::make_shared<DreadedPE::Material>(bad_ambient, bad_diffuse, bad_specular, bad_emmisive));
	
	bad_path_ = new DreadedPE::SceneLeafModel(*this, NULL, bad_line_, bad_material, DreadedPE::LineShader::getShader(), true, true);
	
	filter_line_ = std::make_shared<DreadedPE::Line>(false);
	DreadedPE::MaterialLightProperty filter_ambient(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty filter_diffuse(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty filter_specular(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty filter_emmisive(1, 0, 1, 0.8f);
	std::shared_ptr<DreadedPE::Material> filter_material(std::make_shared<DreadedPE::Material>(filter_ambient, filter_diffuse, filter_specular, filter_emmisive));
	
	filter_path_ = new DreadedPE::SceneLeafModel(*this, NULL, filter_line_, filter_material, DreadedPE::LineShader::getShader(), true, true);
	
	complete_plan_listener_ = ros_node.subscribe("/planning_system/current_plan", 1, &PlanVisualiser::setCurrentPlan, this);
	rviz_plan_publisher_ = ros_node.advertise<nav_msgs::Path>("/plan/complete_plan", 1);
	
	complete_strategic_plan_listener_ = ros_node.subscribe("/planning_system/current_strategic_plan", 1, &PlanVisualiser::setCurrentPlan, this);
	//strategic_plan_dispatcher_ = ros_node.subscribe("/planning_system/strategic_action_dispatch", 1, &StrategicPlanGUIElement::actionDispatch, this);
}

void PlanVisualiser::actionExecutionStarted(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->enableBlinking(true);
	}
}

void PlanVisualiser::actionExecutionFailed(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->setVisible(false);
	}
}

void PlanVisualiser::actionExecutionSucceeded(const PlannerAction& action)
{
	std::map<int, BillBoard*>::iterator i = active_bill_boards_.find(action.getActionMsg().action_id);
	if (i != active_bill_boards_.end())
	{
		(*i).second->setVisible(false);
	}
}

void PlanVisualiser::setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg)
{
	std::vector<glm::vec3> line_segments;
	glm::vec3 previous_point = auv_->getGlobalLocation();
	bool added_first_point = false;
	
	nav_msgs::Path path;
	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";
	
	AUVStatusIcon& auv_status_icon = AUVStatusIcon::getInstance();
	
	// Remove the old bill boards.
	DreadedPE::GUIManager& gui_manager = DreadedPE::GUIManager::getInstance();
	for (std::map<int, BillBoard*>::const_iterator ci = active_bill_boards_.begin(); ci != active_bill_boards_.end(); ++ci)
	{
		//gui_manager.deleteFrame(*((*ci).second));
		(*ci).second->setVisible(false);
	}
	
	std::vector<glm::vec3> bad_line_segments;

	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg->actions.begin(); ci != msg->actions.end(); ++ci)
	{
		const planning_msgs::ActionDispatch& action = *ci;
#ifdef USE_MULTILE_AUVS
		if (action.auv != auv_->getName())
		{
			continue;
		}
#endif

		//std::cout << "(" << action.name << " " << previous_point.x << ", " << previous_point.y << ", " << previous_point.z << ")" << std::endl;
		/*
		std::cout << "(d=" << (*ci).duration << ") " << (*ci).action_id << ": " << (*ci).name << std::endl;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = action.parameters.begin(); ci != action.parameters.end(); ++ci)
		{
			std::cout << "\t*" << (*ci).key << " - " << (*ci).value << std::endl;
		}
		*/
		if (action.name == "goto" || action.name == "goto_structure")
		{
			
			geometry_msgs::PoseStamped ps;
			ps.header.stamp = ros::Time::now();
			
			glm::vec3 point;
			for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = action.parameters.begin(); ci != action.parameters.end(); ++ci)
			{
				if ("north" == (*ci).key)
				{
					point.z = ::atof((*ci).value.c_str());
					ps.pose.position.y = point.z;
				}
				else if ("east" == (*ci).key)
				{
					point.x = ::atof((*ci).value.c_str());
					ps.pose.position.x = -point.x;
				}
				else if ("depth" == (*ci).key)
				{
					point.y = ::atof((*ci).value.c_str());
					ps.pose.position.z = point.y;
				}
			}
			path.poses.push_back(ps);
			
			if (point != previous_point)
			{
				DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), (point + previous_point) / 2.0f));
				const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getGotoIcon();
				/*
				uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
				uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
				uv_mapping.push_back(glm::vec2(0.75f, 0.5f));
				uv_mapping.push_back(glm::vec2(1.0f, 0.5f));
				*/
				BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
				gui_manager.addFrame(*bb);
				active_bill_boards_[action.action_id] = bb;
			}
			
			if (added_first_point)
			{
				std::vector<glm::vec3>* to_add_to = NULL;
				if (scene_manager_->getRoot().doesCollide(auv_, previous_point, point, 1.0f))
				{
					to_add_to = &bad_line_segments;
				}
				else
				{
					to_add_to = &line_segments;
				}
				
				to_add_to->push_back(previous_point + glm::vec3(0.0f, 0.01f, 0.0f));
				to_add_to->push_back(point + glm::vec3(0.0f, 0.01f, 0.0f));
				
				to_add_to->push_back(previous_point + glm::vec3(0.0f, -0.01f, 0.0f));
				to_add_to->push_back(point + glm::vec3(0.0f, -0.01f, 0.0f));
				
				to_add_to->push_back(previous_point + glm::vec3(-0.01f, 0.0f, 0.0f));
				to_add_to->push_back(point + glm::vec3(-0.01f, 0.0f, 0.0f));
				
				to_add_to->push_back(previous_point + glm::vec3(0.01f, 0.0f, 0.0f));
				to_add_to->push_back(point + glm::vec3(0.01f, 0.0f, 0.0f));
				
				to_add_to->push_back(previous_point + glm::vec3(0.00f, 0.0f, 0.01f));
				to_add_to->push_back(point + glm::vec3(0.00f, 0.0f, 0.01f));
				
				to_add_to->push_back(previous_point + glm::vec3(0.00f, 0.0f, -0.01f));
				to_add_to->push_back(point + glm::vec3(0.00f, 0.0f, -0.01f));
			}
			else
			{
				added_first_point = true;
			}
			previous_point = point;
		}
		else if (action.name == "observe" || action.name == "observe_pillar" || action.name == "examine_panel")
		{
			DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), previous_point));
			const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getObserveIcon();
			/*
			uv_mapping.push_back(glm::vec2(0.25f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.5f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.25f, 0.75));
			uv_mapping.push_back(glm::vec2(0.5f, 0.75f));
			*/
			BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
			gui_manager.addFrame(*bb);
			active_bill_boards_[action.action_id] = bb;
		}
		else if (action.name == "illuminate_pillar")
		{
			DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), previous_point));
			const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getEmptyIcon();
			/*
			uv_mapping.push_back(glm::vec2(0.5f, 0.75f));
			uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
			uv_mapping.push_back(glm::vec2(0.5f, 0.5f));
			uv_mapping.push_back(glm::vec2(0.75f, 0.5f));
			*/
			BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
			gui_manager.addFrame(*bb);
			active_bill_boards_[action.action_id] = bb;
		}
		else if (action.name == "turn_valve")
		{
			DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), previous_point));
			const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getTurnValveIcon();
			/*
			uv_mapping.push_back(glm::vec2(0.0f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.25f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.0f, 0.75f));
			uv_mapping.push_back(glm::vec2(0.25f, 0.75f));
			*/
			BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
			gui_manager.addFrame(*bb);
			active_bill_boards_[action.action_id] = bb;
		}
		else
		{
			DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), previous_point));
			std::vector<glm::vec2> uv_mapping;// = auv_status_icon.getTurnValveIcon();

			uv_mapping.push_back(glm::vec2(0.0f, 1.0f));
			uv_mapping.push_back(glm::vec2(1.0f, 1.0f));
			uv_mapping.push_back(glm::vec2(1.0f, 0.0f));
			uv_mapping.push_back(glm::vec2(0.0f, 0.0f));

			BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
			gui_manager.addFrame(*bb);
			active_bill_boards_[action.action_id] = bb;
		}
	}
	
	added_first_point = false;
	std::vector<glm::vec3> filter_segments;
	for (std::vector<Filter*>::const_iterator ci = ontology_->getFilters().begin(); ci != ontology_->getFilters().end(); ++ci)
	{
		const Filter* filter = *ci;
		const std::vector<glm::vec3>& points = filter->getPoints();
		if (points.size() < 2) continue;
		
		for (unsigned int i = 0; i < points.size() - 1; ++i)
		{
			filter_segments.push_back(points[i] + glm::vec3(0.0f, 0.04f, 0.0f));
			filter_segments.push_back(points[i + 1] + glm::vec3(0.0f, 0.04f, 0.0f));
			
			filter_segments.push_back(points[i] + glm::vec3(0.0f, -0.04f, 0.0f));
			filter_segments.push_back(points[i + 1] + glm::vec3(0.0f, -0.04f, 0.0f));
			
			filter_segments.push_back(points[i] + glm::vec3(-0.04f, 0.0f, 0.0f));
			filter_segments.push_back(points[i + 1] + glm::vec3(-0.04f, 0.0f, 0.0f));
			
			filter_segments.push_back(points[i] + glm::vec3(0.04f, 0.0f, 0.0f));
			filter_segments.push_back(points[i + 1] + glm::vec3(0.04f, 0.0f, 0.0f));
			
			filter_segments.push_back(points[i] + glm::vec3(0.00f, 0.0f, 0.04f));
			filter_segments.push_back(points[i + 1] + glm::vec3(0.00f, 0.0f, 0.04f));
			
			filter_segments.push_back(points[i] + glm::vec3(0.00f, 0.0f, -0.04f));
			filter_segments.push_back(points[i + 1] + glm::vec3(0.00f, 0.0f, -0.04f));
		}
	}
	
	line_->setVertexBuffer(line_segments);
	bad_line_->setVertexBuffer(bad_line_segments);
	filter_line_->setVertexBuffer(filter_segments);
	
	rviz_plan_publisher_.publish(path);
}
