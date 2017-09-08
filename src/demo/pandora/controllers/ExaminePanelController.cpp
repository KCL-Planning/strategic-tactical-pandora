#include "ExaminePanelController.h"

#include <cstdlib>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include <planning_msgs/ActionFeedback.h>

#include "../AUV.h"
#include "../ontology/OntologyInterface.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/shaders/ShadowShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shapes/Line.h"
#include "dpengine/shapes/FrustumShape.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"
#include "../structures/Structure.h"
#include "../structures/Valve.h"

ExaminePanelController::ExaminePanelController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub)
	: auv_(&auv), time_(0), ontology_(&ontology), panel_(NULL), action_feedback_pub_(&action_feedback_pub)
{
	/*
	MaterialLightProperty* ambient = new MaterialLightProperty(0.3f, 0.3f, 0.3f, 0.0f);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0.1f, 0.1f, 0.1f, 0.0f);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0.0f);
	MaterialLightProperty* emmisive = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 0.0f);
	material_ = new Material(*ambient, *diffuse, *specular, *emmisive, 1.0f);
	
	Texture* observe_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	
	material_->add2DTexture(*observe_texture);
	
	FrustumShape* frustum_shape = new FrustumShape(0.1f, 5.0f, 0.1f, 0.1f, 1.5f, 1.5f);
	scene_node_ = new SceneNode(scene_manager, &auv_->getAUVModel(), glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
	model_ = new SceneLeafModel(*scene_node_, NULL, *frustum_shape, *material_, BasicShadowShader::getShader(), true, true, OBJECT, ShadowRenderer::NO_SHADOW);
	*/
}
/*
void ExaminePanelController::observe(const glm::vec3& inspection_point)
{
	time_ = 0;
	inspection_point_ = inspection_point;
}
*/
PlannerAction::PLANNER_ACTION_STATUS ExaminePanelController::getStatus()
{
	if (time_ > 10)
	{
		/*
		//ontology_->observedInspectionPoint(inspection_point_);
		// "valve_n_in_panel" : "[int]"
		planning_msgs::ActionFeedback feedback;
		feedback.action_id = action_msg_.action_id;
		feedback.status = "whatever";
		
		for (std::vector<Valve*>::const_iterator ci = panel_->getValves().begin(); ci != panel_->getValves().end(); ++ci)
		{
			const Valve* valve = *ci;
			std::stringstream ss;
			ss << "valve_" << valve->getId() << "_in_panel";
			diagnostic_msgs::KeyValue kv;
			kv.key = ss.str();
			
			ss.str("");
			ss << panel_->getId();
			kv.value = ss.str();
			
			feedback.information.push_back(kv);
			
			ss << "valve_" << valve->getId() << "_angle";
			kv.key = ss.str();
			ss.str("");
			ss << glm::roll(valve->getGlobalRotation()) / 180.0f * M_PI;
			kv.value = ss.str();
			feedback.information.push_back(kv);
		}
		action_feedback_pub_->publish(feedback);
		*/
		panel_->setExamined(true);
		
		return SUCCEEDED;
	}
	return EXECUTING;
}

void ExaminePanelController::amendFeedback(planning_msgs::ActionFeedback& feedback, PLANNER_ACTION_STATUS status)
{
	std::cout << "Examine panel feedback!" << std::endl;
	if (status == SUCCEEDED)
	{
		for (std::vector<Valve*>::const_iterator ci = panel_->getValves().begin(); ci != panel_->getValves().end(); ++ci)
		{
			const Valve* valve = *ci;
			std::stringstream ss;
			ss << "valve_" << valve->getName() << "_in_panel";
			diagnostic_msgs::KeyValue kv;
			kv.key = ss.str();
			ss.str("");
			ss << panel_->getName();
			kv.value = ss.str();
			//std::cout << "key: " << kv.key << "; value=" << kv.value << std::endl;;
			
			feedback.information.push_back(kv);
			ss.str("");
			ss << "valve_" << valve->getName() << "_angle";
			kv.key = ss.str();
			ss.str("");
			ss << glm::roll(valve->getGlobalRotation()) / 180.0f * M_PI;
			kv.value = ss.str();
			//std::cout << "key: " << kv.key << "; value=" << kv.value << std::endl;;
			feedback.information.push_back(kv);
		}
	}
	else
	{
		std::cout << "No feedback amended!" << std::endl;
	}
}

void ExaminePanelController::update(float dt)
{
	time_ += dt;
	int t = time_;
	auv_->setLightOn(true);
	/*
	if (time_ > 10)
	{
		material_->setTransparency(1.0f);
	}
	else
	{
		if (t % 2 == 0)
		{
			material_->setTransparency(0.1f + (time_ - t) * 0.8f);
		}
		else
		{
			material_->setTransparency(0.9f - (time_ - t) * 0.8f);
		}
		
		//auv_->setDirection(inspection_point_ - auv_->getGlobalLocation());
		auv_->setVelocity(0);
	}
	*/
}
