#include "ObserveController.h"

#include <cstdlib>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "../AUV.h"
#include "../ontology/OntologyInterface.h"
#include "../../../core/scene/SceneNode.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/Material.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../core/shaders/ShadowShader.h"
#include "../../../core/shaders/BasicShadowShader.h"
#include "../../../shapes/Line.h"
#include "../../../shapes/FrustumShape.h"
#include "../../../core/texture/Texture.h"
#include "../../../core/texture/TargaTexture.h"

ObserveController::ObserveController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology)
	: auv_(&auv), time_(0), inspection_point_(glm::vec3(0, 0, 0)), ontology_(&ontology)
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

void ObserveController::observe(const glm::vec3& inspection_point)
{
	time_ = 0;
	inspection_point_ = inspection_point;
}

PlannerAction::PLANNER_ACTION_STATUS ObserveController::getStatus()
{
	if (time_ > 10)
	{
		auv_->setLightOn(false);
		//material_->setTransparency(1.0f);
		ontology_->observedInspectionPoint(inspection_point_);
		return SUCCEEDED;
	}
	return EXECUTING;
}

void ObserveController::update(float dt)
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
		//auv_->setVelocity(0);
	}
	*/
	/*
	// Cast lines towards the target.
	std::vector<glm::vec3> line_breaks;
	
	if (time_ < 10)
	{
		glm::vec3 relative_point =  glm::vec3(glm::inverse(auv_->getCompleteTransformation()) * glm::vec4(inspection_point_, 1.0f));
		
		for (unsigned int i = 0; i < 10; i++)
		{
			float x = (float)rand() / (float)RAND_MAX * 0.5f;
			float y = (float)rand() / (float)RAND_MAX * 0.5f;
			float z = (float)rand() / (float)RAND_MAX * 1.0f;
			
			//line_breaks.push_back(auv_->getGlobalLocation());
			line_breaks.push_back(glm::vec3(0, 0, 0));
			line_breaks.push_back(glm::vec3(x, y, z));
			line_breaks.push_back(glm::vec3(x, y, z));
			line_breaks.push_back(relative_point);
		}
	}
	
	line_->setVertexBuffer(line_breaks);
	*/
}
