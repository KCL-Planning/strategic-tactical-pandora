#include "RechargeController.h"

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

RechargeController::RechargeController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology)
	: auv_(&auv), duration_(0), time_(0), ontology_(&ontology)
{
	
}

void RechargeController::setDuration(float duration)
{
	time_ = 0;
	duration_ = duration;
}

PlannerAction::PLANNER_ACTION_STATUS RechargeController::getStatus()
{
	//std::cout << time_ << ">" << duration_ << std::endl;
	if (time_ > duration_)
	{
		//std::cout << "Done!" << std::endl;
		return SUCCEEDED;
	}
	return EXECUTING;
}

void RechargeController::update(float dt)
{
	time_ += dt;
}
