#include "IlluminateController.h"

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

IlluminateController::IlluminateController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology)
	: auv_(&auv), duration_(0), time_(0), ontology_(&ontology)
{
	
}

void IlluminateController::setDuration(float duration)
{
	time_ = 0;
	duration_ = duration;
	auv_->setLightOn(true);
}

PlannerAction::PLANNER_ACTION_STATUS IlluminateController::getStatus()
{
	if (time_ > duration_)
	{
		auv_->setLightOn(false);
		return SUCCEEDED;
	}
	return EXECUTING;
}

void IlluminateController::update(float dt)
{
	auv_->setLightOn(true);
	time_ += dt;
}
