#include "IlluminateController.h"

#include <cstdlib>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

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

IlluminateController::IlluminateController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology)
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
