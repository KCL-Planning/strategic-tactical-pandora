#include "RechargeController.h"

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

RechargeController::RechargeController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology)
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
