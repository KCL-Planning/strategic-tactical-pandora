#ifndef DEMO_PANDORA_CONTROLLERS_ILLUMINATE_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_ILLUMINATE_CONTROLLER_H

#include <glm/glm.hpp>

#include "PlannerAction.h"

class AUV;
namespace DreadedPE
{
	class Line;
	class SceneNode;
	class SceneLeafModel;
	class SceneManager;
	class Material;
};
class OntologyInterface;

class IlluminateController : public PlannerAction
{
public:
	IlluminateController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology);
	
	/**
	 * Set the duration of how long the illumination should take.
	 * @param duratino The duration of the illumination.
	 */
	void setDuration(float duration);
	
	/**
	 * Return true if we are done observing.
	 */
	PlannerAction::PLANNER_ACTION_STATUS getStatus();
	
	/**
	 * Update the observe visual.
	 */
	void update(float dt);
	
private:
	AUV* auv_;
	
	float duration_; // The time that we should be illuminating.
	float time_; // Time that we have been illuminating.

	OntologyInterface* ontology_; // The ontology that stores all the things we care about.
};

#endif
