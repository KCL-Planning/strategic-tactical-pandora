#ifndef DEMO_PANDORA_CONTROLLERS_OBSERVE_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_OBSERVE_CONTROLLER_H

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

class ObserveController : public PlannerAction
{
public:
	ObserveController(DreadedPE::SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology);
	
	/**
	 * Create a cool visualisation of the AUV inspecting the given point.
	 * @param inspection_point The point that must be inspected.
	 */
	void observe(const glm::vec3& inspection_point);
	
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
	float time_; // Time that we have been observing.
	glm::vec3 inspection_point_; // The location of the point we want to inspect.
	//Line* line_; // We visualise the inspection with a few lines (for now, it will look crap, but who cares :).
	//SceneNode* scene_node_; // The location where the lines will start.
	//SceneLeafModel* model_; // The nodes that visualises the lines.
	
	
	DreadedPE::Material* material_; // The material used to render the observation frustum.
	OntologyInterface* ontology_; // The ontology that stores all the things we care about.
};

#endif
