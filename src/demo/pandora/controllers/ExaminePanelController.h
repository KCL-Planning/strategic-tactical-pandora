#ifndef DEMO_PANDORA_CONTROLLERS_EXIMINE_PANEL_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_EXIMINE_PANEL_CONTROLLER_H

#include <ros/ros.h>
#include <glm/glm.hpp>

#include "PlannerAction.h"

class AUV;
class Line;
class SceneNode;
class SceneLeafModel;
class SceneManager;
class Material;
class OntologyInterface;
class Structure;

class ExaminePanelController : public PlannerAction
{
public:
	ExaminePanelController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub);
	
	void setValvePanel(Structure& valve_panel) { panel_ = &valve_panel; time_ = 0; }
	
	/**
	 * Create a cool visualisation of the AUV inspecting the given point.
	 * @param inspection_point The point that must be inspected.
	 */
	//void observe(const glm::vec3& inspection_point);
	
	/**
	 * Return true if we are done observing.
	 */
	PlannerAction::PLANNER_ACTION_STATUS getStatus();
	
	/**
	 * Update the observe visual.
	 */
	void update(float dt);
	
	void amendFeedback(planning_msgs::ActionFeedback& feedback, PLANNER_ACTION_STATUS status);
	
private:
	AUV* auv_;
	
	//glm::vec3 inspection_point_; // The location of the point we want to inspect.
	//Line* line_; // We visualise the inspection with a few lines (for now, it will look crap, but who cares :).
	//SceneNode* scene_node_; // The location where the lines will start.
	//SceneLeafModel* model_; // The nodes that visualises the lines.
	
	float time_; // Time that we have been observing.
	
	//Material* material_; // The material used to render the observation frustum.
	OntologyInterface* ontology_; // The ontology that stores all the things we care about.
	
	Structure* panel_; // Valve panel.
	
	ros::Publisher* action_feedback_pub_;
};

#endif
