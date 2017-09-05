#ifndef DEMO_PANDORA_STRUCTURES_VALVE_PANEL_H
#define DEMO_PANDORA_STRUCTURES_VALVE_PANEL_H

#include <glm/glm.hpp>
#include <string>

class RotateBehaviour;
class SceneManager;
class SceneNode;
class Texture;
class Valve;

#include "../../../core/entities/Entity.h"

class ValvePanel : public Entity
{
public:
	/**
	 * Initialise the valve panel.
	 */
	ValvePanel(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::string& name, Texture& texture);
	
	const glm::vec3& getInteractLocation() const { return interact_location_; }
	
	void setExamined(bool is_examined) { is_examined_ = is_examined; }
	bool isExamined() const { return is_examined_; }
	
	const std::string& getId() const { return id_; }
	
	void addValve(Valve& valve);
	
	const std::vector<Valve*>& getValves() const { return valves_; }
	
	/**
	 * This function will be called when the AUV activates this valve panel.
	 */
	//bool activate(Entity& activator);
	
	/**
	 * Update the valve and rotate when it needs to.
	 */
	//void prepare(float dt);
private:
	std::string id_;                          // An unique ID to identify this valve panel.
	SceneNode* valve_node_;                   // The node that contains the valve that can rotate.
	RotateBehaviour* valve_rotate_behaviour_; // Behaviour to rotate the valve.
	glm::vec3 interact_location_;             // The location the AUV needs to be in order to interact with the valve.
	bool is_examined_;                        // Track whether this valve panel has been examined or not.
	std::vector<Valve*> valves_;              // The valves that are part of this panel.
	
	static int global_valve_panel_id_;
};

#endif