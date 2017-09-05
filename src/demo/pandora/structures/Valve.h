#ifndef DEMO_PANDORA_STRUCTURES_VALVE_H
#define DEMO_PANDORA_STRUCTURES_VALVE_H

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "../../../core/entities/Entity.h"

class RotateBehaviour;
class SceneNode;
class Texture;
class Structure;
class ValveGoal;

class Valve : public Entity
{
public:
	Valve(Structure& structure, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::string& name);
	
	//const std::string& getId() const { return id_; }
	
	void addValveGoal(ValveGoal& valve_goal) { valve_goals_.push_back(&valve_goal); }
	const std::vector<ValveGoal*>& getValveGoals() const { return valve_goals_; }
	
	Structure& getStructure() const { return *structure_; }
	
	void setRotation(float r) { desired_rotation_ = r; }
	
	bool doneRotating() const { return desired_rotation_ == 0.0f; }
	
	void prepare(float dt);
	
	int getTimesBlocked() const { return nr_times_blocked_; }
	void setBlocked() { ++nr_times_blocked_; }
private:
	//std::string id_;
	float desired_rotation_;
	Structure* structure_;
	RotateBehaviour* valve_rotate_behaviour_;
	
	std::vector<ValveGoal*> valve_goals_;
	
	int nr_times_blocked_;
	
	//static int global_valve_id_;
};

#endif
