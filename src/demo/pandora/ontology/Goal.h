#ifndef DEMO_PANDORA_ONTOLOGY_GOAL_H
#define DEMO_PANDORA_ONTOLOGY_GOAL_H

#include <string>
#include <vector>

#include <glm/glm.hpp>

class Structure;

/**
 * Interface for all the goals that the planner needs to achieve.
 */
class Goal
{
public:
	/**
	 * Set the goal ID automatically.
	 */
	Goal(Structure& mission_site);
	
	virtual ~Goal();
	
	/**
	 * Reset the goal on a new run.
	 */
	virtual void reset() { }
	
	/**
	 * @return The goal ID.
	 */
	const std::string& getId() const { return id_; }
	
	/**
	 * @return The mission site this goal is a part of.
	 */
	Structure& getStructure() const { return *structure_; }
	
	/**
	 * Return whether the goal should be exposed to the AUV.
	 */
	virtual bool isEnabled() const { return true; }
	
	/**
	 * Get all the strategic points that relevant for achieving this goal.
	 */
	virtual void getStrategicPoints(std::vector<glm::vec3>& points) const { }
private:
	std::string id_; // The goal id
	Structure* structure_; // The mission site where this goal resides.
	static int next_goal_id_;
};

#endif
