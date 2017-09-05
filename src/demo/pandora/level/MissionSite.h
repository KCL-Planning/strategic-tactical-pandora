#ifndef PANDORA_LEVEL_MISSION_SITE_H
#define PANDORA_LEVEL_MISSION_SITE_H

#include <vector>

#include "../../../core/entities/Entity.h"
#include "../ontology/InspectionGoal.h"

class Waypoint;
class InspectionGoal;
class InspectionPoint;
class OntologyInterface;
class Pillar;
class Chain;
class Mission;
class Structure;

/**
 * Load a mission site from a level description in the PLF format.
 */
class MissionSite : public Entity
{
public:

	/**
	 * Create a mission site in the world, without any mesh.
	 * @param id The mession site id (to be used in the ontology).
	 * @param scene_manager The scene manager.
	 * @param parent The parent node of this entity.
	 * @param transformation The transformation relative to its parent.
	 */
	MissionSite(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const glm::vec3& start_point, OntologyInterface& ontology);
	
	/**
	 * Get methods.
	 */
	const std::string& getId() const { return id_; }
	Waypoint& getStartWaypoint() const { return *start_waypoint_; }
	
	//const std::vector<ValvePanel*>& getPanels() const { return panels_; }
	const std::vector<Pillar*>& getPillars() const { return pillars_; }
	//InspectionGoal& getInspectionGoal() { return inspection_goal_; }
	const std::vector<Waypoint*>& getWaypoints() const { return waypoints_; }
	const std::vector<Chain*>& getChains() const { return chains_; }
	const std::vector<Mission*>& getMissions() const { return missions_; }
	const std::vector<Structure*>& getStructures() const { return structures_; }
	
	/**
	 * Remove methods.
	 */
	void removeStructure(const Structure& structure);
	
	// Stuff related to recharging actions.
	bool canRecharge() const;
	
	/**
	 * Set methods.
	 */
	//void addPanel(ValvePanel& panel) { panels_.push_back(&panel); }
	void addPillar(Pillar& pillar) { pillars_.push_back(&pillar); }
	//void addInspectionPoint(InspectionPoint& inspection_point) { inspection_goal_.addInspectionPoint(inspection_point); }
	void addWaypoint(Waypoint& waypoint) { waypoints_.push_back(&waypoint); }
	void addChain(Chain& chain) { chains_.push_back(&chain); }
	void addMission(Mission& mission) { missions_.push_back(&mission); }
	
	void addStructure(Structure& structure);
	
	
	//void setPanels(const std::vector<ValvePanel*>& panels) { panels_ = panels; }
	void setPillars(const std::vector<Pillar*>& pillars) { pillars_ = pillars; }
	//void setInspectionPoints(const std::vector<InspectionPoint*>& inspection_points) { inspection_goal_.setInspectionPoints(inspection_points); }
	void setWaypoints(const std::vector<Waypoint*>& waypoints) { waypoints_ = waypoints; }
	void setChains(const std::vector<Chain*>& chains) { chains_ = chains; }
	
protected:
	static int global_mission_site_id_;

	std::string id_;
	Waypoint* start_waypoint_;
	//std::vector<ValvePanel*> panels_;
	std::vector<Pillar*> pillars_;
	std::vector<Chain*> chains_;
	//InspectionGoal inspection_goal_;
	std::vector<Waypoint*> waypoints_;
	std::vector<Mission*> missions_;
	
	std::vector<Structure*> structures_;
	bool can_recharge_;
	
	OntologyInterface* ontology_;
};

#endif
