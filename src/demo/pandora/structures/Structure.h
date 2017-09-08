#ifndef DEMO_PANDORA_STRUCTURES_STRUCTURE_H
#define DEMO_PANDORA_STRUCTURES_STRUCTURE_H

#include <vector>
#include <glm/glm.hpp>
#include "dpengine/entities/Entity.h"
#include "../level/MissionSite.h"

namespace DreadedPE
{
	class SceneNode;
	class SceneManager;
};

class MissionSite;
class Valve;

class InspectionPoint;
class InspectionGoal;
class OntologyInterface;

class Structure : public DreadedPE::Entity
{
public:
	Structure(const std::string& name, const std::string& plf_file_name, const std::string& texture_file_name, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation, const std::vector<InspectionPoint*>& inspection_points, bool separate_inspection_goals = false);
	Structure(const std::string& name, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, MissionSite& mission_site, const glm::mat4& transformation);
	
	const std::vector<InspectionPoint*>& getInspectionPoints() const { return inspection_points_; }
	const std::vector<InspectionGoal*>& getInspectionGoals() const { return inspection_goals_; }
	MissionSite& getMissionSite() const { return *mission_site_; }
	
	const glm::vec3& getInteractLocation() const { return interact_location_; }
	
	void addValve(Valve& valve);
	const std::vector<Valve*>& getValves() const { return valves_; }
	bool isExamined() const { return is_examined_; }
	void setExamined(bool is_examined) { is_examined_ = is_examined; }
	void setCanRecharge(bool can_recharge) { can_recharge_ = can_recharge; }
	bool canRecharge() const { return can_recharge_; }
	
	void makeBright(SceneNode& scene_node);
	
	void destroy();
	
protected:
	MissionSite* mission_site_;                        // The mission site this structure is part of.
	
	std::vector<InspectionPoint*> inspection_points_;  // The inspection points of this structure that must be explored.
	std::vector<InspectionGoal*> inspection_goals_;    // The actual inspection goal.
	
	bool is_examined_;                                 // Track whether this structure has been examined or not.
	std::vector<Valve*> valves_;                       // The valves that are part of this panel.
	glm::vec3 interact_location_;                      // The location the AUV needs to be in order to interact with the valve.
	bool can_recharge_;                                // Whether this structure can recharge the AUV.
};

#endif
