#ifndef DEMO_PANDORA_STRUCTURES_CHAIN_H
#define DEMO_PANDORA_STRUCTURES_CHAIN_H

#include <glm/glm.hpp>
#include <string>

#include "../../../core/entities/Entity.h"
#include "Structure.h"

class OntologyInterface;
class SceneManager;
class SceneNode;
class InspectionPoint;
class MissionSite;
class ChainGoal;
class ChainLink;
class HeightMap;

/**
 * Waypoint:"canSeePillar" -> Pillar
 * Waypoint:"canSee" -> InspectionPoint
 * InspectionPoint:"isPartOfPillar" -> Pillar
 * 
 * 
 */
class Chain : public Structure
{
public:
	Chain(MissionSite& mission_site, SceneManager& scene_manager, SceneNode* parent, HeightMap& height_map, const glm::mat4& transformation, unsigned int nr_chain_links);
	const std::string& getId() const { return id_; }
	void setObserved();// { has_been_observed_ = true; }
	bool hasBeenObserved() const { return has_been_observed_; }
	void setNotificationSent(bool notification_sent) { notification_sent_ = notification_sent; }
	bool notifactionHasBeenSent() const { return notification_sent_; }
	ChainGoal& getGoal() const { return *chain_goal_; }
	
	void setExamined() { has_been_examined_ = true; }
	bool isExamined() const { return has_been_examined_; }
	MissionSite& getMissionSite() const { return *mission_site_; }
	
private:
	std::string id_;
	bool has_been_observed_;
	bool notification_sent_;
	bool has_been_examined_;
	MissionSite* mission_site_;
	ChainGoal* chain_goal_;
	std::vector<ChainLink*> chain_links_;
	
	static int global_chain_id_;
};

#endif
