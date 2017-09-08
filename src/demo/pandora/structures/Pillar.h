#ifndef DEMO_PANDORA_STRUCTURES_PILLAR_H
#define DEMO_PANDORA_STRUCTURES_PILLAR_H

#include <memory>
#include <glm/glm.hpp>
#include <string>

#include "dpengine/entities/Entity.h"

class OntologyInterface;
namespace DreadedPE
{
	class SceneManager;
	class SceneNode;
	class Texture;
};
class InspectionPoint;
class MissionSite;

/**
 * Waypoint:"canSeePillar" -> Pillar
 * Waypoint:"canSee" -> InspectionPoint
 * InspectionPoint:"isPartOfPillar" -> Pillar
 * 
 * 
 */
class Pillar : public DreadedPE::Entity
{
public:
	Pillar(const std::string& name, MissionSite& mission_site, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::string& level_file_name, DreadedPE::Texture& texture);
	//int getId() const { return id_; }
	void setObserved();// { has_been_observed_ = true; }
	bool hasBeenObserved() const { return has_been_observed_; }
	void setNotificationSent(bool notification_sent) { notification_sent_ = notification_sent; }
	bool notifactionHasBeenSent() const { return notification_sent_; }
	void addInspectionPoint(InspectionPoint& inspection_point) { inspection_points_.push_back(&inspection_point); }
	const std::vector<InspectionPoint*>& getInspectionPoint() const { return inspection_points_; }
	void prepare(float dt);
private:

	float shinny_timer_;
	void makeBright(SceneNode& scene_node);
	//int id_;
	bool has_been_observed_;
	bool notification_sent_;
	std::vector<InspectionPoint*> inspection_points_;
	MissionSite* mission_site_;
	std::shared_ptr<DreadedPE::Material> wfl_material_;
};

#endif
