#ifndef DEMO_PANDORA_CONTROLLERS_CHAIN_FOLLOW_CONTROLLER_H
#define DEMO_PANDORA_CONTROLLERS_CHAIN_FOLLOW_CONTROLLER_H

#include <set>
#include <glm/glm.hpp>

#include <ros/ros.h>


#include "PlannerAction.h"

class AUV;
class Chain;
class Line;
class SceneManager;
class SceneNode;
class SceneLeafModel;
class OntologyInterface;
class HeightMap;

class ChainFollowController : public PlannerAction
{
public:
	
	ChainFollowController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub, HeightMap& height_map);
	
	void followChain(Chain& chain, float max_time);
	
	void amendFeedback(planning_msgs::ActionFeedback& feedback, PLANNER_ACTION_STATUS status);
	
	void update(float dt);
	
	PlannerAction::PLANNER_ACTION_STATUS getStatus();

private:
	SceneManager* scene_manager_;
	AUV* auv_;
	Chain* chain_;
	glm::vec3 latest_discovered_chain_loc_;
	glm::vec3 general_heading_;

	std::set<SceneNode*> observed_chain_links_;
	float time_, max_time_;
	
	SceneLeafModel* path_;
	Line* line_;
	Line* colliding_line_;
	Line* grid_line_;
	HeightMap* height_map_;
	
	bool turn_left_;
	float turning_time_;
	
	float rotating_time_;
	
	std::vector<glm::vec3> search_pattern_;
};

#endif
