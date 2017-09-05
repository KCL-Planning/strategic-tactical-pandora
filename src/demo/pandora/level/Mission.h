#ifndef DEMO_PANDORA_LEVEL_MISSION_H
#define DEMO_PANDORA_LEVEL_MISSION_H

#include <string>
#include <vector>

#include <glm/glm.hpp>

class Goal;
class MissionSite;

class Mission
{
public:
	Mission(MissionSite& mission_site);
	
	const std::string& getId() const { return id_; }
	void addGoal(Goal& goal);
	const std::vector<Goal*>& getGoals() const { return goals_; }
	void getStrageticPoints(std::vector<glm::vec3>& points) const;
private:
	static int global_mission_id_;
	std::string id_;
	MissionSite* mission_site_;
	
	std::vector<Goal*> goals_;
};

#endif
