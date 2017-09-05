#ifndef DEMO_PANDORA_WAYPOINT_H
#define DEMO_PANDORA_WAYPOINT_H

#include <sstream>
#include <vector>
#include <set>
#include <glm/glm.hpp>

struct Waypoint
{
	Waypoint(const std::string& id, const glm::vec3& position, const std::vector<bool>& is_connected_to_start)
		: id_(id), position_(position), is_connected_to_start_(is_connected_to_start)
	{

	}
	
	Waypoint(int id, const glm::vec3& position, const std::vector<bool>& is_connected_to_start)
		: position_(position), is_connected_to_start_(is_connected_to_start)
	{
		std::stringstream ss;
		ss << "W" << id;
		id_ = ss.str();
	}
	
	Waypoint(const std::string& id, const glm::vec3& position)
		: id_(id), position_(position)
	{
		
	}
	
	void updateConnectivity(std::set<Waypoint*>& processed_waypoints, unsigned int start_id)
	{
		is_connected_to_start_[start_id] = true;
		processed_waypoints.insert(this);
		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = edges_.begin(); ci != edges_.end(); ++ci)
		{
			if (processed_waypoints.count((*ci).first) != 1)
			{
				(*ci).first->updateConnectivity(processed_waypoints, start_id);
			}
		}
	}
	
	std::string id_;
	glm::vec3 position_;
	std::vector<std::pair<Waypoint*, float> > edges_;
	std::vector<bool> is_connected_to_start_;
};

std::ostream& operator<<(std::ostream& os, const Waypoint& waypoint);

#endif
