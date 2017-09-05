#include "ConnectionFilter.h"
#include "../OctomapBuilder.h"

ConnectionFilter::ConnectionFilter(const knowledge_msgs::Filter& filter_msg, const glm::vec3& begin, const glm::vec3& end, OctomapBuilder& octomap, float min_distance_to_obstacle)
	: Filter(filter_msg), begin_(begin), end_(end), octomap_(&octomap), min_distance_to_obstacle_(min_distance_to_obstacle)
{
	points_.push_back(begin);
	points_.push_back(end);
}

bool ConnectionFilter::checkFilter()
{
	if (has_been_triggered_) return true;
	if (octomap_->isBlocked(begin_, end_, min_distance_to_obstacle_))
	{
		has_been_triggered_ = true;
		return false;
	}
	return true;
}
