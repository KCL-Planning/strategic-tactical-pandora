#ifndef DEMO_PANDORA_ONTOLOGY_FILTER_CONNECTION_FILTER_H
#define DEMO_PANDORA_ONTOLOGY_FILTER_CONNECTION_FILTER_H

#include <glm/glm.hpp>

#include "Filter.h"

class OctomapBuilder;

/**
 * A filter that checks the connectivity between waypoints.
 */
class ConnectionFilter : public Filter
{
public:
	/**
	 * Construct the filter that checks the connections between @ref{begin} and @ref{end} with 
	 * the @ref{octomap}, if this connection ever becomes blocked the filter is violated.
	 * @param filter_msg The filter message this filter is based on.
	 * @param begin The start point of the line segment.
	 * @param end The end point of the line segment.
	 * @param octomap The octomap which contains the observed blocked areas.
	 * @param min_distance_to_obstacle The minimal 
	 */
	ConnectionFilter(const knowledge_msgs::Filter& filter_msg, const glm::vec3& begin, const glm::vec3& end, OctomapBuilder& octomap, float min_distance_to_obstacle);
	
	/**
	 * This function returns false if the connectivity between the waypoints is broken.
	 * @return False if the waypoints are no longer connected, true otherwise.
	 */
	bool checkFilter();
private:
	glm::vec3 begin_, end_; // The begin and end point of the edge we need to check.
	OctomapBuilder* octomap_; // The octomap.
	float min_distance_to_obstacle_;
};

#endif
