#ifndef CORE_AI_PATHFINDING_NAV_MESH_NODE_H
#define CORE_AI_PATHFINDING_NAV_MESH_NODE_H

#include <vector>
#include <glm/glm.hpp>

#include "ConvexNavigationArea.h"

struct NavMeshNode
{
	NavMeshNode()
		: parent_(NULL), area_(NULL), distance_from_start_(0), distance_to_end_(0)
	{

	}

	NavMeshNode(const ConvexNavigationArea& area, float distance_to_start, float distance_to_end)
		: parent_(NULL), area_(&area), distance_from_start_(distance_to_start), distance_to_end_(distance_to_end)
	{

	}

	NavMeshNode(NavMeshNode* parent, const ConvexNavigationArea& area, const CNA_Adjacent* entry_point, const glm::vec3& waypoint, float distance_from_start, float distance_to_end)
		: parent_(parent), area_(&area), entry_point_(entry_point), waypoint_(waypoint), distance_from_start_(distance_from_start), distance_to_end_(distance_to_end)
	{

	}
	/*
	NavMeshNode(NavMeshNode* parent, const ConvexNavigationArea& area, const CNA_Adjacent* entry_point, const std::vector<glm::vec3>& waypoints, float distance_from_start, float distance_to_end)
		: parent_(parent), area_(&area), entry_point_(entry_point), waypoints_(waypoints), distance_from_start_(distance_from_start), distance_to_end_(distance_to_end)
	{

	}
	*/

	bool operator() (const NavMeshNode* lhs, const NavMeshNode* rhs) const
	{
		return lhs->distance_from_start_ + lhs->distance_to_end_ > rhs->distance_from_start_ + rhs->distance_to_end_;
	}

	NavMeshNode* parent_;
	const ConvexNavigationArea* area_;
	const CNA_Adjacent* entry_point_;
	//std::vector<glm::vec3> waypoints_;
	glm::vec3 waypoint_;
	float distance_from_start_;
	float distance_to_end_;
};

std::ostream& operator<<(std::ostream& os, const NavMeshNode& node)
{
	os << *node.area_ << std::endl;
	os << "Waypoint: ";
	os << "(" << node.waypoint_.x << "," << node.waypoint_.y << "," << node.waypoint_.z << ")" << " - ";
	//os << "Waypoints: ";
	//for (std::vector<glm::vec3>::const_iterator ci = node.waypoints_.begin(); ci != node.waypoints_.end(); ++ci)
	//{
	//	os << "(" << (*ci).x << "," << (*ci).y << "," << (*ci).z << ")" << " - ";
	//}
	os << std::endl;
	os << "Distance to start: " << node.distance_from_start_ << "." << std::endl;
	os << "Distance to end: " << node.distance_to_end_ << "." << std::endl;
	return os;
}

#endif
