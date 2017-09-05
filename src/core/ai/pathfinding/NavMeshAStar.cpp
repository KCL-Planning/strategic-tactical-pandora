#include <queue>
#include <set>
#include <sstream>

#ifdef _WIN32
#include <windows.h>
#endif

#include "NavMeshAStar.h"

#include "ConvexNavigationArea.h"
#include "NavMeshNode.h"
#include "../../math/Plane.h"
#include "../../math/Math.h"

//#define NAV_MESH_A_STAR_DEBUG


NavMeshAStar::NavMeshAStar(const std::vector<ConvexNavigationArea>& areas)
	: areas_(&areas)
{

}

bool NavMeshAStar::findPath(const glm::vec3& begin_point, const glm::vec3& end_point, std::vector<glm::vec3>& waypoints)
{
	// Find the convex navigation area to start at and end with.
	const ConvexNavigationArea* begin = getArea(begin_point);
		
	const ConvexNavigationArea* end = getArea(end_point);
	
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Find path from: (" << begin_point.x << "," << begin_point.y << "," << begin_point.z << ") to (" << end_point.x << "," << end_point.y << "," << end_point.z << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#else
	std::cout << "Find path from: (" << begin_point.x << "," << begin_point.y << "," << begin_point.z << ") to (" << end_point.x << "," << end_point.y << "," << end_point.z << ")" << std::endl;
#endif

	if (begin != NULL)
	{
		std::cout << "Begin is at: " << *begin << std::endl;
	}
	else
	{
		std::cout << "Begin is not found!" << std::endl;
	}
	
	if (end != NULL)
	{
		std::cout << "End is at: " << *end << std::endl;
	}
	else
	{
		std::cout << "End is not found!" << std::endl;
	}
#endif
	
	if (begin == NULL || end == NULL)
	{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		std::stringstream ss;
		ss << "Begin or end is NULL :((" << std::endl;
		OutputDebugString(ss.str().c_str());
#else
		std::cout << "Begin or end is NULL :((" << std::endl;
#endif
#endif
		return false;
	}
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Begin: " << std::endl;
	ss << *begin << std::endl;
	ss << "End: " << std::endl;
	ss << *end << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#else
	std::cout << "Begin: " << std::endl;
	std::cout << *begin << std::endl;
	std::cout << "End: " << std::endl;
	std::cout << *end << std::endl;
#endif
#endif
	std::set<const CNA_Adjacent*> closed_list;
	//waypoints.push_back(begin_point);
	std::priority_queue<NavMeshNode*, std::vector<NavMeshNode*>,  NavMeshNode> open_list;

	//NavMeshNode* start_node = new NavMeshNode(NULL, *begin, static_cast<CNA_Adjacent*>(NULL), waypoints, 0, glm::distance(begin_point, end_point));
	NavMeshNode* start_node = new NavMeshNode(NULL, *begin, static_cast<CNA_Adjacent*>(NULL), begin_point, 0, glm::distance(begin_point, end_point));
	open_list.push(start_node);

	while (open_list.size() > 0)
	{
		NavMeshNode* current_node = open_list.top();
		open_list.pop();

		if (closed_list.find(current_node->entry_point_) != closed_list.end())
		{
			continue;
		}

		closed_list.insert(current_node->entry_point_);
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		std::stringstream ss;
		ss << "Process: " << *current_node << std::endl;
		ss << "Adjacent areas: " << current_node->area_->getAdjacentAreas().size() << std::endl;
		OutputDebugString(ss.str().c_str());
#else
		std::cout << "Process: " << *current_node << std::endl;
		std::cout << "Adjacent areas: " << current_node->area_->getAdjacentAreas().size() << std::endl;
#endif
#endif
		if (current_node->area_ == end)
		{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			OutputDebugString("We are done! :D");
#else
			std::cout << "We are done! :D" << std::endl;
			for (std::vector<glm::vec3>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
			{
				std::cout << "\t* (" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
			}
#endif
#endif
			postOptimise(*current_node, begin_point, end_point, waypoints);
			
			return true;
		}

		// Check if the end point is reachable from this point.
		for (std::vector<const CNA_Adjacent*>::const_iterator ci = current_node->area_->getAdjacentAreas().begin(); ci != current_node->area_->getAdjacentAreas().end(); ++ci)
		{
			const CNA_Adjacent* adjacent = *ci;
			ConvexNavigationArea* next_area = adjacent->adjacent1_ == current_node->area_ ? adjacent->adjacent2_ : adjacent->adjacent1_;

			glm::vec3 waypoint((adjacent->p1_ + adjacent->p2_) / 2.0f);
			NavMeshNode* next_node = new NavMeshNode(current_node, *next_area, adjacent, waypoint, current_node->distance_from_start_ + glm::distance(current_node->waypoint_, waypoint), glm::distance(waypoint, end_point));
			open_list.push(next_node);
		}
	}
	return false;
}

void NavMeshAStar::postOptimise(const NavMeshNode& node, const glm::vec3& begin_point, const glm::vec3& end_point, std::vector<glm::vec3>& waypoints)
{
	glm::vec3 current_waypoint = end_point;
	waypoints.push_back(end_point);
	const NavMeshNode* current_node = &node;
	
	while (current_node->parent_ != NULL)
	{
		NavMeshNode* next_node = current_node->parent_;
		if (next_node->parent_ == NULL)
		{
			waypoints.insert(waypoints.begin(), next_node->waypoint_);
			break;
		}
		
		glm::vec3 next_next_waypoint = next_node->parent_->waypoint_;
		
		// If the next waypoint does not lie in line with the current direction, we move the waypoint a little
		// further away so the monster does not get stuck on corners.
		/*
		if (glm::dot(glm::normalize(next_next_waypoint - next_node->waypoint_), glm::normalize(next_node->waypoint_ - current_node->waypoint_)) < 0.9f)
		{
			next_node->waypoint_ = next_node->waypoint_ + glm::normalize(current_node->waypoint_ - next_node->waypoint_) * 3.0f;
		}
		else
		*/
		{
			// Check if the line between current_waypoint and next_next_waypoint intersects with
			// the adjacent line of the current node. If it does, then we use the intersection as
			// the waypoint.
			glm::vec2 intersection;
			glm::vec2 begin(current_waypoint.x, current_waypoint.z);
			glm::vec2 end(next_next_waypoint.x, next_next_waypoint.z);
			
			glm::vec2 line_begin(next_node->entry_point_->p1_.x, next_node->entry_point_->p1_.z);
			glm::vec2 line_end(next_node->entry_point_->p2_.x, next_node->entry_point_->p2_.z);
			
			glm::vec2 safe_line_begin(line_begin + glm::normalize(line_end - line_begin));
			glm::vec2 safe_line_end(line_end + glm::normalize(line_begin - line_end));
			
			if (/*glm::distance(line_begin, line_end) > 5.0f && */Math::getIntersectionSegments(begin, end, safe_line_begin, safe_line_end, intersection))
			{
				next_node->waypoint_ = glm::vec3(intersection.x, line_begin.y, intersection.y);
			}
			// If there is no intersection, then we will pick the location on the adjacent line that is closest to the next point.
			else
			{
				/*
				float d1 = glm::distance(next_next_waypoint, next_node->entry_point_->p1_);
				float d2 = glm::distance(next_next_waypoint, next_node->entry_point_->p2_);
				
				if (d1 < d2)
				{
					next_node->waypoint_ = d1 + glm::normalize(next_node->entry_point_->p2_ - next_node->entry_point_->p1_) * 0.1f;
					
				}
				else
				{
					next_node->waypoint_ = d2 + glm::normalize(next_node->entry_point_->p1_ - next_node->entry_point_->p2_) * 0.1f;
				}
				*/
			}
		}
		waypoints.insert(waypoints.begin(), next_node->waypoint_);
		current_node = next_node;
	}
}

const ConvexNavigationArea* NavMeshAStar::getArea(const glm::vec3& point) const
{
	//std::cout << "[NavMeshAStar::getArea] (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	const ConvexNavigationArea* current_area = NULL;
	float closest_distance = std::numeric_limits<float>::max();
	for (std::vector<ConvexNavigationArea>::const_iterator ci = areas_->begin(); ci != areas_->end(); ++ci)
	{
		const ConvexNavigationArea& area = *ci;
		Plane area_plane(area.getPoints());
		
		//std::cout << "Check against: " << area << std::endl;

		glm::vec3 intersecting_point;
		if (!area_plane.intersectsWithRay(point, glm::vec3(0, -1, 0), intersecting_point))
		{
			continue;
		}
		float t = glm::distance(intersecting_point, point);
		if (t < closest_distance)
		{
			closest_distance = t;
			current_area = &area;
		}
	}
	return current_area;
}
