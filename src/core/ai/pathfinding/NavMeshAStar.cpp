#include <queue>
#include <algorithm>
#include <set>
#include <sstream>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

#include "dpengine/ai/pathfinding/NavMeshAStar.h"

#include "dpengine/ai/pathfinding/ConvexNavigationArea.h"
#include "dpengine/ai/pathfinding/NavMeshNode.h"
#include "dpengine/math/Plane.h"
#include "dpengine/math/Math.h"

//#define NAV_MESH_A_STAR_DEBUG

namespace DreadedPE
{

NavMeshAStar::NavMeshAStar(const std::vector<ConvexNavigationArea*>& areas)
	: areas_(&areas), start_stack_(NULL), end_stack_(NULL)
{

}

bool NavMeshAStar::findPath(const glm::vec3& begin_point, const glm::vec3& end_point, std::vector<glm::vec3>& waypoints)
{
	// Find the convex navigation area to start at and end with.
	const ConvexNavigationArea* begin = getArea(begin_point);

	for (unsigned int i = 0; i < 10; ++i)
	{
		if (begin != NULL)
			break;
		begin = getArea(begin_point + ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f);
	}
		
	const ConvexNavigationArea* end = getArea(end_point);
	for (unsigned int i = 0; i < 10; ++i)
	{
		if (end != NULL)
			break;
		end = getArea(begin_point + ((float)rand() / (float)RAND_MAX - 0.5f) * 2.0f);
	}

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
	std::priority_queue<NavMeshNode*, std::vector<NavMeshNode*>,  NavMeshNode> open_list;

	NavMeshNode* start_node = new NavMeshNode(NULL, *begin, static_cast<CNA_Adjacent*>(NULL), begin_point, 0, glm::distance(begin_point, end_point));
	open_list.push(start_node);

	while (open_list.size() > 0)
	{
		NavMeshNode* current_node = open_list.top();
		open_list.pop();

		if (current_node->entry_point_ != NULL && closed_list.find(current_node->entry_point_) != closed_list.end())
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
			NavMeshNode* next_node = new NavMeshNode(current_node, *current_node->area_, NULL, end_point, current_node->distance_from_start_ + glm::distance(current_node->waypoint_, end_point), 0.0f);
			open_list.push(next_node);
		}

		if (glm::distance(current_node->waypoint_, end_point) < 0.01f)
		{
			//NavMeshNode* end_node = new NavMeshNode(current_node, *current_node->area_, NULL, end_point, current_node->distance_from_start_ + glm::distance(current_node->waypoint_, end_point), 0);

#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			std::stringstream ss;
			ss << "We are done! :D" << std::endl;
			ss << "Non-optimised path: {";
			NavMeshNode* tmp_node2 = current_node;
			while (tmp_node2 != NULL)
			{
				ss << "(" << tmp_node2->waypoint_.x << ", " << tmp_node2->waypoint_.y << ", " << tmp_node2->waypoint_.z << "), ";
				tmp_node2 = tmp_node2->parent_;
			}
			ss << "}" << std::endl;

			OutputDebugString(ss.str().c_str());
			ss.str(std::string());
#else
			std::cout << "We are done! :D" << std::endl;
			for (std::vector<glm::vec3>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
			{
				std::cout << "\t* (" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
			}
#endif
#endif

			postOptimise(*current_node, waypoints);
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "After 1st pass: {";
			for (const glm::vec3& p : waypoints)
			{
				ss << "(" << p.x << ", " << p.y << ", " << p.z << "), ";
			}
			ss << "}" << std::endl;
			OutputDebugString(ss.str().c_str());
			ss.str(std::string());
#endif
#endif
			// If there is only a single point (begin and end are the same), just return it.
			if (waypoints.size() < 2) return true;
			/*
			// Flip parents and run again.
			NavMeshNode* tmp_node = current_node;
			NavMeshNode* next_node = tmp_node->parent_;
			NavMeshNode* next_parent = next_node->parent_;
			tmp_node->parent_ = NULL;
			while (next_parent != NULL)
			{
				next_node->parent_ = tmp_node;
				tmp_node = next_node;
				next_node = next_parent;
				next_parent = next_parent->parent_;
			}
			next_node->parent_ = tmp_node;
			waypoints.clear();
			postOptimise(*next_node, waypoints);

			std::reverse(waypoints.begin(), waypoints.end());
			*/
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

void NavMeshAStar::postOptimise(NavMeshNode& node, std::vector<glm::vec3>& waypoints)
{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	std::stringstream ss;
	ss << "Post Optimise." << std::endl;
#endif
#endif
	waypoints.push_back(node.waypoint_);
	NavMeshNode* current_node = &node;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	ss << "Add: (" << node.waypoint_.x << ", " << node.waypoint_.y << ", " << node.waypoint_.z << ")" << std::endl;
#endif
#endif
	
	while (current_node->parent_ != NULL)
	{
		NavMeshNode* next_node = current_node->parent_;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		ss << "Current node: " << *current_node << std::endl;
		ss << "Next node: " << *next_node << std::endl;
#endif
#endif
		// If this is the last node, we have arrived at the initial location.
		if (next_node->parent_ == NULL)
		{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "The last node, we are done (" << next_node->waypoint_.x << ", " << next_node->waypoint_.y << ", " << next_node->waypoint_.z << ")!" << std::endl;
			ss << "Add: (" << next_node->waypoint_.x << ", " << next_node->waypoint_.y << ", " << next_node->waypoint_.z << ")" << std::endl;
#endif
#endif
			if (start_stack_ != NULL)
			{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
				OutputDebugString(ss.str().c_str());
				ss.str(std::string());
#endif
#endif
				resolveStack(waypoints);
			}
			waypoints.insert(waypoints.begin(), next_node->waypoint_);	
			break;
		}
		
		// Test whether we can skip this node and go straight from the current waypoint to the
		// parent of next_node. This will optimise the actual path and make the path look more 
		// 'natural'.
		glm::vec3 next_next_waypoint = next_node->parent_->waypoint_;
		
		// Check if the line between current_waypoint and next_next_waypoint intersects with
		// the adjacent line of the current node. If it does, then we use the intersection as
		// the waypoint.
		glm::vec2 intersection;
		glm::vec2 begin(current_node->waypoint_.x, current_node->waypoint_.z);
		if (start_stack_ != NULL)
		{
			begin.x = start_stack_->waypoint_.x;
			begin.y = start_stack_->waypoint_.z;
		}
		glm::vec2 end(next_next_waypoint.x, next_next_waypoint.z);
		
		glm::vec2 line_begin(next_node->entry_point_->p1_.x, next_node->entry_point_->p1_.z);
		glm::vec2 line_end(next_node->entry_point_->p2_.x, next_node->entry_point_->p2_.z);
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		ss << "Check the line: (" << begin.x << ", " << begin.y << ") - (" << end.x << ", " << end.y << ")" << std::endl;
		ss << "Intersection: (" << line_begin.x << ", " << line_begin.y << ") - (" << line_end.x << ", " << line_end.y << ")" << std::endl;
#endif
#endif
		
		if (Math::getIntersectionSegments(begin, end, line_begin, line_end, intersection))
		{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "Intersection found on the segments at: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
#endif
#endif
			bool is_valid = true;
			if (start_stack_ == NULL)
			{
				start_stack_ = current_node;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
				ss << "New start of the stack. " << std::endl;
				ss << *start_stack_ << std::endl;
#endif
#endif
			}
			else
			{
				// The point must pass through each intersection of all points.
				NavMeshNode* n = start_stack_;
				while (n != end_stack_)
				{
					glm::vec2 n_line_begin(n->parent_->entry_point_->p1_.x, n->parent_->entry_point_->p1_.z);
					glm::vec2 n_line_end(n->parent_->entry_point_->p2_.x, n->parent_->entry_point_->p2_.z);
					if (!Math::getIntersectionSegments(begin, end, n_line_begin, n_line_end, intersection))
					{
						is_valid = false;
						break;
					}
					n = n->parent_;
				}
			}

			if (is_valid)
			{
				end_stack_ = next_node->parent_;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
				ss << "Add to the end of the stack: " << std::endl;
				ss << *end_stack_ << std::endl;
#endif
#endif
				current_node = next_node;
				continue;
			}
		}
		
		// Resolve the stack.
		if (start_stack_ != NULL)
		{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			OutputDebugString(ss.str().c_str());
			ss.str(std::string());
#endif
#endif
			resolveStack(waypoints);
			current_node = current_node->parent_;
			continue;
		}

		Math::getIntersection(begin, end, line_begin, line_end, intersection);
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32

		ss << "No intersections found on the segments!(" << intersection.x << ", " << intersection.y << ")" << std::endl;
#endif
#endif
		// If there is no intersection, then we will pick the location on the  adjacent line that minimizes the distances between the current and next-next waypoint.
		float d1 = glm::distance(next_next_waypoint, next_node->entry_point_->p1_) + glm::distance(current_node->waypoint_, next_node->entry_point_->p1_);
		float d2 = glm::distance(next_next_waypoint, next_node->entry_point_->p2_) + glm::distance(current_node->waypoint_, next_node->entry_point_->p2_);
			
		glm::vec3 original_waypoint = next_node->waypoint_;
		if (d1 < d2)
		{
			next_node->waypoint_ = next_node->entry_point_->p1_ + glm::normalize(next_node->entry_point_->p2_ - next_node->entry_point_->p1_) * 1.5f;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "Intermediate waypoint is close to p1, updating waypoint." << std::endl;
#endif
#endif
		}
		else
		{
			next_node->waypoint_ = next_node->entry_point_->p2_ + glm::normalize(next_node->entry_point_->p1_ - next_node->entry_point_->p2_) * 1.5f;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "Intermediate waypoint is close to p2, updating waypoint." << std::endl;
#endif
#endif
		}

		// Check if this waypoints falls between the p1 - p2 line segment.
		if (Math::dist3D_Segment_to_Point(next_node->entry_point_->p1_, next_node->entry_point_->p2_, next_node->waypoint_) > 0.1f)
		{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "The updated waypoint: (" << next_node->waypoint_.x << ", " << next_node->waypoint_.y << ", " << next_node->waypoint_.z << ") does not lie on the intersection. Reverting to original waypoint." << std::endl;
#endif
#endif
			next_node->waypoint_ = original_waypoint;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
			ss << "Updated waypoint: (" << next_node->waypoint_.x << ", " << next_node->waypoint_.y << ", " << next_node->waypoint_.z << ")" << std::endl;
#endif
#endif
		}
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		ss << "Add: (" << next_node->waypoint_.x << ", " << next_node->waypoint_.y << ", " << next_node->waypoint_.z << ")" << std::endl;
#endif
#endif
		waypoints.insert(waypoints.begin(), next_node->waypoint_);
		current_node = next_node;
	}
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	OutputDebugString(ss.str().c_str());
#endif
#endif
}

void NavMeshAStar::resolveStack(std::vector<glm::vec3>& waypoints)
{
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	std::stringstream ss;
	ss << "Resolve the stack:" << std::endl;
	ss << *start_stack_ << std::endl;
	ss << " === TO === " << std::endl;
	ss << *end_stack_ << std::endl;
	ss << " ========== " << std::endl;
	ss << "Add: (" << start_stack_->waypoint_.x << ", " << start_stack_->waypoint_.y << ", " << start_stack_->waypoint_.z << ")" << std::endl;
#endif
#endif
	//waypoints.insert(waypoints.begin(), start_stack->waypoint_);

	glm::vec2 intersection;
	glm::vec2 begin(start_stack_->waypoint_.x, start_stack_->waypoint_.z);
	glm::vec2 end(end_stack_->waypoint_.x, end_stack_->waypoint_.z);

	NavMeshNode* tmp_node = start_stack_->parent_;
	while (tmp_node != end_stack_)
	{
		glm::vec2 line_begin(tmp_node->entry_point_->p1_.x, tmp_node->entry_point_->p1_.z);
		glm::vec2 line_end(tmp_node->entry_point_->p2_.x, tmp_node->entry_point_->p2_.z);

#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		ss << "Update: " << std::endl;
		ss << *tmp_node << std::endl;
		ss << "Check the line: (" << begin.x << ", " << begin.y << ") - (" << end.x << ", " << end.y << ")" << std::endl;
		ss << "Intersection: (" << line_begin.x << ", " << line_begin.y << ") - (" << line_end.x << ", " << line_end.y << ")" << std::endl;
#endif
#endif

		//if (!Math::getIntersectionSegments(begin, end, line_begin, line_end, intersection))
		if (!Math::getIntersection(begin, end, line_begin, line_end, intersection))
		{
#ifdef _WIN32
			OutputDebugString("ERROR! This intersection MUST exist!");
#endif
			exit(1);
		}
		tmp_node->waypoint_ = glm::vec3(intersection.x, tmp_node->waypoint_.y, intersection.y);
		waypoints.insert(waypoints.begin(), tmp_node->waypoint_);
		tmp_node = tmp_node->parent_;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
		ss << "New node: " << std::endl;
		ss << *tmp_node << std::endl;
		ss << "Add: (" << tmp_node->waypoint_.x << ", " << tmp_node->waypoint_.y << ", " << tmp_node->waypoint_.z << ")" << std::endl;
#endif
#endif
	}
	waypoints.insert(waypoints.begin(), end_stack_->waypoint_);
	//ss << "Add: (" << end_stack->waypoint_.x << ", " << end_stack->waypoint_.y << ", " << end_stack->waypoint_.z << ")" << std::endl;
	start_stack_ = NULL;
	end_stack_ = NULL;
#ifdef NAV_MESH_A_STAR_DEBUG
#ifdef _WIN32
	ss << "=== Stack resolved ===" << std::endl;
	OutputDebugString(ss.str().c_str());
#endif
#endif
}

const ConvexNavigationArea* NavMeshAStar::getArea(const glm::vec3& point) const
{
	//std::cout << "[NavMeshAStar::getArea] (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	const ConvexNavigationArea* current_area = NULL;
	float closest_distance = std::numeric_limits<float>::max();
	for (std::vector<ConvexNavigationArea*>::const_iterator ci = areas_->begin(); ci != areas_->end(); ++ci)
	{
		const ConvexNavigationArea* area = *ci;
		glm::vec3 intersecting_point;
		if (!area->getPlane().intersectsWithRay(point, glm::vec3(0, -1, 0), intersecting_point))
		{
			continue;
		}
		float t = glm::distance(intersecting_point, point);
		if (t < closest_distance)
		{
			closest_distance = t;
			current_area = area;
		}
	}
	return current_area;
}

};
