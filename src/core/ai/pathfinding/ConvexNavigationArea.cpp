#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <fstream>
#endif
#include <sstream>


#include <algorithm>
#include <limits>
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <set>

#include "dpengine/ai/pathfinding/ConvexNavigationArea.h"

#include "dpengine/loaders/PortalLevelFormatLoader.h"

#include "dpengine/math/Math.h"
#include "dpengine/math/Plane.h"

//#define CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
//#define CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO

namespace DreadedPE
{

float ConvexNavigationArea::EPSILON = 0.02f;

ConvexNavigationArea::ConvexNavigationArea(const Plane& plane)
	: plane_(&plane)
{
	
}

bool ConvexNavigationArea::merge(const ConvexNavigationArea& rhs)
{
	if (this == &rhs) return false;

	// The planes need to have the same normal, else they cannot be merged.
	if (std::abs(glm::dot(plane_->getNormal(), rhs.plane_->getNormal())) < 0.99f)
	{
		return false;
	}

	// First of all check if the resulting shape would be convex.
	for (int i = 0; i < plane_->getPoints().size(); ++i)
	{
		const glm::vec3& p1 = plane_->getPoints()[i];
		for (int j = 0; j < rhs.plane_->getPoints().size(); ++j)
		{
			const glm::vec3& p2 = rhs.plane_->getPoints()[j];

			// Check if any edge that does not contain p1 or p2 collides with this line.
			for (int k = 0; k < plane_->getPoints().size(); ++k)
			{
				if (k == i || k == (i + 1) % plane_->getPoints().size() || k == (i - 1) % plane_->getPoints().size())
				{
					continue;
				}

				const glm::vec3& begin_edge = plane_->getPoints()[k];
				const glm::vec3& end_edge = plane_->getPoints()[(k + 1) % plane_->getPoints().size()];

				if (Math::dist3D_Segment_to_Segment(p1, p2, begin_edge, end_edge) < EPSILON)
				{
					return false;
				}
			}

			for (int k = 0; k < rhs.plane_->getPoints().size(); ++k)
			{
				if (k == i || k == (i + 1) % rhs.plane_->getPoints().size() || k == (i - 1) % rhs.plane_->getPoints().size())
				{
					continue;
				}

				const glm::vec3& begin_edge = rhs.plane_->getPoints()[k];
				const glm::vec3& end_edge = rhs.plane_->getPoints()[(k + 1) % rhs.plane_->getPoints().size()];

				if (Math::dist3D_Segment_to_Segment(p1, p2, begin_edge, end_edge) < EPSILON)
				{
					return false;
				}
			}
		}
	}

	// Since both areas are convex, there must be two points shared between both CNAs.
	int p1_index = -1;
	int p2_index = -1;
	int rhs_p1_index = -1;
	int rhs_p2_index = -1;

	for (int i = 0; i < plane_->getPoints().size(); ++i)
	{
		const glm::vec3& p1 = plane_->getPoints()[i];
		for (int j = 0; j < rhs.plane_->getPoints().size(); ++j)
		{
			const glm::vec3& p2 = rhs.plane_->getPoints()[j];

			if (glm::distance(p1, p2) < EPSILON)
			{
				if (p1_index == -1)
				{
					p1_index = i;
					rhs_p1_index = j;
				}
				else if (p2_index == -1)
				{
					p2_index = i;
					rhs_p2_index = j;
				}
			}
		}
	}

	// If no such point could be found, we cannot merge.
	if (p1_index == -1 || p2_index == -1)
	{
		return false;
	}

	// Otherwise we proceed and merge these two convex navigation areas. We assume that the connecting edge only
	// contains two vertexes (i.e. no extra vertexes on a straight line).
	std::vector<glm::vec3> convex_shape;
	if (p2_index != (p1_index + 1) % plane_->getPoints().size())
	{
		for (int i = p1_index; i != p2_index; i = (i + 1) % plane_->getPoints().size())
		{
			convex_shape.push_back(plane_->getPoints()[i]);
		}
	}
	else
	{
		for (int i = p1_index; i != p2_index; i = (i - 1) % plane_->getPoints().size())
		{
			convex_shape.push_back(plane_->getPoints()[i]);
		}
	}

	if (rhs_p1_index != (rhs_p2_index + 1) % rhs.plane_->getPoints().size())
	{
		for (int i = rhs_p2_index; i != rhs_p1_index; i = (i + 1) % rhs.plane_->getPoints().size())
		{
			convex_shape.push_back(rhs.plane_->getPoints()[i]);
		}
	}
	else
	{
		for (int i = rhs_p2_index; i != rhs_p1_index; i = (i - 1) % rhs.plane_->getPoints().size())
		{
			convex_shape.push_back(rhs.plane_->getPoints()[i]);
		}
	}
	
	// Out with the old, in with the new. Preserve the normal vector.
	glm::vec3 normal = plane_->getNormal();
	delete plane_;
	plane_ = new Plane(convex_shape, normal);
	return true;
}

void ConvexNavigationArea::inflateObstacles(std::vector<const ConvexPolygon*>& inflated_obstacles, const std::vector<const ConvexPolygon*>& obstacles, float max_distance) const //, const glm::vec3& walkable_normal) const
{
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG) || defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO)
	std::stringstream ss;
	ss << "[ConvexNavigationArea::inflateObstacles] Max Distance: " << max_distance << std::endl;
#endif
	// TODO: Solve walkable normal later.
	for (const ConvexPolygon* cp : obstacles)
	{
		glm::vec3 centre = cp->getCentre();
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG)
		ss << "Inflate the Convex Polygon: " << *cp << std::endl;
		ss << "Centre: (" << centre.x << ", " << centre.y << ", " << centre.z << ")" << std::endl;
#endif
		std::vector<const Plane*> inflated_planes;
		for (const Plane* plane : cp->getPlanes())
		{
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG)
			ss << "Inflate " << *plane << std::endl;
#endif
			std::vector<glm::vec3> inflated_plane_points;
			for (const glm::vec3& p : plane->getPoints())
			{
				glm::vec3 direction = p * glm::vec3(1, 0, 1) - centre * glm::vec3(1, 0, 1);
				if (glm::length(direction) != 0)
				{
					direction = glm::normalize(direction) * max_distance;
				}
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG)
				ss << "\tp: (" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
				ss << "\tDirection: (" << direction.x << ", " << direction.y << ", " << direction.z << ")" << std::endl;
#endif
				inflated_plane_points.push_back(p + direction);
			}

			// Make sure the plane's normal is preserved.
			glm::vec3 inflated_normal = glm::normalize(glm::cross(inflated_plane_points[1] - inflated_plane_points[0], inflated_plane_points[2] - inflated_plane_points[0]));
			if (inflated_normal.x * plane->getNormal().x < 0) inflated_normal.x = -inflated_normal.x;
			if (inflated_normal.y * plane->getNormal().y < 0) inflated_normal.y = -inflated_normal.y;
			if (inflated_normal.z * plane->getNormal().z < 0) inflated_normal.z = -inflated_normal.z;

			Plane* inflated_plane = new Plane(inflated_plane_points, inflated_normal);
			inflated_planes.push_back(inflated_plane);
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG) || defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO)
			ss << "Result " << *inflated_plane << std::endl;
#endif
		}
		ConvexPolygon* inflated_cp = new ConvexPolygon(*cp->getEntity(), inflated_planes);
		inflated_obstacles.push_back(inflated_cp);
	}
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG) || defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO)
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
#endif
}

void ConvexNavigationArea::orderIntersections(const Plane& existing_plane, glm::vec3& projected_line, std::vector<glm::vec2>& projected_plane_points, std::vector<std::vector<glm::vec2>*>& ordered_projected_intersection_sets, std::vector<std::vector<glm::vec3>*>& ordered_projected_intersection_3d_sets, const std::vector<std::pair<glm::vec3, glm::vec3> >& intersecting_lines) const
{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	std::stringstream ss;
	ss << "[ConvexNavigationArea::orderIntersections]" << std::endl;
#endif
	
	// We project the plane and intersection line onto the 2D plane, based on the largest normal vector of the plane.
	std::vector<std::pair<glm::vec2, glm::vec2> > projected_intersections;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "Normal of this plane: (" << existing_plane.getNormal().x << ", " << existing_plane.getNormal().y << ", " << existing_plane.getNormal().z << ")." << std::endl;
#endif
	if (fabs(existing_plane.getNormal().x) >= fabs(existing_plane.getNormal().y) && fabs(existing_plane.getNormal().x) >= fabs(existing_plane.getNormal().z))
	{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Project on the X-plane." << std::endl;
#endif
		projected_line = glm::vec3(1, 0, 0);
		for (std::vector<glm::vec3>::const_iterator ci = existing_plane.getPoints().begin(); ci != existing_plane.getPoints().end(); ++ci)
		{
			glm::vec2 projected_point((*ci).y, (*ci).z);
			projected_plane_points.push_back(projected_point);
		}
		for (std::vector<std::pair<glm::vec3, glm::vec3> >::const_iterator ci = intersecting_lines.begin(); ci != intersecting_lines.end(); ++ci)
		{
			const glm::vec3& begin_intersection = (*ci).first;
			const glm::vec3& end_intersection = (*ci).second;
			projected_intersections.push_back(std::make_pair(glm::vec2(begin_intersection.y, begin_intersection.z), glm::vec2(end_intersection.y, end_intersection.z)));
		}
	}
	else if (fabs(existing_plane.getNormal().y) >= fabs(existing_plane.getNormal().x) && fabs(existing_plane.getNormal().y) >= fabs(existing_plane.getNormal().z))
	{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Project on the Y-plane." << std::endl;
#endif
		projected_line = glm::vec3(0, 1, 0);
		for (std::vector<glm::vec3>::const_iterator ci = existing_plane.getPoints().begin(); ci != existing_plane.getPoints().end(); ++ci)
		{
			glm::vec2 projected_point((*ci).x, (*ci).z);
			projected_plane_points.push_back(projected_point);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Projected point: (" << projected_point.x << ", " << projected_point.y << ")" << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
		}
		for (std::vector<std::pair<glm::vec3, glm::vec3> >::const_iterator ci = intersecting_lines.begin(); ci != intersecting_lines.end(); ++ci)
		{
			const glm::vec3& begin_intersection = (*ci).first;
			const glm::vec3& end_intersection = (*ci).second;
			projected_intersections.push_back(std::make_pair(glm::vec2(begin_intersection.x, begin_intersection.z), glm::vec2(end_intersection.x, end_intersection.z)));
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Begin intersection: (" << begin_intersection.x << ", " << begin_intersection.y << ", " << begin_intersection.z << ")" << std::endl;
			ss << "End intersection: (" << end_intersection.x << ", " << end_intersection.y << ", " << end_intersection.z << ")" << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
		}
	}
	else
	{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Project on the Z-plane." << std::endl;
#endif
		projected_line = glm::vec3(0, 0, 1);
		for (std::vector<glm::vec3>::const_iterator ci = existing_plane.getPoints().begin(); ci != existing_plane.getPoints().end(); ++ci)
		{
			glm::vec2 projected_point((*ci).x, (*ci).y);
			projected_plane_points.push_back(projected_point);
		}
		for (std::vector<std::pair<glm::vec3, glm::vec3> >::const_iterator ci = intersecting_lines.begin(); ci != intersecting_lines.end(); ++ci)
		{
			const glm::vec3& begin_intersection = (*ci).first;
			const glm::vec3& end_intersection = (*ci).second;
			projected_intersections.push_back(std::make_pair(glm::vec2(begin_intersection.x, begin_intersection.y), glm::vec2(end_intersection.x, end_intersection.y)));
		}
	}

	// We rearange the intersections such that they form a connected sequence. There could be more than
	// one sequence as the obstacle could intersect as seperate areas of this plane (e.g. clip its edges).
	std::vector<std::pair<glm::vec3, glm::vec3> > intersecting_lines_copy(intersecting_lines);
	
	std::vector<glm::vec2>* ordered_projected_intersections = new std::vector<glm::vec2>();
	std::vector<glm::vec3>* ordered_projected_intersections_3d = new std::vector<glm::vec3>();

	bool start_new_set = true;
	while (projected_intersections.size() > 0)
	{
		if (start_new_set)
		{
			ordered_projected_intersections = new std::vector<glm::vec2>();
			ordered_projected_intersections->push_back(projected_intersections[0].first);
			ordered_projected_intersections->push_back(projected_intersections[0].second);
			projected_intersections.erase(projected_intersections.begin());
			ordered_projected_intersection_sets.push_back(ordered_projected_intersections);

			ordered_projected_intersections_3d = new std::vector<glm::vec3>();
			ordered_projected_intersections_3d->push_back(intersecting_lines_copy[0].first);
			ordered_projected_intersections_3d->push_back(intersecting_lines_copy[0].second);
			intersecting_lines_copy.erase(intersecting_lines_copy.begin());
			ordered_projected_intersection_3d_sets.push_back(ordered_projected_intersections_3d);
		}
		start_new_set = true;

		for (int i = projected_intersections.size() - 1; i >= 0; --i)
		{
			const glm::vec2& intersection_begin = projected_intersections[i].first;
			const glm::vec2& intersection_end = projected_intersections[i].second;

			// Check if this fits with any of the existing links.
			const glm::vec2& start_other_begin = (*ordered_projected_intersections)[0];
			const glm::vec2& end_other_end = (*ordered_projected_intersections)[ordered_projected_intersections->size() - 1];
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Intersection: (" << intersection_begin.x << ", " << intersection_begin.y << ") - (" << intersection_end.x << ", " << intersection_end.y << ")" << std::endl;
			ss << "Other Intersection: (" << start_other_begin.x << ", " << start_other_begin.y << ") - (" << end_other_end.x << ", " << end_other_end.y << ")" << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
			if (glm::distance(intersection_begin, start_other_begin) < EPSILON)
			{
				ordered_projected_intersections->insert(ordered_projected_intersections->begin(), intersection_end);
				ordered_projected_intersections_3d->insert(ordered_projected_intersections_3d->begin(), intersecting_lines_copy[i].second);
				projected_intersections.erase(projected_intersections.begin() + i);
				intersecting_lines_copy.erase(intersecting_lines_copy.begin() + i);
				start_new_set = false;
				break;
			}
			else if (glm::distance(intersection_end, start_other_begin) < EPSILON)
			{
				ordered_projected_intersections->insert(ordered_projected_intersections->begin(), intersection_begin);
				ordered_projected_intersections_3d->insert(ordered_projected_intersections_3d->begin(), intersecting_lines_copy[i].first);
				projected_intersections.erase(projected_intersections.begin() + i);
				intersecting_lines_copy.erase(intersecting_lines_copy.begin() + i);
				start_new_set = false;
				break;
			}
			else if (glm::distance(intersection_begin, end_other_end) < EPSILON)
			{
				ordered_projected_intersections->insert(ordered_projected_intersections->end(), intersection_end);
				ordered_projected_intersections_3d->insert(ordered_projected_intersections_3d->end(), intersecting_lines_copy[i].second);
				projected_intersections.erase(projected_intersections.begin() + i);
				intersecting_lines_copy.erase(intersecting_lines_copy.begin() + i);
				start_new_set = false;
				break;
			}
			else if (glm::distance(intersection_end, end_other_end) < EPSILON)
			{
				ordered_projected_intersections->insert(ordered_projected_intersections->end(), intersection_begin);
				ordered_projected_intersections_3d->insert(ordered_projected_intersections_3d->end(), intersecting_lines_copy[i].first);
				projected_intersections.erase(projected_intersections.begin() + i);
				intersecting_lines_copy.erase(intersecting_lines_copy.begin() + i);
				start_new_set = false;
				break;
			}
		}
	}

	// Show the ordered list.
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << " --- Ordered Lists ---" << std::endl;
	for (std::vector<std::vector<glm::vec2>*>::const_iterator ci = ordered_projected_intersection_sets.begin(); ci != ordered_projected_intersection_sets.end(); ++ci)
	{
		ss << " --- Ordered List ---" << std::endl;
		std::vector<glm::vec2>* ordered_projected_intersections = *ci;
		for (std::vector<glm::vec2>::const_iterator ci = ordered_projected_intersections->begin(); ci != ordered_projected_intersections->end(); ++ci)
		{
			const glm::vec2& p = *ci;
			ss << "(" << p.x << ", " << p.y << ")" << std::endl;
		}
		ss << " -+- Ordered List -+-" << std::endl;
	}
	ss << " -+- Ordered Lists -+-" << std::endl;
	ss << " --- Ordered Lists 3D ---" << std::endl;
	for (std::vector<std::vector<glm::vec3>*>::const_iterator ci = ordered_projected_intersection_3d_sets.begin(); ci != ordered_projected_intersection_3d_sets.end(); ++ci)
	{
		ss << " --- Ordered List 3D ---" << std::endl;
		std::vector<glm::vec3>* ordered_projected_intersections_3d = *ci;
		for (std::vector<glm::vec3>::const_iterator ci = ordered_projected_intersections_3d->begin(); ci != ordered_projected_intersections_3d->end(); ++ci)
		{
			const glm::vec3& p = *ci;
			ss << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
		}
		ss << " -+- Ordered List 3D -+-" << std::endl;
	}
	ss << " -+- Ordered Lists 3D -+-" << std::endl;
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
#endif
}

void ConvexNavigationArea::getIntersections(std::vector<std::pair<glm::vec3, glm::vec3> >& intersecting_lines, const Plane& plane, const ConvexPolygon& obstacle) const
{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	std::stringstream ss;
	ss << "[ConvexNavigationArea::getIntersections]" << std::endl;
#endif
	// Get the intersection between the existing plane and the obstacle.
	glm::vec3 begin_intersection;
	glm::vec3 end_intersection;
	for (std::vector<const Plane*>::const_iterator ci = obstacle.getPlanes().begin(); ci != obstacle.getPlanes().end(); ++ci)
	{
		const Plane* obstacle_plane = *ci;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << " --- Compare against obstacle plane --- " << std::endl;
		ss << *obstacle_plane << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif
		// Check if this plane's hight is the same as the walkable surface. If so, ignore it.
		float highest_point = -std::numeric_limits<float>::max();
		float lowest_point = std::numeric_limits<float>::max();
		for (const glm::vec3& p : obstacle_plane->getPoints())
		{
			if (p.y > highest_point)
				highest_point = p.y;
			if (p.y < lowest_point)
				lowest_point = p.y;
		}

		float highest_plane_point = -std::numeric_limits<float>::max();
		float lowest_plane_point = std::numeric_limits<float>::max();
		for (const glm::vec3& p : plane.getPoints())
		{
			if (p.y > highest_plane_point)
				highest_plane_point = p.y;
			if (p.y < lowest_plane_point)
				lowest_plane_point = p.y;
		}

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Lowest points: Existing Plane / Obstacle: " << lowest_plane_point << " / " << lowest_point << "Highest points: Existing Plane / Obstacle: " << highest_plane_point << " / " << highest_point << std::endl;
#endif

		if (std::abs(highest_point - highest_plane_point) < EPSILON ||
			std::abs(lowest_point - lowest_plane_point) < EPSILON)
		{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Skip this plane as it is a the same hight as the walking surface!" << std::endl;
#endif
			continue;
		}

		if (plane.intersectsWith(*obstacle_plane, begin_intersection, end_intersection))
		{
			intersecting_lines.push_back(std::make_pair(begin_intersection, end_intersection));
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Found intersection: (" << begin_intersection.x << ", " << begin_intersection.y << ", " << begin_intersection.z << ") - (" << end_intersection.x << ", " << end_intersection.y << ", " << end_intersection.z << ")" << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
		}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		else
		{
			ss << "(No intersections found!)" << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
		}
#endif
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << " -+- FOUND INTERSECTIONS -+- " << std::endl;
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif
}

bool ConvexNavigationArea::split(std::vector<ConvexNavigationArea*>& new_areas, const std::vector<const ConvexPolygon*>& obstacles, float max_distance) const
{
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG) || defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO)
	std::stringstream ss;
	ss << "[ConvexNavigationArea::split] Max distance: " << max_distance << std::endl;
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif
	
	//inflateObstacles(obstacles, basic_obstacles, max_distance);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "=== " << obstacles.size() << " Obstacles:" << std::endl;
	for (const ConvexPolygon* o : obstacles)
	{
		ss << *o << std::endl;
	}
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif

	// Every obstacle will puncture out a shape out of this plane. This can either be at the edge or in the middle. After we altered this plane we triangulate 
	// it and construct new convex planes out of it. These become the new convex navigation areas.
	std::vector<Plane*> new_planes;
	new_planes.push_back(new Plane(*plane_));

	for (unsigned int obstacle_index = 0; obstacle_index < obstacles.size(); ++obstacle_index)
	{
		const ConvexPolygon* obstacle = obstacles[obstacle_index];
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << " --- Obstacle ---" << std::endl;
		ss << *obstacle << std::endl;
		ss << " -+- Obstacle -+-" << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif

		// Process all the known planes.
		std::vector<Plane*> planes_to_process(new_planes);
		new_planes.clear();
		for (std::vector<Plane*>::const_iterator ci = planes_to_process.begin(); ci != planes_to_process.end(); ++ci)
		{
			Plane* existing_plane = *ci;
			std::vector<std::pair<glm::vec3, glm::vec3> > intersecting_lines;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << " ----------------------------- Existing plane ----------------------------- " << std::endl;
			ss << *existing_plane << std::endl;
			ss << " -+- Existing plane -+- " << std::endl;

			ss << " --- FOUND INTERSECTIONS --- " << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
			// Get the intersection between the existing plane and the obstacle.
			getIntersections(intersecting_lines, *existing_plane, *obstacle);

			// Remove those lines that lie ON the plane.
			for (int i = intersecting_lines.size() - 1; i >= 0; --i)
			{
				glm::vec3 begin = intersecting_lines[i].first;
				glm::vec3 end = intersecting_lines[i].second;

				for (int j = 0; j < existing_plane->getPoints().size(); ++j)
				{
					const glm::vec3& p1 = existing_plane->getPoints()[j];
					const glm::vec3& p2 = existing_plane->getPoints()[(j + 1) % existing_plane->getPoints().size()];

					if (std::abs(glm::dot(glm::normalize(begin - end), glm::normalize(p2 - p1))) > 0.999f &&
					    Math::dist3D_Segment_to_Segment(begin, end, p1, p2) < EPSILON)
					{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "Remove the intersection: (" << begin.x << ", " << begin.y << ", " << begin.z << ") - (" << end.x << ", " << end.y << ", " << end.z << ")" << std::endl;
						ss << "It lies on the plane edge: (" << p1.x << ", " << p1.y << ", " << p1.z << ") - (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
#endif
						intersecting_lines.erase(intersecting_lines.begin() + i);
						break;
					}
				}
			}

			// If we found nothing, move on.
			if (intersecting_lines.empty())
			{
				new_planes.push_back(existing_plane);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "No intersections found..." << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				continue;
			}

			// We rearrange the intersections such that they form a connected sequence. There could be more than
			// one sequence as the obstacle could intersect as seperate areas of this plane (e.g. clip its edges).
			glm::vec3 projected_line;
			std::vector<glm::vec2> projected_plane_points;
			std::vector<std::vector<glm::vec2>*> ordered_projected_intersection_sets;
			std::vector<std::vector<glm::vec3>*> ordered_projected_intersection_3d_sets;

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif

			orderIntersections(*existing_plane, projected_line, projected_plane_points, ordered_projected_intersection_sets, ordered_projected_intersection_3d_sets, intersecting_lines);
			
			// Split the plan into subplanes, based on the found intersections. We follow the plane's edges and split according to found intersections.
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Ordered Intersection sets: " << std::endl;
#endif
			//for (std::vector<glm::vec2>* ordered_projected_intersection_set : ordered_projected_intersection_sets)
			for (int i = ordered_projected_intersection_sets.size() - 1; i >= 0; --i)
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				std::vector<glm::vec2>* set = ordered_projected_intersection_sets[i];
				ss << "{ ";
				for (const glm::vec2& p : *set)
				{
					ss << "(" << p.x << ", " << p.y << "), ";
				}
				ss << "}" << std::endl;
#endif
				/*
				if (set->size() == 2 || set->size() == 1)
				{
					ordered_projected_intersection_sets.erase(ordered_projected_intersection_sets.begin() + i);
					ordered_projected_intersection_3d_sets.erase(ordered_projected_intersection_3d_sets.begin() + i);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "- Ignore this set, as it is only a point or line." << std::endl;
#endif
				}
				*/
			}
			
			if (ordered_projected_intersection_sets.empty())
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "No intersections found, skipping it!" << std::endl;
#endif
				new_planes.push_back(existing_plane);
				continue;
			}
			
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "Updated set: " << std::endl;
			for (std::vector<glm::vec2>* ordered_projected_intersection_set : ordered_projected_intersection_sets)
			{
				ss << "{ ";
				for (const glm::vec2& p : *ordered_projected_intersection_set)
				{
					ss << "(" << p.x << ", " << p.y << "), ";
				}
				ss << "}" << std::endl;
			}
#endif
			
			// <projected_plane_point index, coordinates> key - value list. An index of std::numeric_limits<int>::max() means there 
			// is not related projected plane point index but is rather an intersection.
			std::vector<std::pair<int, glm::vec2> > stack;
			stack.push_back(std::make_pair(0, projected_plane_points[0]));
			std::set<const std::vector<glm::vec2>*> closed_list;

			bool found_intersection = false;
			while (!stack.empty())
			{
				int index = stack[0].first;
				glm::vec2 location = stack[0].second;
				stack.erase(stack.begin());
				std::vector<glm::vec2> new_plane;
				new_plane.push_back(location);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Process the plane: " << std::endl;
				for (int i = 0; i <projected_plane_points.size(); ++i)
				{
					const glm::vec2& next_location = projected_plane_points[i];
					ss << "[" << i << "] (" << next_location.x << ", " << next_location.y << ")" << std::endl;
				}

				ss << "(" << location.x << "," << location.y << ")" << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				for (int i = (index + 1) % projected_plane_points.size(); true; i = (i + 1) % projected_plane_points.size())
				{
					const glm::vec2& next_location = projected_plane_points[i];

					// Check if we have found a loop.
					if (glm::distance(new_plane[0], new_plane[new_plane.size() - 1]) < EPSILON && new_plane.size() > 1)
					{
						new_plane.erase(new_plane.begin() + new_plane.size() - 1);
						break;
					}

					// It can happen that the intersection is identical to the next location, in which case we skip it.
					// This happens if an intersection of an obstacle is identical to a vertex on the plane.
					if (glm::distance(location, next_location) < EPSILON)
					{
						continue;
					}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "Process: [" << index << "](" << location.x << "," << location.y << ") -> [" << i << "](" << next_location.x << "," << next_location.y << ")" << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					// Check for intersections, these can only at the beginning or end of the intersection sequences.
					std::vector<glm::vec2>* closest_intersection = NULL;
					float closest_intersection_distance = std::numeric_limits<float>::max();
					for (std::vector<std::vector<glm::vec2>*>::const_iterator ci = ordered_projected_intersection_sets.begin(); ci != ordered_projected_intersection_sets.end(); ++ci)
					{
						std::vector<glm::vec2>* ordered_projected_intersection = *ci;

						const glm::vec2& begin_intersection = (*ordered_projected_intersection)[0];
						const glm::vec2& end_intersection = (*ordered_projected_intersection)[ordered_projected_intersection->size() - 1];

						glm::vec2 target(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						float tmp1 = Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(begin_intersection.x, begin_intersection.y, 0.0f));
						float tmp2 = glm::distance(location, begin_intersection);
						float tmp3 = Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(end_intersection.x, end_intersection.y, 0.0f));
						float tmp4 = glm::distance(location, end_intersection);

						ss << "Location -> Next location to (" << begin_intersection.x << "," << begin_intersection.y << ") = " << tmp1 << "; d=" << tmp2 << std::endl;
						ss << "Location -> Next location to (" << end_intersection.x << "," << end_intersection.y << ") = " << tmp3 << "; d=" << tmp4 << std::endl;
#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
#endif

						// Intersection with the beginning of the sequence.
						if (Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(begin_intersection.x, begin_intersection.y, 0.0f)) < EPSILON &&
							glm::distance(location, begin_intersection) > EPSILON)
						{
							if (closest_intersection_distance > glm::distance(location, begin_intersection))
							{
								closest_intersection_distance = glm::distance(location, begin_intersection);
								closest_intersection = ordered_projected_intersection;
							}
						}
						// Intersection with the end of the sequence.
						else if (Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(end_intersection.x, end_intersection.y, 0.0f)) < EPSILON &&
							glm::distance(location, end_intersection) > EPSILON)
						{
							if (closest_intersection_distance > glm::distance(location, end_intersection))
							{
								closest_intersection_distance = glm::distance(location, end_intersection);
								closest_intersection = ordered_projected_intersection;
							}
						}
					}

					if (closest_intersection != NULL)
					{
						found_intersection = true;
						const glm::vec2& begin_intersection = (*closest_intersection)[0];
						const glm::vec2& end_intersection = (*closest_intersection)[closest_intersection->size() - 1];

						float distance_to_begin_intersection = glm::distance(location, begin_intersection);
						float distance_to_end_intersection = glm::distance(location, end_intersection);
						bool found_begin_intersection = Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(begin_intersection.x, begin_intersection.y, 0.0f)) < EPSILON && distance_to_begin_intersection >= EPSILON;
						bool found_end_intersection = Math::dist3D_Segment_to_Point(glm::vec3(location.x, location.y, 0.0f), glm::vec3(next_location.x, next_location.y, 0.0f), glm::vec3(end_intersection.x, end_intersection.y, 0.0f)) < EPSILON && distance_to_end_intersection >= EPSILON;


						glm::vec2 target(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

						// Intersection with the beginning of the sequence.
						if (found_begin_intersection &&
						   (!found_end_intersection || (found_end_intersection && distance_to_begin_intersection <= distance_to_end_intersection)))
						{
							new_plane.insert(new_plane.end(), closest_intersection->begin(), closest_intersection->end());
							target = end_intersection;

							if (closed_list.count(closest_intersection) == 0)
							{
								stack.push_back(std::make_pair(i == 0 ? projected_plane_points.size() - 1 : i - 1, begin_intersection));
								closed_list.insert(closest_intersection);
							}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "Found intersection at: (" << begin_intersection.x << "," << begin_intersection.y << ") [" << i << "]; Target: (" << target.x << "," << target.y << ")" << std::endl;
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());
#endif
						}
						// Intersection with the end of the sequence.
						else if (found_end_intersection && 
							(!found_begin_intersection || (found_begin_intersection && distance_to_end_intersection < distance_to_begin_intersection)))
						{
							new_plane.insert(new_plane.end(), closest_intersection->rbegin(), closest_intersection->rend());
							target = begin_intersection;
							if (closed_list.count(closest_intersection) == 0)
							{
								stack.push_back(std::make_pair(i == 0 ? projected_plane_points.size() - 1 : i - 1, end_intersection));
								closed_list.insert(closest_intersection);
							}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "Found intersection at: (" << end_intersection.x << "," << end_intersection.y << ") [" << i << "]; Target: (" << target.x << "," << target.y << ")" << std::endl;
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());
#endif
						}

						// If we found an intersection, we find the edge where the end point of the sequence intersects with an edge of the plane.
						if (target.x != std::numeric_limits<float>::max())
						{
							for (; true; i = (i + 1) % projected_plane_points.size())
							{
								const glm::vec2& p1 = projected_plane_points[i];
								const glm::vec2& p2 = projected_plane_points[(i + 1) % projected_plane_points.size()];

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								float d = Math::dist3D_Segment_to_Point(glm::vec3(p1.x, p1.y, 0.0f), glm::vec3(p2.x, p2.y, 0.0f), glm::vec3(target.x, target.y, 0.0f));
								ss << "Compare the line: (" << p1.x << "," << p1.y << ") -> (" << p2.x << "," << p2.y << ") = " << d << std::endl;
#ifdef WIN32
								OutputDebugString(ss.str().c_str());
#else
								std::cout << ss.str() << std::endl;
#endif
								ss.str(std::string());
#endif
								if (Math::dist3D_Segment_to_Point(glm::vec3(p1.x, p1.y, 0.0f), glm::vec3(p2.x, p2.y, 0.0f), glm::vec3(target.x, target.y, 0.0f)) < EPSILON)
								{
									break;
								}
							}
							location = target;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "Continue from: (" << location.x << "," << location.y << ")" << std::endl;
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());
#endif
						}
					}

					// If we found not intersection, we move on to the next vertex on the plane.
					else
					{
						location = next_location;
						new_plane.push_back(location);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
						ss << "Next: (" << location.x << "," << location.y << ")" << std::endl;
#endif
					}
				}

				if (new_plane.size() <= 2)
				{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "Found plane only has " << new_plane.size() << " points. Ignore!" << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					continue;
				}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "New plane: " << std::endl;
				for (std::vector<glm::vec2>::const_iterator ci = new_plane.begin(); ci != new_plane.end(); ++ci)
				{
					ss << "\t(" << (*ci).x << "," << (*ci).y << ")" << std::endl;
				}
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());

				ss << "Same plane in 3D: " << std::endl;
#endif
				std::vector<glm::vec3> points_3d;
				glm::vec3 intersection(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
				for (std::vector<glm::vec2>::const_iterator ci = new_plane.begin(); ci != new_plane.end(); ++ci)
				{
					const glm::vec2& plane_2d = *ci;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "\t(" << (*ci).x << "," << (*ci).y << ") -> ";
#endif
					if (projected_line.x == 1)
					{
						existing_plane->intersectsWith(glm::vec3(0.0f, plane_2d.x, plane_2d.y), glm::vec3(projected_line.x, plane_2d.x, plane_2d.y), intersection, false);
					}
					else if (projected_line.y == 1)
					{
						existing_plane->intersectsWith(glm::vec3(plane_2d.x, 0.0f, plane_2d.y), glm::vec3(plane_2d.x, projected_line.y, plane_2d.y), intersection, false);
					}
					else if (projected_line.z == 1)
					{
						existing_plane->intersectsWith(glm::vec3(plane_2d.x, plane_2d.y, 0.0f), glm::vec3(plane_2d.x, plane_2d.y, projected_line.z), intersection, false);
					}
					else
					{
#ifdef WIN32
						OutputDebugString("This situation should never have happened! ERROR 0012");
#else
						std::cerr << "This situation should never have happened!" << std::endl;
#endif
						assert(false);
						exit(1);
					}

					if (intersection.x == std::numeric_limits<float>::max())
					{
#ifdef WIN32
						OutputDebugString("This situation should never have happened! ERROR 0013");
#else
						std::cerr << "Could not find the intersection with the original plane!" << std::endl;
#endif
						assert(false);
						exit(1);
					}

					points_3d.push_back(intersection);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "\t(" << intersection.x << ", " << intersection.y << ", " << intersection.z << ")" << std::endl;
#endif
				}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				if (found_intersection)
				{
					Plane* new_3d_plane = new Plane(points_3d);
					
					// Triangulate these planes to make sure they are convex.
					std::vector<Plane*> new_plane_set;
					new_plane_set.push_back(new_3d_plane);
					triangulateAndMerge(new_plane_set, NULL, new_planes);
				}
			}

			// We found not intersection with the edges of the convex area, so the intersection is a hole in the middle.
			if (!found_intersection)
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "[ConvexNavigationArea::split] Found no intersections, must be a hole!" << std::endl;
				for (std::vector<glm::vec3>* points : ordered_projected_intersection_3d_sets)
				{
					ss << "{";
					for (const glm::vec3& p : *points)
					{
						ss << "(" << p.x << ", " << p.y << ", " << p.z << "), ";
					}
					ss << "}" << std::endl;
				}
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				std::vector<Plane*> original_plane;
				original_plane.push_back(existing_plane);
				for (std::vector<glm::vec3>* points : ordered_projected_intersection_3d_sets)
				{
					std::vector<glm::vec3> new_plane_points(points->begin(), points->end() - 1);

					Plane* new_3d_plane = new Plane(new_plane_points);
					new_planes.push_back(new_3d_plane);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "[ConvexNavigationArea::split] work on plane: " << *new_3d_plane << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					// Triangulate this planes to make sure they are convex.
					triangulateAndMerge(original_plane, new_3d_plane, new_planes);
				}
			}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
		}
	}
#if defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG) || defined(CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_INFO)
	ss << "[ConvexNavigationArea::split] Final triangle and merge:" << std::endl;
	for (Plane* plane : new_planes)
	{
		ss << "\t" << *plane << std::endl;
	}
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif
	std::vector<Plane*> merged_planes;
	triangulateAndMerge(new_planes, NULL, merged_planes);
	
	std::vector<const Plane*> obstacle_planes;
	for (const ConvexPolygon* p : obstacles)
	{
		obstacle_planes.insert(obstacle_planes.end(), p->getPlanes().begin(), p->getPlanes().end());
	}
	std::vector<Plane*> finalised_planes;
	merge(merged_planes, obstacle_planes, finalised_planes);

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "=== CHECK WHICH PLANES ARE INSIDE OBSTACLES ===" << std::endl;
#endif

	for (Plane* plane : finalised_planes)
	{
		// Check if this plane is inside an obstacle, if so then we will not add it.
		glm::vec3 centre_point;
		for (const glm::vec3& p : plane->getPoints())
		{
			centre_point += p;
		}
		centre_point /= plane->getPoints().size();

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Plane: " << *plane << "; Centre point: (" << centre_point.x << ", " << centre_point.y << ", " << centre_point.z << ")" << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif

		bool is_inside_an_obstacle = false;
		for (const ConvexPolygon* cp : obstacles)
		{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			//ss << "Compare against : " << *cp << std::endl;
#endif
			if (cp->isInside(centre_point))
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Is inside: " << *cp << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				//float height_difference = -std::numeric_limits<float>::max();
				is_inside_an_obstacle = true;
				break;
			}
			else
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				//ss << "Point does not lie inside!" << std::endl;
#endif
			}
		}

		if (is_inside_an_obstacle)
		{
			continue;
		}
		
		ConvexNavigationArea* new_cna = new ConvexNavigationArea(*plane);
		new_areas.push_back(new_cna);
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif

#endif
	return true;
}

void ConvexNavigationArea::triangulateAndMerge(const std::vector<Plane*>& planes, const Plane* hole, std::vector<Plane*>& result) const
{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	std::stringstream ss; // Debug.
	ss << "[ConvexNavigationArea::triangulateAndMerge] " << planes.size() << " planes." << std::endl;
	for (const Plane* p : planes)
	{
		ss << "\t" << *p << std::endl;
	}
#endif
	std::vector<const Plane*> obstacles;
	if (hole != NULL)
	{
		// Check whether the hole is clock-wise.
		Plane* cw_hole = NULL;

		if (!isCounterClockWise(*hole))
		{
			cw_hole = new Plane(*hole);
		}
		else
		{
			std::vector<glm::vec3> points = hole->getPoints();
			std::reverse(points.begin(), points.end());
			cw_hole= new Plane(points, hole->getNormal());
		}
		obstacles.push_back(cw_hole);

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "[ConvexNavigationArea::triangulateAndMerge] Add hole " << *cw_hole << std::endl;
#endif
	}

	// Use earclipping method (https://www.geometrictools.com/Documentation/TriangulationByEarClipping.pdf).
	for (Plane* plane : planes)
	{
		std::vector<Plane*> triangles;

		// Make sure the edges in this plane go counter-clockwise.
		Plane* ccw_plane = NULL;
		glm::vec3 ccw_plan_normal;
		
		if (isCounterClockWise(*plane))
		{
			ccw_plane = new Plane(*plane);
		}
		else
		{
			std::vector<glm::vec3> points = plane->getPoints();
			std::reverse(points.begin(), points.end());
			ccw_plane = new Plane(points, plane->getNormal());
		}
		ccw_plan_normal = ccw_plane->getNormal();

		// If holes are present, we update the edges of the plane to take these into account.
		glm::vec3 intersection;
		for (const Plane* obstacle : obstacles)
		{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "[ConvexNavigationArea::triangulateAndMerge] Find connecting edge to " << *obstacle << std::endl;
#ifdef WIN32
			OutputDebugString(ss.str().c_str());
#else
			std::cout << ss.str() << std::endl;
#endif
			ss.str(std::string());
#endif
			const glm::vec3* connecting_p1 = NULL;
			const glm::vec3* connecting_p2 = NULL;

			// Find an edge that connects the plane with the obstacle.
			for (const glm::vec3& p1 : ccw_plane->getPoints())
			{
				for (const glm::vec3& p2 : obstacle->getPoints())
				{
					// Update this edge so it does not collide with the vertexes it is part of.
					glm::vec3 nudged_p1 = p1 + glm::normalize(p2 - p1) * 0.01f;
					glm::vec3 nudged_p2 = p2 + glm::normalize(p1 - p2) * 0.01f;
					if (!ccw_plane->intersectsWith(nudged_p1, nudged_p2, intersection, true) &&
					    !obstacle->intersectsWith(nudged_p1, nudged_p2, intersection, true))
					{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "nudged_p1: (" << nudged_p1.x << ", " << nudged_p1.y << ", " << nudged_p1.z << ")" << std::endl;
						ss << "nudged_p2: (" << nudged_p2.x << ", " << nudged_p2.y << ", " << nudged_p2.z << ")" << std::endl;
#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
#endif
						connecting_p1 = &p1;
						connecting_p2 = &p2;
						break;
					}
				}
				if (connecting_p1 != NULL) break;
			}

			if (connecting_p1 == NULL)
			{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
				OutputDebugString("[ConvexNavigationArea::triangulateAndMerge] Could not find a connecting edge, aborting!");
#else
				std::cout << "[ConvexNavigationArea::triangulateAndMerge] Could not find a connecting edge, aborting!" << std::endl;
#endif
#endif
				exit(-1);
			}

#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
			ss << "connecting_p1: (" << connecting_p1->x << ", " << connecting_p1->y << ", " << connecting_p1->z << ")" << std::endl;
			ss << "connecting_p2: (" << connecting_p2->x << ", " << connecting_p2->y << ", " << connecting_p2->z << ")" << std::endl;
#endif

			// Reconstruct the plane to include this edge.
			std::vector<glm::vec3> updated_points;
			for (const glm::vec3& p1 : ccw_plane->getPoints())
			{
				updated_points.push_back(p1);
				if (connecting_p1 == &p1)
				{
					int i;
					for (i = 0; i < obstacle->getPoints().size(); ++i)
					{
						if (&obstacle->getPoints()[i] == connecting_p2)
							break;
					}

					for (int counter = 0; counter < obstacle->getPoints().size(); ++counter)
					{
						updated_points.push_back(obstacle->getPoints()[(i + counter) % obstacle->getPoints().size()]);
					}
					updated_points.push_back(*connecting_p2);
					updated_points.push_back(*connecting_p1);
				}
			}

			ccw_plane->setPoints(updated_points, ccw_plan_normal);
		}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "[ConvexNavigationArea::triangulateAndMerge] Updated plane (ccw): " << *ccw_plane << std::endl;
#endif
		// Iteratively find 'ears' can clip them off.
		while (ccw_plane->getPoints().size() > 3)
		{
			for (unsigned int i = 0; i < ccw_plane->getPoints().size(); ++i)
			{
				const glm::vec3& p1 = ccw_plane->getPoints()[i];
				const glm::vec3& p2 = ccw_plane->getPoints()[(i + 1) % ccw_plane->getPoints().size()];
				const glm::vec3& p3 = ccw_plane->getPoints()[(i + 2) % ccw_plane->getPoints().size()];

				std::vector<glm::vec3> triangle_points;
				triangle_points.push_back(p1);
				triangle_points.push_back(p2);
				triangle_points.push_back(p3);

				glm::vec2 v1(p1.x - p2.x, p1.z - p2.z);
				glm::vec2 v2(p3.x - p2.x, p3.z - p2.z);

				float angle1 = Math::signedAngle(v1, v2);
				
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				float angle2 = Math::signedAngle(v2, v1);
				ss << "[ConvexNavigationArea::triangulateAndMerge] Consider the triangle: (" << p1.x << "," << p1.y << "," << p1.z << "), (" << p2.x << ", " << p2.y << ", " << p2.z << "), (" << p3.x << ", " << p3.y << ", " << p3.z << ")" << std::endl;
				ss << "\t[ConvexNavigationArea::triangulateAndMerge] Angle (" << v1.x << ", " << v1.y << ") - (" << v2.x << ", " << v2.y << "): " << angle1 << "; Angle p3, p2: " << angle2 << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				
				bool valid_edge = false;

				// An angle between 0 and -PI means it's a potential angle for the triangle.
				if (angle1 < 0)
				{
					
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "\t[ConvexNavigationArea::triangulateAndMerge] Angle OK!" << std::endl;
#endif
					
					valid_edge = true;

					// Check if this new edge collides with other edges of this plane.
					for (unsigned int j = 0; j < ccw_plane->getPoints().size(); ++j)
					{
						const glm::vec3& other_p1 = ccw_plane->getPoints()[j];
						const glm::vec3& other_p2 = ccw_plane->getPoints()[(j + 1) % ccw_plane->getPoints().size()];

						// Ignore edges that share the veretx v1 or v2.
						bool other_p1_is_on_edge = false;
						bool other_p2_is_on_edge = false;
						
						if (glm::distance(other_p1, p1) < EPSILON ||
							glm::distance(other_p1, p2) < EPSILON ||
							glm::distance(other_p1, p3) < EPSILON)
						{
							other_p1_is_on_edge = true;
						}
						// Check if this point is on the inside of the triangle, if so then we cannot form a triangle.
						else if (Plane::isInsidePlane(triangle_points, other_p1))
						{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "\tIn the triangle: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ")!" << std::endl;
#endif
							valid_edge = false;
							break;
						}

						if (glm::distance(other_p2, p1) < EPSILON ||
							glm::distance(other_p2, p2) < EPSILON ||
							glm::distance(other_p2, p3) < EPSILON)
						{
							other_p2_is_on_edge = true;
						}
						// Check if this point is on the inside of the triangle, if so then we cannot form a triangle.
						else if (Plane::isInsidePlane(triangle_points, other_p2))
						{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "\tIn the triangle: (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << ")!" << std::endl;
#endif
							valid_edge = false;
							break;
						}

						if (other_p1_is_on_edge && other_p2_is_on_edge)
						{
							continue;
						}
						
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "\tother_p1 = (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ")" << std::endl;
						ss << "\tother_p2 = (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << ")" << std::endl;

#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
#endif
						
						float distance = Math::dist3D_Segment_to_Segment(p1, p2, other_p1, other_p2);

						if (distance <= EPSILON)
						{
							// Make sure we do not collide with p1 or p2.
							glm::vec3 intersection_p1, intersection_p2;
							Math::getIntersection(p1, p2, other_p1, other_p2, intersection_p1, intersection_p2);

							if (((glm::distance(p1, intersection_p1) >= EPSILON && glm::distance(p2, intersection_p1) >= EPSILON)) ||
								((glm::distance(p1, intersection_p2) >= EPSILON && glm::distance(p2, intersection_p2) >= EPSILON)))
							{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Line: (" << p1.x << ", " << p1.y << ", " << p1.z << ") - (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Collides with: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Intersections: (" << intersection_p1.x << ", " << intersection_p1.y << ", " << intersection_p1.z << ")-  (" << intersection_p2.x << ", " << intersection_p2.y << ", " << intersection_p2.z << ")" << std::endl;
#endif

								valid_edge = false;
								break;
							}
						}
						
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						else
						{
							ss << "\t[ConvexNavigationArea::triangulateAndMerge] No collision: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << "); Distance=" << distance << std::endl;
						}
#endif
						
						distance = Math::dist3D_Segment_to_Segment(p3, p2, other_p1, other_p2);

						if (distance <= EPSILON)
						{
							// Make sure we do not collide with p1 or p2.
							glm::vec3 intersection_p1, intersection_p2;
							Math::getIntersection(p3, p2, other_p1, other_p2, intersection_p1, intersection_p2);

							if (((glm::distance(p3, intersection_p1) >= EPSILON && glm::distance(p2, intersection_p1) >= EPSILON)) ||
								((glm::distance(p3, intersection_p2) >= EPSILON && glm::distance(p2, intersection_p2) >= EPSILON)))
							{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Line: (" << p3.x << ", " << p3.y << ", " << p3.z << ") - (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Collides with: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Intersections: (" << intersection_p1.x << ", " << intersection_p1.y << ", " << intersection_p1.z << ")-  (" << intersection_p2.x << ", " << intersection_p2.y << ", " << intersection_p2.z << ")" << std::endl;
#endif
								valid_edge = false;
								break;
							}
						}
						
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						else
						{
							ss << "\t[ConvexNavigationArea::triangulateAndMerge] No collision: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << "); Distance=" << distance << std::endl;
						}
#endif
						
						distance = Math::dist3D_Segment_to_Segment(p1, p3, other_p1, other_p2);

						if (distance <= EPSILON)
						{
							// Make sure we do not collide with p1 or p2.
							glm::vec3 intersection_p1, intersection_p2;
							Math::getIntersection(p1, p3, other_p1, other_p2, intersection_p1, intersection_p2);

							if (((glm::distance(p1, intersection_p1) >= EPSILON && glm::distance(p3, intersection_p1) >= EPSILON)) ||
								((glm::distance(p1, intersection_p2) >= EPSILON && glm::distance(p3, intersection_p2) >= EPSILON)))
							{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Line: (" << p1.x << ", " << p1.y << ", " << p1.z << ") - (" << p3.x << ", " << p3.y << ", " << p3.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Collides with: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << ")" << std::endl;
								ss << "\t[ConvexNavigationArea::triangulateAndMerge] Intersections: (" << intersection_p1.x << ", " << intersection_p1.y << ", " << intersection_p1.z << ")-  (" << intersection_p2.x << ", " << intersection_p2.y << ", " << intersection_p2.z << ")" << std::endl;
#endif
								valid_edge = false;
								break;
							}
						}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						else
						{
							ss << "\t[ConvexNavigationArea::triangulateAndMerge] No collision: (" << other_p1.x << ", " << other_p1.y << ", " << other_p1.z << ") - (" << other_p2.x << ", " << other_p2.y << ", " << other_p2.z << "); Distance=" << distance << std::endl;
						}
#endif
					}
				}

				// If this a valid ear, clip it off.
				if (valid_edge)
				{
					std::vector<glm::vec3> triangle_points;
					triangle_points.push_back(p1);
					triangle_points.push_back(p2);
					triangle_points.push_back(p3);
					Plane* triangle = new Plane(triangle_points, ccw_plane->getNormal());
					triangles.push_back(triangle);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "\t\t[ConvexNavigationArea::triangulateAndMerge] Found a triangle: " << *triangle << std::endl;
#endif
					// Update the plane by removing these edges.
					std::vector<glm::vec3> updated_points = ccw_plane->getPoints();
					updated_points.erase(updated_points.begin() + (i + 1) % ccw_plane->getPoints().size());
					ccw_plane->setPoints(updated_points, ccw_plan_normal);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "\t\t[ConvexNavigationArea::triangulateAndMerge] Create new plane with: " << updated_points.size() << " points." << std::endl;
					ss << *ccw_plane << std::endl;
					OutputDebugString(ss.str().c_str());
					ss.str(std::string());
#endif
					break;
				}
			}
		}

		// The remainder forms the last triangle.
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "\t\tFound a triangle: " << *ccw_plane << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif
		triangles.push_back(ccw_plane);

		// Merge these triangles into convex shapes.
		merge(triangles, obstacles, result);

		//result.insert(result.end(), triangles.begin(), triangles.end()); // DEBUG.
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "[ConvexNavigationArea::triangulateAndMerge] Result:" << std::endl;
	for (Plane* plane : result)
	{
		ss << "\t" << *plane << std::endl;
	}
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
#endif
}

/**
 * For now we assume that a plane is projected y-plane.
 */
bool ConvexNavigationArea::isCounterClockWise(const Plane& plane) const
{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	std::stringstream ss;
	ss << "[ConvexNavigationArea::isCounterClockWise] " << plane << std::endl;
#endif
	if (plane.getPoints().size() <= 2)
	{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "[ConvexNavigationArea::isCounterClockWise] Plane is a line or a point. Returning false." << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
#endif
		return false;
	}

	// Select the left-most point.
	int left_most_index = 0;
	float left_most_value = std::numeric_limits<float>::max();

	// Determine on which axis to project this plane.
	glm::vec3 mask;
	if (std::abs(plane.getNormal().x) >= std::abs(plane.getNormal().y) && std::abs(plane.getNormal().x) >= std::abs(plane.getNormal().z))
	{
		mask = glm::vec3(0, 1, 1);
	}
	else if (std::abs(plane.getNormal().y) >= std::abs(plane.getNormal().x) && std::abs(plane.getNormal().y) >= std::abs(plane.getNormal().z))
	{
		mask = glm::vec3(1, 0, 1);
	}
	else if (std::abs(plane.getNormal().z) >= std::abs(plane.getNormal().x) && std::abs(plane.getNormal().z) >= std::abs(plane.getNormal().y))
	{
		mask = glm::vec3(1, 1, 0);
	}
	else
	{
		std::stringstream ss_d;
		ss_d << "Could not find a mask, this is impossible!" << std::endl;
		ss_d << plane << std::endl;
#ifdef WIN32
		OutputDebugString(ss_d.str().c_str());
#else
		std::cout << ss_d.str() << std::endl;
#endif
		assert(false);
		exit(-1);
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "Mask: (" << mask.x << ", " << mask.y << ", " << mask.z << ")" << std::endl;
#endif
	// Find the minimal value on the predominate axis.
	for (int i = 0; i < plane.getPoints().size(); ++i)
	{
		//const glm::vec3& p = plane.getPoints()[i];
		for (int j = 0; j < 3; ++j)
		{
			if (mask[j] == 1)
			{
				float value;
				if (j == 0) value = plane.getPoints()[i].x;
				else if (j == 1) value = plane.getPoints()[i].y;
				else if (j == 2) value = plane.getPoints()[i].z;
				if (value < left_most_value)
				{
					left_most_value = value;
					left_most_index = i;
				}
				break;
			}
		}
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "[" << left_most_index << "] Value=" << left_most_value << std::endl;
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
#endif
	// Check the angle of the edges that start or end at the found vertex.
	int previous_edge_index = left_most_index == 0 ? plane.getPoints().size() - 1 : left_most_index - 1;
	int next_edge_index = (left_most_index + 1) % plane.getPoints().size();

	if (mask.x == 0)
	{
		glm::vec2 v2(plane.getPoints()[left_most_index].y - plane.getPoints()[next_edge_index].y, plane.getPoints()[left_most_index].z - plane.getPoints()[next_edge_index].z);
		glm::vec2 v1(plane.getPoints()[left_most_index].y - plane.getPoints()[previous_edge_index].y, plane.getPoints()[left_most_index].z - plane.getPoints()[previous_edge_index].z);

		return Math::signedAngle(v1, v2) < 0;
	}
	else if (mask.y == 0)
	{
		glm::vec2 v2(plane.getPoints()[left_most_index].x - plane.getPoints()[next_edge_index].x, plane.getPoints()[left_most_index].z - plane.getPoints()[next_edge_index].z);
		glm::vec2 v1(plane.getPoints()[left_most_index].x - plane.getPoints()[previous_edge_index].x, plane.getPoints()[left_most_index].z - plane.getPoints()[previous_edge_index].z);

		return Math::signedAngle(v1, v2) < 0;
	}
	else if (mask.z == 0)
	{
		glm::vec2 v2(plane.getPoints()[left_most_index].y - plane.getPoints()[next_edge_index].y, plane.getPoints()[left_most_index].z - plane.getPoints()[next_edge_index].z);
		glm::vec2 v1(plane.getPoints()[left_most_index].y - plane.getPoints()[previous_edge_index].y, plane.getPoints()[left_most_index].z - plane.getPoints()[previous_edge_index].z);

		return Math::signedAngle(v1, v2) < 0;
	}
	else
	{
		assert(false);
		exit(-1);
	}
}

void ConvexNavigationArea::merge(const std::vector<Plane*>& start_triangles, const std::vector<const Plane*>& obstacles, std::vector<Plane*>& result) const
{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	std::stringstream ss;
	ss << "[ConvexNavigationArea::merge]" << std::endl;
	for (Plane* plane : start_triangles)
	{
		ss << "- " << *plane << std::endl;
	}
#endif
	if (start_triangles.size() == 1)
	{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Only a single triangle, nothing to merge." << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif
		result.push_back(start_triangles[0]);
		return;
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif
	// Create a copy to modify.
	std::vector<Plane*> triangles(start_triangles);

	// Next try to merge all triangles.
	bool done_merging = false;
	while (!done_merging)
	{
		done_merging = true;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
		ss << "Start merging, #trinagles: " << triangles.size() << std::endl;
#ifdef WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
		ss.str(std::string());
#endif
		for (int i = triangles.size() - 1; i >= 0; --i)
		{
			Plane* triangle = triangles[i];

			for (int j = 0; j < triangles.size(); ++j)
			{
				if (i == j) break;

				Plane* other_triangle = triangles[j];
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Try to merge: " << std::endl << *triangle << std::endl << "and" << std::endl << *other_triangle << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				// Check if these triangles share an edge.
				glm::vec3 shared_p1(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
				glm::vec3 shared_p2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

				for (int k = 0; k < triangle->getPoints().size(); ++k)
				{
					for (const glm::vec3& other_triangle_p : other_triangle->getPoints())
					{
						if (glm::distance(other_triangle_p, triangle->getPoints()[k]) < EPSILON)
						{
							if (shared_p1.x != std::numeric_limits<float>::max())
							{
								shared_p2 = other_triangle_p;
							}
							else
							{
								shared_p1 = other_triangle_p;
							}
						}
					}
				}

				if (shared_p2.x == std::numeric_limits<float>::max())
				{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "Merging failed!" << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					continue;
				}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Shared points: p1=(" << shared_p1.x << ", " << shared_p1.y << ", " << shared_p1.z << ")" << std::endl;
				ss << "Shared points: p2=(" << shared_p2.x << ", " << shared_p2.y << ", " << shared_p2.z << ")" << std::endl;

#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				// Check if the remaining two points do not intersect with any of the hole.
				bool collides_with_hole = false;
				glm::vec3 intersection;
				glm::vec3 intersection2;
				for (std::vector<glm::vec3>::const_iterator ci = triangle->getPoints().begin(); ci != triangle->getPoints().end(); ++ci)
				{
					const glm::vec3& p1 = *ci;
					if (p1 == shared_p1 || p1 == shared_p2) continue;
					for (std::vector<glm::vec3>::const_iterator ci = other_triangle->getPoints().begin(); ci != other_triangle->getPoints().end(); ++ci)
					{
						const glm::vec3& p2 = *ci;
						if (p2 == shared_p1 || p2 == shared_p2) continue;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "Check connection: (" << p1.x << " " << p1.y << ", " << p1.z << ") - (" << p2.x << " " << p2.y << ", " << p2.z << ")" << std::endl;
#ifdef WIN32
						OutputDebugString(ss.str().c_str());
#else
						std::cout << ss.str() << std::endl;
#endif
						ss.str(std::string());
#endif
						for (std::vector<const Plane*>::const_iterator ci = obstacles.begin(); ci != obstacles.end(); ++ci)
						{
							const Plane* obstacle_plane = *ci;
							
							// TODO: This does not work!
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "Plane: " << *obstacle_plane << std::endl;
							ss << "Distance to plane: " << obstacle_plane->getDistance(p1, p2) << std::endl;
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());

							if (!obstacle_plane->intersectsWith(p1, p2, intersection))
							{
								ss << "intersection1! (" << intersection.x << ", " << intersection.y << ", " << intersection.z << ")" << std::endl;
							}
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());
							if (!obstacle_plane->intersectsWith(p2, p1, intersection2))
							{
								ss << "intersection2! (" << intersection2.x << ", " << intersection2.y << ", " << intersection2.z << ")" << std::endl;
							}

#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());

							ss << "Intersections: (" << intersection.x << ", " << intersection.y << ", " << intersection.z << ") - (" << intersection2.x << " " << intersection2.y << ", " << intersection2.z << ")" << std::endl;
#ifdef WIN32
							OutputDebugString(ss.str().c_str());
#else
							std::cout << ss.str() << std::endl;
#endif
							ss.str(std::string());
#endif
							// TODO: Check when we find an intersection with an edge of a plane, such that the line is orthorgonal to the plane. 
							// In that case we should not count it as an interesection.
							if (obstacle_plane->getDistance(p1, p2) < EPSILON && obstacle_plane->intersectsWith(p1, p2, intersection) && obstacle_plane->intersectsWith(p2, p1, intersection2))
							{
								collides_with_hole = true;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "Intersects with obstacle plane! (" << intersection.x << ", " << intersection.y << ", " << intersection.z << ")" << std::endl;
#endif
								// Ignore the intersection if the intersections are actually part of the start and end point of the lines we are checking.
								if ((glm::distance(intersection, p1) <= EPSILON || glm::distance(intersection, p2) <= EPSILON) &&
									(glm::distance(intersection2, p1) <= EPSILON || glm::distance(intersection2, p2) <= EPSILON))
								{
									collides_with_hole = false;
								}

								// Ignore the intersection, if they are part of a plane's edge.
								for (int j = 0; j < obstacle_plane->getPoints().size(); ++j)
								{
									const glm::vec3& p1 = obstacle_plane->getPoints()[j];
									const glm::vec3& p2 = obstacle_plane->getPoints()[(j + 1) % obstacle_plane->getPoints().size()];
									if ((glm::distance(intersection, p1) <= EPSILON || glm::distance(intersection, p2) <= EPSILON) &&
										(glm::distance(intersection2, p1) <= EPSILON || glm::distance(intersection2, p2) <= EPSILON))
									{
										collides_with_hole = false;
									}
								}

								if (collides_with_hole) break;
							}
						}
						if (collides_with_hole) break;
					}

					if (collides_with_hole) break;
				}

				if (collides_with_hole)
				{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "This line collides with an obstacle." << std::endl;
#endif
					continue;
				}

				// Merge the polygons. We first order the polygons such that one goes cw and the other goes ccw.
				bool triangle_in_sequence = false;
				for (int k = 0; k < triangle->getPoints().size(); ++k)
				{
					if (glm::distance(triangle->getPoints()[k], shared_p1) < EPSILON && glm::distance(triangle->getPoints()[(k + 1) % triangle->getPoints().size()], shared_p2) < EPSILON)
					{
						triangle_in_sequence = true;
						break;
					}
				}

				bool other_triangle_in_sequence = false;
				for (int k = 0; k < other_triangle->getPoints().size(); ++k)
				{
					if (glm::distance(other_triangle->getPoints()[k], shared_p1) < EPSILON && glm::distance(other_triangle->getPoints()[(k + 1) % other_triangle->getPoints().size()], shared_p2))
					{
						other_triangle_in_sequence = true;
						break;
					}
				}

				std::vector<glm::vec3> other_triangle_points = other_triangle->getPoints();
				if (triangle_in_sequence != other_triangle_in_sequence)
				{
					std::reverse(other_triangle_points.begin(), other_triangle_points.end());
				}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Ready to merge!" << std::endl;
#endif
				// Now we are ready to merge them.
				std::vector<glm::vec3> new_triangle;

				// Start adding the points of the first triangle, before
				for (int k = 0; k < triangle->getPoints().size(); ++k)
				{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "Add [" << k << "] (" << triangle->getPoints()[k].x << " " << triangle->getPoints()[k].y << ", " << triangle->getPoints()[k].z << ")" << std::endl;
#endif
					new_triangle.push_back(triangle->getPoints()[k]);
					if ((glm::distance(triangle->getPoints()[k], shared_p1) < EPSILON && glm::distance(triangle->getPoints()[(k + 1) % triangle->getPoints().size()], shared_p2) < EPSILON) ||
					    (glm::distance(triangle->getPoints()[k], shared_p2) < EPSILON && glm::distance(triangle->getPoints()[(k + 1) % triangle->getPoints().size()], shared_p1) < EPSILON))
					{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "Swap to the other triangle!" << std::endl;
#endif
						// Find the begin point of the other triangle.
						int l = 0;
						for (; l <other_triangle_points.size(); l = (l + 1) % other_triangle_points.size())
						{
							if (glm::distance(other_triangle_points[l], triangle->getPoints()[k]) < EPSILON)
							{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "Start from: [" << l << "] (" << other_triangle_points[l].x << " " << other_triangle_points[l].y << ", " << other_triangle_points[l].z << ")" << std::endl;
#endif
								l = (l + 1) % other_triangle_points.size();
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
								ss << "Next to inspect: [" << l << "] (" << other_triangle_points[l].x << " " << other_triangle_points[l].y << ", " << other_triangle_points[l].z << ")" << std::endl;
#endif
								break;
							}
						}

						// Traverse this triangle, until we reach the other shared point.
						while (glm::distance(other_triangle_points[l], shared_p1) >= EPSILON && glm::distance(other_triangle_points[l], shared_p2) >= EPSILON)
						{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
							ss << "Add [" << l << "] (" << other_triangle_points[l].x << " " << other_triangle_points[l].y << ", " << other_triangle_points[l].z << ")" << std::endl;
#endif
							new_triangle.push_back(other_triangle_points[l]);
							l = (l + 1) % other_triangle_points.size();
						}
					}
				}

				// Remove each vertex that lies on an edge of this plane (angles are PI).
				for (int i = new_triangle.size() - 1; i >= 0; --i)
				{
					const glm::vec3& p1 = new_triangle[(i == 0 ? new_triangle.size() - 1 : i - 1)];
					const glm::vec3& p2 = new_triangle[i];
					const glm::vec3& p3 = new_triangle[(i + 1) % new_triangle.size()];
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					OutputDebugString(ss.str().c_str());
					ss.str(std::string());
#endif

					// If p2 lies on the edge p1-p3, then we can remove p2.
					float distance = Math::dist3D_Segment_to_Point(p1, p3, p2);
					if (distance < EPSILON)
					{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
						ss << "Remove the vertex: (" << p2.x << ", " << p2.y << ", " << p2.z << "); Distance=" << distance << std::endl;
#endif
						new_triangle.erase(new_triangle.begin() + i);
					}
				}

				// Check that the shape is convex.
				bool shape_is_convex = true;
				for (unsigned int i = 0; i < new_triangle.size(); ++i)
				{
					const glm::vec3& p1 = new_triangle[i];
					const glm::vec3& p2 = new_triangle[(i + 1) % new_triangle.size()];
					const glm::vec3& p3 = new_triangle[(i + 2) % new_triangle.size()];

					glm::vec2 v1(p1.x - p2.x, p1.z - p2.z);
					glm::vec2 v2(p3.x - p2.x, p3.z - p2.z);

					float angle1 = Math::signedAngle(v1, v2);
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					float angle2 = Math::signedAngle(v2, v1);
					ss << "Consider the merged triangle: (" << p1.x << "," << p1.y << "," << p1.z << "), (" << p2.x << ", " << p2.y << ", " << p2.z << "), (" << p3.x << ", " << p3.y << ", " << p3.z << ")" << std::endl;
					ss << "Angle (" << v1.x << ", " << v1.y << ") - (" << v2.x << ", " << v2.y << "): " << angle1 << "; Angle p3, p2: " << angle2 << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					// An angle between 0 and PI means it's a potential angle for the triangle.
					if (angle1 > 0)
					{
						shape_is_convex = false;
						break;
					}
				}

				if (!shape_is_convex)
				{
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
					ss << "Triangle is not convex!" << std::endl;
#ifdef WIN32
					OutputDebugString(ss.str().c_str());
#else
					std::cout << ss.str() << std::endl;
#endif
					ss.str(std::string());
#endif
					continue;
				}

				// Mark this algorithm as not done yet.
				done_merging = false;
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Create a new plane with: " << new_triangle.size() << " points!" << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());

#endif
				// Add the new triangle.
				Plane* plane = new Plane(new_triangle, triangle->getNormal());
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
				ss << "Merged plane: " << std::endl << *plane << std::endl;
				ss << "Update the triangle: " << std::endl;
				for (const Plane* p : triangles)
				{
					ss << "\t" << *p << std::endl;
				}
				ss << "Remove the " << i << "th and " << j << "th triangle!" << std::endl;
#ifdef WIN32
				OutputDebugString(ss.str().c_str());
#else
				std::cout << ss.str() << std::endl;
#endif
				ss.str(std::string());
#endif
				triangles.erase(triangles.begin() + i);
				triangles.erase(triangles.begin() + j);
				triangles.push_back(plane);
				break;
			}

			// More elements have been erased from the list, restart to avoid errors.
			if (!done_merging) break;
		}
	}
#ifdef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H_ENABLE_DEBUG
	ss << "[merge] Done!" << std::endl;
	for (Plane* merged_plane : triangles)
	{
		ss << "\t- " << *merged_plane << std::endl;
	}
#ifdef WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif
	ss.str(std::string());
#endif
	result.insert(result.end(), triangles.begin(), triangles.end());
}

std::ostream& operator<<(std::ostream& os, const ConvexNavigationArea& area)
{
	os << "Area: ";
	for (std::vector<glm::vec3>::const_iterator ci = area.plane_->getPoints().begin(); ci != area.plane_->getPoints().end(); ++ci)
	{
		os << "(" << (*ci).x << "," << (*ci).y << "," << (*ci).z << ")";
		if (ci + 1 != area.plane_->getPoints().end()) os << " - ";
	}
	return os;
}

};

