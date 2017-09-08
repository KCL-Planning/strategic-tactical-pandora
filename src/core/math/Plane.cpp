#include "dpengine/math/Plane.h"
#include "dpengine/math/Math.h"

#include <sstream>
#include <map>
#include <algorithm>
#include <iostream>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

//#define ENABLE_CORE_COLLISION_PLANE_H_DEBUG

namespace DreadedPE
{
Plane::Plane(const std::vector<glm::vec3>& convex_shape)
{
	setPoints(convex_shape);
}

Plane::Plane(const std::vector<glm::vec3>& convex_shape, const glm::vec3& normal)
{
	setPoints(convex_shape);
	normal_ = normal;
	d_ = -glm::dot(normal_, convex_shape[0]);
}

void Plane::setPoints(const std::vector<glm::vec3>& convex_shape)
{
	assert(convex_shape.size() > 2);
	points_ = convex_shape;
	bounded_lines_.clear();
	for (unsigned int i = 0; i < convex_shape.size(); ++i)
	{
		bounded_lines_.push_back(convex_shape[(i + 1) % convex_shape.size()] - convex_shape[i]);
	}

	normal_ = glm::normalize(glm::cross(convex_shape[1] - convex_shape[0], convex_shape[2] - convex_shape[0]));
	d_ = -glm::dot(normal_, convex_shape[0]);

	/*
	// Calculate the centre of this plane.
	float area = 0;
	centre_.x = 0;
	centre_.y = 0;
	for (int i = 0; i < points_.size(); ++i)
	{
		area += points_[i].x * points_[(i + 1) % points_.size()].y - points_[(i + 1) % points_.size()].x * points_[i].y;
		centre_.x += (points_[i].x + points_[(i + 1) % points_.size()].x) * (points_[i].x * points_[(i + 1) % points_.size()].y - points_[(i + 1) % points_.size()].x * points_[i].y);
		centre_.y += (points_[i].y + points_[(i + 1) % points_.size()].y) * (points_[i].x * points_[(i + 1) % points_.size()].y - points_[(i + 1) % points_.size()].x * points_[i].y);
	}
	area *= 0.5f;
	centre_.x = centre_.x / (6 * area);
	centre_.y = centre_.y / (6 * area);
	*/

	for (int i = 0; i < points_.size(); ++i)
	{
		centre_ += points_[i];
	}
	centre_ /= points_.size();

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	std::stringstream ss;
	ss << "Create a plane with the points: " << std::endl;
	for (const glm::vec3 p : convex_shape)
	{
		ss << "(" << p.x << ", " << p.y << ", " << p.z << ")" << std::endl;
	}
	//ss << "Area: " << area << std::endl;
	ss << "Centre point: (" << centre_.x << ", " << centre_.y << ", " << centre_.z << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
#endif
}

void Plane::setPoints(const std::vector<glm::vec3>& points, const glm::vec3& normal)
{
	setPoints(points);
	normal_ = normal;
}

bool Plane::intersectsWith(const glm::vec3& begin_point, const glm::vec3& end_point, glm::vec3& intersection_point, bool check_segment_only) const
{
	static float EPSILON = 0.001f;
	glm::vec3 direction(end_point - begin_point);

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	std::stringstream ss;
	ss << "[Plane::intersectsWith] " << (check_segment_only ? "segment" : "") << ": (" << begin_point.x << ", " << begin_point.y << ", " << begin_point.z << ") - (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")" << std::endl;
	ss << "Plane: " << *this<< std::endl;
#endif

	// If the direction is orthorgonal to the normal vector, we check collisitions with the line edges.
	// We return the point closest to the begin_point.
	if (std::abs(glm::dot(normal_, direction)) < EPSILON)
	{

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "Normal is orthogonal to the direction!" << (check_segment_only ? "segment" : "") << ": (" << begin_point.x << ", " << begin_point.y << ", " << begin_point.z << ") - (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")" << std::endl;
		ss << "Direction: (" << direction.x << ", " << direction.y << ", " << direction.z << ");" << std::endl;
#endif

		glm::vec3 intersection_p1, intersection_p2;
		float closest_intersection = std::numeric_limits<float>::max();
		for (int i = 0; i < points_.size(); ++i)
		{
			const glm::vec3& p1 = points_[i];
			const glm::vec3& p2 = points_[(i + 1) % points_.size()];

			float distance;
			
			if (check_segment_only)
			{
				distance = Math::dist3D_Segment_to_Segment(begin_point, end_point, p1, p2);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Distance to intersection is: " << distance << std::endl;
#endif
				if (distance > EPSILON)
				{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "Distance is too long. No intersection!" << std::endl;
#endif
					continue;
				}
				Math::getIntersection(begin_point, end_point, p1, p2, intersection_p1, intersection_p2);
				
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Intersection p1: (" << intersection_p1.x << ", " << intersection_p1.y << ", " << intersection_p1.z << ")" << std::endl;
				ss << "Intersection p2: (" << intersection_p2.x << ", " << intersection_p2.y << ", " << intersection_p2.z << ")" << std::endl;
#endif
				
				float distance_to_begin = glm::distance(intersection_p1, begin_point);
				if (distance_to_begin < closest_intersection)
				{
					closest_intersection = distance_to_begin;
					intersection_point = intersection_p1;
				}
			}
			else
			{
				// TODO: Need more checking!
				distance = Math::getIntersection(begin_point, end_point, p1, p2, intersection_p1, intersection_p2);
				if (distance > EPSILON) continue;

				if (distance < closest_intersection)
				{
					closest_intersection = distance;
					intersection_point = intersection_p1;
				}
			}

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "Distance is: " << distance << std::endl;
#endif

		}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		OutputDebugString(ss.str().c_str());
#endif
		return closest_intersection != std::numeric_limits<float>::max();
	}

	float t = -(glm::dot(normal_, begin_point) + d_) / glm::dot(normal_, direction);
	
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	ss << "Line " << (check_segment_only ? "segment" : "" ) << ": (" << begin_point.x << ", " << begin_point.y << ", " << begin_point.z << ") - (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")" << std::endl;
	ss << "Direction: (" << direction.x << ", " << direction.y << ", " << direction.z << "); t=" << t << std::endl;
#endif
	
	// If the line intersects outside the given segment then there is no collision.
	if (check_segment_only && (t < 0 || t > 1))
	{
		
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "No intersection found. t == " << t << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
		return false;
	}

	glm::vec3 point(begin_point + direction * t);
	intersection_point = point;
	
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	ss << "Plane intersection: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	for (int i = 0; i < points_.size(); ++i)
	{
		ss << "(" << points_[i].x << ", " << points_[i].y << ", " << points_[i].z << ") -- ";
	}
	ss << std::endl;
	ss << "Bounded lines: " << std::endl;
	for (int i = 0; i < points_.size(); ++i)
	{
		ss << "(" << bounded_lines_[i].x << ", " << bounded_lines_[i].y << ", " << bounded_lines_[i].z << ") -- ";
	}
	ss << std::endl;
#endif
	
	// Calculate if the point is inside the bounded box, we do this by calculating cross products
	// from each edge of the bounded plane and the calculated point. Iff the sign is the same the
	// point is inside the bounded plane, otherwise it must be outside of it.
	bool signs[3];
	for (unsigned int i = 0; i < points_.size(); ++i)
	{
		glm::vec3 cross = glm::cross(bounded_lines_[i], point - points_[i]);
		float signed_distance = glm::dot(normal_, point) + d_;

		// Solve floating point rounding for problems.
		static float EPSILON = 0.001f;
		if (signed_distance < EPSILON && signed_distance > -EPSILON)
		{
			signed_distance = 0;
		}

		for (unsigned int i = 0; i < 3; ++i)
		{
			if (cross[i] < EPSILON && cross[i] > -EPSILON)
			{
				cross[i] = 0;
			}
		}
		
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "[" << i << "] Cross: (" << cross.x << ", " << cross.y << ", " << cross.z << ") Signed distance: " << signed_distance << std::endl;
		ss << "Signs: " << signs[0] << ", " << signs[1] << ", " << signs[2] << std::endl;
#endif
		
		if (i == 0)
		{
			signs[0] = cross[0] < 0.0f ? false : true;
			signs[1] = cross[1] < 0.0f ? false : true;
			signs[2] = cross[2] < 0.0f ? false : true;
		}
		else if (cross[0] >= 0 != signs[0] ||
			     cross[1] >= 0 != signs[1] ||
			     cross[2] >= 0 != signs[2]) 
		{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			OutputDebugString(ss.str().c_str());
#endif
			return false;			 
		}
	}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	OutputDebugString(ss.str().c_str());
#endif
	return true;
}

bool Plane::intersectsWithRay(const glm::vec3& begin_point, const glm::vec3& direction, glm::vec3& intersection_point) const
{
/*
#ifdef _WIN32
{
	float d = glm::dot(glm::normalize(normal_), glm::normalize(direction));
	float t = -(glm::dot(normal_, begin_point) + d_) / glm::dot(normal_, direction);
	std::stringstream ss;
	ss << "intersectWithRay: " << std::endl;
	ss << begin_point.x << "," << begin_point.y << "," << begin_point.z << std::endl;
	ss << "direction: " << std::endl;
	ss << direction.x << "," << direction.y << "," << direction.z << std::endl;
	ss << "d: " << d << std::endl;
	ss << "t: " << t << std::endl;
	OutputDebugString(ss.str().c_str());
} 
#endif
*/

	// The line is parallel to the plane.
	float d = glm::dot(glm::normalize(normal_), glm::normalize(direction));
	if (abs(d) < 0.001f) return false;
	float t = -(glm::dot(normal_, begin_point) + d_) / glm::dot(normal_, direction);
	if (t < 0) return false;
	glm::vec3 point(begin_point + direction * t);
	intersection_point = point;

	// Calculate if the point is inside the bounded box, we do this by calculating cross products
	// from each edge of the bounded plane and the calculated point. Iff the sign is the same the
	// point is inside the bounded plane, otherwise it must be outside of it.
	bool signs[3];
	for (unsigned int i = 0; i < points_.size(); ++i)
	{
		glm::vec3 cross = glm::cross(bounded_lines_[i], point - points_[i]);
		float signed_distance = glm::dot(normal_, point) + d_;

		// Solve floating point rounding for problems.
		static float EPSILON = 0.001f;
		if (signed_distance < EPSILON && signed_distance > -EPSILON)
		{
			signed_distance = 0;
		}

		for (unsigned int i = 0; i < 3; ++i)
		{
			if (cross[i] < EPSILON && cross[i] > -EPSILON)
			{
				cross[i] = 0;
			}
		}

		if (i== 0)
		{
			signs[0] = cross[0] < 0.0f ? false : true;
			signs[1] = cross[1] < 0.0f ? false : true;
			signs[2] = cross[2] < 0.0f ? false : true;
		}
		else if (cross[0] >= 0 != signs[0] ||
			     cross[1] >= 0 != signs[1] ||
			     cross[2] >= 0 != signs[2]) 
		{
			return false;
		}
	}

	return true;
}

bool Plane::isInsidePlane(const glm::vec3& point) const
{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	std::stringstream ss;
	ss << "Plane::isInsidePlane(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
	ss << *this << std::endl;
#endif

	// First check if the point is not too far from the plane.
	static float EPSILON = 0.001f;
	if (getDistance(point) > EPSILON)
	{
		return false;
	}

	// Calculate if the point is inside the bounded box, we do this by calculating cross products
	// from each edge of the bounded plane and the calculated point. Iff the sign is the same the
	// point is inside the bounded plane, otherwise it must be outside of it.
	bool signs[3];
	for (unsigned int i = 0; i < points_.size(); ++i)
	{
		glm::vec3 cross = glm::cross(bounded_lines_[i], point - points_[i]);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "Cross product (" << bounded_lines_[i].x << ", " << bounded_lines_[i].y << ", " << bounded_lines_[i].z << ") - (" << (point - points_[i]).x << ", " << (point - points_[i]).y << ", " << (point - points_[i]).z << ")" << std::endl;
#endif
		// Solve floating point rounding for problems.
		for (unsigned int i = 0; i < 3; ++i)
		{
			if (cross[i] < EPSILON && cross[i] > -EPSILON)
			{
				cross[i] = 0;
			}
		}

		if (i == 0)
		{
			signs[0] = cross[0] < 0.0f ? false : true;
			signs[1] = cross[1] < 0.0f ? false : true;
			signs[2] = cross[2] < 0.0f ? false : true;
		}
		else if (cross[0] >= 0 != signs[0] ||
			cross[1] >= 0 != signs[1] ||
			cross[2] >= 0 != signs[2])
		{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			OutputDebugString(ss.str().c_str());
#endif
			return false;
		}
	}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	OutputDebugString(ss.str().c_str());
#endif
	return true;
}

bool Plane::intersectsWith(const Plane& plane, glm::vec3& begin_point, glm::vec3& end_point) const
{
	static float EPSILON = 0.001f;

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	std::stringstream ss;
	ss << "\tPlane::intersectsWith: " << std::endl;
	ss << "\t\t" << *this << std::endl;
	ss << "\tOther plane" << std::endl;
	ss << "\t\t" << plane << std::endl;
#endif
	glm::vec3 direction = glm::cross(normal_, plane.normal_);

	// If the cross product is 0, this means that the normals of te planes are identical.
	// NOTE: Might need to change this, if they overlap there is an intersection but it
	// cannot be characterised by a line.
	if (glm::length(direction) == 0.0f)
	{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "\tPlane normals are equal, no intersections!" << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
		return false;
	}
	direction = glm::normalize(direction);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	//ss << "\t\tDirection: (" << direction.x << ", " << direction.y << ", " << direction.z << ")" << std::endl;
#endif

	// Next we need to find a point on this line. We do this by finding a point that intersects 3 planes:
	// this one: <normal, d>, plane: <plane.normal, plane.d> and the plane given by <direction, 0>. For 
	// a point p to be on each of this planes the follow equation must hold:
	//
	// p DOT normal = -d;
	// 
	// In matrix form:
	// [  Nx  Ny  Nz ] [ px ]   [  -d ]
	// [ pNx pNy pNz ] [ py ] = [ -pd ] = NP = D
	// [ sNx xNy xNz ] [ pz ]   [ -nd ]
	//
	// To get P: P = N^{-1}D
	glm::mat3x3 N(normal_.x, plane.normal_.x, direction.x,
		normal_.y, plane.normal_.y, direction.y,
		normal_.z, plane.normal_.z, direction.z);
	glm::vec3 D(-d_, -plane.d_, 0);

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	//ss << "\t\t     [ " << N[0][0] << ", " << N[1][0] << ", " << N[2][0] << "]" << std::endl;
	//ss << "\t\t N = [ " << N[0][1] << ", " << N[1][1] << ", " << N[2][1] << "]" << std::endl;
	//ss << "\t\t     [ " << N[0][2] << ", " << N[1][2] << ", " << N[2][2] << "]" << std::endl;
	//glm::mat3x3 N_inverse = glm::inverse(N);
	//ss << "\t\t        [ " << N_inverse[0][0] << ", " << N_inverse[1][0] << ", " << N_inverse[2][0] << "]" << std::endl;
	//ss << "\t\t N^-1 = [ " << N_inverse[0][1] << ", " << N_inverse[1][1] << ", " << N_inverse[2][1] << "]" << std::endl;
	//ss << "\t\t        [ " << N_inverse[0][2] << ", " << N_inverse[1][2] << ", " << N_inverse[2][2] << "]" << std::endl;
#endif
	glm::vec3 P = glm::inverse(N) * D;
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	//ss << "\t\tP=(" << P.x << ", " << P.y << ", " << P.z << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	ss.str(std::string());
#endif

	// Next we find the intervals at which this line intersects both planes, then we find the smallest interval.
	glm::vec3 intersections[4];
	if (!getIntersectionInterval(direction, P, intersections[0], intersections[1]) ||
		!plane.getIntersectionInterval(direction, P, intersections[2], intersections[3]))
	{
		return false;
	}


	// Get the points that have the minimum distance to the other plane. These points for the 
	// line that intersect with both planes.
	glm::vec3* p1 = NULL;
	glm::vec3* p2 = NULL;

	for (int i = 0; i < 2; ++i)
	{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "Check (" << intersections[i].x << ", " << intersections[i].y << ", " << intersections[i].z << ")" << std::endl;
#endif
		bool is_close = false;
		if (plane.isInsidePlane(intersections[i]))
		{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "Point is inside the plane..." << std::endl;
#endif
			is_close = true;
		}

		// Check distance to the lines.
		if (!is_close)
		{
			for (int j = 0; j < plane.getPoints().size(); ++j)
			{
				const glm::vec3& plane_p1 = plane.getPoints()[j];
				const glm::vec3& plane_p2 = plane.getPoints()[(j + 1) % plane.getPoints().size()];

				float distance = Math::dist3D_Segment_to_Point(plane_p1, plane_p2, intersections[i]);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Distance to line (" << plane_p1.x << ", " << plane_p1.y << " ," << plane_p1.z << ") - (" << plane_p2.x << ", " << plane_p2.y << " ," << plane_p2.z << ") = " << distance << std::endl;
#endif
				if (distance < 0.0001f)
				{
					is_close = true;
					break;
				}
			}
		}

		// If this point is close to the other plane it is added.
		if (is_close)
		{
			if (p1 == NULL)
			{
				p1 = &intersections[i];
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "p1=(" << p1->x << ", " << p1->y << ", " << p1->z << ")" << std::endl;
#endif
			}
			else if (p2 == NULL && glm::distance(*p1, intersections[i]) > EPSILON)
			{
				p2 = &intersections[i];
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "p2=(" << p2->x << ", " << p2->y << ", " << p2->z << ")" << std::endl;
#endif
			}
			else if (glm::distance(*p1, intersections[i]) > EPSILON && glm::distance(*p2, intersections[i]) > EPSILON)
			{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Found a 3rd point (" << intersections[i].x << ", " << intersections[i].y << ", " << intersections[i].z << ")?" << std::endl;
				OutputDebugString(ss.str().c_str());
#endif
				assert(false);
			}
		}
	}

	for (int i = 2; i < 4; ++i)
	{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "Check (" << intersections[i].x << ", " << intersections[i].y << ", " << intersections[i].z << ")" << std::endl;
#endif
		bool is_close = false;
		if (isInsidePlane(intersections[i]))
		{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "Point is NOT inside the plane..." << std::endl;
#endif
			is_close = true;
		}

		// Check distance to the lines.
		if (!is_close)
		{
			for (int j = 0; j < getPoints().size(); ++j)
			{
				const glm::vec3& plane_p1 = getPoints()[j];
				const glm::vec3& plane_p2 = getPoints()[(j + 1) % getPoints().size()];

				float distance = Math::dist3D_Segment_to_Point(plane_p1, plane_p2, intersections[i]);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Distance to line (" << plane_p1.x << ", " << plane_p1.y << " ," << plane_p1.z << ") - (" << plane_p2.x << ", " << plane_p2.y << " ," << plane_p2.z << ") = " << distance << std::endl;
#endif
				if (distance < 0.0001f)
				{
					is_close = true;
					break;
				}
			}
		}

		// If this point is close to the other plane it is added.
		if (is_close)
		{
			if (p1 == NULL)
			{
				p1 = &intersections[i];
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "p1=(" << p1->x << ", " << p1->y << ", " << p1->z << ")" << std::endl;
#endif
			}
			else if (p2 == NULL && glm::distance(*p1, intersections[i]) > EPSILON)
			{
				p2 = &intersections[i];
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "p2=(" << p2->x << ", " << p2->y << ", " << p2->z << ")" << std::endl;
#endif
			}
			else if (glm::distance(*p1, intersections[i]) > EPSILON && glm::distance(*p2, intersections[i]) > EPSILON)
			{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "Found a 3rd point (" << intersections[i].x << ", " << intersections[i].y << ", " << intersections[i].z << ")?" << std::endl;
				OutputDebugString(ss.str().c_str());
#endif
				assert(false);
			}
		}
	}

#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	OutputDebugString(ss.str().c_str());
#endif
	if (p1 != NULL && p2 != NULL)
	{
		begin_point = *p1;
		end_point = *p2;
		return true;
	}
	return false;
}

bool Plane::getIntersectionInterval(const glm::vec3& direction, const glm::vec3& point, glm::vec3& intersection_p1, glm::vec3& intersection_p2) const
{
	static float EPSILON = 0.001f;
	// Now we need to find where this line intersects with the plane, due to floating point errors we cannot
	// do this in 3D space. So we project the plane and line on a 2D plane and once we found a collision point 
	// we project it back into 3D space.
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	std::stringstream ss;
	ss << "Plane::getIntersectionInterval:" << std::endl;
	ss << *this << std::endl;
	ss << "\tDirection (" << direction.x << ", " << direction.y << ", " << direction.z << ")" << std::endl;
	ss << "\tPoint (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
#endif
	// First we establish which axis to project the plane on, we pick the plane for which the normal vector has 
	// the largest component.
	glm::vec2 intersection;
	intersection_p1 = glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	intersection_p2 = glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

	if (std::abs(normal_.y) >= std::abs(normal_.x) && std::abs(normal_.y) >= std::abs(normal_.z))
	{
		glm::vec2 other_begin(point.x, point.z);
		glm::vec2 other_end(point.x + direction.x, point.z + direction.z);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "\t\tProject on the Y-plane: (" << other_begin.x << ", " << other_begin.y << ") - (" << other_end.x << ", " << other_end.y << ")" << std::endl;
#endif
		for (int i = 0; i < points_.size(); ++i)
		{
			glm::vec2 begin(points_[i].x, points_[i].z);
			glm::vec2 end(points_[(i + 1) % points_.size()].x, points_[(i + 1) % points_.size()].z);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "\t\tCheck edge: (" << begin.x << ", " << begin.y << ") - (" << end.x << ", " << end.y << ")" << std::endl;
			ss << "\t\tThe line to check against: (" << other_begin.x << ", " << other_begin.y << ") - (" << other_end.x << ", " << other_end.y << ")" << std::endl;
			OutputDebugString(ss.str().c_str());
			ss.str(std::string());
#endif
			if (Math::getIntersection(begin, end, other_begin, other_end, intersection))
			{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "\t\tFound an intersection at: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
#endif
				// Check if this intersection lies on the edge of this plane.
				if (((intersection.x >= begin.x && intersection.x <= end.x) || (intersection.x <= begin.x && intersection.x >= end.x))
				    &&
					((intersection.y >= begin.y && intersection.y <= end.y) || (intersection.y <= begin.y && intersection.y >= end.y)))
				{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tIntersection is connecting with the plane!" << std::endl;
#endif
					// Project the point back on the plane.
					glm::vec3 projected_intersection;
					if (!intersectsWith(glm::vec3(intersection.x, 0.0f, intersection.y), glm::vec3(intersection.x, 1.0f, intersection.y), projected_intersection, false))
					{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						std::cerr << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
						ss << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
						//OutputDebugString(ss.str().c_str());
#endif
						//assert(false);
					}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tProjected on the plane at(" << projected_intersection.x << ", " << projected_intersection.y << ", " << projected_intersection.z << ")" << std::endl;
#endif
					if (intersection_p1.x == std::numeric_limits<float>::max())
					{
						intersection_p1 = projected_intersection;
					}
					// Ignore the point if it is identical to the point we already found. This happens when the line goes through a vertex of the plane.
					else if (glm::distance(intersection_p1, projected_intersection) < EPSILON || glm::distance(intersection_p2, projected_intersection) < EPSILON)
					{
						continue;
					}
					else if (intersection_p2.x == std::numeric_limits<float>::max())
					{
						intersection_p2 = projected_intersection;
					}
					else
					{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						std::cerr << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex)." << std::endl;
						ss << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex)." << std::endl;
						OutputDebugString(ss.str().c_str());
#endif
						assert(false);
					}
				}
			}
		}
	}
	else if (std::abs(normal_.x) >= std::abs(normal_.y) && std::abs(normal_.x) >= std::abs(normal_.z))
	{
		glm::vec2 other_begin(point.y, point.z);
		glm::vec2 other_end(point.y + direction.y, point.z + direction.z);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "\t\tProject on the X-plane: (" << other_begin.x << ", " << other_begin.y << ") - (" << other_end.x << ", " << other_end.y << ")" << std::endl;
#endif
		for (int i = 0; i < points_.size(); ++i)
		{
			glm::vec2 begin(points_[i].y, points_[i].z);
			glm::vec2 end(points_[(i + 1) % points_.size()].y, points_[(i + 1) % points_.size()].z);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "\t\tCheck line: (" << begin.x << ", " << begin.y << ") - (" << end.x << ", " << end.y << ")" << std::endl;
#endif
			if (Math::getIntersection(begin, end, other_begin, other_end, intersection))
			{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "\t\tFound an intersection at: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
#endif
				// Check if this intersection lies on the edge of this plane.
				if (((intersection.x >= begin.x && intersection.x <= end.x) || (intersection.x <= begin.x && intersection.x >= end.x))
					&&
					((intersection.y >= begin.y && intersection.y <= end.y) || (intersection.y <= begin.y && intersection.y >= end.y)))
				{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tIntersection is connecting with the plane!" << std::endl;
#endif
					// Project the point back on the plane.
					glm::vec3 projected_intersection;
					//if (!intersectsWithRay(glm::vec3(0.0f, intersection.x, intersection.y), glm::vec3(1, 0, 0), projected_intersection))
					if (!intersectsWith(glm::vec3(0.0f, intersection.x, intersection.y), glm::vec3(1.0f, intersection.x, intersection.y), projected_intersection, false))
					{
						std::cerr << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						ss << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
						//OutputDebugString(ss.str().c_str());
#endif
						//assert(false);
					}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tProjected on the plane at(" << projected_intersection.x << ", " << projected_intersection.y << ", " << projected_intersection.z << ")" << std::endl;
#endif
					if (intersection_p1.x == std::numeric_limits<float>::max())
					{
						intersection_p1 = projected_intersection;
					}
					// Ignore the point if it is identical to the point we already found. This happens when the line goes through a vertex of the plane.
					else if (glm::distance(intersection_p1, projected_intersection) < EPSILON || glm::distance(intersection_p2, projected_intersection) < EPSILON)
					{
						continue;
					}
					else if (intersection_p2.x == std::numeric_limits<float>::max())
					{
						intersection_p2 = projected_intersection;
					}
					else
					{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						std::cerr << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex." << std::endl;
						ss << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex." << std::endl;
						OutputDebugString(ss.str().c_str());
#endif
						assert(false);
					}
				}
			}
		}
	}
	else if (std::abs(normal_.z) >= std::abs(normal_.x) && std::abs(normal_.z) >= std::abs(normal_.y))
	{
		glm::vec2 other_begin(point.x, point.y);
		glm::vec2 other_end(point.x + direction.x, point.y + direction.y);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "\t\tProject on the Z-plane: (" << other_begin.x << ", " << other_begin.y << ") - (" << other_end.x << ", " << other_end.y << ")" << std::endl;
#endif
		for (int i = 0; i < points_.size(); ++i)
		{
			glm::vec2 begin(points_[i].x, points_[i].y);
			glm::vec2 end(points_[(i + 1) % points_.size()].x, points_[(i + 1) % points_.size()].y);
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
			ss << "\t\tCheck line: (" << begin.x << ", " << begin.y << ") - (" << end.x << ", " << end.y << ")" << std::endl;
#endif
			if (Math::getIntersection(begin, end, other_begin, other_end, intersection))
			{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
				ss << "\t\tFound an intersection at: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
#endif
				// Check if this intersection lies on the edge of this plane.
				if (((intersection.x >= begin.x && intersection.x <= end.x) || (intersection.x <= begin.x && intersection.x >= end.x))
					&&
					((intersection.y >= begin.y && intersection.y <= end.y) || (intersection.y <= begin.y && intersection.y >= end.y)))
				{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tIntersection is connecting with the plane!" << std::endl;
#endif
					// Project the point back on the plane.
					glm::vec3 projected_intersection;
					//if (!intersectsWithRay(glm::vec3(intersection.x, intersection.y, 0.0f), glm::vec3(0, 0, 1), projected_intersection))
					if (!intersectsWith(glm::vec3(intersection.x, intersection.y, 0.0f), glm::vec3(intersection.x, intersection.y, 1.0f), projected_intersection, false))
					{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						std::cerr << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
						ss << "We found a intersection in 2D space, but when we project it back to 3D we do not find an intersection. This is impossible! (probably a floating point error)" << std::endl;
						//OutputDebugString(ss.str().c_str());
#endif
						//assert(false);
					}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
					ss << "\t\tProjected on the plane at(" << projected_intersection.x << ", " << projected_intersection.y << ", " << projected_intersection.z << ")" << std::endl;
#endif
					if (intersection_p1.x == std::numeric_limits<float>::max())
					{
						intersection_p1 = projected_intersection;
					}
					// Ignore the point if it is identical to the point we already found. This happens when the line goes through a vertex of the plane.
					else if (glm::distance(intersection_p1, projected_intersection) < EPSILON || glm::distance(intersection_p2, projected_intersection) < EPSILON)
					{
						continue;
					}
					else if (intersection_p2.x == std::numeric_limits<float>::max())
					{
						intersection_p2 = projected_intersection;
					}
					else
					{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
						std::cerr << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex." << std::endl;
						ss << "We found 3 intersection points with a plane, this is impossible (assuming the plane is convex." << std::endl;
						OutputDebugString(ss.str().c_str());
#endif
						assert(false);
					}
				}
			}
		}
	}
	else
	{
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
		ss << "Could not find a plane to project on." << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
		assert(false);
	}
#ifdef ENABLE_CORE_COLLISION_PLANE_H_DEBUG
	OutputDebugString(ss.str().c_str());
#endif
	return intersection_p1.x != std::numeric_limits<float>::max() && intersection_p2.x != std::numeric_limits<float>::max();
}

std::ostream& operator<<(std::ostream& os, const Plane& plane)
{
	os << plane.normal_.x << "," << plane.normal_.y << "," << plane.normal_.z << "," << plane.d_ << ":|";// << std::endl;
	for (std::vector<glm::vec3>::const_iterator ci = plane.points_.begin(); ci != plane.points_.end(); ++ci)
	{
		os << "*** (" << (*ci).x << "," << (*ci).y << "," << (*ci).z << ")|";// << std::endl;
	}
	return os;
}

float Plane::getDistance(const glm::vec3& point) const
{
	return glm::dot(glm::vec4(point, 1), glm::vec4(normal_, d_));
}

float Plane::getDistance(const glm::vec4& plane, const glm::vec3& point)
{
	return glm::dot(glm::vec4(point, 1), plane);
}

float Plane::getDistance(const glm::vec3& begin_point, const glm::vec3& end_point) const
{
	glm::vec3 intersection;
	if (intersectsWith(begin_point, end_point, intersection)) return 0;
#ifdef _WIN32
	float min_distance = 10000000.0f;
#else
	float min_distance = std::numeric_limits<float>::max();
#endif
	for (unsigned int i = 0; i < points_.size(); ++i)
	{
		const glm::vec3& p1 = points_[i];
		const glm::vec3& p2 = points_[(i + 1) % points_.size()];
		
		float d = Math::dist3D_Segment_to_Segment(begin_point, end_point, p1, p2);
		if (d < min_distance) min_distance = d;
	}
	return min_distance;
}

bool Plane::intersectsWithPolygon(const std::vector<glm::vec3>& points, const glm::vec3& begin_point, const glm::vec3& end_point, glm::vec3& intersection_point)
{
	assert (points.size() > 2);
	glm::vec3 normal = glm::normalize(glm::cross(points[1] - points[0], points[2] - points[0]));
	float d = -glm::dot(normal, points[0]);

	glm::vec3 direction(end_point - begin_point);
	float t = -(glm::dot(normal, begin_point) + d) / glm::dot(normal, direction);

	// If the line intersects outside the given segment then there is no collision.
	if (t < 0 || t > 1)
	{
		return false;
	}

	glm::vec3 point(begin_point + direction * t);
	intersection_point = point;

	// Calculate if the point is inside the bounded box, we do this by calculating cross products
	// from each edge of the bounded plane and the calculated point. Iff the sign is the same the
	// point is inside the bounded plane, otherwise it must be outside of it.
	bool signs[3];
	for (int i = 0; i < points.size(); ++i)
	{
		glm::vec3 bounded_line = points[(i + 1) % points.size()] - points[i];
		glm::vec3 cross = glm::cross(bounded_line, point - points[i]);
		float signed_distance = glm::dot(normal, point) + d;

		// Solve floating point rounding for problems.
		static float EPSILON = 0.001;
		if (signed_distance < EPSILON && signed_distance > -EPSILON)
		{
			signed_distance = 0;
		}

		for (unsigned int i = 0; i < 3; ++i)
		{
			if (cross[i] < EPSILON && cross[i] > -EPSILON)
			{
				cross[i] = 0;
			}
		}

		if (i== 0)
		{
			signs[0] = cross[0] < 0.0f ? false : true;
			signs[1] = cross[1] < 0.0f ? false : true;
			signs[2] = cross[2] < 0.0f ? false : true;
		}
		else if (cross[0] >= 0 != signs[0] ||
			     cross[1] >= 0 != signs[1] ||
			     cross[2] >= 0 != signs[2]) 
		{
			return false;			 
		}
	}

	return true;
}

float Plane::getDistanceFromPolygon(const std::vector<glm::vec3>& points, const glm::vec3& begin_point, const glm::vec3& end_point)
{
	glm::vec3 intersection;
	if (intersectsWithPolygon(points, begin_point, end_point, intersection)) return 0;
#ifdef _WIN32
	float min_distance = 10000000.0f;
#else
	float min_distance = std::numeric_limits<float>::max();
#endif
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		const glm::vec3& p1 = points[i];
		const glm::vec3& p2 = points[(i + 1) % points.size()];
		
		float d = Math::dist3D_Segment_to_Segment(begin_point, end_point, p1, p2);
		if (d < min_distance) min_distance = d;
	}
	return min_distance;
}

bool Plane::isInsidePlane(const std::vector<glm::vec3>& points, const glm::vec3& point)
{
	Plane plane(points);
	return plane.isInsidePlane(point);
}

glm::vec3 Plane::project(const glm::vec3& point) const
{
	glm::vec4 L(normal_, d_);
	glm::vec4 p(point, 1.0f);
	
	float t = -glm::dot(L, p) / glm::dot(normal_, -normal_);
	return point + -normal_ * t;
}

glm::vec3 Plane::project(const glm::vec4& plane, const glm::vec3& point)
{
	glm::vec4 p(point, 1.0f);
	glm::vec3 normal(plane.x, plane.y, plane.z);

	float t = -glm::dot(plane, p) / glm::dot(normal, -normal);
	return point + -normal * t;
}

glm::vec3 Plane::project(const Plane& plane, const glm::vec3& point)
{
	glm::vec4 L(plane.normal_, plane.d_);
	glm::vec4 p(point, 1.0f);

	float t = -glm::dot(L, p) / glm::dot(plane.normal_, -plane.normal_);
	return point + -plane.normal_ * t;
}

};
