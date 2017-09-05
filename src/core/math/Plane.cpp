#include "Plane.h"
#include "Math.h"

#include <sstream>

#ifdef _WIN32
#include <windows.h>
#endif

Plane::Plane(const glm::vec3& bottom_left, const glm::vec3& bottom_right, const glm::vec3& top_left)
{
	
	glm::vec3 top_right = bottom_right + (top_left - bottom_left);
	//bounded_lines_.reserve(4);
	bounded_lines_.push_back(bottom_right - bottom_left);
	bounded_lines_.push_back(top_right - bottom_right);
	bounded_lines_.push_back(top_left - top_right);
	bounded_lines_.push_back(bottom_left - top_left);
	//points_.reserve(4);
	points_.push_back(bottom_left);
	points_.push_back(bottom_right);
	points_.push_back(top_right);
	points_.push_back(top_left);
	normal_ = glm::normalize(glm::cross(bottom_right - bottom_left, top_left - bottom_left));
	glm::vec3 point = (bottom_left + bottom_right + top_left + top_right) / 4.0f;
	d_ = -glm::dot(normal_, point);
}

Plane::Plane(const std::vector<glm::vec3>& convex_shape)
{
	assert (convex_shape.size() > 2);
	points_ = convex_shape;
	bounded_lines_.reserve(convex_shape.size());
	for (unsigned int i = 0; i < convex_shape.size(); ++i)
	{
		bounded_lines_.push_back(convex_shape[(i + 1) % convex_shape.size()] - convex_shape[i]);
	}

	normal_ = glm::normalize(glm::cross(convex_shape[1] - convex_shape[0], convex_shape[2] - convex_shape[0]));
	d_ = -glm::dot(normal_, convex_shape[0]);
}

bool Plane::intersectsWith(const glm::vec3& begin_point, const glm::vec3& end_point, glm::vec3& intersection_point) const
{
	glm::vec3 direction(end_point - begin_point);
	float t = -(glm::dot(normal_, begin_point) + d_) / glm::dot(normal_, direction);

	// If the line intersects outside the given segment then there is no collision.
	if (t < 0 || t > 1)
	{
		return false;
	}

	glm::vec3 point(begin_point + direction * t);
	intersection_point = point;
/*
#ifdef _WIN32
	{
	std::stringstream ss;
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
	OutputDebugString(ss.str().c_str());
	}
#endif
*/
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
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "[" << i << "] Cross: (" << cross.x << ", " << cross.y << ", " << cross.z << ") Signed distance: " << signed_distance << std::endl;
		ss << "Signs: " << signs[0] << ", " << signs[1] << ", " << signs[2] << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
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
			return false;			 
		}
	}

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

float Plane::getDistance(const glm::vec3& begin_point, const glm::vec3& end_point) const
{
	glm::vec3 intersection;
	if (intersectsWith(begin_point, end_point, intersection)) return 0;
#ifdef _WIN32
	float min_distance = 10000000.0f;
#else
	float min_distance = std::numeric_limits<float>::max();
#endif
	//for (std::vector<glm::vec3>::const_iterator ci = bounded_lines_.begin(); ci != bounded_lines_.end(); ++ci)
	for (unsigned int i = 0; i < points_.size(); ++i)
	{
		const glm::vec3& p1 = points_[i];
		const glm::vec3& p2 = points_[(i + 1) % points_.size()];
		
		float d = Math::dist3D_Segment_to_Segment(begin_point, end_point, p1, p2);
		if (d < min_distance) min_distance = d;
	}
	
	/*
	float distance_begin_point = getDistance(begin_point);
	float distance_end_point = getDistance(end_point);
	if (distance_begin_point < min_distance) min_distance = distance_begin_point;
	if (distance_end_point < min_distance) min_distance = distance_end_point;
	*/
	
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
	//for (std::vector<glm::vec3>::const_iterator ci = bounded_lines_.begin(); ci != bounded_lines_.end(); ++ci)
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		const glm::vec3& p1 = points[i];
		const glm::vec3& p2 = points[(i + 1) % points.size()];
		
		float d = Math::dist3D_Segment_to_Segment(begin_point, end_point, p1, p2);
		if (d < min_distance) min_distance = d;
	}
	return min_distance;
}
