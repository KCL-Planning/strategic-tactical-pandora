#ifndef CORE_COLLISION_PLANE_H
#define CORE_COLLISION_PLANE_H

#include <glm/glm.hpp>
#include <ostream>
#include <vector>

class Plane
{
public:

	/**
	 * Create a plane given three points that form the corners of the plane. The 4th corners is created
	 * using these points such that the plane forms a rectangle.
	 */
	Plane(const glm::vec3& bottom_left, const glm::vec3& bottom_right, const glm::vec3& top_left);

	/**
	 * Create a plane based on a sequence of points that lie on a similar plane.
	 */
	Plane(const std::vector<glm::vec3>& shape);

	/**
	 * Check if this plane intersects with a line segement, we only check the bounded plane given by the 
	 * corners in the constructor.
	 */
	bool intersectsWith(const glm::vec3& begin_point, const glm::vec3& end_point, glm::vec3& intersection_point) const;

	/**
	 * Get the intersection of a ray with this plane. The returned value is the value of t such that the equation
	 * point + t * direction = the intersecting point. If the ray does not intersect, then std::numeric_limits<float>::max()
	 * is returned.
	 */
	bool intersectsWithRay(const glm::vec3& point, const glm::vec3& direction, glm::vec3& intersection_point) const;

	const glm::vec3& getNormal() const { return normal_; }

	float getD() const { return d_; }

	float getDistance(const glm::vec3& point) const;
	
	/**
	 * Get the distance of the line segment to the plane.
	 */
	float getDistance(const glm::vec3& begin_point, const glm::vec3& end_point) const;

	const std::vector<glm::vec3>& getPoints() const { return points_; }

	/**
	 * Check if a line intersects with an infinate plane given by three points.
	 */
	static bool intersectsWithPolygon(const std::vector<glm::vec3>& points, const glm::vec3& begin_point, const glm::vec3& end_point, glm::vec3& intersection_point);

	static float getDistanceFromPolygon(const std::vector<glm::vec3>& points, const glm::vec3& begin_point, const glm::vec3& end_point);
private:
	// A set of lines which form the circumference of the plane. They all go counter clockwise.
	// The latter fact is important because we use it to test if a point is inside the plane
	// bounded by this circumference using the cross products.
	//glm::vec3 bounded_lines_[4];
	std::vector<glm::vec3> bounded_lines_;

	// The points that make up the corners of the plane.
	//glm::vec3 points_[4];
	std::vector<glm::vec3> points_;

	// Parameters which make up the plane.
	// normal_ is the normal vector pendicular to the plane.
	// point_ is an arbitrary point on the plane.
	// d_ is calculated as -(normal dot point_).
	glm::vec3 normal_;//, point_;
	float d_;

	friend std::ostream& operator<<(std::ostream& os, const Plane& plane);
};

std::ostream& operator<<(std::ostream& os, const Plane& plane);

#endif
