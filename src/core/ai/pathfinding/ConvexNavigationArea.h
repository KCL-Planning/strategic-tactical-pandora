#ifndef CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H
#define CORE_AI_PATHFINDING_CONVEX_NAVIGATION_AREA_H

#include <vector>
#include <glm/glm.hpp>
#include <iostream>
//#include <GL/glfw.h>

class ConvexNavigationArea;

class PLF_Cube;
typedef unsigned int GLuint;

struct CNA_Adjacent
{
	CNA_Adjacent(ConvexNavigationArea& adjacent1, ConvexNavigationArea& adjacent2, const glm::vec3& p1, const glm::vec3& p2)
		: adjacent1_(&adjacent1), adjacent2_(&adjacent2), p1_(p1), p2_(p2)
	{

	}

	ConvexNavigationArea* adjacent1_, *adjacent2_;

	// The line segment that lies along the adjacent border.
	glm::vec3 p1_;
	glm::vec3 p2_;
};

class ConvexNavigationArea
{
public:
	/**
	 * Construct a convex area, given a set of points that are ordered counter clockwise.
	 */
	ConvexNavigationArea(const std::vector<glm::vec3>& points_cw);

	ConvexNavigationArea(const std::vector<glm::vec3>& points_cw, const std::vector<GLuint>& indicies);

	bool merge(const ConvexNavigationArea& rhs);

	const std::vector<glm::vec3>& getPoints() const { return points_cw_; }

	void addAdjacentArea(const CNA_Adjacent& adjacent) { adjacent_areas_.push_back(&adjacent); }

	const std::vector<const CNA_Adjacent*>& getAdjacentAreas() const { return adjacent_areas_; }

	const std::vector<GLuint>& getIndices() const { return indicies_; }

	bool isTotallyBlocked(const std::vector<PLF_Cube*>& processed_cubes, float max_distance_from_obstacles) const;
	bool needsSplitting(const std::vector<PLF_Cube*>& processed_cubes, float max_distance_from_obstacles) const;
	void split(const std::vector<PLF_Cube*>& processed_cubes, std::vector<ConvexNavigationArea>& new_areas) const;
	float getSurfaceArea() const;

	glm::vec3 getCentre() const;

private:

	float getSurfaceAreaTriangle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) const;
	std::vector<glm::vec3> points_cw_; // Points that make up the area in counter clockwise order.
	std::vector<const CNA_Adjacent*> adjacent_areas_;

	// Debug, keep track of the vertex indicis.
	std::vector<GLuint> indicies_;

	friend std::ostream& operator<<(std::ostream& os, const ConvexNavigationArea& area);
};

std::ostream& operator<<(std::ostream& os, const ConvexNavigationArea& area);

#endif
