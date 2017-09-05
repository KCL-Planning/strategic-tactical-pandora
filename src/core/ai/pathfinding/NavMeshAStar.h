#ifndef CORE_AI_PATHFINDING_NAV_MESH_A_STAR_H
#define CORE_AI_PATHFINDING_NAV_MESH_A_STAR_H

#include <vector>
#include <glm/glm.hpp>

class ConvexNavigationArea;
class NavMeshNode;

/**
 * Algorithm to find a path in the navigation mesh given two points.
 */
class NavMeshAStar
{
public:
	/**
	 *
	 */
	NavMeshAStar(const std::vector<ConvexNavigationArea>& areas);

	/**
	 * @begin_point The begin point.
	 * @end_point The end point.
	 * @waypoints The waypoints 
	 */
	bool findPath(const glm::vec3& begin_point, const glm::vec3& end_point, std::vector<glm::vec3>& waypoints);


	const ConvexNavigationArea* getArea(const glm::vec3& point) const;
private:
	void postOptimise(const NavMeshNode& node, const glm::vec3& begin_point, const glm::vec3& end_point, std::vector<glm::vec3>& waypoints);
	const std::vector<ConvexNavigationArea>* areas_;
};

#endif
