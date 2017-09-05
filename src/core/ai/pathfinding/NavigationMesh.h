#ifndef CORE_AI_PATHFINDING_NAVIGATION_MESH_H
#define CORE_AI_PATHFINDING_NAVIGATION_MESH_H

#include <vector>
#include <fstream>

#include <glm/glm.hpp>
//#include <GL/glfw.h>

class ConvexNavigationArea;
class PLF_Cube;
typedef unsigned int GLuint;

/**
 * Given a mesh, this module will create a navigation mesh that can be used for pathfinding purposes.
 */
class NavigationMesh
{
public:
	/**
	 * Create a navigation mesh, given a set of vertices and indices that form triangles. We limit the areas that we considder
	 * as 'walkable' by limiting the angle to the ground floot (i.e. no slopes!).
	 * @param vertices The list of all vertices.
	 * @param indices The indices that refer to the vertices, they come in tripplets and form triangles.
	 * @param normal The normal that is considered to be opposite to the 'gravity'.
	 * @param max_angle The cos(rad) of the surface relative to the normal that we consider to be walkable.
	 * @param max_distance_from_obstacle Make sure the resulting navigation mesh does not goes closer than @ref{max_distance_from_obstacle} from any obstacle.
	 */
	NavigationMesh(const std::vector<PLF_Cube*>& processed_cubes, const std::vector<glm::vec3>& vertices, const std::vector<GLuint>& indices, const glm::vec3& normal, float max_angle, float max_distance_from_obstacle);

	/**
	 * Find a path between two points.
	 */
	bool findPath(const glm::vec3& from, const glm::vec3& to, std::vector<ConvexNavigationArea*>& path) const;

	const std::vector<ConvexNavigationArea>& getAreas() const { return areas_; }

private:
	/**
	 * Merge those areas that can be merged.
	 */
	void merge();

	/**
	 * Perform 3-2 merging.
	 */
	void merge32();

	/**
	 * Split the areas such that they do not intersect with any geometry that has an angle between the two areas is
	 * less that max_angle. At the same time an area should not become smaller that min_area.
	 * @param processed_cubes All the obstacles we need to take into account.
	 * @param vertices The list of all vertices.
	 * @param indices The indices that refer to the vertices, they come in tripplets and form triangles.
	 * @param max_angle The cos(rad) of the surface relative to the normal that we consider to be walkable.
	 * @param min_area The minimum area we allow any part convex area to cover.
	 * @param max_distance_from_obstacle Make sure the resulting navigation mesh does not goes closer than @ref{max_distance_from_obstacle} from any obstacle.
	 */
	void split(const std::vector<PLF_Cube*>& processed_cubes, const std::vector<glm::vec3>& vertices, const std::vector<GLuint>& indices, float max_angle, float min_area, float max_distance_from_obstacle);

	std::vector<ConvexNavigationArea> areas_; // Contains all the areas that can be used for navigation purposes.
};

#endif
