#ifdef _WIN32
#include <windows.h>
#include <fstream>
#endif
#include <sstream>

#include <algorithm>
#include <limits>

#include "ConvexNavigationArea.h"

#include "../../loaders/PortalLevelFormatLoader.h"

#include "../../math/Math.h"

ConvexNavigationArea::ConvexNavigationArea(const std::vector<glm::vec3>& points_cw)
	: points_cw_(points_cw)
{
	// Construct the indicies, the points are ordered counter clockwise.
	for (unsigned int i = 1; i < points_cw.size() - 1; ++i)
	{
		indicies_.push_back(0); indicies_.push_back(i); indicies_.push_back(i + 1);
	}
}

ConvexNavigationArea::ConvexNavigationArea(const std::vector<glm::vec3>& points_cw, const std::vector<GLuint>& indicies)
	: points_cw_(points_cw), indicies_(indicies)
{
	if (points_cw_.size() < 3)
	{
#ifdef _WIN32
		MessageBox(NULL, "OEPS!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
}


bool ConvexNavigationArea::merge(const ConvexNavigationArea& rhs)
{
	if (this == &rhs) return false;
/*
	if (indicies_.size() > 6 || rhs.indicies_.size() > 6)
	{
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "merge: " << indicies_.size() << "(" << points_cw_.size() << ") with "  << rhs.indicies_.size() << "(" << rhs.points_cw_.size() << ")" << std::endl;
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
		}
#endif
	}
*/
	// We can only merge if the convex area to merge with shares an edge where both endpoints
	// overlap.
	glm::vec3 begin_edge(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	glm::vec3 end_edge(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	unsigned int lhs_edge_index = std::numeric_limits<unsigned int>::max();
	unsigned int rhs_edge_index = std::numeric_limits<unsigned int>::max();

	unsigned int nr_vertices = rhs.points_cw_.size() + points_cw_.size();

	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		for (unsigned int j = 0; j < rhs.points_cw_.size(); ++j)
		{
			if (glm::distance(points_cw_[i], rhs.points_cw_[j]) < 0.01f)
			{
				// Check if the next vertices match up as well.
				const glm::vec3& lhs_n_point = points_cw_[(i + 1) % points_cw_.size()];
				const glm::vec3& rhs_n_point = rhs.points_cw_[(j == 0 ? rhs.points_cw_.size() - 1 : j - 1)];
				//const glm::vec3& rhs_n_point = rhs.points_cw_[(j + 1) % rhs.points_cw_.size()];
				if (glm::distance(lhs_n_point, rhs_n_point) < 0.01f)
				{
					begin_edge = points_cw_[i];
					lhs_edge_index = i;
					rhs_edge_index = j;
					end_edge = points_cw_[(i + 1)% points_cw_.size()];
					break;
				}
			}
		}
		if (lhs_edge_index != std::numeric_limits<unsigned int>::max())
		{
			break;
		}
	}

	if (lhs_edge_index == std::numeric_limits<unsigned int>::max())
	{
		return false;
	}

	// Check if the combined area is convex or not. We check this as follows. Given the areas lhs and rhs,
	// every line segment between point p \in lhs and p' \in rhs must intersect with the line that form the
	// boundery between these two areas.
	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		if (glm::distance(points_cw_[i], begin_edge) < 0.01f || glm::distance(points_cw_[i], end_edge) < 0.01f) continue;
		for (unsigned int j = 0; j < rhs.points_cw_.size(); ++j)
		{
			if (glm::distance(rhs.points_cw_[j], begin_edge) < 0.01f || glm::distance(rhs.points_cw_[j], end_edge) < 0.01f) continue;

			if (Math::dist3D_Segment_to_Segment(begin_edge, end_edge, points_cw_[i], rhs.points_cw_[j]) >= 0.01f)
			{
				//return false;
			}
		}
	}

	/*
	if (points_cw_.size() > 4 || rhs.points_cw_.size() > 4)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "t3h fuck!?";
	//	ss << "Old: " << *this << " -> "<< getSurfaceArea() << " is larger than the new area: " << new_area << " -> " << new_area.getSurfaceArea();
		MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	*/
	// Find the first point where it crosses into the rhs area.
	std::vector<glm::vec3> new_points;
	for (unsigned int i = 0; i < points_cw_.size() - 2; ++i)
	{
		new_points.push_back(points_cw_[(lhs_edge_index + 2 + i) % points_cw_.size()]);
	}

	for (unsigned int i = 0; i < rhs.points_cw_.size(); ++i)
	{
		new_points.push_back(rhs.points_cw_[(i + rhs_edge_index) % rhs.points_cw_.size()]);
	}
	
	points_cw_.clear();
	points_cw_.insert(points_cw_.end(), new_points.begin(), new_points.end());

	// Post process the structure, remove any edges that are made redundant. We can check this by
	// comparing the angles between a vertex and its adjacent vertices. If they form a straight line
	// then we can remove this edge.
	//std::stringstream ss;
	std::vector<unsigned int> vertices_to_remove;
	for (unsigned int i = 1; i < points_cw_.size() + 1; ++i)
	{
		//float angle = glm::dot(glm::normalize(points_cw_[i % points_cw_.size()] - points_cw_[i - 1]), glm::normalize(points_cw_[(i + 1) % points_cw_.size()] - points_cw_[i - 1]));
		//glm::vec3 p1 = points_cw_[i % points_cw_.size()] - points_cw_[i - 1];
		//glm::vec3 p2 = points_cw_[(i + 1) % points_cw_.size()] - points_cw_[i - 1];

		float distance = Math::dist3D_Segment_to_Point(points_cw_[i - 1], points_cw_[(i + 1) % points_cw_.size()], points_cw_[i % points_cw_.size()]);

		if (distance < 0.01f)
		{
//			ss << "Remove the point: " << "(" << points_cw_[i % points_cw_.size()].x << "," << points_cw_[i % points_cw_.size()].y << "," << points_cw_[i % points_cw_.size()].z << ");";
			vertices_to_remove.push_back(i % points_cw_.size());
		}
		/*
		else
		{
			ss << "The point: " << "(" << points_cw_[i % points_cw_.size()].x << "," << points_cw_[i % points_cw_.size()].y << "," << points_cw_[i % points_cw_.size()].z << "); is: " << distance << " removed from: ";
			ss << "(" << points_cw_[i - 1].x << "," << points_cw_[i - 1].y << "," << points_cw_[i - 1].z << ") -";
			ss << "(" << points_cw_[(i + 1) % points_cw_.size()].x << "," << points_cw_[(i + 1) % points_cw_.size()].y << "," << points_cw_[(i + 1) % points_cw_.size()].z << ")" << std::endl;
		}
		*/
	}

	unsigned int nr_vertices2 = points_cw_.size();
	if (points_cw_.size() - vertices_to_remove.size() < 3)
	{
#ifdef _WIN32
		//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	else
	{
		std::sort(vertices_to_remove.begin(), vertices_to_remove.end());
		for (std::vector<unsigned int>::const_reverse_iterator ci = vertices_to_remove.rbegin(); ci != vertices_to_remove.rend(); ++ci)
		{
			points_cw_.erase(points_cw_.begin() + *ci);
		}
	}

	// Update the indices.
	indicies_.clear();
	for (unsigned int i = 1; i < points_cw_.size() - 1; ++i)
	{
		indicies_.push_back(0); indicies_.push_back(i); indicies_.push_back(i + 1);
	}
/*
	if (points_cw_.size() > 4)
	{
		std::stringstream ss;
		ss << "Result of merge: " << indicies_.size() << "(" << points_cw_.size() << ") with "  << rhs.indicies_.size() << "(" << rhs.points_cw_.size() << ")" << std::endl;
		ss << nr_vertices2  << " - " << vertices_to_remove.size() << std::endl;
		for (unsigned int i = 0; i < points_cw_.size(); ++i)
		{
			ss << "(" << points_cw_[i].x << ", " << points_cw_[i].y << ", " << points_cw_[i].z << ")" << std::endl;
		}

		for (unsigned int i = 1; i < points_cw_.size() + 1; ++i)
		{

			//float angle = glm::dot(glm::normalize(points_cw_[i % points_cw_.size()] - points_cw_[i - 1]), glm::normalize(points_cw_[(i + 1) % points_cw_.size()] - points_cw_[i - 1]));
			//glm::vec3 p1 = points_cw_[i % points_cw_.size()] - points_cw_[i - 1];
			//glm::vec3 p2 = points_cw_[(i + 1) % points_cw_.size()] - points_cw_[i - 1];

			float distance = Math::dist3D_Segment_to_Point(points_cw_[i - 1], points_cw_[(i + 1) % points_cw_.size()], points_cw_[i % points_cw_.size()]);

			if (distance < 0.01f)
			{
				ss << "Remove the point: " << "(" << points_cw_[i % points_cw_.size()].x << "," << points_cw_[i % points_cw_.size()].y << "," << points_cw_[i % points_cw_.size()].z << ");";
			}
			else
			{
				ss << "The point: " << "(" << points_cw_[i % points_cw_.size()].x << "," << points_cw_[i % points_cw_.size()].y << "," << points_cw_[i % points_cw_.size()].z << "); is: " << distance << " removed from: ";
				ss << "(" << points_cw_[i - 1].x << "," << points_cw_[i - 1].y << "," << points_cw_[i - 1].z << ") -";
				ss << "(" << points_cw_[(i + 1) % points_cw_.size()].x << "," << points_cw_[(i + 1) % points_cw_.size()].y << "," << points_cw_[(i + 1) % points_cw_.size()].z << ")" << std::endl;
				*
				glm::vec3 p = points_cw_[i % points_cw_.size()] - points_cw_[i - 1];
				glm::vec3 line = points_cw_[(i + 1) % points_cw_.size()] - points_cw_[i - 1];
				float line_length = glm::length(line);
				float dis_to_point = glm::length(p);

				float l = glm::dot(p, line) / line_length;
				float cos_angle = glm::dot(p, line) / (line_length * dis_to_point);

				// 1st case: the point is 'behind' the line.
				if (cos_angle < 0)
				{
					//return dis_to_point;
				}
				// 2nd case: the point is 'beyond' the line.
				else if (line_length < l)
				{
					//return glm::length(point - p1_end);
				}
				// The projection is on the line.
				else
				{
					ss << "p=(" << p.x << "," << p.y << "," << p.z << ")" << std::endl;
					ss << "line=" << line.x << "," << line.y << "," << line.z << ")" << std::endl;
					ss << "line_length=" << line_length << "; dis_to_point=" << dis_to_point << ". l=" << l << " cos_angle=" << cos_angle << "sqrt(" << dis_to_point*dis_to_point - l*l << ") " << std::endl;
					ss << sqrt(dis_to_point * dis_to_point - l * l);
				}
				*
			}
		} 
#ifdef _WIN32
		//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
*/
	return true;
}

bool ConvexNavigationArea::isTotallyBlocked(const std::vector<PLF_Cube*>& processed_cubes, float max_distance_from_obstacles) const
{
	for (std::vector<PLF_Cube*>::const_iterator ci = processed_cubes.begin(); ci != processed_cubes.end(); ++ci)
	{
		const PLF_Cube* cube = *ci;

		bool is_inside = true;
		for (std::vector<glm::vec3>::const_iterator ci = points_cw_.begin(); ci != points_cw_.end(); ++ci)
		{
			const glm::vec3& point = *ci;
			
			// If the point is not inside the plane and is far enough away then this convex area cannot be totally blocked.
			if (!cube->isInside(point))
			{
				/*
				bool outside_max_distance_range = true;
				for (std::vector<Plane>::const_iterator ci = cube.planes_.begin(); ci != cube.planes_.end(); ++ci)
				{
					const Plane& plane = *ci;
					if (plane.getDistance(point) < max_distance_from_obstacles)
					{
						outside_max_distance_range = false;
						break;
					}
				}
				
				if (outside_max_distance_range)
				*/
				{
					is_inside = false;
					break;
				}
			}
		}
		
		if (is_inside)
		{
			return true;
		}
	}
	return false;
}

//#define CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
bool ConvexNavigationArea::needsSplitting(const std::vector<PLF_Cube*>& processed_cubes, float max_distance_from_obstacles) const
{
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
	std::ofstream debug;
	debug.open("needsSplitting.txt");
	debug << *this << std::endl;
#endif
	glm::vec3 normal = glm::normalize(glm::cross(points_cw_[1] - points_cw_[0], points_cw_[2] - points_cw_[0]));
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
	debug << "Debug: " << normal.x << "," << normal.y << "," <<normal.z << std::endl;
	debug << "Area plane: " << area_plane << std::endl;
#endif
	
	Plane area_plane(points_cw_);

	for (std::vector<PLF_Cube*>::const_iterator ci = processed_cubes.begin(); ci != processed_cubes.end(); ++ci)
	{
		const PLF_Cube* cube = *ci;
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
		//const PLF_Cube& cube = processed_cubes[0];
		debug << "Collision cube: " << *cube << std::endl;
#endif
		/*
		std::vector<const Plane*> planes_to_check;
		for (std::vector<Plane>::const_iterator ci = cube.planes_.begin(); ci != cube.planes_.end(); ++ci)
		{
			const Plane& plane = *ci;
			bool plane_is_above = false;
			bool plane_is_below = false;
			for (std::vector<glm::vec3>::const_iterator ci = plane.getPoints().begin(); ci != plane.getPoints().end(); ++ci)
			{
				
				// A plane that is below this area.
				//if (glm::dot(normal, glm::normalize(*ci - points_cw_[0])) < -0.1)
				if ((*ci).y < -1.5f)
				{
					ignore_cube = true;
					break;
				}
			}
			if (ignore_cube) break;
		}

		if (ignore_cube) continue;
		*/
		for (std::vector<Plane>::const_iterator ci = cube->getPlanes().begin(); ci != cube->getPlanes().end(); ++ci)
		{
			const Plane& plane = *ci;
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
			debug << "Check out plane: " << plane << std::endl;
#endif

			// Only consider planes that are not parallel to this convex area.
			if (fabs(glm::dot(normal, plane.getNormal())) > 0.1f)
			{
				continue;
			}
			
			// Make sure that the plane's height is not too high (is far above the surface and we can pass underneath) or is too small
			// and we can easily step on top of it. We assume that all walkable surfaces have a normal that is around (0, 1, 0).
			float min_distance = std::numeric_limits<float>::max();
			float max_distance = -std::numeric_limits<float>::max();
			for (std::vector<glm::vec3>::const_iterator ci = plane.getPoints().begin(); ci != plane.getPoints().end(); ++ci)
			{
				const glm::vec3& plane_point = *ci;
				for (std::vector<glm::vec3>::const_iterator ci = points_cw_.begin(); ci != points_cw_.end(); ++ci)
				{
					const glm::vec3& area_point = *ci;
					
					float distance = plane_point.y - area_point.y;
					if (distance < min_distance) min_distance = distance;
					if (distance > max_distance) max_distance = distance;
				}
			}
			
			// If the plane is too high we will never collide and ignore it.
			if (min_distance > 2.0f) continue;
			
			// If the plane is too low we can use it as a step, so no need to split!
			if (max_distance < 1.0f) continue;

			// Check if all points are inside the given area.
			//bool contains_a_close_point = false;
			//bool all_points_above = true;

			for (unsigned int i = 0; i < points_cw_.size(); ++i)
			{
				const glm::vec3& p1 = points_cw_[i];
				const glm::vec3& p2 = points_cw_[(i + 1) % points_cw_.size()];

				// Check if these points are too close to any of the planes.
				if (plane.getDistance(p1, p2) < max_distance_from_obstacles)
				{
					return true;
				}
				
				//if (cube.isInside)
				
				/*
				// Make the lines a little longer to account for the length of the player / monster.
				glm::vec3 long_p1 = p1 + glm::normalize(p1 - p2) * 0.01f + glm::vec3(0, 0.1f, 0);
				glm::vec3 long_p2 = p2 + glm::normalize(p2 - p1) * 0.01f + glm::vec3(0, 0.1f, 0);
				glm::vec3 intersection;

#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
				debug << "Does the line: (" << long_p1.x << "," << long_p1.y << "," << long_p1.z << ") - (" << long_p2.x << "," << long_p2.y << "," << long_p2.z << ") intersect with the plane?" << std::endl;
#endif

				if (plane.intersectsWith(long_p2, long_p1, intersection))
				{
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
					debug << "Yes! At: (" << intersection.x << "," << intersection.y << "," << intersection.z << ")" << std::endl;
#endif

					// Check if the plane has any points higher than the intersection point, otherwise it is not blocking.
					bool has_higher_point = false;
					for (std::vector<glm::vec3>::const_iterator ci = plane.getPoints().begin(); ci != plane.getPoints().end(); ++ci)
					{
						const glm::vec3& point = *ci;
						if (point.y - 0.01f > intersection.y)
						{
							has_higher_point = true;
							break;
						}
					}

					if (!has_higher_point)
					{
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
						debug << "But has no higher point :(" << std::endl;
#endif
						continue;
					}
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
					debug.close();
					exit(0);
#endif
					return true;
				}
				*/
			}

			/*
			// Check if the lines of the plane intersect with this area.
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
			debug << "Compare against the area plane!" << std::endl;
#endif
			for (unsigned int i = 0; i < plane.getPoints().size(); ++i)
			{
				
				const glm::vec3& p1 = plane.getPoints()[i];
				const glm::vec3& p2 = plane.getPoints()[(i + 1) % plane.getPoints().size()];

				// Make the lines a little longer to account for the length of the player / monster.
				glm::vec3 long_p1 = p1;// + glm::normalize(p2 - p1);
				glm::vec3 long_p2 = p2;// + glm::normalize(p1 - p);
				glm::vec3 intersection;
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
				debug << "Does the line: (" << long_p1.x << "," << long_p1.y << "," << long_p1.z << ") - (" << long_p2.x << "," << long_p2.y << "," << long_p2.z << ") intersect with the plane?" << std::endl;
#endif
				if (area_plane.intersectsWith(long_p2, long_p1, intersection))
				{
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
					debug << "Yes! At: (" << intersection.x << "," << intersection.y << "," << intersection.z << ")" << std::endl;
#endif
					// Check if the plane has any points higher than the intersection point, otherwise it is not blocking.
					bool has_higher_point = false;
					for (std::vector<glm::vec3>::const_iterator ci = plane.getPoints().begin(); ci != plane.getPoints().end(); ++ci)
					{
						const glm::vec3& point = *ci;
						if (point.y - 0.01f > intersection.y)
						{
							has_higher_point = true;
							break;
						}
					}

					if (!has_higher_point)
					{
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
						debug << "But has no higher point :(" << std::endl;
#endif
						continue;
					}
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
					debug.close();
					exit(0);
#endif
					return true;
				}
			}
			*/
		}
	}
#ifdef CONVEX_NAVIGATION_AREA__NEEDS_SPLITTING_DEBUG
	debug.close();
	exit(0);
#endif
	return false;
}

void ConvexNavigationArea::split(const std::vector<PLF_Cube*>& processed_cubes, std::vector<ConvexNavigationArea>& new_areas) const
{
	if (points_cw_.empty())
	{
		return;
	}

	glm::vec3 middle_point;
	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		middle_point += points_cw_[i];
	}
	middle_point /= (float)(points_cw_.size());

	// Split the area from the middle of edges; all new areas will be  square.

	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		const glm::vec3& p0 = i == 0 ? points_cw_[points_cw_.size() - 1] : points_cw_[i - 1];
		const glm::vec3& p1 = points_cw_[i];
		const glm::vec3& p2 = points_cw_[(i + 1) % points_cw_.size()];

		std::vector<glm::vec3> new_points;
		new_points.push_back(p1); new_points.push_back((p1 + p2) / 2.0f); new_points.push_back(middle_point); new_points.push_back((p0 + p1) / 2.0f);
		std::vector<GLuint> new_indicies;
		new_indicies.push_back(0); new_indicies.push_back(1); new_indicies.push_back(2); 
		new_indicies.push_back(0); new_indicies.push_back(2); new_indicies.push_back(3); 

		ConvexNavigationArea new_area(new_points, new_indicies);
		new_areas.push_back(new_area);
	}

	// Triangle code.
	/*
	float surface_area = 0;
	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		const glm::vec3& p1 = points_cw_[i];
		const glm::vec3& p2 = points_cw_[(i + 1) % points_cw_.size()];
		const glm::vec3& p3 = points_cw_[(i + 2) % points_cw_.size()];

		std::vector<glm::vec3> new_points;
		new_points.push_back(p1); new_points.push_back(p2); new_points.push_back(middle_point);
		std::vector<GLuint> new_indicies;
		new_indicies.push_back(0); new_indicies.push_back(1); new_indicies.push_back(2); 

		ConvexNavigationArea new_area(new_points, new_indicies);

		// Debug check :).
		if (new_area.getSurfaceArea() >= getSurfaceArea())
		{
#ifdef _WIN32
			std::stringstream ss;
			ss << "Old: " << *this << " -> "<< getSurfaceArea() << " is larger than the new area: " << new_area << " -> " << new_area.getSurfaceArea();
			MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
			exit(1);
		}
		new_areas.push_back(new_area);
	}
	*/
}

float ConvexNavigationArea::getSurfaceArea() const
{
	if (points_cw_.empty())
	{
		return 0;
	}

	if (points_cw_.size() == 3)
	{
		return getSurfaceAreaTriangle(points_cw_[0], points_cw_[1], points_cw_[2]);
	}

	glm::vec3 middle_point;
	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		middle_point += points_cw_[i];
	}
	middle_point /= (float)(points_cw_.size());

	float surface_area = 0;
	for (unsigned int i = 0; i < points_cw_.size(); ++i)
	{
		const glm::vec3& p1 = points_cw_[i];
		const glm::vec3& p2 = points_cw_[(i + 1) % points_cw_.size()];

		surface_area += getSurfaceAreaTriangle(p1, p2, middle_point);
	}
	return surface_area;
}

float ConvexNavigationArea::getSurfaceAreaTriangle(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3) const
{
	// Find the longest edge.
	float d1 = glm::distance(p2, p1);
	float d2 = glm::distance(p3, p2);
	float d3 = glm::distance(p1, p3);

	const glm::vec3* rp1, *rp2, *rp3;

	// p1 - p2 is the longest.
	if (d1 >= d2 && d1 >= d3)
	{
		rp1 = &p1;
		rp2 = &p2;
		rp3 = &p3;
	}
	// p2 - p3 is the longest.
	else if (d2 >= d1 && d2 >= 3)
	{
		rp1 = &p2;
		rp2 = &p3;
		rp3 = &p1;
	}
	// p3 - p1 is the longest.
	else
	{
		rp1 = &p3;
		rp2 = &p1;
		rp3 = &p2;
	}

	float d = glm::distance(*rp1, *rp2);
	float l = glm::distance(*rp1, *rp3);
	float c = glm::dot(*rp3 - *rp1, *rp2 - *rp1) / d;
	float h = sqrt(l*l - c*c);
	return h*d/2.0f;
	/*
		float b = glm::length(p2 - p1);
		float l = glm::length(middle_point - p1);
		float b2 = glm::dot(p2 - p1, middle_point - p1) / b;
		float a = sqrt(l * l - b2 * b2) * b2 / 2.0f;
		surface_area += sqrt(l * l - b2 * b2) * b2 / 2.0f;
	*/
}

glm::vec3 ConvexNavigationArea::getCentre() const
{
	glm::vec3 centre;
	for (std::vector<glm::vec3>::const_iterator ci = points_cw_.begin(); ci != points_cw_.end(); ++ci)
	{
		centre += *ci;
	}
	return centre /= points_cw_.size();
}

std::ostream& operator<<(std::ostream& os, const ConvexNavigationArea& area)
{
	os << "Area: ";
	for (std::vector<glm::vec3>::const_iterator ci = area.points_cw_.begin(); ci != area.points_cw_.end(); ++ci)
	{
		os << "(" << (*ci).x << "," << (*ci).y << "," << (*ci).z << ")";
		if (ci + 1 != area.points_cw_.end()) os << " - ";
	}
	return os;
}
