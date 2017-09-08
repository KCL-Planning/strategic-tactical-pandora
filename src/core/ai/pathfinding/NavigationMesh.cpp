#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#include <sstream>
#endif

#include <algorithm>

#include "dpengine/ai/pathfinding/NavigationMesh.h"
#include "dpengine/ai/pathfinding/ConvexNavigationArea.h"
#include "dpengine/loaders/PortalLevelFormatLoader.h"
#include "dpengine/collision/ConvexPolygon.h"

#include "dpengine/math/Math.h"

//#define NAVIGATION_MESH_DEBUG

namespace DreadedPE
{

NavigationMesh::NavigationMesh(const std::vector<const ConvexPolygon*>& obstacles, const glm::vec3& normal, float max_angle, float max_distance_from_obstacle, float max_height)
{
	float EPSILON = 0.01f;
#ifdef NAVIGATION_MESH_DEBUG
	std::stringstream ss;
	ss << "[NavigationMesh::NavigationMesh] " << obstacles.size() << " obstacles!" << std::endl;
#endif
	// Create the Convex Navigation Areas.
	std::vector<std::pair<const Plane*, std::vector<const ConvexPolygon*> > > walkable_surfaces;
	for (const ConvexPolygon* convex_polygon : obstacles)
	{
		// Check if any of these are walkable surfaces (e.g. a normal vector that is close to (0, 1, 0)).
		for (const Plane* plane : convex_polygon->getPlanes())
		{
#ifdef NAVIGATION_MESH_DEBUG
			ss << "Process plane: " << *plane << std::endl;
#endif
			if (glm::dot(glm::normalize(plane->getNormal()), glm::normalize(normal)) > max_angle)
			{
#ifdef NAVIGATION_MESH_DEBUG
				ss << "Is walkable!" << std::endl;
#endif
				// Don't include the convex polygon this walkable surface is part of as a potential collision object (colliding with itself!).
				std::vector<const ConvexPolygon*> cp_copy = obstacles;
				cp_copy.erase(std::find(cp_copy.begin(), cp_copy.end(), convex_polygon));
				walkable_surfaces.push_back(std::make_pair(plane, cp_copy));
			}
		}
	}

#ifdef NAVIGATION_MESH_DEBUG
	ss << "Found " << walkable_surfaces.size() << " walkable surfaces!" << std::endl;
	OutputDebugString(ss.str().c_str());
	ss.str(std::string());
#endif

	// Construct the Convex Navigation Areas.
#ifdef NAVIGATION_MESH_DEBUG
	int nr_areas = 0;
#endif
	for (const std::pair<const Plane*, std::vector<const ConvexPolygon*> > plane_and_c : walkable_surfaces)
	{
		ConvexNavigationArea* cna = new ConvexNavigationArea(*plane_and_c.first);
		cna->split(areas_, plane_and_c.second, max_distance_from_obstacle);
#ifdef NAVIGATION_MESH_DEBUG
		ss << "========" << std::endl;
		ss << "Extra areas: " << areas_.size() - nr_areas << std::endl;
		ss << "The plane: " << *plane_and_c.first << std::endl;
		ss << "transforms to:" << std::endl;
		ss << *cna << std::endl;
		ss << "========" << std::endl;
		OutputDebugString(ss.str().c_str());
		ss.str(std::string());

		//if (areas_.size() - nr_areas == 0) exit(0);

		nr_areas = areas_.size();
#endif
	}
#ifdef NAVIGATION_MESH_DEBUG
	ss << areas_.size() << " split CNAs: " << std::endl;
	for (const ConvexNavigationArea* cna : areas_)
	{
		ss << " ======================================= " << std::endl;
		ss << *cna << std::endl;
		ss << " ======================================= " << std::endl;
	}
	OutputDebugString(ss.str().c_str());
	ss.str(std::string());
#endif

	if (areas_.empty())
	{
#ifdef NAVIGATION_MESH_DEBUG
		ss << "No areas were found." << std::endl;
		OutputDebugString(ss.str().c_str());
		ss.str(std::string());
#endif
		return;
	}

	// We now have a set of convex areas, check which areas are adjacent.
	for (unsigned int i = 0; i < areas_.size() - 1; ++i)
	{
		ConvexNavigationArea& lhs = *areas_[i];
#ifdef NAVIGATION_MESH_DEBUG
		ss << "Area1: " << lhs << std::endl;
		OutputDebugString(ss.str().c_str());
		ss.str(std::string());
#endif
		for (unsigned int j = i + 1; j < areas_.size(); ++j)
		{
			ConvexNavigationArea& rhs = *areas_[j];
#ifdef NAVIGATION_MESH_DEBUG
			ss << "Area2: " << rhs << std::endl;
			OutputDebugString(ss.str().c_str());
			ss.str(std::string());
#endif
			bool are_adjacent = false;
			for (unsigned int k = 0; k < lhs.getPlane().getPoints().size(); ++k)
			{
				// We ignore the height, in case of staircases we want these to be connectable.
				glm::vec3 lhs_p1(lhs.getPlane().getPoints()[k].x, 0.0f, lhs.getPlane().getPoints()[k].z);
				glm::vec3 lhs_p2(lhs.getPlane().getPoints()[(k + 1) % lhs.getPlane().getPoints().size()].x, 0.0f, lhs.getPlane().getPoints()[(k + 1) % lhs.getPlane().getPoints().size()].z);
				for (unsigned int l = 0; l < rhs.getPlane().getPoints().size(); ++l)
				{
					// Do not connect areas with too much of a hight difference.
					if (std::abs(lhs.getPlane().getPoints()[k].y - rhs.getPlane().getPoints()[l].y) > max_height ||
						std::abs(lhs.getPlane().getPoints()[(k + 1) % lhs.getPlane().getPoints().size()].y - rhs.getPlane().getPoints()[l].y) > max_height ||
						std::abs(lhs.getPlane().getPoints()[k].y - rhs.getPlane().getPoints()[(l + 1) % rhs.getPlane().getPoints().size()].y) > max_height ||
						std::abs(lhs.getPlane().getPoints()[(k + 1) % lhs.getPlane().getPoints().size()].y - rhs.getPlane().getPoints()[(l + 1) % rhs.getPlane().getPoints().size()].y) > max_height)
					{
						continue;
					}

					glm::vec3 rhs_p1(rhs.getPlane().getPoints()[l].x, 0.0f, rhs.getPlane().getPoints()[l].z);
					glm::vec3 rhs_p2(rhs.getPlane().getPoints()[(l + 1) % rhs.getPlane().getPoints().size()].x, 0.0f, rhs.getPlane().getPoints()[(l + 1) % rhs.getPlane().getPoints().size()].z);
					
#ifdef NAVIGATION_MESH_DEBUG
					ss << "Compare the lines: (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") <->";
					ss << " (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ")" << std::endl;
					OutputDebugString(ss.str().c_str());
					ss.str(std::string());
#endif
					// Make sure that the lines are going in the same direction.
					if (std::abs(glm::dot(glm::normalize(lhs_p1 - lhs_p2), glm::normalize(rhs_p1 - rhs_p2))) < 0.999f)
					{
#ifdef NAVIGATION_MESH_DEBUG
						ss << "The lines are not parallel!" << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						continue;
					}
#ifdef NAVIGATION_MESH_DEBUG
					ss << "The lines are parallel! " << glm::dot(glm::normalize(lhs_p1 - lhs_p2), glm::normalize(rhs_p1 - rhs_p2)) << std::endl;
					OutputDebugString(ss.str().c_str());
					ss.str(std::string());
#endif	
					// Make sure that the lines 'overlap' so they are not just in each other's extension.
					int nr_projected_points = 0;
					nr_projected_points += Math::projectsOnLine(lhs_p1, lhs_p2, rhs_p1);
					nr_projected_points += Math::projectsOnLine(lhs_p1, lhs_p2, rhs_p2);
					nr_projected_points += Math::projectsOnLine(rhs_p1, rhs_p2, lhs_p1);
					nr_projected_points += Math::projectsOnLine(rhs_p1, rhs_p2, lhs_p2);
					
					if (nr_projected_points < 2)
					{
#ifdef NAVIGATION_MESH_DEBUG
						ss << "The lines lie in eachother's extension and do not overlap parallel." << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						continue;
					}

					// TODO: Check for height differences.
					float d = Math::dist3D_Segment_to_Segment(lhs_p1 * glm::vec3(1, 0, 1), lhs_p2 * glm::vec3(1, 0, 1), rhs_p1 * glm::vec3(1, 0, 1), rhs_p2 * glm::vec3(1, 0, 1));
					if (d < EPSILON)
					{
#ifdef NAVIGATION_MESH_DEBUG
						ss << "Distance is within " << EPSILON << "!" << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						glm::vec3 p1;
						
#ifdef NAVIGATION_MESH_DEBUG
						ss << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						ss << "Distance from (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") to (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						// Find out which line segment connect both areas.
						//if (Math::dist3D_Segment_to_Point(rhs.getPoints()[l], rhs.getPoints()[(l + 1) % rhs.getPoints().size()], lhs.getPoints()[k]) < 0.01f)
						if (Math::dist3D_Segment_to_Point(rhs_p1 * glm::vec3(1, 0, 1), rhs_p2 * glm::vec3(1, 0, 1), lhs_p1 * glm::vec3(1, 0, 1)) < EPSILON)
						{
							p1 = lhs.getPlane().getPoints()[k];
						}
						//else if (Math::dist3D_Segment_to_Point(lhs.getPoints()[k], lhs.getPoints()[(k + 1) % lhs.getPoints().size()], rhs.getPoints()[(l + 1) % rhs.getPoints().size()]) < 0.01f)
						else if (Math::dist3D_Segment_to_Point(lhs_p1 * glm::vec3(1, 0, 1), lhs_p2 * glm::vec3(1, 0, 1), rhs_p2 * glm::vec3(1, 0, 1)) < EPSILON)
						{
							p1 = rhs.getPlane().getPoints()[(l + 1) % rhs.getPlane().getPoints().size()];
						}
						// These lines are not parallel!
						else
						{
							continue;
						}

						glm::vec3 p2;
						
#ifdef NAVIGATION_MESH_DEBUG
						ss << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						ss << "Distance from (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") to (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						
						//if (Math::dist3D_Segment_to_Point(rhs.getPoints()[l], rhs.getPoints()[(l + 1) % rhs.getPoints().size()], lhs.getPoints()[(k + 1) % lhs.getPoints().size()]) < 0.01f)
						if (Math::dist3D_Segment_to_Point(rhs_p1 * glm::vec3(1, 0, 1), rhs_p2 * glm::vec3(1, 0, 1), lhs_p2 * glm::vec3(1, 0, 1)) < EPSILON)
						{
							p2 = lhs.getPlane().getPoints()[(k + 1) % lhs.getPlane().getPoints().size()];
							//p2 = lhs_p2;
						}
						//else if (Math::dist3D_Segment_to_Point(lhs.getPoints()[k], lhs.getPoints()[(k + 1) % lhs.getPoints().size()], rhs.getPoints()[l]) < 0.01f)
						else if (Math::dist3D_Segment_to_Point(lhs_p1 * glm::vec3(1, 0, 1), lhs_p2 * glm::vec3(1, 0, 1), rhs_p1 * glm::vec3(1, 0, 1)) < EPSILON)
						{
							p2 = rhs.getPlane().getPoints()[l];
							//p2 = rhs_p1;
						}
						// These lines are not parallel!
						else
						{
							continue;
						}

						if (glm::distance(p1, p2) < EPSILON)
						{
							continue;
						}
												
#ifdef NAVIGATION_MESH_DEBUG
						ss << "Adjacent! (" << p1.x << ", " << p1.y << ", " << p1.z << ") --- (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						// Are adjacent! :D
						CNA_Adjacent* adjacent = new CNA_Adjacent(lhs, rhs, p1, p2);
						lhs.addAdjacentArea(*adjacent);
						rhs.addAdjacentArea(*adjacent);
						
#ifdef NAVIGATION_MESH_DEBUG
						ss << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
#endif
						are_adjacent = true;
						break;
					}
#ifdef NAVIGATION_MESH_DEBUG
					else
					{
						ss << "Lines are too far apart: " << d << " > 0.5." << std::endl;
						OutputDebugString(ss.str().c_str());
						ss.str(std::string());
					}
#endif
				}
				if (are_adjacent) break;
			}
		}
	}
#ifdef NAVIGATION_MESH_DEBUG
	ss << "Final number of areas: " << areas_.size() << std::endl;
	for (ConvexNavigationArea* cna : areas_)
	{
		ss << "==================================" << std::endl;
		for (const CNA_Adjacent* adjacent : cna->getAdjacentAreas())
		{
			ss << *adjacent->adjacent1_ << std::endl;
			ss << " NEXT TO " << std::endl;
			ss << *adjacent->adjacent2_ << std::endl;
			ss << "Connected by: (" << adjacent->p1_.x << ", " << adjacent->p1_.y << ", " << adjacent->p1_.z << ") - (" << adjacent->p2_.x << ", " << adjacent->p2_.y << ", " << adjacent->p2_.z << ")" << std::endl;
		}
	}
	OutputDebugString(ss.str().c_str());
#endif
}

};
