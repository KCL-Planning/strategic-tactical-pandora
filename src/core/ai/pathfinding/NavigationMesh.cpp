#ifdef _WIN32
#include <windows.h>
#include <sstream>
#endif

#include "NavigationMesh.h"
#include "ConvexNavigationArea.h"
#include "../../loaders/PortalLevelFormatLoader.h"

#include "../../math/Math.h"

//#define NAVIGATION_MESH_DEBUG

NavigationMesh::NavigationMesh(const std::vector<PLF_Cube*>& processed_cubes, const std::vector<glm::vec3>& vertices, const std::vector<GLuint>& indices, const glm::vec3& normal, float max_angle, float max_distance_from_obstacle)
{
	// Convert the triangles into convex areas.
	assert ((indices.size() % 3) == 0); // Make sure that we only consider triangles :).
	assert (glm::length(normal) == 1);

	std::cout << "[NavigationMesh::NavigationMesh] " << processed_cubes.size() << " " << vertices.size() << ", " << indices.size() << "." << std::endl;
/*
	for (std::vector<glm::vec3>::const_iterator ci = vertices.begin(); ci != vertices.end(); ++ci)
	{
		std::cout << "v: " << (*ci).x << ", " << (*ci).y << ", " <<  (*ci).z << std::endl;
	}
	for (std::vector<GLuint>::const_iterator ci = indices.begin(); ci != indices.end(); ++ci)
	{
		std::cout << "i: " << *ci << std::endl;
	}
*/
	for (unsigned int i = 0; i < indices.size() / 3; ++i)
	{
		// Check if this part is 'walkable' or not :).
		glm::vec3 n = glm::normalize(glm::cross(vertices[indices[i * 3 + 1]] - vertices[indices[i * 3]], vertices[indices[i * 3 + 2]] - vertices[indices[i * 3]]));
		if (glm::dot(n, glm::normalize(normal)) < max_angle) continue;
		
		std::vector<glm::vec3> points;
		points.push_back(vertices[indices[i * 3]]);
		points.push_back(vertices[indices[i * 3 + 1]]);
		points.push_back(vertices[indices[i * 3 + 2]]);

		std::vector<GLuint> area_indices;
		area_indices.push_back(0);
		area_indices.push_back(1);
		area_indices.push_back(2);

		ConvexNavigationArea cna(points, area_indices);
		areas_.push_back(cna);
		
		std::cout << i << ": " << cna << std::endl;
	}

	
	// Next we reduce the areas by 'merging' those areas that can be merged using the Hertel-Mehlhorn algorithm.
	merge();
	split(processed_cubes, vertices, indices, max_angle, 0.01f, max_distance_from_obstacle);
	merge();
	merge32();

	//std::cout << "Connect the adjacent areas." << std::endl;
	// We now have a set of convex areas, check which areas are adjacent.
	for (unsigned int i = 0; i < areas_.size() - 1; ++i)
	{
		ConvexNavigationArea& lhs = areas_[i];
#ifdef NAVIGATION_MESH_DEBUG
		std::cout << "Area1: " << lhs << std::endl;
#endif
		for (unsigned int j = i + 1; j < areas_.size(); ++j)
		{
			ConvexNavigationArea& rhs = areas_[j];
#ifdef NAVIGATION_MESH_DEBUG
			std::cout << "Area2: " << rhs << std::endl;
#endif
			bool are_adjacent = false;
			for (unsigned int k = 0; k < lhs.getPoints().size(); ++k)
			{
				// We ignore the height, in case of staircases we want these to be connectable.
				glm::vec3 lhs_p1(lhs.getPoints()[k].x, 0.0f, lhs.getPoints()[k].z);
				glm::vec3 lhs_p2(lhs.getPoints()[(k + 1) % lhs.getPoints().size()].x, 0.0f, lhs.getPoints()[(k + 1) % lhs.getPoints().size()].z);
				for (unsigned int l = 0; l < rhs.getPoints().size(); ++l)
				{
					// Do not connect areas with too much of a hight difference.
					if (std::abs(lhs.getPoints()[k].y - rhs.getPoints()[l].y) > 1.0f) continue;

					glm::vec3 rhs_p1(rhs.getPoints()[l].x, 0.0f, rhs.getPoints()[l].z);
					glm::vec3 rhs_p2(rhs.getPoints()[(l + 1) % rhs.getPoints().size()].x, 0.0f, rhs.getPoints()[(l + 1) % rhs.getPoints().size()].z);
					
#ifdef NAVIGATION_MESH_DEBUG
					std::cout << "Compare the lines: (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") <->";
					std::cout << " (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ")" << std::endl;
#endif
					// Make sure that the lines are going in the same direction.
					if (std::abs(glm::dot(glm::normalize(lhs_p1 - lhs_p2), glm::normalize(rhs_p1 - rhs_p2))) < 0.9f)
					{
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "The lines are not parallel!" << std::endl;
#endif
						continue;
					}
#ifdef NAVIGATION_MESH_DEBUG
					std::cout << "The lines are parallel! " << glm::dot(glm::normalize(lhs_p1 - lhs_p2), glm::normalize(rhs_p1 - rhs_p2)) << std::endl;
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
						std::cout << "The lines lie in eachother's extension and do not overlap parallel." << std::endl;
#endif
						continue;
					}

					//float d = Math::dist3D_Segment_to_Segment(lhs.getPoints()[k], lhs.getPoints()[(k + 1) % lhs.getPoints().size()], rhs.getPoints()[l], rhs.getPoints()[(l + 1) % rhs.getPoints().size()]);
					float d = Math::dist3D_Segment_to_Segment(lhs_p1, lhs_p2, rhs_p1, rhs_p2);
					if ((std::abs(rhs.getPoints()[l].y - lhs.getPoints()[k].y) < 0.01 && d < 0.05) || (std::abs(rhs.getPoints()[l].y - lhs.getPoints()[k].y) >= 0.01 && d < 0.5f))
					//if (d < 0.5f)
					{
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "Distance is within 0.5!" << std::endl;
#endif
						glm::vec3 p1;
						
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						std::cout << "Distance from (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") to (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
#endif
						// Find out which line segment connect both areas.
						//if (Math::dist3D_Segment_to_Point(rhs.getPoints()[l], rhs.getPoints()[(l + 1) % rhs.getPoints().size()], lhs.getPoints()[k]) < 0.01f)
						if (Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) < 0.5f)
						{
							p1 = lhs.getPoints()[k];
							//p1 = lhs_p1;
						}
						//else if (Math::dist3D_Segment_to_Point(lhs.getPoints()[k], lhs.getPoints()[(k + 1) % lhs.getPoints().size()], rhs.getPoints()[(l + 1) % rhs.getPoints().size()]) < 0.01f)
						else if (Math::dist3D_Segment_to_Point(lhs_p1, lhs_p2, rhs_p2) < 0.5f)
						{
							p1 = rhs.getPoints()[(l + 1) % rhs.getPoints().size()];
							//p1 = rhs_p2;
						}
						// These lines are not parallel!
						else
						{
							continue;
						}

						glm::vec3 p2;
						
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
						std::cout << "Distance from (" << lhs_p1.x << ", " << lhs_p1.y << ", " << lhs_p1.z << ") - (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") to (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
#endif
						
						//if (Math::dist3D_Segment_to_Point(rhs.getPoints()[l], rhs.getPoints()[(l + 1) % rhs.getPoints().size()], lhs.getPoints()[(k + 1) % lhs.getPoints().size()]) < 0.01f)
						if (Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p2) < 0.5f)
						{
							p2 = lhs.getPoints()[(k + 1) % lhs.getPoints().size()];
							//p2 = lhs_p2;
						}
						//else if (Math::dist3D_Segment_to_Point(lhs.getPoints()[k], lhs.getPoints()[(k + 1) % lhs.getPoints().size()], rhs.getPoints()[l]) < 0.01f)
						else if (Math::dist3D_Segment_to_Point(lhs_p1, lhs_p2, rhs_p1) < 0.5f)
						{
							p2 = rhs.getPoints()[l];
							//p2 = rhs_p1;
						}
						// These lines are not parallel!
						else
						{
							continue;
						}

						if (glm::distance(p1, p2) < 0.01f)
						{
							continue;
						}
						
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "Adjacent! (" << p1.x << ", " << p1.y << ", " << p1.z << ") --- (" << p2.x << ", " << p2.y << ", " << p2.z << ")" << std::endl;
#endif
						// Are adjacent! :D
						CNA_Adjacent* adjacent = new CNA_Adjacent(lhs, rhs, p1, p2);
						lhs.addAdjacentArea(*adjacent);
						rhs.addAdjacentArea(*adjacent);
						
#ifdef NAVIGATION_MESH_DEBUG
						std::cout << "Distance from (" << rhs_p1.x << ", " << rhs_p1.y << ", " << rhs_p1.z << ") - (" << rhs_p2.x << ", " << rhs_p2.y << ", " << rhs_p2.z << ") to (" << lhs_p2.x << ", " << lhs_p2.y << ", " << lhs_p2.z << ") is " << Math::dist3D_Segment_to_Point(rhs_p1, rhs_p2, lhs_p1) << std::endl;
#endif
						are_adjacent = true;
						break;
					}
#ifdef NAVIGATION_MESH_DEBUG
					else
					{
						std::cout << "Lines are too far apart: " << d << " > 0.5." << std::endl;
					}
#endif
				}
				if (are_adjacent) break;
			}
		}
	}
	std::cout << "Final number of areas: " << areas_.size() << std::endl;
}

void NavigationMesh::merge()
{
/*
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Merge: " << areas_.size() << " areas!" << std::endl;
	MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
	}
#endif
*/
	/*
	for (unsigned int i = 0; i < areas_.size(); ++i)
	{
		ConvexNavigationArea& active_area = areas_[i];
		if (active_area.getIndices().size() > 4)
		{
#ifdef _WIN32
			MessageBox(NULL, "WRONG!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
		}
	}
	*/

	// Next we reduce the areas by 'merging' those areas that can be merged using the Hertel-Mehlhorn algorithm.
	bool done_merging = false;
	while (!done_merging)
	{
		done_merging = true;

		for (unsigned int i = 0; i < areas_.size(); ++i)
		{
			ConvexNavigationArea& active_area = areas_[i];
			bool done_merging2 = false;
			while (!done_merging2 && done_merging)
			{
				done_merging2 = true;
				for (unsigned int j = areas_.size() - 1; j != 0; --j)
				{
					ConvexNavigationArea& next_area = areas_[j];
					if (active_area.merge(next_area))
					{
						areas_.erase(areas_.begin() + j);
//						done_merging2 = false;
						done_merging = false;
						break;
					}
				}
			}
		}
	}
	/*
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Merged: " << areas_.size() << " areas!" << std::endl;
	MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
	}
#endif
	*/
}

void NavigationMesh::merge32()
{
	if (areas_.size() < 3) return;
	bool done_merging = false;
	while (!done_merging)
	{
		std::vector<ConvexNavigationArea> new_areas;
		done_merging = true;
		// Find two areas that share exacty one vertex and has an edge running from it
		// that runs parallel.
		for (unsigned int area1_i = 0; area1_i < areas_.size() - 1; ++area1_i)
		{
			ConvexNavigationArea& area1 = areas_[area1_i];
			ConvexNavigationArea* shared_area = NULL;
			ConvexNavigationArea* area_to_intersect_with = NULL;
			ConvexNavigationArea* area_to_extend = NULL;
			ConvexNavigationArea* third_area = NULL;

			glm::vec3 shared_point(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
			glm::vec3 line_middle_point;
			glm::vec3 line_begin_point;
			glm::vec3 point_to_extend_to;
			for (unsigned int area2_i = area1_i + 1; area2_i < areas_.size(); ++area2_i)
			{
				ConvexNavigationArea& area2 = areas_[area2_i];
				unsigned int shared_points = 0;
				for (unsigned int a1pi = 0; a1pi < area1.getPoints().size(); ++a1pi)
				{
					const glm::vec3& a1p = area1.getPoints()[a1pi];
					for (unsigned int a2pi = 0; a2pi < area2.getPoints().size(); ++a2pi)
					{
						const glm::vec3& a2p = area2.getPoints()[a2pi];
						if (glm::distance(a1p, a2p) < 0.01f)
						{
							++shared_points;
						}
					}
				}

				if (shared_points != 1)
				{
					continue;
				}

				// If there is one point, then we check if this point lies on two edges of both areas that 
				// are parallel.
				for (unsigned int a1pi = 0; a1pi < area1.getPoints().size(); ++a1pi)
				{
					const glm::vec3& a1p = area1.getPoints()[a1pi];
					for (unsigned int a2pi = 0; a2pi < area2.getPoints().size(); ++a2pi)
					{
						const glm::vec3& a2p = area2.getPoints()[a2pi];

						if (glm::distance(a1p, a2p) < 0.01f)
						{
							// Check if the edges from a1p and a2p are parallel.
							const glm::vec3& a1p1 = area1.getPoints()[(a1pi + 1) % area1.getPoints().size()];
							const glm::vec3& a2p1 = area2.getPoints()[a2pi == 0 ? area2.getPoints().size() - 1 : a2pi - 1];
							if (glm::dot(a1p1 - a1p, a2p1 - a1p) > 0.999f)
							{
								if (glm::distance(a1p, a1p1) < glm::distance(a1p, a2p1))
								{
									line_middle_point = a1p1;
									line_begin_point = area1.getPoints()[(a1pi + 2) % area1.getPoints().size()];
									point_to_extend_to = area2.getPoints()[(a2pi + 1) % area2.getPoints().size()];
									area_to_intersect_with = &area2;
									area_to_extend = &area1;
								}
								else
								{
									line_middle_point = a2p1;
									if (a2pi == 0)
										line_begin_point = area2.getPoints()[area2.getPoints().size() - 2];
									else if (a2pi == 1)
										line_begin_point = area2.getPoints()[area2.getPoints().size() - 1];
									else
										line_begin_point = area2.getPoints()[a2pi - 2];

									if (a1pi == 0)
										point_to_extend_to = area1.getPoints()[area1.getPoints().size() - 1];
									else
										point_to_extend_to = area1.getPoints()[a1pi - 1];
									
									area_to_intersect_with = &area1;
									area_to_extend = &area2;
								}
								shared_point = a1p;
								shared_area = &area2;
								break;
							}

							const glm::vec3& a1p2 = area1.getPoints()[a1pi == 0 ? area1.getPoints().size() - 1 : a1pi - 1];
							const glm::vec3& a2p2 = area2.getPoints()[(a2pi + 1) % area2.getPoints().size()];
							if (glm::dot(a1p2 - a1p, a2p2 - a1p) > 0.999f)
							{
								if (glm::distance(a1p, a1p2) < glm::distance(a1p, a2p2))
								{
									line_middle_point = a1p2;
									if (a1pi == 0)
										line_begin_point = area1.getPoints()[area1.getPoints().size() - 2];
									else if (a1pi == 1)
										line_begin_point = area1.getPoints()[area1.getPoints().size() - 1];
									else
										line_begin_point = area1.getPoints()[a1pi - 2];
									
									if (a2pi == 0)
										point_to_extend_to = area2.getPoints()[area2.getPoints().size() - 1];
									else
										point_to_extend_to = area2.getPoints()[a2pi - 1];
									area_to_intersect_with = &area2;
									area_to_extend = &area1;
								}
								else
								{
									line_middle_point = a2p2;
									line_begin_point = area2.getPoints()[(a2pi + 2) % area2.getPoints().size()];
									point_to_extend_to = area1.getPoints()[(a1pi + 1) % area1.getPoints().size()];
									area_to_intersect_with = &area1;
									area_to_extend = &area2;
								}
								shared_point = a1p;
								shared_area = &area2;
								break;
							}
						}
					}

					if (shared_area != NULL) break;
				}

				if (shared_area == NULL)
				{
					continue;
				}
				
				// Find the intersecting point that will form the new vertex.
				glm::vec3 line_end_point;
				for (unsigned int points_i = 0; points_i != area_to_intersect_with->getPoints().size(); ++points_i)
				{
					const glm::vec3& p2_begin = area_to_intersect_with->getPoints()[points_i];
					const glm::vec3& p2_end = area_to_intersect_with->getPoints()[(points_i + 1) % area_to_intersect_with->getPoints().size()];

					// Ignore the line where line_middle_point is on.
					if (Math::dist3D_Segment_to_Point(p2_begin, p2_end, line_middle_point) < 0.01f)
					{
						continue;
					}

					// Bit of a hack, we use line segments instead of line -> line segment; will change if necessary.
					float line_distance = Math::dist3D_Segment_to_Segment(line_begin_point, line_middle_point + glm::normalize(line_middle_point - line_begin_point) * 1000.0f, p2_begin, p2_end);
					if (line_distance < 0.01f)
					{
						glm::vec3 area1_p;
						glm::vec3 area2_p;
						Math::getIntersection(line_begin_point, line_middle_point + line_middle_point - line_begin_point, p2_begin, p2_end, area1_p, area2_p);
						// Found the point where we split the area! :D
						if (area_to_intersect_with == &area1)
						{
							line_end_point = area1_p;
						}
						else
						{
							line_end_point = area2_p;
						}
						break;
					}
				}

				// Merge the two areas :).
							
				// Start with the area that needs to be split. This area has the property that it contains the only vertex
				// that is shared between two areas.
				unsigned int area_to_be_merged_id;
				for (unsigned int i = 0; i < area_to_intersect_with->getPoints().size(); ++i)
				{
					if (glm::distance(area_to_intersect_with->getPoints()[i], shared_point) < 0.01f)
					{
						area_to_be_merged_id = i;
						break;
					}
				}
				/*
				std::stringstream ss;
				ss << "The area: " << area1;
				if (&area1 == area_to_extend) ss << "EXTEND!";
				if (&area1 == area_to_intersect_with) ss << "Separate!";
				ss << std::endl;
				ss << "and" << std::endl;
				ss << "The area: " << area2;
				if (&area2 == area_to_extend) ss << "EXTEND!";
				if (&area2 == area_to_intersect_with) ss << "Separate!";
				ss << "Share exactly one vertex, which is: (" << shared_point.x << "," << shared_point.y << "," << shared_point.z << ")" << std::endl;
				ss << "The area that must be split is: " << *area_to_intersect_with << std::endl;
				ss << "The begining of the line is: (" << line_begin_point.x << "," << line_begin_point.y << "," << line_begin_point.z << ")" << std::endl;
				ss << "The middle of the line is: (" << line_middle_point.x << "," << line_middle_point.y << "," << line_middle_point.z << ")" << std::endl;
				ss << "The end of the line is: (" << line_end_point.x << "," << line_end_point.y << "," << line_end_point.z << ")" << std::endl;
				ss << "The point to extend to is: (" << point_to_extend_to.x << "," << point_to_extend_to.y << "," << point_to_extend_to.z << ")" << std::endl;
				ss << "The index of the point that starts at the end of the line is: " << area_to_be_merged_id << std::endl;

#ifdef _WIN32
				//MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif

				ss << "Construct the merged area: " << std::endl;
				*/
				std::vector<glm::vec3> new_area_to_merge_points;
				for (unsigned int i = 0; i < area_to_extend->getPoints().size(); ++i)
				{
					//ss << "(" << area_to_extend->getPoints()[(i + area_to_be_merged_id) % area_to_intersect_with->getPoints().size()].x << "," << area_to_extend->getPoints()[(i + area_to_be_merged_id) % area_to_intersect_with->getPoints().size()].y << "," << area_to_extend->getPoints()[(i + area_to_be_merged_id) % area_to_intersect_with->getPoints().size()].z << ")";
					if (glm::distance(area_to_extend->getPoints()[i], shared_point) < 0.01f)
					{
						new_area_to_merge_points.push_back(point_to_extend_to);
					}
					else if (glm::distance(area_to_extend->getPoints()[i], line_middle_point) < 0.01f)
					{
						new_area_to_merge_points.push_back(line_end_point);
					}
					else
					{
						new_area_to_merge_points.push_back(area_to_extend->getPoints()[i]);
					}
				}

				//ss << "Construct the separated area: " << std::endl;
				std::vector<glm::vec3> new_area_to_separate_points;
				for (unsigned int i = 0; i < area_to_intersect_with->getPoints().size(); ++i)
				{
					//ss << "(" << area_to_intersect_with->getPoints()[i].x << "," << area_to_intersect_with->getPoints()[i].y << "," << area_to_intersect_with->getPoints()[i].z << ") " << std::endl;
					//ss << "Distance to shared point: " << glm::distance(area_to_extend->getPoints()[i], shared_point) << "." << std::endl;
					//ss << "Distance to to_extend_to_point: "  << glm::distance(area_to_extend->getPoints()[i], point_to_extend_to) << std::endl;
					if (glm::distance(area_to_intersect_with->getPoints()[i], shared_point) < 0.01f)
					{
						new_area_to_separate_points.push_back(line_middle_point);
					}
					else if (glm::distance(area_to_intersect_with->getPoints()[i], point_to_extend_to) < 0.01f)
					{
						new_area_to_separate_points.push_back(line_end_point);
					}
					else
					{
						new_area_to_separate_points.push_back(area_to_intersect_with->getPoints()[i]);
					}
				}
				
				ConvexNavigationArea new_area_to_merge(new_area_to_merge_points);
				std::cout << "The new convex area that has been merged: " << new_area_to_merge << std::endl;

				ConvexNavigationArea new_area_to_separate(new_area_to_separate_points);
				std::cout << "The new convex area that will be seperated: " << new_area_to_separate << std::endl;
//#ifdef _WIN32
				//MessageBox(NULL, ss.str().c_str(), "INFO! :D", MB_ICONERROR | MB_OK);
//#endif

				// Check if we can find a 3rd area that can be merged with the generated area.
				bool found_merging_area = false;
				for (std::vector<ConvexNavigationArea>::iterator ci = areas_.begin(); ci != areas_.end(); ++ci)
				{
					ConvexNavigationArea& area3 = *ci;
					if (&area3 == &area1 || &area3 == &area2) continue;

					if (area3.merge(new_area_to_merge))
					{
						found_merging_area = true;
						areas_.push_back(new_area_to_separate);

						// Remove area 1 and area 2.
						areas_.erase(areas_.begin() + area2_i);
						areas_.erase(areas_.begin() + area1_i);
						new_areas.push_back(new_area_to_merge);
						new_areas.push_back(new_area_to_separate);
						--area1_i;
						done_merging = false;
						
						//std::cout << "After merge: " << area3 << std::endl;
						if (area3.getPoints().size() > 4)
						{
							std::cout << "ERROR! Area 3 has too many points after merge!" << std::endl;
							exit(0);
						}
						
						if (new_area_to_separate.getPoints().size() > 4)
						{
							std::cout << "ERROR! Area to seperate has too many points!" << std::endl;
							exit(0);
						}
//#ifdef _WIN32
						//MessageBox(NULL, ss.str().c_str(), "INFO! :D", MB_ICONERROR | MB_OK);
//#endif
						break;
					}
				}

				if (found_merging_area)
				{	
					break;
				}

				// Otherwise, reset all variables and continue looking! :)
				shared_area = NULL;
				shared_points = 0;
			}
		}
		merge();
	}
}

void NavigationMesh::split(const std::vector<PLF_Cube*>& processed_cubes, const std::vector<glm::vec3>& vertices, const std::vector<GLuint>& indices, float max_angle, float min_area, float max_distance_from_obstacle)
{
	std::vector<ConvexNavigationArea> splitted_areas;

	for (std::vector<ConvexNavigationArea>::const_iterator ci = areas_.begin(); ci != areas_.end(); ++ci)
	{
		//debug_file_ << "Process. " << *ci << std::endl;
		// Check if this area is clipped by any collision boxes. If so then we need to split them.
		std::vector<std::pair<ConvexNavigationArea, unsigned int> > open_queue;
		open_queue.push_back(std::make_pair(*ci, 0));
		while (open_queue.size() > 0)
		{
			std::pair<ConvexNavigationArea, unsigned int> item = *open_queue.begin();
			open_queue.erase(open_queue.begin());

			ConvexNavigationArea area = item.first;
			unsigned int nr = item.second;
			
			if (area.getSurfaceArea() < min_area || area.isTotallyBlocked(processed_cubes, max_distance_from_obstacle))
			{
				//debug_file_ << "Surface is too small or totally inside the blocking area." << std::endl;
				continue;
			}
			else if (area.needsSplitting(processed_cubes, max_distance_from_obstacle))
			{
				std::vector<ConvexNavigationArea> splitted_areas2;
				//debug_file_ << "Area needs to be splitted!" << std::endl;
				area.split(processed_cubes, splitted_areas2);

				for (std::vector<ConvexNavigationArea>::const_iterator ci = splitted_areas2.begin(); ci != splitted_areas2.end(); ++ci)
				{
					open_queue.push_back(std::make_pair(*ci, nr + 1));
				}
			}
			else
			{
				//debug_file_ << "Finalised area: " << area << std::endl;
				splitted_areas.push_back(area);
			}
			
		}
	}
	areas_.clear();
	areas_.insert(areas_.end(), splitted_areas.begin(), splitted_areas.end());
	//debug_file_ << "Total number of areas: " << areas_.size() << std::endl;
}
