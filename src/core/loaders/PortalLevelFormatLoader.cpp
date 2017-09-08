#define _CRT_SECURE_NO_WARNINGS // Get rid of stupid warnings regarding the use of printf functions under Visual Studio.

#include <memory>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include "dpengine/loaders/PortalLevelFormatLoader.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <limits>

#include "dpengine/light/SpotLight.h"
#include "dpengine/shapes/Shape.h"
#include "dpengine/shapes/Cube.h"
#include "dpengine/shapes/Piramid.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneLeafLight.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/scene/portal/Portal.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/math/Math.h"

#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/scene/Material.h"

#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shaders/ToonShader.h"

#include "dpengine/ai/pathfinding/ConvexNavigationArea.h"
#include "dpengine/ai/pathfinding/NavigationMesh.h"

#define PLF_LOADER_ENABLE_OUTPUT

namespace DreadedPE
{

std::vector<glm::vec3> PortalDef::findConvexShape(std::vector<glm::vec3> open_list, const std::vector<glm::vec3>& w, const std::vector<glm::vec3>& points)
{
	if (open_list.empty())
	{
		return w;
	}
	else if (w.empty())
	{
		std::vector<glm::vec3> new_open_list(open_list.begin() + 1, open_list.end());
		std::vector<glm::vec3> new_w;
		new_w.push_back(open_list[0]);
		return findConvexShape(new_open_list, new_w, points);
	}

	//while (open_list.size() > 0)
	for (unsigned int a = 0; a < open_list.size(); ++a)
	{
		std::vector<glm::vec3> s(w);
		s.push_back(open_list[a]);
		//open_list.erase(open_list.begin());
		std::vector<glm::vec3> new_open_list(open_list);
		new_open_list.erase(new_open_list.begin() + a);

		glm::vec3 e_line1 = s[s.size() - 1];

		// Check if it is consistent.
		bool is_consistent = true;
		if (s.size() > 3)
		{
			for (unsigned int h = 0; h < s.size() - 2; ++h)
			{
				glm::vec3 b_line1 = s[h];

				for (unsigned int i = 0; i < s.size() - 2; ++i)
				{
					if (h == i || h == i + 1) continue;
					glm::vec3 b_line2 = s[i];
					glm::vec3 e_line2 = s[i + 1];

					if (Math::dist3D_Segment_to_Segment(b_line1, e_line1, b_line2, e_line2) < 0.01f)
					{
/*
#ifdef _WIN32
						std::stringstream ss;
						for (std::vector<glm::vec3>::const_iterator ci = points.begin(); ci != points.end(); ++ci)
						{
							ss << "- (" << (*ci).x << " " << (*ci).y << " " << (*ci).z << ")" << std::endl;
						}
						ss << "Collision!: (" << b_line1.x << " " << b_line1.y << " " << b_line1.z << ") - (" << e_line1.x << " " << e_line1.y << " " << e_line1.z << ")" << std::endl;
						ss << "and: (" << b_line2.x << " " << b_line2.y << " " << b_line2.z << ") - (" << e_line2.x << " " << e_line2.y << " " << e_line2.z << ")" << std::endl;
						MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
*/
						is_consistent = false;
						break;
					}
				}
				if (!is_consistent) break;
			}
		}

		// TODO: Check if any points fall inside the polygon.
		if (!is_consistent) continue;

		std::vector<glm::vec3> result = findConvexShape(new_open_list, s, points);
		if (result.size() > 0) return result;
	}

	return std::vector<glm::vec3>();
}

SceneNode* PortalLevelFormatLoader::importLevel(const std::string& filename, std::shared_ptr<Material> material, ShaderInterface* shader, SceneManager& scene_manager, SceneNode& terrain_node, bool create_regions, bool load_lights, std::shared_ptr<Material> portal_material)
{
#ifdef PLF_LOADER_ENABLE_OUTPUT
	std::stringstream ss;
	ss << "[PortalLevelFormatLoader::importLevel] " << filename << std::endl;
#endif
	root_node_ = NULL;
	std::ifstream mesh_file;
	mesh_file.open(filename.c_str(), std::ios::in);

	if (!mesh_file.is_open())
	{
#ifdef PLF_LOADER_ENABLE_OUTPUT
		ss << "Could not open: " << filename << std::endl;
#ifdef _WIN32
		OutputDebugString(ss.str().c_str());
#else
		std::cout << ss.str() << std::endl;
#endif
#endif
		return NULL;
	}

	std::string line;

	/**
	 * File format.
	 *
	 * <file>      ::= <texture> <vertex>* <section>*
	 * <texture>   ::= tex <string>
	 * <vertex>    ::= v <float> <float> <float>
	 */
	//std::vector<glm::vec3> vertices;
	vertices_.clear();
	while (std::getline(mesh_file, line))
	{
		std::vector<std::string> tokens;
		std::istringstream iss(line);
		std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));

		if (tokens.empty())
		{
			continue;
		}

		// Texture.
		if ("tex" == tokens[0] && tokens.size() == 2)
		{
			// Load the texture.
			if (material != NULL)
			{
				Texture* texture = TargaTexture::loadTexture(tokens[1]);
				material->add2DTexture(*texture);
			}
		}

		// Vertices.
		else if ("v" == tokens[0] && tokens.size() == 4)
		{
			vertices_.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
		}

		// Collisions.
		else if ("c_id" == tokens[0] && tokens.size() == 2)
		{
			std::string collision_name = tokens[1];
			tokens.clear();
			{
				std::getline(mesh_file, line);
				std::istringstream iss(line);
				std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));
			}

			if ("cs" == tokens[0] && tokens.size() == 2)
			{
				unsigned int nr_planes = ::atoi(tokens[1].c_str());
				std::vector<const Plane*>* planes = new std::vector<const Plane*>();
				for (unsigned int i = 0; i < nr_planes; ++i)
				{
					std::vector<std::string> plane_tokens;
					{
						std::getline(mesh_file, line);
						std::istringstream iss(line);
						std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(plane_tokens));
					}
					if ("cp" != plane_tokens[0] || plane_tokens.size() != 2)
					{
						std::stringstream ss;
						ss << "Unknown line: " << line;
#ifdef _WIN32
						MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
					}

					// Next line is the normal.
					std::vector<std::string> normal_tokens;
					{
						std::getline(mesh_file, line);
						std::istringstream iss(line);
						std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(normal_tokens));
					}
					if ("cn" != normal_tokens[0] || normal_tokens.size() != 4)
					{
						std::stringstream ss;
						ss << "Unknown line: " << line;
#ifdef _WIN32
						MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
					}
					glm::vec3 normal(::atof(normal_tokens[1].c_str()), ::atof(normal_tokens[2].c_str()), ::atof(normal_tokens[3].c_str()));

					unsigned int nr_points = ::atoi(plane_tokens[1].c_str());
					std::vector<glm::vec3> points;
					for (unsigned int i = 0; i < nr_points; ++i)
					{
						std::vector<std::string> point_tokens;
						{
							std::getline(mesh_file, line);
							std::istringstream iss(line);
							std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(point_tokens));
						}

						if ("cv" != point_tokens[0] || point_tokens.size() != 4)
						{
							std::stringstream ss;
							ss << "Unknown line: " << line;
#ifdef _WIN32
							MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
						}
						assert("cv" == point_tokens[0] && point_tokens.size() == 4);
						glm::vec3 point(::atof(point_tokens[1].c_str()), ::atof(point_tokens[2].c_str()), ::atof(point_tokens[3].c_str()));
						points.push_back(point);
						//ss2 << "p " << point.x << " " << point.y << " " << point.z << std::endl;
					}
					Plane* plane = new Plane(points, normal);
					planes->push_back(plane);
				}
#ifdef PLF_LOADER_ENABLE_OUTPUT
				ss << "Add a collision polygon of: " << planes->size() << " planes called " << collision_name << "." << std::endl;
#endif
				collision_mapping_[collision_name] = planes;
			}
		}
		// Lights.
		else if ("l" == tokens[0] && tokens.size() == 2 && load_lights)
		{
			unsigned int nr_lights = ::atoi(tokens[1].c_str());
			glm::mat4 light_matrix;
			glm::vec3 light_colour;
			float angle;
			std::string name;
			for (unsigned int i = 0; i < nr_lights; ++i)
			{
				while (true)
				{
					tokens.clear();
					{
						std::getline(mesh_file, line);
						std::istringstream iss(line);
						std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));
					}

					if ("lm0" == tokens[0] && tokens.size() == 5)
					{
						light_matrix[0][0] = ::atof(tokens[1].c_str());
						light_matrix[1][0] = ::atof(tokens[2].c_str());
						light_matrix[2][0] = ::atof(tokens[3].c_str());
						light_matrix[3][0] = ::atof(tokens[4].c_str());
					}
					if ("lm1" == tokens[0] && tokens.size() == 5)
					{
						light_matrix[0][1] = ::atof(tokens[1].c_str());
						light_matrix[1][1] = ::atof(tokens[2].c_str());
						light_matrix[2][1] = ::atof(tokens[3].c_str());
						light_matrix[3][1] = ::atof(tokens[4].c_str());
					}
					if ("lm2" == tokens[0] && tokens.size() == 5)
					{
						light_matrix[0][2] = ::atof(tokens[1].c_str());
						light_matrix[1][2] = ::atof(tokens[2].c_str());
						light_matrix[2][2] = ::atof(tokens[3].c_str());
						light_matrix[3][2] = ::atof(tokens[4].c_str());
					}
					if ("lm3" == tokens[0] && tokens.size() == 5)
					{
						light_matrix[0][3] = ::atof(tokens[1].c_str());
						light_matrix[1][3] = ::atof(tokens[2].c_str());
						light_matrix[2][3] = ::atof(tokens[3].c_str());
						light_matrix[3][3] = ::atof(tokens[4].c_str());
					}
					if ("lc" == tokens[0] && tokens.size() == 4)
					{
						light_colour.r = ::atof(tokens[1].c_str());
						light_colour.g = ::atof(tokens[2].c_str());
						light_colour.b = ::atof(tokens[3].c_str());
					}
					if ("la" == tokens[0] && tokens.size() == 2)
					{
						angle = ::atof(tokens[1].c_str());
						break;
					}
					if ("ln" == tokens[0] && tokens.size() == 2)
					{
						name = tokens[1];
					}
				}
				//std::stringstream ss;
				//ss << "Before rotating: ( " << light_matrix[3][0] << ", " << light_matrix[3][1] << ", " << light_matrix[3][2] << ") -> ";
				
				// Need to flip the yaw.
				// Given as Pitch, Yaw, Roll.
				//light_matrix = glm::rotate(light_matrix, 180.0f, glm::vec3(0, 1, 0));
				light_matrix = glm::rotate(light_matrix, glm::radians(90.0f), glm::vec3(-1, 0, 0));
				
				/*
				glm::fquat fq = glm::toQuat(light_matrix);
				glm::vec3 euler = glm::eulerAngles(fq);
				
				// Yaw, Pitch, Roll
				glm::mat4 updated_light_matrix = glm::yawPitchRoll(euler.y, -euler.x, euler.z);
				updated_light_matrix[3][0] = light_matrix[3][0];
				updated_light_matrix[3][1] = light_matrix[3][1];
				updated_light_matrix[3][2] = light_matrix[3][2];
				
				light_matrix = updated_light_matrix;
				*/
				// Get the yaw and invert it.
				float yaw = glm::yaw(glm::toQuat(light_matrix));
				float pitch = glm::pitch(glm::toQuat(light_matrix));
				float roll = glm::roll(glm::toQuat(light_matrix));
				
				//ss << "Float: " << yaw << ", " << pitch << ", " << roll << std::endl;
				//ss << " ( " << light_matrix[3][0] << ", " << light_matrix[3][1] << ", " << light_matrix[3][2] << ")" << std::endl;

				SpotLight* light = new SpotLight(scene_manager, angle * 180.0f / M_PI, glm::vec3(0, 0, 0), light_colour, glm::vec3(0, 0, 0), 0.5f, 0.1f, 0.001f, 0.1f, 60.0f);
				SceneNode* light_node = new SceneNode(scene_manager, NULL, light_matrix);
				
				new SceneLeafLight(*light_node, NULL, *light);

				/**
				* Store to be used later.
				*/
				light_mapping_[name] = light_node;
			}
		}
	}
		
	mesh_file.close();
	mesh_file.open(filename.c_str(), std::ios::in);
#ifdef PLF_LOADER_ENABLE_OUTPUT	
	ss << "Loaded: " << vertices_.size() << " vertices." << std::endl;
#endif

	// Once we have all the vertices, we can process the rest of the file.
	normals_ = std::vector<glm::vec3>(vertices_.size(), glm::vec3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
	texture_coordinates_ = std::vector<glm::vec2>(vertices_.size(), glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()));
	
	/**
	 * File format for the sections.
	 *
	 * <section>   ::= s<uint> <face>*
	 * <face>      ::= <indexes> <normals> <uv_coords>[3]
	 * <indexes>   ::= i <uint> <uint> <uint>
	 * <normals>   ::= n <float> <float> <float>
	 * <uv_coords> ::= u <float> <float>
	 */
	//std::vector<Section*> sections;
	sections_.clear();
	Section* current_section = NULL;
	GLuint last_indexes[3];
	last_indexes[0] = std::numeric_limits<GLuint>::max();
	last_indexes[1] = std::numeric_limits<GLuint>::max();
	last_indexes[2] = std::numeric_limits<GLuint>::max();
	unsigned int vertex_index = 0;
	PLF_Triangle current_triangle;

	//std::stringstream ssd;

	while (std::getline(mesh_file, line))
	{
		std::vector<std::string> tokens;
		std::istringstream iss(line);
		std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));

		if (tokens.empty())
		{
			continue;
		}
		//ssd << line << std::endl;
		// Sections.
		if ("s" == tokens[0] && tokens.size() == 2)
		{
			unsigned int section_id = ::atoi(tokens[1].c_str());
			current_section = new Section(section_id);
			sections_.push_back(current_section);
#ifdef PLF_LOADER_ENABLE_OUTPUT
			ss << "Process section #" << section_id << std::endl;
#endif
		}
		else if ("i" == tokens[0] && tokens.size() == 4)
		{
			assert (current_section != NULL);
			last_indexes[0] = ::atoi(tokens[1].c_str());
			last_indexes[1] = ::atoi(tokens[2].c_str());
			last_indexes[2] = ::atoi(tokens[3].c_str());

			current_triangle.vertex_ = glm::ivec3(last_indexes[0], last_indexes[1], last_indexes[2]);
		}
		else if ("n" == tokens[0] && tokens.size() == 4)
		{
			assert (current_section != NULL);
			assert (last_indexes[0] != std::numeric_limits<GLuint>::max());
			assert (last_indexes[1] != std::numeric_limits<GLuint>::max());
			assert (last_indexes[2] != std::numeric_limits<GLuint>::max());

			glm::vec3 normal(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str()));
			current_triangle.normal_ = normal;
		}
		else if ("u" == tokens[0] && tokens.size() == 3)
		{
			assert (current_section != NULL);
			assert (last_indexes[0] != std::numeric_limits<GLuint>::max());
			assert (last_indexes[1] != std::numeric_limits<GLuint>::max());
			assert (last_indexes[2] != std::numeric_limits<GLuint>::max());

			glm::vec2 uv(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()));

			current_triangle.uv_[vertex_index] = uv;

			// Update the normals.
			++vertex_index;
		}
		else if ("c_nr" == tokens[0] && tokens.size() == 2)
		{
			unsigned int nr_collisions = ::atoi(tokens[1].c_str());
			for (unsigned int i = 0; i < nr_collisions; ++i)
			{
				std::vector<std::string> plane_tokens;
				{
					std::getline(mesh_file, line);
					std::istringstream iss(line);
					std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(plane_tokens));
				}
				if ("c_ref" != plane_tokens[0] || plane_tokens.size() != 2)
				{
					std::stringstream ss;
					ss << "Unknown line: " << line;
#ifdef _WIN32
					MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
				}

				current_section->addCollisionPlane(*collision_mapping_[plane_tokens[1]]);
			}
		}
		else if ("l_nr" == tokens[0] && tokens.size() == 2 && load_lights)
		{
			unsigned int nr_lights = ::atoi(tokens[1].c_str());
			for (unsigned int i = 0; i < nr_lights; ++i)
			{
				std::vector<std::string> light_tokens;
				{
					std::getline(mesh_file, line);
					std::istringstream iss(line);
					std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(light_tokens));
				}
				if ("l_ref" != light_tokens[0] || light_tokens.size() != 2)
				{
					std::stringstream ss;
					ss << "Unknown line: " << line;
#ifdef _WIN32
					MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
				}

				if (light_mapping_.find(light_tokens[1]) == light_mapping_.end())
				{
					std::stringstream ss;
					ss << "Unknown light: " << light_tokens[1];
#ifdef _WIN32
					MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#else
					ss << ss.str() << std::endl;
#endif
					exit(1);
				}
				current_section->addLight(*light_mapping_[light_tokens[1]]);
			}
		}

		// Complete face :).
		if (vertex_index == 3)
		{
			vertex_index = 0;

			// Check if the normals and uv coordinates match with the values we already have stored.
			for (unsigned int i = 0; i < 3; ++i)
			{
				bool match_data = true;
				unsigned int vertex_id = current_triangle.vertex_[i];
				if (normals_[vertex_id].x != std::numeric_limits<float>::max() && glm::dot(normals_[vertex_id], current_triangle.normal_) < .66f)
				{
					match_data = false;
				}

				if (texture_coordinates_[vertex_id].x != std::numeric_limits<float>::max() && std::abs(glm::distance(current_triangle.uv_[i], texture_coordinates_[vertex_id])) > 0.01f)
				{
					match_data = false;
				}

				// Duplicate the vertex!
				if (!match_data)
				{
					current_triangle.vertex_[i] = vertices_.size();
					vertices_.push_back(vertices_[vertex_id]);
					normals_.push_back(current_triangle.normal_);
					texture_coordinates_.push_back(current_triangle.uv_[i]);
				}
				else
				{
					normals_[current_triangle.vertex_[i]] = current_triangle.normal_;
					texture_coordinates_[current_triangle.vertex_[i]] = current_triangle.uv_[i];
				}
			}

			current_section->addIndex(current_triangle.vertex_[0]);
			current_section->addIndex(current_triangle.vertex_[1]);
			current_section->addIndex(current_triangle.vertex_[2]);
			
			current_triangle = PLF_Triangle();
		}
	}

	mesh_file.close();
	// Build the sections of the level.
	
	regions_.clear();
	Entity* root_node = new Entity(scene_manager, &terrain_node, glm::mat4(1.0), OBSTACLE, "root");
	for (std::vector<Section*>::const_iterator ci = sections_.begin(); ci != sections_.end(); ++ci)
	{
#ifdef PLF_LOADER_ENABLE_OUTPUT
		ss << "load... " << vertices_.size() << std::endl;
#endif
		Section* section = *ci;
		section->calculateCentre(vertices_);

		// TODO: Move to the centre of the actual section.
		Entity* section_node = new Entity(scene_manager, root_node, glm::mat4(1.0), OBSTACLE, "section");

		if (material != NULL && shader != NULL)
		{
			std::shared_ptr<Shape> shape(std::make_shared<Shape>(vertices_, texture_coordinates_, section->getIndices(), normals_));
			new SceneLeafModel(*section_node, NULL, shape, material, *shader, false, false);
		}

		std::stringstream region_name_ss;
		region_name_ss << "s" << section->getSectionId() << "; c=" << section->getCollisionPlanes().size();

		Region* region = NULL;
		if (create_regions)
		{
			region = new Region(*section_node, region_name_ss.str());
			section_node->setRegion(*region);
			regions_.push_back(region);
			region->addCollidableEntity(*section_node);
		}

		// Created the bounded planes.
		for (std::vector<std::vector<const Plane*> >::const_iterator ci = section->getCollisionPlanes().begin(); ci != section->getCollisionPlanes().end(); ++ci)
		{
			ConvexPolygon* convex_polygon = new ConvexPolygon(*section_node, *ci);
			section_node->addCollision(*convex_polygon);
		}

		// Add the lights.
		for (SceneNode* light : section->getLights())
		{
			section_node->addChild(*light);
		}
	}
#ifdef PLF_LOADER_ENABLE_OUTPUT
	ss << "Process portals" << std::endl;
#endif
	//std::vector<PortalDef> portals;
	portals_.clear();
	if (create_regions)
	{
		mesh_file.open(filename.c_str(), std::ios::in);
		std::vector<glm::vec3> points;
		unsigned int section_id_1 = std::numeric_limits<unsigned int>::max();
		unsigned int section_id_2 = std::numeric_limits<unsigned int>::max();
		while (std::getline(mesh_file, line))
		{
			std::vector<std::string> tokens;
			std::istringstream iss(line);
			std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(tokens));

			if (tokens.empty())
			{
				continue;
			}

			// Portals.
			if ("p" == tokens[0] && tokens.size() == 3)
			{
				if (section_id_1 != std::numeric_limits<unsigned int>::max())
				{
#ifdef PLF_LOADER_ENABLE_OUTPUT
					ss << "Add a portal from " << section_id_1 << " to " << section_id_2 << std::endl;
#endif
					portals_.push_back(PortalDef(section_id_1, section_id_2, points));
					section_id_1 = std::numeric_limits<unsigned int>::max();
					section_id_2 = std::numeric_limits<unsigned int>::max();
				}
				points.clear();
				section_id_1 = ::atoi(tokens[1].c_str());
				section_id_2 = ::atoi(tokens[2].c_str());
#ifdef PLF_LOADER_ENABLE_OUTPUT		
				ss << "Potential portal from " << section_id_1 << " to " << section_id_2 << std::endl;
#endif
			}
			else if ("pv" == tokens[0] && tokens.size() == 4)
			{
				points.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
			}
		}
		if (section_id_1 != std::numeric_limits<unsigned int>::max())
		{
#ifdef PLF_LOADER_ENABLE_OUTPUT
			ss << "Add a portal from " << section_id_1 << " to " << section_id_2 << std::endl;
#endif
			portals_.push_back(PortalDef(section_id_1, section_id_2, points));
		}
		mesh_file.close();
	}
	
	for (std::vector<PortalDef>::iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		PortalDef& portal = *ci;

#ifdef PLF_LOADER_ENABLE_OUTPUT
		ss << "Pre ordering portal: ";
		for (std::vector<glm::vec3>::const_iterator ci = portal.points_.begin(); ci != portal.points_.end(); ++ci)
		{
			ss << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
		}
		ss << std::endl;
#endif

		// First thing to do is to make sure the points are such that any line sequence, created by
		// linking any adjacent points, do not intersect. Blender is not consistent with its output...
		std::vector<glm::vec3> points_to_process = portal.points_;
		std::vector<glm::vec3> all_points = portal.points_;

		if (points_to_process.size() > 0)
		{
			glm::vec3 current_point = points_to_process[0];
			points_to_process.erase(points_to_process.begin());
			portal.points_.clear();

#ifdef PLF_LOADER_ENABLE_OUTPUT
			ss << "# points: " << portal.points_.size() << std::endl;
#endif

			portal.points_.push_back(current_point);
#ifdef PLF_LOADER_ENABLE_OUTPUT
			ss << portal.points_.size() << std::endl;
#endif

			// Find the next point that does not intersect with any of the other points.
			while (points_to_process.size() > 0)
			{
				for (std::vector<glm::vec3>::iterator i = points_to_process.begin(); i != points_to_process.end(); ++i)
				{
					const glm::vec3& next_point = *i;
					bool is_valid_point = true;
#ifdef PLF_LOADER_ENABLE_OUTPUT
					ss << "Next point: (" << next_point.x << ", " << next_point.y << ", " << next_point.z << ")" << std::endl;
#endif

					// Check if the line formed by these points intersects with any other possible lines.
					for (std::vector<glm::vec3>::const_iterator ci = all_points.begin(); ci != all_points.end(); ++ci)
					{
						const glm::vec3& control_point1 = *ci;
						if (control_point1 == current_point || control_point1 == next_point) continue;

#ifdef PLF_LOADER_ENABLE_OUTPUT
						ss << "Control point1: (" << control_point1.x << ", " << control_point1.y << ", " << control_point1.z << ")" << std::endl;
#endif

						for (std::vector<glm::vec3>::const_iterator ci = all_points.begin(); ci != all_points.end(); ++ci)
						{
							const glm::vec3& control_point2 = *ci;
							if (control_point2 == current_point || control_point2 == next_point || control_point2 == control_point1) continue;

#ifdef PLF_LOADER_ENABLE_OUTPUT
							ss << "Control point1: (" << control_point2.x << ", " << control_point2.y << ", " << control_point2.z << ")" << std::endl;
#endif

							float distance = Math::dist3D_Segment_to_Segment(current_point, next_point, control_point1, control_point2);

#ifdef PLF_LOADER_ENABLE_OUTPUT
							ss << "Compare: (" << current_point.x << ", " << current_point.y << ", " << current_point.z << ") -- (" << next_point.x << ", " << next_point.y << ", " << next_point.z << ") ***";
							ss << " (" << control_point1.x << ", " << control_point1.y << ", " << control_point1.z << ") -- (" << control_point2.x << ", " << control_point2.y << ", " << control_point2.z << ")  = " << distance;
							ss << " -- " << portal.points_.size() << std::endl;
#endif


							if (distance < 0.01f)
							{
								is_valid_point = false;
								break;
							}
						}

						if (!is_valid_point) break;
					}

					if (is_valid_point)
					{
#ifdef PLF_LOADER_ENABLE_OUTPUT
						ss << "Add: (" << next_point.x << ", " << next_point.y << ", " << next_point.z << ") # points: " << portal.points_.size() << std::endl;
#endif
						portal.points_.push_back(next_point);
						points_to_process.erase(i);
						current_point = next_point;
						break;
					}
				}
			}
		}

#ifdef PLF_LOADER_ENABLE_OUTPUT
		ss << "Post ordering portal: ";
		for (std::vector<glm::vec3>::const_iterator ci = portal.points_.begin(); ci != portal.points_.end(); ++ci)
		{
			ss << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
		}
		ss << std::endl;
#endif

		std::vector<glm::vec3> reverse_points(portal.points_);
		std::reverse(reverse_points.begin(), reverse_points.end());
		
		glm::vec3 normal = glm::cross(portal.points_[1] - portal.points_[0], portal.points_[portal.points_.size() - 1] - portal.points_[0]);
		float angle_1 = glm::dot(glm::normalize(normal), glm::normalize(sections_[portal.section_id_1_]->getCentre() - portal.points_[0]));

		const std::vector<glm::vec3>* points_to_use = &portal.points_;
		// Check if the angle is less than 90 degrees.
		if (angle_1 < 0)
		{
			points_to_use = &reverse_points;
		}

		// Check which way the portal should face.
		Portal& portal1 = regions_[portal.section_id_1_]->addPortalToOtherRegion(*regions_[portal.section_id_2_], *points_to_use);

		float angle_2 = glm::dot(glm::normalize(normal), glm::normalize(sections_[portal.section_id_2_]->getCentre() - portal.points_[0]));

		points_to_use = &portal.points_;
		// Check if the angle is less than 90 degrees.
		if (angle_2 < 0)
		{
			points_to_use = &reverse_points;
		}
			
		Portal& portal2 = regions_[portal.section_id_2_]->addPortalToOtherRegion(*regions_[portal.section_id_1_], *points_to_use);
		portal1.setMirrorPortal(portal2);
		portal2.setMirrorPortal(portal1);

		if (portal_material != NULL)
		{
			// Construct the portals (debug).
			std::vector<glm::vec3> portal_vertices;
			std::vector<glm::vec2> portal_texture_coordinates;
			std::vector<GLuint> portal_indices;
			std::vector<glm::vec3> portal_normals;

			for (std::vector<glm::vec3>::const_iterator ci = reverse_points.begin(); ci != reverse_points.end(); ++ci)
			{
				portal_vertices.push_back(*ci);
				portal_texture_coordinates.push_back(glm::vec2(0, 0));
				portal_normals.push_back(normal);
			}
			portal_indices.push_back(0);
			portal_indices.push_back(1);
			portal_indices.push_back(2);
			portal_indices.push_back(0);
			portal_indices.push_back(2);
			portal_indices.push_back(3);

			SceneNode* section_node = new SceneNode(scene_manager, root_node, glm::rotate(glm::mat4(1.0), glm::radians(-0.0f), glm::vec3(1.0f, 0.0f, 0.0f)));
			std::shared_ptr<Shape> shape(std::make_shared<Shape>(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals));
			new SceneLeafModel(*section_node, NULL, shape, portal_material, BasicShadowShader::getShader(), true, true);
		}

#ifdef PLF_LOADER_ENABLE_OUTPUT
		ss << "Loaded portal: ";
		for (std::vector<glm::vec3>::const_iterator ci = points_to_use->begin(); ci != points_to_use->end(); ++ci)
		{
			ss << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
		}
		ss << std::endl;
#endif
	}
	root_node_ = root_node;
	
#ifdef _WIN32
	OutputDebugString(ss.str().c_str());
#else
	std::cout << ss.str() << std::endl;
#endif

	return root_node;
}

NavigationMesh* PortalLevelFormatLoader::createNavigationMesh(SceneManager* scene_manager, float max_distance)
{
	if (root_node_ == NULL)
	{
		return NULL;
	}

	// Create a list of vertices that make up the collision boxes.
	std::vector<const ConvexPolygon*> convex_polygons;
	for (const std::pair<std::string, std::vector<const Plane*>* >& mp : collision_mapping_)
	{
		convex_polygons.push_back(new ConvexPolygon(*root_node_, *mp.second));
	}

	// Now we have our navigation mesh.
	return new NavigationMesh(convex_polygons, glm::vec3(0, 1, 0), 45.0f * M_PI / 180.0f, max_distance, 3.0f);
}

};
