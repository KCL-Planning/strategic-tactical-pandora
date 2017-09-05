#include "PortalLevelFormatLoader.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <limits>

#include "../../shapes/Shape.h"
#include "../../shapes/Cube.h"
#include "../scene/SceneManager.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneLeafModel.h"
#include "../entities/Entity.h"
#include "../scene/portal/Portal.h"
#include "../scene/portal/Region.h"
#include "../collision/BoxCollision.h"
#include "../math/Math.h"

#include "../texture/Texture.h"
#include "../texture/TargaTexture.h"
#include "../scene/Material.h"

#include "../shaders/BasicShadowShader.h"
#include "../shaders/ToonShader.h"
#include "../shaders/LineShader.h"

#include "../ai/pathfinding/ConvexNavigationArea.h"
#include "../ai/pathfinding/NavigationMesh.h"

//#define PLF_LOADER_ENABLE_OUTPUT

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

SceneNode* PortalLevelFormatLoader::importLevel(const std::string& filename, Material& material, ShaderInterface& shader, SceneManager& scene_manager, SceneNode& terrain_node, bool create_regions)
{
	std::cout << "[PortalLevelFormatLoader::importLevel] " << filename << std::endl;
	root_node_ = NULL;
	std::ifstream mesh_file;
	mesh_file.open(filename.c_str(), std::ios::in);

	if (!mesh_file.is_open())
	{
		std::cerr << "Could not open: " << filename << std::endl;
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

		// Vertices.
		if ("v" == tokens[0] && tokens.size() == 4)
		{
			vertices_.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
		}
	}
	mesh_file.close();
	mesh_file.open(filename.c_str(), std::ios::in);
	
	std::cout << "Loaded: " << vertices_.size() << " vertices." << std::endl;

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
			std::cout << "Process section #" << section_id << std::endl;
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
		else if ("c" == tokens[0] && tokens.size() == 2)
		{
			unsigned int nr_points = ::atoi(tokens[1].c_str());
			assert (nr_points == 8); // We only deal with cubes at the moment.
			std::vector<glm::vec3> bb_points;
			//std::stringstream ss2;
			for (unsigned int i = 0; i < nr_points; ++i)
			{
				std::getline(mesh_file, line);
				std::vector<std::string> bb_tokens;
				std::istringstream iss(line);
				std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter<std::vector<std::string> >(bb_tokens));

				if ("cv" != bb_tokens[0] || bb_tokens.size() != 4)
				{
					std::stringstream ss;
					ss << "Unknown line: " << line;
#ifdef _WIN32
					MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
				}
				assert ("cv" == bb_tokens[0] && bb_tokens.size() == 4);
				glm::vec3 point(::atof(bb_tokens[1].c_str()), ::atof(bb_tokens[2].c_str()), ::atof(bb_tokens[3].c_str()));
				bb_points.push_back(point);
				//ss2 << "p " << point.x << " " << point.y << " " << point.z << std::endl;
			}
/*
#ifdef _WIN32
			MessageBox(NULL, ss2.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
*/
			std::cout << "Add a collision box of: " << bb_points.size() << " vertices." << std::endl;
			current_section->addCollisionBox(bb_points);
		}
/*		else if ("v" != tokens[0] && "p" != tokens[0] && "pv" != tokens[0])
		{
			std::stringstream ss;
			ss << "Unknown line: " << line;
#ifdef _WIN32
			MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		}*/

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
/*
	Texture* wfl_texture = TargaTexture::loadTexture("data/models/levels/modular/atlas.tga");
	//Texture* wfl_texture = new Texture("data/textures/water.tga");

	// Initialise the texture to use.
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.4f, 0.4f, 0.4f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.3f, 0.3f, 0.3f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*wfl_texture);
*/
	// Build the sections of the level.

	//std::stringstream ss;

	//std::vector<Region*> regions;
	regions_.clear();
	Entity* root_node = new Entity(scene_manager, &terrain_node, glm::mat4(1.0), OBSTACLE, "root");
	for (std::vector<Section*>::const_iterator ci = sections_.begin(); ci != sections_.end(); ++ci)
	{
		std::cout << "load... " << vertices_.size() << std::endl;
		Section* section = *ci;
		section->calculateCentre(vertices_);

		//Section* section = sections[0];

		//ss << section->getSectionId() << " " << section->getCentre().x << " " << section->getCentre().y << " " << section->getCentre().z << std::endl;

		// TODO: Move to the centre of the actual section.
		Entity* section_node = new Entity(scene_manager, root_node, glm::mat4(1.0), OBSTACLE, "section");
		Shape* shape = new Shape(vertices_, texture_coordinates_, section->getIndices(), normals_);
		
		//Cube* shape = new Cube(5.0f);
		//SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *wfl_material_, ToonShader::getShader(), false, false);
		SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, material, shader, false, false);

		std::stringstream ss;
		ss << "s" << section->getSectionId() << "; c=" << section->getCollisionBoxes().size();

		if (create_regions)
		{
			Region* region = new Region(*section_node, ss.str());
			section_node->setRegion(*region);
			regions_.push_back(region);
		}

		// Create the bounded boxes.
		for (std::vector<PLF_Cube*>::const_iterator ci = section->getCollisionBoxes().begin(); ci != section->getCollisionBoxes().end(); ++ci)
		{
			const PLF_Cube* cube = *ci;
			BoxCollision* bc = new BoxCollision(*section_node,
			                           cube->bottom_left_away_, cube->bottom_right_away_, 
			                           cube->top_left_away_, cube->top_right_away_,                            
			                           cube->bottom_left_close_, cube->bottom_right_close_, 
			                           cube->top_left_close_, cube->top_right_close_);

			section_node->addCollision(*bc, *SceneNode::bright_material_, BasicShadowShader::getShader());
			//section_node->addCollision(*bc);
/*
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "Bounding box: {";
			for (unsigned int i = 0; i < 8; ++i)
			{
				ss << "(" << bc->getPoints()[i].x << ", " << bc->getPoints()[i].y << ", " << bc->getPoints()[i].z << "), ";
			}
			ss << "}" << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#endif
*/
			//section_node->addCollision(*bc);
/*
#ifdef _WIN32
			std::stringstream ss2;
			ss2 << cube << std::endl;
			MessageBox(NULL, ss2.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
*/
		}
	}
	
	std::cout << "Process portals" << std::endl;
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
					std::cout << "Add a portal from " << section_id_1 << " to " << section_id_2 << std::endl;
					portals_.push_back(PortalDef(section_id_1, section_id_2, points));
					section_id_1 = std::numeric_limits<unsigned int>::max();
					section_id_2 = std::numeric_limits<unsigned int>::max();
				}
				points.clear();
				section_id_1 = ::atoi(tokens[1].c_str());
				section_id_2 = ::atoi(tokens[2].c_str());
				
				std::cout << "Potential portal from " << section_id_1 << " to " << section_id_2 << std::endl;
			}
			else if ("pv" == tokens[0] && tokens.size() == 4)
			{
				points.push_back(glm::vec3(::atof(tokens[1].c_str()), ::atof(tokens[2].c_str()), ::atof(tokens[3].c_str())));
			}
		}
		if (section_id_1 != std::numeric_limits<unsigned int>::max())
		{
			std::cout << "Add a portal from " << section_id_1 << " to " << section_id_2 << std::endl;
			portals_.push_back(PortalDef(section_id_1, section_id_2, points));
		}
		mesh_file.close();
	}
	
	for (std::vector<PortalDef>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		const PortalDef& portal = *ci;
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

		// Construct the portals (debug).
		std::vector<glm::vec3> portal_vertices;
		std::vector<glm::vec2> portal_texture_coordinates;
		std::vector<GLuint> portal_indices;
		std::vector<glm::vec3> portal_normals;

		for (std::vector<glm::vec3>::const_iterator ci = reverse_points.begin(); ci != reverse_points.end(); ++ci)
		{
			portal_vertices.push_back(*ci);
			portal_texture_coordinates.push_back(glm::vec2(0,0));
			portal_normals.push_back(normal);
		}
		portal_indices.push_back(0);
		portal_indices.push_back(1);
		portal_indices.push_back(2);
		portal_indices.push_back(0);
		portal_indices.push_back(2);
		portal_indices.push_back(3);

		SceneNode* section_node = new SceneNode(scene_manager, root_node, glm::rotate(glm::mat4(1.0), -0.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
		Shape* shape = new Shape(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals);
		SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *SceneNode::bright_material_, BasicShadowShader::getShader(), true, true);
		
		

#ifdef _WIN32
		std::stringstream ss;
		ss << "Loaded portal: ";
		for (std::vector<glm::vec3>::const_iterator ci = portal_vertices.begin(); ci != portal_vertices.end(); ++ci)
		{
			ss << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
		}
		ss << std::endl;
		OutputDebugString(ss.str().c_str());
#else
		std::cout << "Loaded portal: ";
		for (std::vector<glm::vec3>::const_iterator ci = portal_vertices.begin(); ci != portal_vertices.end(); ++ci)
		{
			std::cout << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "), ";
		}
		std::cout << std::endl;
#endif

	}
	root_node_ = root_node;
	
	// Visualise the collision boxes.
	MaterialLightProperty* ambient = new MaterialLightProperty(1, 1, 0 , 1);
	MaterialLightProperty* diffuse = new MaterialLightProperty(1, 1, 0 , 1);
	MaterialLightProperty* specular = new MaterialLightProperty(1, 1, 0 , 1);
	MaterialLightProperty* emissive = new MaterialLightProperty(1, 1, 0 , 1);
	Material* collision_material = new Material(*ambient, *diffuse, *specular, *emissive);
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	collision_material->add2DTexture(*grass_texture);
	for (std::vector<Section*>::const_iterator ci = sections_.begin(); ci != sections_.end(); ++ci)
	{
		Section* section = *ci;
		for (std::vector<PLF_Cube*>::const_iterator ci = section->getCollisionBoxes().begin(); ci != section->getCollisionBoxes().end(); ++ci)
		{
			PLF_Cube* cube = *ci;
			SceneLeafModel* collision_box = new SceneLeafModel(terrain_node, NULL, *cube, *collision_material, BasicShadowShader::getShader(), false, false, COLLISION);
		}
		
		std::cout << "Section: " << section->getSectionId() << " has " << section->getCollisionBoxes().size() << " collision boxes!" << std::endl;
	}
	
	return root_node;
}

NavigationMesh* PortalLevelFormatLoader::createNavigationMesh(SceneManager* scene_manager)
{
	if (root_node_ == NULL)
	{
		return NULL;
	}

	std::vector<glm::vec3> collision_vertices;
	std::vector<GLuint> collision_vertices_indexes;
	std::vector<PLF_Cube*> processed_cubes;

	// Create a list of vertices that make up the collision boxes.
	unsigned int index_offset = 0;
	for (std::vector<Section*>::const_iterator ci = sections_.begin(); ci != sections_.end(); ++ci)
	{
		Section* section = *ci;
		
		// Create the bounded boxes.
		for (std::vector<PLF_Cube*>::const_iterator ci = section->getCollisionBoxes().begin(); ci != section->getCollisionBoxes().end(); ++ci)
		{
			PLF_Cube* cube = *ci;
			bool is_already_processed = false;
			for (std::vector<PLF_Cube*>::const_iterator ci = processed_cubes.begin(); ci != processed_cubes.end(); ++ci)
			{
				if (cube == *ci)
				{
					is_already_processed = true;
					break;
				}
			}

			if (is_already_processed)
			{
				continue;
			}
			
			std::cout << "Add the plf cube: " << *cube << std::endl;
			
			collision_vertices.insert(collision_vertices.end(), cube->getVertices().begin(), cube->getVertices().end());
			
			for (std::vector<unsigned int>::const_iterator ci = cube->getIndices().begin(); ci != cube->getIndices().end(); ++ci)
			{
				collision_vertices_indexes.push_back(*ci + index_offset);
			}
			
			processed_cubes.push_back(cube);
			
			index_offset += cube->getVertices().size();
		}
	}

	NavigationMesh* navigation_mesh = new NavigationMesh(processed_cubes, collision_vertices, collision_vertices_indexes, glm::vec3(0.0f, 1.0f, 0.0f), 0.9f, 0.50f);
	return navigation_mesh;
}
