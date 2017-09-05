#include "RegionWorld.h"
#include "FrustumEntity.h"

#include <iostream>
#include <algorithm>

#include "../../core/entities/Entity.h"
#include "../../core/entities/camera/FreeMovingCamera.h"
#include "../../core/scene/Material.h"
#include "../../core/scene/SceneManager.h"
#include "../../core/scene/SceneNode.h"
#include "../../core/scene/SceneLeafLight.h"
#include "../../core/scene/SceneLeafModel.h"
#include "../../core/scene/SkyBoxLeaf.h"
#include "../../core/scene/portal/Region.h"
#include "../../core/scene/portal/Portal.h"
#include "../../core/shaders/SkyBoxShader.h"
#include "../../core/shaders/TerrainShader.h"
#include "../../core/shaders/BasicShadowShader.h"
#include "../../core/texture/TargaTexture.h"
#include "../../core/collision/BoxCollision.h"
#include "../../shapes/Cube.h"
#include "../../shapes/terrain.h"
#include "../../shapes/SkyBox.h"

#include "../physical_sim/PhysicsCube.h"

RegionWorld::RegionWorld(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	
}

bool RegionWorld::init(int argc, char** argv)
{
	#ifdef _WIN32
	if (!GLEW_VERSION_4_3)
	{
		std::cerr << "Right OpenGL version is not suppored!" << std::endl;
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
        	std::cerr << "VBOs are not supported by your graphics card" << std::endl;
	        MessageBox(NULL, "VBOs are not supported by your graphics card!", "An error occurred", MB_ICONERROR | MB_OK);
        	return false;
	}
	std::cout << "Right version is supported! :D" << std::endl;
#endif
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	
	terrain_ = new Terrain();
	if (!terrain_->createHeightmap(129, -0.02f))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the terrain heightmap", "Error", MB_OK);
#endif
		std::cout << "Could not load the terrain heightmap" << std::endl;
		return false;
	}
	
	terrain_node_ = new Entity(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0), OBSTACLE, "terrain");
	
	Texture* wfl_texture = TargaTexture::loadTexture("data/models/levels/modular/atlas.tga");
	//Texture* wfl_texture = new Texture("data/textures/water.tga");

	// Initialise the texture to use.
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.4f, 0.4f, 0.4f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.3f, 0.3f, 0.3f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*wfl_texture);
	
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.3f, 1.0f);

	Material* terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);
	
	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, TerrainShader::getShader(), false, false);

	// Initialise the camera:
	camera_ = new FreeMovingCamera(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(2, 1, 2)), 90.0f, 1024, 768, 0.1f, 60.0f);
	scene_manager_->addUpdateableEntity(*camera_);
	
	// Initialise the sky box to render.
	MaterialLightProperty* skybox_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* skybox_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* skybox_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* skybox_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	Material* skybox_material = new Material(*skybox_ambient, *skybox_diffuse, *skybox_specular, *skybox_emmisive);
	Texture* skybox_texture = TargaTexture::loadTexture("data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");
	skybox_material->addCubeTexture(*skybox_texture);

	SkyBox* inverted_cube = new SkyBox(5.0f);
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*camera_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material);
	
	// Create a number of regions.
	float cube_size = 2;
	Cube* cube = new Cube(cube_size, cube_size, cube_size);
	std::vector<Region*> regions;
	int size = 6;
	
	for (int z = 0; z < size; ++z)
	{
		for (int y = 0; y < size; ++y)
		{
			for (int x = 0; x < size; ++x)
			{
				std::stringstream ss;
				ss << "Region(" << x << ", " << y << ", " << z << ")";
				Entity* scene_node = new Entity(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), glm::vec3(x * cube_size, y * cube_size, z * cube_size)), OBSTACLE, ss.str());
				SceneLeafModel* scene_leaf_model = new SceneLeafModel(*scene_node, NULL, *cube, *wfl_material_, BasicShadowShader::getShader(), true, true);
				//scene_leaf_model->setVisible(false);
				
				Region* region = new Region(*scene_node, ss.str());
				scene_node->setRegion(*region);
				
				regions.push_back(region);
				
				// Connect left-right neighbours.
				if (x > 0)
				{
					Region* to_region = regions[regions.size() - 2];
					std::vector<glm::vec3> points, reverse_points;
					
					points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					
					Portal& p0 = region->addPortalToOtherRegion(*to_region, points);
					Portal& p1 = to_region->addPortalToOtherRegion(*region, reverse_points);
					p0.setMirrorPortal(p1);
					p1.setMirrorPortal(p0);
					
					std::cout << p0 << std::endl;
					std::cout << p1 << std::endl;
					
					/*
					{
					// Construct the portals (debug).
					std::vector<glm::vec3> portal_vertices;
					std::vector<glm::vec2> portal_texture_coordinates;
					std::vector<GLuint> portal_indices;
					std::vector<glm::vec3> portal_normals;

					glm::vec3 normal = glm::cross(points[1] - points[0], points[2] - points[0]);
					for (std::vector<glm::vec3>::const_iterator ci = points.begin(); ci != points.end(); ++ci)
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

					SceneNode* section_node = new SceneNode(*scene_manager_, scene_node, glm::mat4(1.0));
					Shape* shape = new Shape(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals);
					SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *terrain_material_, BasicShadowShader::getShader(), true, true);
					}
					
					{
					// Construct the portals (debug).
					std::vector<glm::vec3> portal_vertices;
					std::vector<glm::vec2> portal_texture_coordinates;
					std::vector<GLuint> portal_indices;
					std::vector<glm::vec3> portal_normals;

					glm::vec3 normal = glm::cross(reverse_points[1] - reverse_points[0], reverse_points[2] - reverse_points[0]);
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

					SceneNode* section_node = new SceneNode(*scene_manager_, &to_region->getSceneNode(), glm::mat4(1.0));
					Shape* shape = new Shape(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals);
					SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *terrain_material_, BasicShadowShader::getShader(), true, true);
					}
					*/
				}
				
				// Connect down-up neighbours.
				if (y > 0)
				{
					Region* to_region = regions[regions.size() - (1 + size)];
					std::vector<glm::vec3> points, reverse_points;
					/*
					points.push_back(glm::vec3(0.5f, -0.5f, 0.5f));
					points.push_back(glm::vec3(-0.5f, -0.5f, 0.5f));
					points.push_back(glm::vec3(-0.5f, -0.5f, -0.5f));
					points.push_back(glm::vec3(0.5f, -0.5f, -0.5f));
					*/
					
					//
					points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					
					/*
					reverse_points.push_back(glm::vec3(0.5f, 0.5f, -0.5f));
					reverse_points.push_back(glm::vec3(-0.5f, 0.5f, -0.5f));
					reverse_points.push_back(glm::vec3(-0.5f, 0.5f, 0.5f));
					reverse_points.push_back(glm::vec3(0.5f, 0.5f, 0.5f));
					*/
					
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					
					Portal& p0 = region->addPortalToOtherRegion(*to_region, points);
					Portal& p1 = to_region->addPortalToOtherRegion(*region, reverse_points);
					p0.setMirrorPortal(p1);
					p1.setMirrorPortal(p0);
					
					/*
					// Construct the portals (debug).
					std::vector<glm::vec3> portal_vertices;
					std::vector<glm::vec2> portal_texture_coordinates;
					std::vector<GLuint> portal_indices;
					std::vector<glm::vec3> portal_normals;

					glm::vec3 normal = glm::cross(points[1] - points[0], points[2] - points[0]);
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

					SceneNode* section_node = new SceneNode(*scene_manager_, NULL, glm::mat4(1.0));
					Shape* shape = new Shape(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals);
					SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *terrain_material_, BasicShadowShader::getShader(), true, true);
					*/
				}
				
				// Connect back-front neighbours.
				if (z > 0)
				{
					Region* to_region = regions[regions.size() - (1 + size * size)];
					std::vector<glm::vec3> points, reverse_points;
					points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, -cube_size / 2.0f));
					
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(-cube_size / 2.0f, -cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(-cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					reverse_points.push_back(glm::vec3(cube_size / 2.0f, cube_size / 2.0f, cube_size / 2.0f));
					
					Portal& p0 = region->addPortalToOtherRegion(*to_region, points);
					Portal& p1 = to_region->addPortalToOtherRegion(*region, reverse_points);
					p0.setMirrorPortal(p1);
					p1.setMirrorPortal(p0);
					
					/*
					// Construct the portals (debug).
					std::vector<glm::vec3> portal_vertices;
					std::vector<glm::vec2> portal_texture_coordinates;
					std::vector<GLuint> portal_indices;
					std::vector<glm::vec3> portal_normals;

					glm::vec3 normal = glm::cross(points[1] - points[0], points[2] - points[0]);
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

					SceneNode* section_node = new SceneNode(*scene_manager_, NULL, glm::mat4(1.0));
					Shape* shape = new Shape(portal_vertices, portal_texture_coordinates, portal_indices, portal_normals);
					SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*section_node, NULL, *shape, *terrain_material_, BasicShadowShader::getShader(), true, true);
					*/
				}
				
				region_to_visible_model_[region] = scene_leaf_model;
			}
		}
	}
	
	// Setup the movable node.
	Cube* big_cube = new Cube(3, 1, 2);
	movable_node_ = new Entity(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 1)), OBSTACLE, "The Cube!");
	SceneLeafModel* scene_leaf_model = new SceneLeafModel(*movable_node_, NULL, *big_cube, *wfl_material_, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*movable_node_);
	
	FrustumEntity* frustum_entity = new FrustumEntity(*scene_manager_, *camera_);
	scene_manager_->addUpdateableEntity(*frustum_entity);
	
	BoxCollision* bounded_box = new BoxCollision(*movable_node_, 3, 1, 2);
	movable_node_->addCollision(*bounded_box);

	cube_ = new PhysicsCube(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 5.0f, 0.0f)));
	scene_manager_->addUpdateableEntity(*cube_);
	//scene_manager_->addPlayer(*cube_);

	return true;
}

bool RegionWorld::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
}

GLuint RegionWorld::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	glfwSetMousePos(width_ / 2, height_ / 2);
	return 0;
}

void RegionWorld::tick(float dt)
{
	if (glfwGetKey('M') == GLFW_PRESS)
	{
		for (std::vector<Region*>::const_iterator ci = Region::getAllRegions().begin(); ci != Region::getAllRegions().end(); ++ci)
		{
			Region* region = *ci;
			region_to_visible_model_[region]->setVisible(false);
		}
	}

	if (glfwGetKey('N') == GLFW_PRESS)
	{
		for (std::vector<Region*>::const_iterator ci = Region::getAllRegions().begin(); ci != Region::getAllRegions().end(); ++ci)
		{
			Region* region = *ci;
			region_to_visible_model_[region]->setVisible(true);
		}
	}
	/*
	std::vector<Region*>* collidable_regions = Region::getRegionsCollidableIn(*movable_node_);
	if (collidable_regions != NULL)
	{
		for (std::vector<Region*>::const_iterator ci = collidable_regions->begin(); ci != collidable_regions->end(); ++ci)
		{
			Region* region = *ci;
			region_to_visible_model_[region]->setVisible(true);
		}
	}
	
	std::vector<Region*>* visible_regions = Region::getRegionsVisibleIn(*movable_node_);
	if (visible_regions != NULL)
	{
		for (std::vector<Region*>::const_iterator ci = visible_regions->begin(); ci != visible_regions->end(); ++ci)
		{
			Region* region = *ci;
			region_to_visible_model_[region]->setVisible(true);
		}
	}
	*/
#ifdef _WIN32
	if (movable_node_->getRegion() != NULL)
	{
		std::stringstream ss;
		ss << movable_node_->getRegion()->getName() << std::endl;
		OutputDebugString(ss.str().c_str());
	}
#endif

	if (glfwGetKey('E') == GLFW_PRESS)
	{
		cube_->setTransformation(glm::translate(cube_->getLocalTransformation(), glm::vec3(0, 1, 0)));
		cube_->updateTransformations();
		cube_->transition(terrain_node_);
	}
	
	else if (glfwGetKey('4') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(dt, 0, 0)));
	}
	else if (glfwGetKey('6') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(-dt, 0, 0)));
	}
	if (glfwGetKey('2') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(0, -dt, 0)));
	}
	else if (glfwGetKey('8') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(0, dt, 0)));
	}
	else if (glfwGetKey('7') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(0, 0, -dt)));
	}
	else if (glfwGetKey('9') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::translate(movable_node_->getLocalTransformation(), glm::vec3(0, 0, dt)));
	}
	else if (glfwGetKey('Y') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), dt * 10.0f, glm::vec3(0, 1, 0)));
	}
	else if (glfwGetKey('U') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), -dt * 10.0f, glm::vec3(0, 1, 0)));
	}
	else if (glfwGetKey('O') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), dt * 10.0f, glm::vec3(0, 0, 1)));
	}
	else if (glfwGetKey('P') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), -dt * 10.0f, glm::vec3(0, 0, 1)));
	}
	else if (glfwGetKey('N') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), dt * 10.0f, glm::vec3(1, 0, 0)));
	}
	else if (glfwGetKey('M') == GLFW_PRESS)
	{
		movable_node_->setTransformation(glm::rotate(movable_node_->getLocalTransformation(), -dt * 10.0f, glm::vec3(1, 0, 0)));
	}
}

void RegionWorld::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_->onResize(width, height);
}
