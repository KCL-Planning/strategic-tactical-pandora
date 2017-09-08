#include "FlatDemo.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../core/entities/camera/FreeMovingCamera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"

#include "../core/scene/SceneLeafLight.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/scene/SkyBoxLeaf.h"
#include "../shapes/terrain.h"
#include "../core/scene/Material.h"
#include "../core/shaders/TerrainShader.h"
#include "../core/shaders/AnimatedShadowShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/WaterShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/shaders/ShadowShader.h"
#include "../core/shaders/CreateAnimatedShadowMapShader.h"
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "../core/entities/Player.h"
#include "../core/entities/Monster.h"
#include "../core/collision/ConvexPolygon.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"

#include "../core/scene/portal/Region.h"
#include "../core/scene/portal/Portal.h"

#include "../core/entities/behaviours/RotateBehaviour.h"

#include "../core/loaders/AssimpLoader.h"
#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"
#include "../core/models/AnimatedModel.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Label.h"
#include "../core/gui/Container.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "gui_demo/FPSLabel.h"

#include "flat/Wall.h"
#include "shooter/ArmedPlayer.h"

FlatDemo::FlatDemo(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool FlatDemo::init(int argc, char** argv)
{
/*
	if (!GL_VERSION_4_3)
	{
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
	        std::cerr << "VBOs are not supported by your graphics card" << std::endl;
#ifdef _WIN32
		MessageBox(NULL, "VBOs are not supported by your graphics card!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
 	       return false;
	}
*/

	glEnable(GL_DEPTH_TEST);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	// Load the textures.
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_TEXTURE_CUBE_MAP);

	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");

	AssimpLoader* loader = new AssimpLoader();
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/grunt_beast/grunt beast2.dae");

	//checkError("Loaded textures.");

	glEnableVertexAttribArray(0); // Vertex.
	glEnableVertexAttribArray(1); // Colour.
	glEnableVertexAttribArray(2); // Texture.
	glEnableVertexAttribArray(3); // Normals.

	// Start building a simple scene to render.
	Terrain* terrain = new Terrain();
	if (!terrain->loadHeightmap("data/heightmaps/heightmap.raw", 65))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the terrain heightmap", "Error", MB_OK);
#endif
		return false;
	}

	//LinearAnimation* terrain_linear_animation = new LinearAnimation(&scene_manager_->getRoot(), glm::vec3(0.1f, 0.5f, -0.1f), 0, 10, 0);
	//SceneNode* terrain_node = new SceneNode(terrain_linear_animation, glm::translate(glm::mat4(1.0), glm::vec3(0, 0, 0)));
	Entity* terrain_node = new Entity(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0), OBSTACLE, "terrain");

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);

	//SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node, NULL, *terrain, *terrain_material_, TerrainShader::getShader(), &ShadowShader::getShader(), false, false);
	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node, NULL, *terrain, *terrain_material_, TerrainShader::getShader(), false, false);
	/*
	// Create the monster.
	SceneNode* rotate_node = new SceneNode(*scene_manager_, terrain_node, glm::scale(glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 3.0f, 0.0f)), -90.0f, glm::vec3(1.0f, 0.0f, 0.0f)), glm::vec3(0.1f, 0.1f, 0.1f)));
	Entity* gingerman_entity = new Entity(*scene_manager_, rotate_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f, 0.0f)), OBSTACLE, "gingerman");
	SceneLeafModel* chair_leaf_node = new SceneLeafModel(*gingerman_entity, NULL, *gingerman_properties.first, *gingerman_properties.second, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*gingerman_entity);
	*/
	// Create a flat box as the floor.
	MaterialLightProperty* concrete_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* concrete_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* concrete_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* concrete_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	concrete_material_ = new Material(*concrete_ambient, *concrete_diffuse, *concrete_specular, *concrete_emmisive);
	concrete_material_->add2DTexture(*grass_texture);

	MaterialLightProperty* bright_ambient = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(0.0, 1.0, 1.0, 1.0);

	Material* bright_material = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	bright_material->add2DTexture(*grass_texture);

	// Ground floor.
	floor_ = new Cube(10.0f, 0.5f, 10.0f);
	stair_ = new Cube(50.0f, 0.5f, 2.0f);

	// Create a wall.
	Wall* wall_with_door = new Wall(10.0f, 4.0f, 1.0f, false);
	wall_with_door->createHole(4.0f, 1.0f, 1.0f, 2.0f);
	wall_with_door->createHole(2.5f, 2.0f, 1.0f, 1.0f);
	//wall_with_door->createHole(7.5f, 2.0f, 1.0f, 1.0f);
	wall_with_door->finalise();

	Wall* plain_wall = new Wall(10.0f, 4.0f, 1.0f, false);
	plain_wall->finalise();

	Wall* floor_access = new Wall(10.0f, 10.0f, 0.5f, true);
	floor_access->createHole(9.0f, 7.5f, 2.0f, 4.0f);
	floor_access->finalise();

	Wall* ground_floor = new Wall(10.0f, 10.0f, 0.5f, true);
	ground_floor->finalise();
	
	stair_step_ = new Cube(0.5f, 0.3f, 1.0f);

	Cube* door = new Cube(0.2f, 2.0f, 1.0f);

	SceneNode* current_region_node = new SceneNode(*scene_manager_, terrain_node, glm::mat4(1.0f));
	Region* current_region = new Region(*current_region_node);
	current_region_node->setRegion(*current_region);

	// The ground floor.
	//Entity* ground_floor_node = new Entity(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f, 0.0f)), ENTITY_TYPE::OBSTACLE, "ground floor node");
	Entity* ground_floor_node = new Entity(*scene_manager_, current_region_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f, 0.0f)), OBSTACLE, "ground floor node");
	for (std::vector<Quad>::const_iterator ci = ground_floor->getQuads().begin(); ci != ground_floor->getQuads().end(); ++ci)
	{
		const Quad& quad = *ci;
		ConvexPolygon* bc = new ConvexPolygon(*ground_floor_node,
                                            glm::vec3(quad.bottom_left_.x, -ground_floor->getDepth() / 2.0f, quad.bottom_left_.y),
                                            glm::vec3(quad.bottom_right_.x, -ground_floor->getDepth() / 2.0f, quad.bottom_right_.y),
                                            glm::vec3(quad.top_left_.x, -ground_floor->getDepth() / 2.0f, quad.top_left_.y),
                                            glm::vec3(quad.top_right_.x, -ground_floor->getDepth() / 2.0f, quad.top_right_.y),
                                            glm::vec3(quad.bottom_left_.x, ground_floor->getDepth() / 2.0f, quad.bottom_left_.y),
                                            glm::vec3(quad.bottom_right_.x, ground_floor->getDepth() / 2.0f, quad.bottom_right_.y),
                                            glm::vec3(quad.top_left_.x, ground_floor->getDepth() / 2.0f, quad.top_left_.y),
                                            glm::vec3(quad.top_right_.x, ground_floor->getDepth() / 2.0f, quad.top_right_.y)
                                            );
		ground_floor_node->addCollision(*bc);
	}
	//SceneLeafModel* ground_floor_leaf_node = new SceneLeafModel(*ground_floor_node, NULL, *ground_floor, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
	SceneLeafModel* ground_floor_leaf_node = new SceneLeafModel(*ground_floor_node, NULL, *ground_floor, *concrete_material_, BasicShadowShader::getShader(), false, false);

	Entity* current_ground_floor = ground_floor_node;

	for (unsigned int floor_nr = 0; floor_nr < 4; ++floor_nr)
	{
		SceneNode* stair_case = new SceneNode(*scene_manager_, current_ground_floor, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)));
		// Stair case.
		for (unsigned int i = 0; i < 6; ++i)
		{
			std::stringstream ss;
			ss << "Lower stairs: " << i << ". Floor nr: " << floor_nr;
			//Entity* stair_step_entity = new Entity(current_ground_floor, glm::translate(glm::mat4(1.0), glm::vec3(0.5f * i + 5.25f, 0.3f * i, 9.0f)), ENTITY_TYPE::OBSTACLE, ss.str());
			Entity* stair_step_entity = new Entity(*scene_manager_, stair_case, glm::translate(glm::mat4(1.0), glm::vec3(0.5f * i + 5.25f, 0.3f * i, 9.0f)), OBSTACLE, ss.str());
			ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 0.5f, 0.3f, 1.0f);
			stair_step_entity->addCollision(*stair_step_bc);
			//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
			SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
		
		for (unsigned int i = 0; i < 7; ++i)
		{
			std::stringstream ss;
			ss << "Upper stairs: " << i << ". Floor nr: " << floor_nr;
			//Entity* stair_step_entity = new Entity(current_ground_floor, glm::translate(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(0.5f * i - 8.75f, 0.3f * i + 6 * 0.3, 9.0f)), ENTITY_TYPE::OBSTACLE, ss.str());
			Entity* stair_step_entity = new Entity(*scene_manager_, stair_case, glm::translate(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(0.5f * i - 8.75f, 0.3f * i + 6 * 0.3, 9.0f)), OBSTACLE, ss.str());
			ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 0.5f, 0.3f, 1.0f);
			stair_step_entity->addCollision(*stair_step_bc);
			//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
			SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
		
		// Create a stepping stone to connect the two stairs.
		Cube* stepping_stone = new Cube(1.5f, 0.3f, 1.0f);
		{
			std::stringstream ss;
			ss << "Stepping stone: " << floor_nr;
			//Entity* stair_step_entity = new Entity(current_ground_floor, glm::translate(glm::mat4(1.0), glm::vec3(8.75f, 0.3f * 6, 9.0f)), ENTITY_TYPE::OBSTACLE, ss.str());
			Entity* stair_step_entity = new Entity(*scene_manager_, stair_case, glm::translate(glm::mat4(1.0), glm::vec3(8.75f, 0.3f * 6, 9.0f)), OBSTACLE, ss.str());
			ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 1.5f, 0.3f, 1.0f);
			stair_step_entity->addCollision(*stair_step_bc);
			//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stepping_stone, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
			SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stepping_stone, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
	
		{
		std::stringstream ss;
		ss << "Wall: " << floor_nr;
		Entity* wall_entity_ = new Entity(*scene_manager_, current_ground_floor, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.0f, 0.0f)), OBSTACLE, ss.str());

		// Create the bounded boxes.
		for (std::vector<Quad>::const_iterator ci = plain_wall->getQuads().begin(); ci != plain_wall->getQuads().end(); ++ci)
		{
			const Quad& quad = *ci;
			ConvexPolygon* bc = new ConvexPolygon(*wall_entity_,
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, plain_wall->getDepth() / 2.0f)
                                                );
			wall_entity_->addCollision(*bc);
		}

		//SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}

		{
		std::stringstream ss;
		ss << "Wall: " << floor_nr;
		Entity* wall_entity_ = new Entity(*scene_manager_, current_ground_floor, glm::translate(glm::rotate(glm::mat4(1.0), 90.0f, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(-10.0f, 0.0f, 10.0f)), OBSTACLE, ss.str());
		for (std::vector<Quad>::const_iterator ci = plain_wall->getQuads().begin(); ci != plain_wall->getQuads().end(); ++ci)
		{
			const Quad& quad = *ci;
			ConvexPolygon* bc = new ConvexPolygon(*wall_entity_,
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, plain_wall->getDepth() / 2.0f)
                                                );
			wall_entity_->addCollision(*bc);
		}
		//SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
	
		{
		std::stringstream ss;
		ss << "Wall: " << floor_nr;
		Entity* wall_entity_ = new Entity(*scene_manager_, current_ground_floor, glm::translate(glm::rotate(glm::mat4(1.0), 180.0f, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(-10.0f, 0.0f, -10.0f)), OBSTACLE, ss.str());
		for (std::vector<Quad>::const_iterator ci = plain_wall->getQuads().begin(); ci != plain_wall->getQuads().end(); ++ci)
		{
			const Quad& quad = *ci;
			ConvexPolygon* bc = new ConvexPolygon(*wall_entity_,
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, -plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, plain_wall->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, plain_wall->getDepth() / 2.0f)
                                                );
			wall_entity_->addCollision(*bc);
		}
		//SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *plain_wall, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
	
		{
		std::stringstream ss;
		ss << "Wall: " << floor_nr;
		Entity* wall_entity_ = new Entity(*scene_manager_, current_ground_floor, glm::translate(glm::rotate(glm::mat4(1.0), 270.0f, glm::vec3(0.0f, 1.0f, 0.0f)), glm::vec3(0.0f, 0.0f, 0.0f)), OBSTACLE, ss.str());
		for (std::vector<Quad>::const_iterator ci = wall_with_door->getQuads().begin(); ci != wall_with_door->getQuads().end(); ++ci)
		{
			const Quad& quad = *ci;
			ConvexPolygon* bc = new ConvexPolygon(*wall_entity_,
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, -wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, -wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, -wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, -wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_left_.x, quad.bottom_left_.y, wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.bottom_right_.x, quad.bottom_right_.y, wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.top_left_.x, quad.top_left_.y, wall_with_door->getDepth() / 2.0f),
                                                glm::vec3(quad.top_right_.x, quad.top_right_.y, wall_with_door->getDepth() / 2.0f)
                                                );

			wall_entity_->addCollision(*bc);
		}
		//SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *wall_with_door, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* wall_leaf_node = new SceneLeafModel(*wall_entity_, NULL, *wall_with_door, *concrete_material_, BasicShadowShader::getShader(), false, false);
		}
		{
		std::stringstream ss;
		ss << "Floor access: " << floor_nr;

		SceneNode* new_current_region_node = new SceneNode(*scene_manager_, terrain_node, glm::mat4(1.0f));
		Region* new_current_region = new Region(*new_current_region_node);
		new_current_region_node->setRegion(*new_current_region);
		/*
		if (floor_nr == 2)
		{
			// Create the monster.
			std::pair<AnimatedModel*, Material*> monster_properties = loader->LoadModel(*scene_manager_, "data/models/grunt_beast/grunt beast4.dae");
			for (unsigned int i = 0; i < 1; ++ i)
			{
				//Monster* monster = new Monster(current_ground_floor, glm::translate(glm::mat4(1.0), glm::vec3(2.0f + 1.5f * i, 0.25f, 4.0f)), *current_ground_floor, *terrain, *scene_manager_);
				Monster* monster = new Monster(current_ground_floor, glm::translate(glm::mat4(1.0), glm::vec3(2.0f + 1.5f * i, 0.25f, 4.0f)), *scene_manager_);
				SceneLeafModel* chair_leaf_node = new SceneLeafModel(*monster, NULL, *monster_properties.first, *monster_properties.second, AnimatedShadowShader::getShader(), false, false);
				chair_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);

				monster->init(*bright_material, BasicShadowShader::getShader());

				scene_manager_->addUpdateableEntity(*monster);
				monster_properties.first->setAnimation(*monster_properties.first->getAnimations()[0]);
			}
		}*/

		// Create the floor access to the next floor.
		//Entity* floor_access_entity = new Entity(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f + (floor_nr + 1) * 3.8f, 0.0f)), ENTITY_TYPE::OBSTACLE, ss.str());
		Entity* floor_access_entity = new Entity(*scene_manager_, new_current_region_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f + (floor_nr + 1) * 3.8f, 0.0f)), OBSTACLE, ss.str());
		for (std::vector<Quad>::const_iterator ci = floor_access->getQuads().begin(); ci != floor_access->getQuads().end(); ++ci)
		{
			const Quad& quad = *ci;
			ConvexPolygon* bc = new ConvexPolygon(*floor_access_entity,
                                                glm::vec3(quad.bottom_left_.x, -floor_access->getDepth() / 2.0f, quad.bottom_left_.y),
                                                glm::vec3(quad.bottom_right_.x, -floor_access->getDepth() / 2.0f, quad.bottom_right_.y),
                                                glm::vec3(quad.top_left_.x, -floor_access->getDepth() / 2.0f, quad.top_left_.y),
                                                glm::vec3(quad.top_right_.x, -floor_access->getDepth() / 2.0f, quad.top_right_.y),
                                                glm::vec3(quad.bottom_left_.x, floor_access->getDepth() / 2.0f, quad.bottom_left_.y),
                                                glm::vec3(quad.bottom_right_.x, floor_access->getDepth() / 2.0f, quad.bottom_right_.y),
                                                glm::vec3(quad.top_left_.x, floor_access->getDepth() / 2.0f, quad.top_left_.y),
                                                glm::vec3(quad.top_right_.x, floor_access->getDepth() / 2.0f, quad.top_right_.y)
                                                );
			floor_access_entity->addCollision(*bc);
		}
		//SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*floor_access_entity, NULL, *floor_access, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* floor_access_leaf_node = new SceneLeafModel(*floor_access_entity, NULL, *floor_access, *concrete_material_, BasicShadowShader::getShader(), false, false);

		//if (floor_nr == 0)// || floor_nr == 2)
		{
			Cube* light_location = new Cube(0.1f, 0.1f, 0.1f);

			Entity* light_entity = new Entity(*scene_manager_, current_ground_floor, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(4.0f, 3.0f + 0.1f, 4.25f)), -90.0f, glm::vec3(1.0f, 0.0f, 0.0f)), OBSTACLE, ss.str());
			//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stepping_stone, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
			SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*light_entity, NULL, *light_location, *bright_material, BasicShadowShader::getShader(), false, false);

			SceneNode* light_node = new SceneNode(*scene_manager_, current_ground_floor, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(4.0f, 3.0f, 4.25f)), -90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
			PointLight* point_light = new PointLight(*scene_manager_, 65.0f, glm::vec3(0.1f, 0.1f, 0.1f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
			SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);
		}


		// Create the portals.
		glm::mat4 region_transform_matrix;
		SceneNode* current_scene_node = floor_access_entity;
		while (current_scene_node != new_current_region_node)
		{
			region_transform_matrix = current_scene_node->getLocalTransformation() * region_transform_matrix;
			current_scene_node = current_scene_node->getParent();
		}

		for (std::vector<Hole>::const_iterator ci = floor_access->getHoles().begin(); ci != floor_access->getHoles().end(); ++ci)
		{
			const Hole& hole = *ci;
			glm::vec4 near_left = region_transform_matrix * glm::vec4(hole.x_ - hole.width_ / 2.0f, 0.2f, hole.y_ - hole.height_ / 2.0f, 1.0f);
			glm::vec4 near_right = region_transform_matrix * glm::vec4(hole.x_ + hole.width_ / 2.0f, 0.2f, hole.y_ - hole.height_ / 2.0f, 1.0f);
			glm::vec4 far_left = region_transform_matrix * glm::vec4(hole.x_ - hole.width_ / 2.0f, 0.2f, hole.y_ + hole.height_ / 2.0f, 1.0f);
			glm::vec4 far_right = region_transform_matrix * glm::vec4(hole.x_ + hole.width_ / 2.0f, 0.2f, hole.y_ + hole.height_ / 2.0f, 1.0f);

			glm::vec3 diff = new_current_region->getSceneNode().getGlobalLocation() - current_region->getSceneNode().getGlobalLocation();

			glm::vec4 lower_near_left = region_transform_matrix * glm::vec4(glm::vec3(hole.x_ - hole.width_ / 2.0f, 0.2f, hole.y_ - hole.height_ / 2.0f) - diff, 1.0f);
			glm::vec4 lower_near_right = region_transform_matrix * glm::vec4(glm::vec3(hole.x_ + hole.width_ / 2.0f, 0.2f, hole.y_ - hole.height_ / 2.0f) - diff, 1.0f);
			glm::vec4 lower_far_left = region_transform_matrix * glm::vec4(glm::vec3(hole.x_ - hole.width_ / 2.0f, 0.2f, hole.y_ + hole.height_ / 2.0f) - diff, 1.0f);
			glm::vec4 lower_far_right = region_transform_matrix * glm::vec4(glm::vec3(hole.x_ + hole.width_ / 2.0f, 0.2f, hole.y_ + hole.height_ / 2.0f) - diff, 1.0f);

			std::vector<glm::vec3> portal_to_upper_level;
			portal_to_upper_level.push_back(glm::vec3(lower_near_left));
			portal_to_upper_level.push_back(glm::vec3(lower_near_right));
			portal_to_upper_level.push_back(glm::vec3(lower_far_right));
			portal_to_upper_level.push_back(glm::vec3(lower_far_left));
			Portal& upper_portal = current_region->addPortalToOtherRegion(*new_current_region, portal_to_upper_level);

			std::vector<glm::vec3> portal_to_lower_level;
			portal_to_lower_level.push_back(glm::vec3(far_left));
			portal_to_lower_level.push_back(glm::vec3(far_right));			
			portal_to_lower_level.push_back(glm::vec3(near_right));
			portal_to_lower_level.push_back(glm::vec3(near_left));
			Portal& lower_portal = new_current_region->addPortalToOtherRegion(*current_region, portal_to_lower_level);
			
			upper_portal.setMirrorPortal(lower_portal);
			lower_portal.setMirrorPortal(upper_portal);
		}
		
		ss.str(std::string());
		ss << "Door: " << floor_nr;
		// Create a door.
		Entity* door_entity = new Entity(*scene_manager_, current_ground_floor, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 4.0f)), OBSTACLE, "door" + ss.str());
		ConvexPolygon* door_bc = new ConvexPolygon(*door_entity, 0.3f, 2.0f, 1.0f);

		RotateBehaviour* door_rotation = new RotateBehaviour(*door_entity, glm::vec3(0.0f, 0.0f, -0.5f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f, 0.0f, 10.0f);
		door_entity->addBehaviour(*door_rotation);

		door_entity->addCollision(*door_bc, *bright_material, BasicShadowShader::getShader());
		//SceneLeafModel* wall_leaf_node = new SceneLeafModel(*door_entity, NULL, *door, *concrete_material_, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* wall_leaf_node = new SceneLeafModel(*door_entity, NULL, *door, *concrete_material_, BasicShadowShader::getShader(), false, false);
		
		current_ground_floor = floor_access_entity;
		current_region = new_current_region;
		}
	}
	

	// The player.
	player_ = new ArmedPlayer(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(5.0f, 15.0f, 5.0f)), 1.4f, *terrain_node, *terrain, *scene_manager_, *grass_texture);
	scene_manager_->addUpdateableEntity(*player_);

	// Add a camera.
	//camera_ = new Camera(*scene_manager_, player_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.7f, 0.0f)));
	camera_ = new FreeMovingCamera(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.7f, 0.0f)), 90.0f, 1024, 768, 0.1f, 300.0f);

	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, player_,  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.2f, -0.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);
	}

	// Initialise the sky box to render.
	MaterialLightProperty* skybox_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* skybox_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* skybox_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* skybox_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	Material* skybox_material = new Material(*skybox_ambient, *skybox_diffuse, *skybox_specular, *skybox_emmisive);
	Texture* skybox_texture = TargaTexture::loadTexture("data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");
	skybox_material->addCubeTexture(*skybox_texture);

	//skybox_material->addCubeTexture(28);
	SkyBox* inverted_cube = new SkyBox(5.0f);
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*camera_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material);//, "data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");

	// Setup the GUI for the FPS.
	MyGUITheme* theme = new MyGUITheme();
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");
	
	Font* font = new TexturedFont(*font_texture);
	GUIManager& gui_manager = GUIManager::getInstance();
	
	Container* fps_container = new Container(*theme, font->clone(), 10, 10, 120, 20, false);
	Label* fps_label = new Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FPSLabel(*fps_label);

	gui_manager.addFrame(*fps_container);


	return true;
}

bool FlatDemo::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
}

GLuint FlatDemo::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	fps_label_->frameRendered();
	glfwSetMousePos(width_ / 2, height_ / 2);
	return 0;
}

void FlatDemo::tick(float dt)
{

}

void FlatDemo::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_->onResize(width, height);
}

/*
void FlatDemo::generateFloor(Entity& parent)
{
	// Ground floor.
	for (unsigned int i = 0; i < 20; ++i)
	{
		Entity* stair_step_entity = new Entity(&parent, glm::translate(glm::mat4(1.0), glm::vec3(0.5f * i - 5.0f, 0.2f * i, 0.0f)), ENTITY_TYPE::OBSTACLE, *game_world_);
		ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 0.5f, 0.2f, 2.0f);
		stair_step_entity->addCollision(*stair_step_bc, *bright_material, BasicShadowShader::getShader());
		game_world_->addEntity(*stair_step_entity);
		SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *concrete_material_, BasicShadowShader::getShader(), false, false);
	}

	// Stair case.
	{
		Entity* stair_entity_ = new Entity(parent, glm::rotate(glm::translate(glm::mat4(1.0), glm::vec3(5.0f, 2.2f, 2.0f)), 15.0f, glm::vec3(0.0f, 0.0f, 1.0f)), ENTITY_TYPE::OBSTACLE, *game_world_);
		ConvexPolygon* stair_bc = new ConvexPolygon(*stair_entity_, 50.0f, 0.5f, 2.0f);
		stair_entity_->setCollision(*stair_bc);
		game_world_->addEntity(*stair_entity_);
		SceneLeafModel* stair_leaf_node = new SceneLeafModel(*stair_entity_, NULL, *stair, *concrete_material_, BasicShadowShader::getShader(), false, false);
	}
}
*/
