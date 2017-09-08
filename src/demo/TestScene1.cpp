#include "TestScene1.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../effects/Reflection.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"
#include "../effects/GodRays.h"
//#include "../core/gui/FreeTypeFont.h"

#include "../core/scene/SceneLeafLight.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/renderer/SimpleRenderer.h"
#include "../core/scene/SkyBoxLeaf.h"
#include "../shapes/terrain.h"
#include "../core/scene/Material.h"
#include "../core/shaders/TerrainShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/WaterShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "../core/entities/Player.h"
#include "../core/collision/ConvexPolygon.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"
#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"

TestScene::TestScene(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool TestScene::init(int argc, char** argv)
{
#ifdef _WIN32
	if (!GLEW_VERSION_4_3)
	{
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
        std::cerr << "VBOs are not supported by your graphics card" << std::endl;

        MessageBox(NULL, "VBOs are not supported by your graphics card!", "An error occurred", MB_ICONERROR | MB_OK);
        return false;
	}
#endif
	Texture* water_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");
	Texture* beech_texture = TargaTexture::loadTexture("data/textures/beech.tga");
	Texture* light_texture = TargaTexture::loadTexture("data/textures/light_point.tga");
	Texture* coin_texture = TargaTexture::loadTexture("data/textures/gold.tga");

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

	Material* terrain_material = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material->add1DTexture(*height_texture);
	terrain_material->add2DTexture(*grass_texture);

	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node, NULL, *terrain, *terrain_material, TerrainShader::getShader(), false, false);

	// Add some trees.
	Tree* tree = new Tree();
	MaterialLightProperty* tree_ambient = new MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	MaterialLightProperty* tree_diffuse = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* tree_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* tree_emmisive = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);

	Material* tree_material = new Material(*tree_ambient, *tree_diffuse, *tree_specular, *tree_emmisive);
	tree_material->add2DTexture(*beech_texture);
	//SphereCheck* tree_frustrum_check = new SphereCheck(0.5f);
	//sphere_check_ = tree_frustrum_check;

	MaterialLightProperty* bright_ambient = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(0.0, 1.0, 1.0, 1.0);

	Material* bright_material = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	bright_material->add2DTexture(*water_texture);

	for (unsigned int i = 0; i < 15; )
	{
		//float tree_x = 54.0f;
		//float tree_z = 54.0f;
		//float height = 10.0f;
		float tree_x = (rand() / (float)RAND_MAX) * 64;
		float tree_z = (rand() / (float)RAND_MAX) * 64;
		float height = terrain->getHeight(tree_x, tree_z);

		//LinearAnimation* linear_animation = new LinearAnimation(tree_node, glm::vec3(0, 0, 0), 0, 40, 0);

		if (height > 7.0f)
		{
			Entity* tree_node = new Entity(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(tree_x - terrain->getWidth() / 2, height + 1.0f, tree_z - terrain->getWidth() / 2)), OBSTACLE, "tree");

			ConvexPolygon* bc = new ConvexPolygon(*tree_node, 1.0f, 1.0f, 1.0f);
			tree_node->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

			SceneLeafModel* tree_leaf_node = new SceneLeafModel(*tree_node, NULL, *tree, *tree_material, BasicShadowShader::getShader(), false, true);
			leaf_ = tree_leaf_node;
			//linear_animation->addLeaf(*tree_leaf_node);

			++i;
		}
	}

	// Add the water.
	// TODO: Remove the height from water and add it to the scene node.
	Water* water = new Water(65, 65, 6.0f);
	MaterialLightProperty* water_ambient = new MaterialLightProperty(0.2, 0.2, 0.2, 1.0);
	MaterialLightProperty* water_diffuse = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* water_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* water_emmisive = new MaterialLightProperty(0.1, 0.1, 0.1, 1.0);

	Material* water_material = new Material(*water_ambient, *water_diffuse, *water_specular, *water_emmisive);
	water_material->add2DTexture(*water_texture);

	WavingWater* water_node = new WavingWater(*scene_manager_, terrain_node, glm::mat4(1.0f), *water);
	SceneLeafModel* water_leaf_node = new SceneLeafModel(*water_node, NULL, *water, *water_material, WaterShader::getShader(), true, true);

	MaterialLightProperty* geometry_ambient = new MaterialLightProperty(0.2, 0.2, 0.2, 1.0);
	MaterialLightProperty* geometry_diffuse = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* geometry_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* geometry_emmisive = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);

	Material* geometry_material = new Material(*geometry_ambient, *geometry_diffuse, *geometry_specular, *geometry_emmisive);
	geometry_material->add2DTexture(*grass_texture);

	/*
	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 70.0f, 7);
		SceneNode* light_node = new SceneNode(&scene_manager_->getRoot(),  glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 25.0f, 0.0f)), -90.0f, glm::vec3(1.0f, 0.0f, 0.0f)));
		//SceneNode* light_node = new SceneNode(player_node, glm::mat4(1.0f));

		SceneLeafLight* light_leaf = new SceneLeafLight(*point_light);

		//Cube* light_cube = new Cube(5.0f);
		//RenderableEntity* renderable_cube = new RenderableEntity(*light_node, *light_cube, ENTITY_TYPE::OBSTACLE);
		//game_world_.addEntity(*renderable_cube);

		//SceneLeafModel* light_piramid_leaf = new SceneLeafModel(*light_cube, *bright_material, BasicShadowShader::getShader(), false, false);
		light_node->addLeaf(*light_leaf);
		//light_node->addLeaf(*light_piramid_leaf);

		// Test some animation.
		//BouncingBox* bb = new BouncingBox(-12, 3, -12, 12, 15, 12, *light_node);
		//game_world_.addEntity(*bb);
	}
	*/

	/*
	// Add some lighting.
	for (unsigned int i = 0; i < 0; ++i)
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, 6 + i);
		SceneNode* light_node = new SceneNode(&scene_manager_->getRoot(), glm::mat4(1.0f));

		SceneLeafLight* light_leaf = new SceneLeafLight(*point_light);

		Cube* light_cube = new Cube(0.75f);
		RenderableEntity* renderable_cube = new RenderableEntity(*light_node, *light_cube, ENTITY_TYPE::OBSTACLE);
		game_world_.addEntity(*renderable_cube);

		SceneLeafModel* light_piramid_leaf = new SceneLeafModel(*light_cube, *bright_material, BasicShadowShader::getShader(), false, false);
		light_node->addLeaf(*light_leaf);
		light_node->addLeaf(*light_piramid_leaf);

		// Test some animation.
		BouncingBox* bb = new BouncingBox(-12, 3, -12, 12, 15, 12, *light_node);
		game_world_.addEntity(*bb);
	}
	*/

	MaterialLightProperty* coin_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* coin_diffuse = new MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	MaterialLightProperty* coin_specular = new MaterialLightProperty(0.4, 0.4, 0.4, 1.0);
	MaterialLightProperty* coin_emmisive = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);

	Material* coin_material = new Material(*coin_ambient, *coin_diffuse, *coin_specular, *coin_emmisive);
	coin_material->add2DTexture(*coin_texture);
	
	// Some random geometry.
	Sphere* sphere = new Sphere(25, 25, 1);
	//SphereCheck* sphere_frustrum_check = new SphereCheck(1.0f);
	std::vector<glm::vec3> locations_;
	locations_.push_back(glm::vec3(15.0f, -1.0f, 5.0f));
	locations_.push_back(glm::vec3(0.0f, 7.0f, -12.0f));
	locations_.push_back(glm::vec3(-12.0f, -1.0f, -15.0f));
	locations_.push_back(glm::vec3(-3.0f, -1.0f, 20.0f));
	locations_.push_back(glm::vec3(0.0f, 17.0f, 0.0f));

	for (std::vector<glm::vec3>::iterator ci = locations_.begin(); ci != locations_.end(); ++ci)
	{
		glm::vec3& loc = *ci;
		if (loc.y == -1.0f) loc.y = terrain->getHeight(loc.x + terrain->getWidth() / 2, loc.z + terrain->getWidth() / 2) + 2.5f;
		//float x = (rand() / (float)RAND_MAX) * 64;
		//float z = (rand() / (float)RAND_MAX) * 64;
		//float y = (rand() / (float)RAND_MAX) * 10 + 3;

		//float x = 35.0f;
		//float z = 39.0f;
		//float y = terrain->getHeight(x, z) + 2.5f;
		//SceneNode* geometry_node = new SceneNode(terrain_node, glm::translate(glm::mat4(1.0), loc));
		//BouncingBox* bb = new BouncingBox(terrain_node,glm::translate(glm::mat4(1.0), loc)-32, 3, -32, 32, 15, 32, *geometry_node);
		//game_world_.addEntity(*bb);

		Entity* renderable_cube = new Entity(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), loc), COIN, "coin");

		//SphereCheck* sphere_frustrum_check = new SphereCheck(*renderable_cube, 1.0f);

		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 1.0f, 1.0f, 1.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *sphere, *coin_material, BasicShadowShader::getShader(), false, false);
	}
	
	//SphereCheck* cube_frustrum_check = new SphereCheck(1.5f);
	Cube* normal_cube = new Cube(2.0f, 0.5f, 2.0f);
	{
		BouncingBox* bb = new BouncingBox(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(-6.0f, 8.0f, 4.0)), -15, 3, -32, 0, 15, 32, glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(-6.0f, 8.0f, 4.0));
		Entity* renderable_cube = new Entity(*scene_manager_, bb, glm::mat4(1.0), OBSTACLE, "bounded box");

		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 2.0f, 0.5f, 2.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *normal_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		scene_manager_->addUpdateableEntity(*bb);
	}
	{
		BouncingBox* bb = new BouncingBox(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(-12.0f, 7.0f, 0.0)), -15, 3, -3.5, 0, 15, 3, glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(-12.0f, 7.0f, 0.0));
		Entity* renderable_cube = new Entity(*scene_manager_, bb, glm::mat4(1.0), OBSTACLE, "bounded box");

		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 2.0f, 0.5f, 2.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *normal_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		scene_manager_->addUpdateableEntity(*bb);
	}
	{
		BouncingBox* bb = new BouncingBox(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(-5.0f, 7.0f, 8.0)), -15, 7, -32, 0, 15, 32, glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(-5.0f, 7.0f, 8.0));
		Entity* renderable_cube = new Entity(*scene_manager_, bb, glm::mat4(1.0), OBSTACLE, "bounded box");

		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 2.0f, 0.5f, 2.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *normal_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		scene_manager_->addUpdateableEntity(*bb);
	}

	{
		BouncingBox* bb = new BouncingBox(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 15.0f, 8.0)), -5, 10, 0, 5, 20, 12, glm::vec3(0.5f, 0.0f, 0.5f), glm::vec3(0.0f, 15.0f, 8.0));
		Entity* renderable_cube = new Entity(*scene_manager_, bb, glm::mat4(1.0), OBSTACLE, "bounded box");

		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 2.0f, 0.5f, 2.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *normal_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		scene_manager_->addUpdateableEntity(*bb);
	}

	//SphereCheck* bridge_frustrum_check = new SphereCheck(12.0f);
	// Add a lever and a bridge.
	{
		Cube* lever_shape = new Cube(0.25f, 1.0f, 0.25f);
		Cube* base_shape = new Cube(0.5f, 0.25f, 0.5f);
		Sphere* knob_shape = new Sphere(10, 10, 0.25);

		//SceneNode* base_node = new SceneNode(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(12.0f, 7.75f, -12.0f)));
		//SceneNode* lever_node = new SceneNode(base_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.5f, -0.0f)));
		//SceneNode* knob_node = new SceneNode(lever_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.5f, 0.0f)));

		Entity* base = new Entity(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(12.0f, 7.75f, -12.0f)), OBSTACLE, "base");

		SceneLeafModel* base_leaf_node = new SceneLeafModel(*base, NULL, *base_shape, *geometry_material, BasicShadowShader::getShader(), false, false);

		Lever* lever = new Lever(*scene_manager_, base, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.5f, -0.0f)), OBSTACLE);

		ConvexPolygon* bc = new ConvexPolygon(*lever, 0.5f, 1.0f, 0.5f);
		lever->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* lever_leaf_node = new SceneLeafModel(*lever, NULL, *lever_shape, *geometry_material, BasicShadowShader::getShader(), false, false);

		Entity* knob = new Entity(*scene_manager_, lever, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 0.5f, 0.0f)), OBSTACLE, "knob");

		SceneLeafModel* knob_leaf_node = new SceneLeafModel(*knob, NULL, *knob_shape, *geometry_material, BasicShadowShader::getShader(), false, false);

		Bridge* bridge_ = new Bridge(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(-4.0f, 6.0f, -12.0f)), *lever, OBSTACLE, glm::vec3(-4.0f, 6.0f, -12.0f), glm::vec3(4.0f, 6.0f, -12.0f));

		Cube* bridge = new Cube(12.0f, 0.5f, 4.0f);
		//SceneNode* geometry_node = new SceneNode(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(-4.0f, 6.0f, -12.0f)));
		//bridge_ = new Bridge(*lever, *geometry_node, ENTITY_TYPE::OBSTACLE, glm::vec3(-4.0f, 6.0f, -12.0f), glm::vec3(4.0f, 6.0f, -12.0f));

		ConvexPolygon* bridge_bc = new ConvexPolygon(*bridge_, 12.0f, 0.5f, 4.0f);
		bridge_->addCollision(*bridge_bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*bridge_, NULL, *bridge, *geometry_material, BasicShadowShader::getShader(), false, false);
	}

	//SphereCheck* large_frustrum_check = new SphereCheck(5.0f);
	Cube* large_cube = new Cube(3.0f, 1.0f, 3.0f);
	//for (unsigned int i = 0; i < 10; ++i)
	{
		float x = 0.0f;
		float y = 4.0f;
		float z = 0.0f;
		//float x = (rand() / (float)RAND_MAX) * 64;
		//float z = (rand() / (float)RAND_MAX) * 64;
		//float y = terrain->getHeight(x, z) + 2.0f;

		//SceneNode* geometry_node = new SceneNode(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(x - terrain->getWidth() / 2, y, z - terrain->getWidth() / 2)));
		SceneNode* geometry_node = new SceneNode(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(x, y, z)));
		
		LinearAnimation* la = new LinearAnimation(*scene_manager_, geometry_node, glm::vec3(0.0f, 0.0f, 0.0f), 0, 20, 0);
		
		Entity* renderable_cube = new Entity(*scene_manager_, la, glm::mat4(1.0), OBSTACLE, "large cube");
		//Entity* renderable_cube = new Entity(*scene_manager_, geometry_node, glm::rotate(glm::mat4(1.0f), 170.0f, glm::vec3(0.0f, 1.0f, 0.0f)), OBSTACLE, "large cube");
		
		ConvexPolygon* bc = new ConvexPolygon(*renderable_cube, 3.0f, 1.0f, 3.0f);
		renderable_cube->addCollision(*bc, *bright_material, BasicShadowShader::getShader());

		SceneLeafModel* leaf_node = new SceneLeafModel(*la, NULL, *large_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		//SceneLeafModel* leaf_node = new SceneLeafModel(*renderable_cube, NULL, *large_cube, *geometry_material, BasicShadowShader::getShader(), false, false);
		scene_manager_->addUpdateableEntity(*la);
	}
	
	// The player.
	player_ = new Player(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0, 10, 0)), 1.4f, *terrain_node, *terrain, *scene_manager_);
	camera_ = new Camera(*scene_manager_, player_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.7f, 0.0f)));
	
	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, player_,  glm::translate(glm::mat4(1.0f), glm::vec3(0.30f, 0.2f, 0.3f)));
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
	
	SkyBox* inverted_cube = new SkyBox(5.0f);
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*camera_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material);

	return true;
}

bool TestScene::checkError(const std::string& description) const
{
	
	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
	{
		std::ofstream file("errors2.txt");
		file << description << std::endl;
		file << "Error:" << glGetError() << std::endl;
		file.close();
#ifdef _WIN32
        MessageBox(NULL, description.c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
		exit(1);
		return false;
	}
	
	return true;
}
