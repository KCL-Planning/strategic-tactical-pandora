#include "FrustumCullingExample.h"

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
//#include "../shapes/GingerMan.h"
//#include "../core/loaders/MD2Loader.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"
//#include "../core/gui/FreeTypeFont.h"

#include "../core/scene/SceneLeafLight.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/scene/SkyBoxLeaf.h"
#include "../shapes/terrain.h"
#include "../core/scene/Material.h"
#include "../core/shaders/TerrainShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/WaterShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/shaders/ShadowShader.h"
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "../core/entities/Player.h"
#include "../core/collision/ConvexPolygon.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"

#include "../core/entities/behaviours/RotateBehaviour.h"

#include "../core/loaders/WavefrontLoader.h"

#include "../core/loaders/AssimpLoader.h"

#include "flat/Wall.h"

FrustumCullingExample::FrustumCullingExample(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool FrustumCullingExample::init(int argc, char** argv)
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

	// Load some textures.
	// * Water texture.
	// * Grass texture.
	glGenTextures(7, textures);

	// Coin texture.
	glActiveTexture(GL_TEXTURE15);
	glBindTexture(GL_TEXTURE_2D, textures[5]);

	TargaImage targa_loader6;
	if (targa_loader6.load("data/textures/chair_uv.tga"))
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, targa_loader6.getWidth(), targa_loader6.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, targa_loader6.getImageData());
	}
	else
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialise the chair uv!", "Error", MB_OK);
#endif
	}

		
	// Light texture.
	glActiveTexture(GL_TEXTURE28);
	glBindTexture(GL_TEXTURE_2D, textures[4]);
	
	TargaImage targa_loader5;
	if (targa_loader5.load("data/textures/table_uv.tga"))
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, targa_loader5.getWidth(), targa_loader5.getHeight(), GL_RGBA, GL_UNSIGNED_BYTE, targa_loader5.getImageData());
	}
	else
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not initialise the table uv!", "Error", MB_OK);
#endif
	}

	// Water texture.
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textures[0]);

	TargaImage targa_loader;
	if (!targa_loader.load("data/textures/water.tga"))
	{
#ifdef _WIN32
       MessageBox(NULL, "Failed to load the water texture!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, targa_loader.getWidth(), targa_loader.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, targa_loader.getImageData());
	
	// Grass texture.
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textures[1]);

	TargaImage targa_loader2;
	if (!targa_loader2.load("data/textures/grass.tga"))
	{
#ifdef _WIN32
       MessageBox(NULL, "Failed to load the grass texture!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB8, targa_loader2.getWidth(), targa_loader2.getHeight(), GL_RGB, GL_UNSIGNED_BYTE, targa_loader2.getImageData());

	// Height texture.
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, textures[2]);

	TargaImage targa_loader3;
	if (!targa_loader3.load("data/textures/height.tga"))
	{
#ifdef _WIN32
       MessageBox(NULL, "Failed to load the height texture!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB8, targa_loader3.getWidth(), 0, GL_RGB, GL_UNSIGNED_BYTE, targa_loader3.getImageData());
	
	// Tree texture.
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_1D, textures[3]);
	
	TargaImage targa_loader4;
	if (!targa_loader4.load("data/textures/beech.tga"))
	{
#ifdef _WIN32
       MessageBox(NULL, "Failed to load the beech texture!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
	}
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, targa_loader4.getWidth(), targa_loader4.getHeight(), GL_RGBA, GL_UNSIGNED_BYTE, targa_loader4.getImageData());

	/*
	glActiveTexture(GL_TEXTURE10);
	TargaImage targa_loader7;
	if (!targa_loader7.load("data/textures/flashlight.tga"))
	{
		MessageBox(NULL, "Could not initialise the flashlight!", "Error", MB_OK);
	}
	*/
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
	//GingerMan* ginger_man = new GingerMan("models/gingerman.md2", *scene_manager_, terrain_node, glm::mat4(1.0f), OBSTACLE, "Gingerman");

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(2);
	terrain_material_->add2DTexture(1);

	//SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node, NULL, *terrain, *terrain_material_, TerrainShader::getShader(), false, false);

	// Create a flat box as the floor.
	MaterialLightProperty* concrete_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* concrete_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* concrete_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* concrete_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	concrete_material_ = new Material(*concrete_ambient, *concrete_diffuse, *concrete_specular, *concrete_emmisive);
	concrete_material_->add2DTexture(1);

	MaterialLightProperty* bright_ambient = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(0.0, 1.0, 1.0, 1.0);

	Material* bright_material = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	bright_material->add2DTexture(1);

	stair_step_ = new Cube(0.5f, 0.3f, 1.0f);
	Entity* parent = terrain_node;
	for (unsigned int i = 0; i < 10; ++i)
	{
		float x = 1;//((float)(rand()) / RAND_MAX) * 30.0f;
		float y = 1;//((float)(rand()) / RAND_MAX) * 10.0f;
		float z = 1;//((float)(rand()) / RAND_MAX) * 30.0f;

		Entity* stair_step_entity = new Entity(*scene_manager_, parent, glm::translate(glm::mat4(1.0), glm::vec3(x, y, z)), OBSTACLE, "stair");
		ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 0.5f, 0.3f, 1.0f);
		stair_step_entity->addCollision(*stair_step_bc);
		//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *bright_material, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *bright_material, BasicShadowShader::getShader(), false, false);

		parent = stair_step_entity;
		if (i == 0)
		{
			stair_step_entity_ = stair_step_entity;
		}
	}

	parent = terrain_node;
	for (unsigned int i = 0; i < 10; ++i)
	{
		float x = 1;//((float)(rand()) / RAND_MAX) * 30.0f;
		float y = 1;//((float)(rand()) / RAND_MAX) * 10.0f;
		float z = 1;//((float)(rand()) / RAND_MAX) * 30.0f;

		Entity* stair_step_entity = new Entity(*scene_manager_, parent, glm::translate(glm::mat4(1.0), glm::vec3(x, y, z + 10)), OBSTACLE, "stair");
		ConvexPolygon* stair_step_bc = new ConvexPolygon(*stair_step_entity, 0.5f, 0.3f, 1.0f);
		stair_step_entity->addCollision(*stair_step_bc);
		//SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *bright_material, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
		SceneLeafModel* stair_step_leaf_node = new SceneLeafModel(*stair_step_entity, NULL, *stair_step_, *bright_material, BasicShadowShader::getShader(), false, false);

		parent = stair_step_entity;
		if (i == 0)
		{
			stair_step_entity_ = stair_step_entity;
		}
	}

	// Load the table.
	Shape* table = WavefrontLoader::importShape("models/table.obj");
	if (table == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not import table.", "Error", MB_OK);
#endif
	}
	MaterialLightProperty* table_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* table_diffuse = new MaterialLightProperty(0.7, 0.7, 0.7, 1.0);
	MaterialLightProperty* table_specular = new MaterialLightProperty(0.2, 0.2, 0.2, 1.0);
	MaterialLightProperty* table_emmisive = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);

	Material* table_material = new Material(*table_ambient, *table_diffuse, *table_specular, *table_emmisive);
	table_material->add2DTexture(28);

	Entity* table_entity = new Entity(*scene_manager_, terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0, 2, 0)), OBSTACLE, "table");
	//SceneLeafModel* table_leaf_node = new SceneLeafModel(*table_entity, NULL, *table, *table_material, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
	SceneLeafModel* table_leaf_node = new SceneLeafModel(*table_entity, NULL, *table, *table_material, BasicShadowShader::getShader(), false, false);
	
	// Load the chair.
	Shape* chair = WavefrontLoader::importShape("models/chair.obj");
	if (chair == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not import chair.", "Error", MB_OK);
#endif
	}
	MaterialLightProperty* chair_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* chair_diffuse = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);
	MaterialLightProperty* chair_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* chair_emmisive = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);

	Material* chair_material = new Material(*chair_ambient, *chair_diffuse, *chair_specular, *chair_emmisive);
	chair_material->add2DTexture(15);

	Entity* chair_entity = new Entity(*scene_manager_, terrain_node, glm::scale(glm::translate(glm::mat4(1.0), glm::vec3(5, 2, 0)), glm::vec3(0.7f, 0.7f, 0.7f)), OBSTACLE, "chair");
	//SceneLeafModel* chair_leaf_node = new SceneLeafModel(*chair_entity, NULL, *chair, *chair_material, BasicShadowShader::getShader(), &ShadowShader::getShader(), false, false);
	SceneLeafModel* chair_leaf_node = new SceneLeafModel(*chair_entity, NULL, *chair, *chair_material, BasicShadowShader::getShader(), false, false);
	
	// The player.
	player_ = new Player(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 10.0f, 0.0f)), 1.4f, *terrain_node, *terrain, *scene_manager_);

	camera_ = new Camera(*scene_manager_, player_, glm::mat4(1.0f));

	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, 7, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, player_,  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.2f, -0.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);
	}

	// Initialise the sky box to render.
	MaterialLightProperty* skybox_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* skybox_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* skybox_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* skybox_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	/*
	Material* skybox_material = new Material(*skybox_ambient, *skybox_diffuse, *skybox_specular, *skybox_emmisive);
	Texture* skybox_texture = new Texture("data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");
	Material* skybox_material = new Material(*skybox_ambient, *skybox_diffuse, *skybox_specular, *skybox_emmisive);
	skybox_material->addCubeTexture(skybox_texture->getTextureId());

	//skybox_material->addCubeTexture(12);
	SkyBox* inverted_cube = new SkyBox(5.0f);
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*player_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material, "data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");
	*/
	return true;
}
