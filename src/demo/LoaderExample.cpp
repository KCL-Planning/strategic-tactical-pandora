#include "LoaderExample.h"

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

#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

LoaderExample::LoaderExample(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool LoaderExample::init()
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

	glEnableVertexAttribArray(0); // Vertex.
	glEnableVertexAttribArray(1); // Colour.
	glEnableVertexAttribArray(2); // Texture.
	glEnableVertexAttribArray(3); // Normals.
	glEnableVertexAttribArray(4); // Bone weights.
	glEnableVertexAttribArray(5); // Bone ids.

	Texture::initialise();

	Texture* wfl_texture = TargaTexture::loadTexture("data/models/levels/modular/atlas.tga");
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");

	WavefrontLoader* wfl = new WavefrontLoader();
	Shape* shape = wfl->importShape("data/models/levels/modular/basic_elements4.obj");

	// Initialise the texture to use.
	
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*wfl_texture);

	//AssimpLoader* loader = new AssimpLoader();

	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/boblampclean.md5mesh");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/top_rot2.dae");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/simple_rot.dae");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/snake.dae");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/grunt_beast/grunt beast2.dae");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/levels/office/posh_office3.dae");
	//std::pair<AnimatedModel*, Material*> gingerman_properties = loader->LoadModel(*scene_manager_, "data/models/levels/modular/basic_elements4.dae");
	//snake_ = gingerman_properties.first;

	Terrain* terrain = new Terrain();
	if (!terrain->createHeightmap(129, 0.95f))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the terrain heightmap", "Error", MB_OK);
#endif
		return false;
	}

	Entity* terrain_node = new Entity(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0), OBSTACLE, "terrain");

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);

	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node, NULL, *terrain, *terrain_material_, TerrainShader::getShader(), false, false);
	
	/*
	SceneNode* rotate_node = new SceneNode(*scene_manager_, terrain_node, glm::scale(glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 3.0f, 0.0f)), -90.0f, glm::vec3(1.0f, 0.0f, 0.0f)), glm::vec3(1.1f, 1.1f, 1.1f)));
	Entity* gingerman_entity = new Entity(*scene_manager_, rotate_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 5.0f, 0.0f)), OBSTACLE, "gingerman");
	SceneLeafModel* chair_leaf_node = new SceneLeafModel(*gingerman_entity, NULL, *gingerman_properties.first, *gingerman_properties.second, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*gingerman_entity);
	*/

	Entity* gingerman_entity = new Entity(*scene_manager_, terrain_node, glm::mat4(1.0), OBSTACLE, "waveobj");
	SceneLeafModel* chair_leaf_node = new SceneLeafModel(*gingerman_entity, NULL, *shape, *wfl_material_, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*gingerman_entity);
	
	// The player.
	player_ = new Player(terrain_node, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 10.0f, 0.0f)), 1.9f, *terrain_node, *terrain, *scene_manager_);

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
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*player_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material);//, "data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");

	return true;
}
