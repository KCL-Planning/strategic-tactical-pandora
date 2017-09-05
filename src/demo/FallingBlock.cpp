#include "FallingBlock.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "../core/entities/camera/FreeMovingCamera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../shapes/Line.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"

#include "../core/scene/SceneLeafLight.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/scene/SkyBoxLeaf.h"
#include "../core/scene/portal/Region.h"
#include "../shapes/terrain.h"
#include "../core/scene/Material.h"
#include "../core/shaders/TerrainShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/WaterShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/shaders/ShadowShader.h"
#include "../core/shaders/AnimatedShadowShader.h"
#include "../core/shaders/LineShader.h"
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "../core/entities/Player.h"
#include "../core/collision/BoxCollision.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"

#include "../core/entities/behaviours/RotateBehaviour.h"

#include "../core/loaders/WavefrontLoader.h"

#include "../core/loaders/AssimpLoader.h"

#include "../core/texture/TargaTexture.h"
#include "../core/texture/Texture.h"

#include "../core/loaders/PortalLevelFormatLoader.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

#include "shooter/ArmedPlayer.h"

#include "../core/loaders/AssimpLoader.h"
#include "../core/entities/Monster.h"

#include "../core/ai/pathfinding/NavMeshAStar.h"
#include "../core/ai/pathfinding/NavigationMesh.h"
#include "../core/ai/pathfinding/ConvexNavigationArea.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Label.h"
#include "../core/gui/Container.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "gui_demo/FPSLabel.h"
#include "physical_sim/PhysicsCube.h"

FallingBlock::FallingBlock(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool FallingBlock::init(int argc, char** argv)
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
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

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

	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");
	
	MaterialLightProperty* bright_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(1.0, 0.2, 1.0, 1.0);

	Material* bright_material_ = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	bright_material_->add2DTexture(*grass_texture);
	
	glLineWidth(3.0f);
	glEnable(GL_LINE_SMOOTH);

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	Material* terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);

	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, TerrainShader::getShader(), false, false);
	
	
	Cube* rotating_cube = new Cube(5, 0.5, 5);
	rotating_cube_entity_ = new Entity(*scene_manager_, terrain_node_, glm::rotate(glm::translate(glm::mat4(1.0), glm::vec3(10.0f, 1.0f, -10.0f)), 90.0f, glm::vec3(0, 1, 0)), OBSTACLE, "Rotating Cube");
	BoxCollision* stair_step_bc = new BoxCollision(*rotating_cube_entity_, 5.0f, 0.5f, 5.0f);
	rotating_cube_entity_->addCollision(*stair_step_bc);
	Behaviour* behaviour = new RotateBehaviour(*rotating_cube_entity_, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0), 0, 180, 90, 10);
	rotating_cube_entity_->addBehaviour(*behaviour);
	SceneLeafModel* rotating_cube_leaf = new SceneLeafModel(*rotating_cube_entity_, NULL, *rotating_cube, *bright_material_, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*rotating_cube_entity_);
	scene_manager_->addPlayer(*rotating_cube_entity_);
	
	cube_ = new PhysicsCube(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(12.0f, 5.0f, -12.0f)));
	scene_manager_->addUpdateableEntity(*cube_);
	scene_manager_->addPlayer(*cube_);
	
	
	// Initialise the camera:
	camera_ = new FreeMovingCamera(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.7f, 2.0f)), 90.0f, 1024, 768, 0.1f, 60.0f);
	//camera_ = new DynamicCamera(*terrain_node_, *scene_manager_, cube_, glm::mat4(1.0f), 90, 1024, 768, 0.1f, 60.0f);
	scene_manager_->addUpdateableEntity(*camera_);
	
	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, camera_,  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.2f, -0.25f)));
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

bool FallingBlock::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
}

void FallingBlock::tick(float dt)
{
	if (glfwGetKey('E') == GLFW_PRESS)
	{
		cube_->setTransformation(glm::translate(cube_->getLocalTransformation(), glm::vec3(0, 1, 0)));
		cube_->updateTransformations();
		cube_->transition(terrain_node_);
	}
	if (glfwGetKey('R') == GLFW_PRESS)
	{
		rotating_cube_entity_->activate(*cube_);
	}
}

GLuint FallingBlock::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	fps_label_->frameRendered();
	glfwSetMousePos(width_ / 2, height_ / 2);
	return 0;
}

void FallingBlock::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_->onResize(width, height);
}
