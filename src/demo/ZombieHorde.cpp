#include "ZombieHorde.h"

#include <glm/glm.hpp>

#include "../core/entities/camera/FreeMovingCamera.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/Material.h"
#include "../core/shaders/AnimatedShadowShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/loaders/AssimpLoader.h"
#include "../core/texture/TargaTexture.h"
#include "../core/texture/Texture.h"
#include "../core/scene/Material.h"
#include "../core/models/AnimatedModel.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/Container.h"
#include "../core/gui/Label.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "../core/gui/themes/MyGUITheme.h"

#include "../shapes/Cube.h"

#include "../shapes/SkyBox.h"
#include "../core/scene/SkyBoxLeaf.h"

#include "gui_demo/FPSLabel.h"

#include "pandora/models/Shark.h"

#include "instance_rendering/InstanceRenderedShape.h"
#include "instance_rendering/SceneLeafInstanced.h"
#include "instance_rendering/AnimatedInstanceShader.h"
#include "instance_rendering/InstanceRenderedSceneNode.h"

ZombieHorde::ZombieHorde(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	camera_node_ = new FreeMovingCamera(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_node_);

	// Setup the GUI.
	MyGUITheme* theme = new MyGUITheme();
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");

	Font* font = new TexturedFont(*font_texture);

	// Setup the GUI for the FPS.
	GUIManager& gui_manager = GUIManager::getInstance();
	
	Container* fps_container = new Container(*theme, font->clone(), 10, 10, 120, 20, false);
	gui_manager.addFrame(*fps_container);
	Label* fps_label = new Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FPSLabel(*fps_label);

	
	Cube* cube = new Cube(0.2f, 0.2f, 0.2f);
	single_instance_ = new InstanceRenderedShape(*cube);
	MaterialLightProperty* ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* emissive = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	Texture* cube_texture = TargaTexture::loadTexture("data/textures/grass.tga");

	Material* material = new Material(*ambient, *diffuse, *specular, *emissive);
	material->add2DTexture(*cube_texture);

	instance_rendered_node_ = new InstanceRenderedSceneNode(scene_manager, &scene_manager.getRoot(), glm::mat4(1.0f));
	SceneLeafInstanced* leaf = new SceneLeafInstanced(*instance_rendered_node_, InstanceShader::getShader(), *single_instance_, *material);
}

bool ZombieHorde::init(int argc, char** argv)
{
	
	glEnable(GL_DEPTH_TEST);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	float size = 9;
	for (float x = -size; x < size; ++x)
	{
		for (float y = -size; y < size; ++y)
		{
			for (float z = -size; z < size; ++z)
			{
				SceneNode* instance = new SceneNode(*scene_manager_, instance_rendered_node_, glm::translate(glm::mat4(1.0f), glm::vec3(x, y, z)));
				instance_rendered_node_->addChild(*instance);
			}
		}
	}
	
	return true;
}

bool ZombieHorde::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	/*
	// Lets add some sharks :)
	AssimpLoader* loader = new AssimpLoader();
	std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > shark_properties = loader->LoadModel(*scene_manager_, "data/models/Pandora/sea creatures/shark.dae");
	
	// Load the texture for the shark.
	Texture* shark_texture = TargaTexture::loadTexture("data/models/Pandora/sea creatures/shark.tga");
	
	MaterialLightProperty* shark_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* shark_diffuse = new MaterialLightProperty(0.1, 0.3, 0.6, 1.0);
	MaterialLightProperty* shark_specular = new MaterialLightProperty(0.01, 0.03, 0.06, 1.0);
	MaterialLightProperty* shark_emissive = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	
	Material* shark_material = new Material(*shark_ambient, *shark_diffuse, *shark_specular, *shark_emissive);
	shark_material->add2DTexture(*shark_texture);
	
	single_instance_ = new InstanceRenderedShape(*shark_properties.first);
	SceneLeafInstanced* leaf = new SceneLeafInstanced(scene_manager_->getRoot(), AnimatedInstanceShader::getShader(), *single_instance_, *shark_material);
	
	Shark* shark = NULL;

	for (int i = 0; i < 1; ++i)
	{
		shark = new Shark(&scene_manager_->getRoot(), glm::translate(glm::mat4(1.0), glm::vec3(50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f, 50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f, 50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f)), *scene_manager_);
		///SceneLeafModel* chair_leaf_node = new SceneLeafModel(*shark, NULL, *shark_properties.first, *shark_properties.second, AnimatedShadowShader::getShader(), false, false);
		///chair_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);
		
		///leaf->addInstance(*shark);

		shark->init(*shark_material, BasicShadowShader::getShader());
	
		std::vector<glm::vec3> waypoints;
		for (int j = 0; j < 10; ++j)
		{
			waypoints.push_back(glm::vec3(50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f, 50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f, 50.0f * ((float)rand() / (float)RAND_MAX) - 25.0f));
		}
		shark->setWaypoints(waypoints);
		
		single_instance_->addInstance(*shark);
		//if (i == 0)
			scene_manager_->addUpdateableEntity(*shark);
	}
	
	//scene_manager_->addUpdateableEntity(*single_instance_);
	shark_properties.first->setAnimation(*shark_properties.first->getAnimations()[0]);
	single_instance_->finaliseBuffers();
	*/
	return true;
}

GLuint ZombieHorde::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	glfwSetMousePos(width_ / 2, height_ / 2);
	fps_label_->frameRendered();
	return 0;
}

void ZombieHorde::tick(float dt)
{

}

void ZombieHorde::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_node_->onResize(width, height);
}
