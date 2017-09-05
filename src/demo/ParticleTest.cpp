#include "ParticleTest.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <GL/glew.h>
#include <GL/glfw.h>

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../core/entities/camera/FreeMovingCamera.h"

#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/scene/Material.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"

#include "../shapes/Cube.h"

#include "../core/particles/GPUParticleEmitter.h"
#include "../core/particles/GPUParticleDrawShader.h"
#include "../core/particles/GPUParticleComputerShader.h"
#include "../core/particles/Particle.h"

ParticleTest::ParticleTest(SceneManager& scene_manager)
	: scene_manager_(&scene_manager), total_time_(0)
{
	srand (time(NULL));
}

bool ParticleTest::init(int argc, char** argv)
{
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	smoke_texture_ = TargaTexture::loadTexture("data/textures/smoke_particle.tga");
	
	particle_root_ = new Entity(*scene_manager_, &scene_manager_->getRoot(), glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0)), OBSTACLE, "blaat");
	//scene_manager_->addUpdateableEntity(*particle_root_);
	Cube* underwater_volvano_model = new Cube(0.1f, 0.1f, 0.1f);
	std::vector<Particle> initial_particle_state;
	for (unsigned int i = 0; i < 1; ++i)
	{
		float x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 30.0f;
		float z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 30.0f;
		x = 10;
		z = 0;
		SceneNode* dummy_node = new SceneNode(*scene_manager_, particle_root_, glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, z)));
		SceneLeafModel* dummy_cube = new SceneLeafModel(*dummy_node, NULL, *underwater_volvano_model, *SceneNode::bright_material_, BasicShadowShader::getShader(), false, false);

		Particle particle(glm::vec3(x, 0.0f, z), glm::vec3(0, 0, 0), 0, 0);
		initial_particle_state.push_back(particle);
	}
	
	particle_computer_ = new GPUParticleComputerShader("src/demo/pandora/shaders/resources/ParticleInWater.vert", "src/demo/particle_demo/shaders/ParticleCalculator.geom", initial_particle_state, 0.0025f, 2.0f);
	particle_drawer_ = new GPUParticleDrawShader(*particle_computer_, *smoke_texture_, "src/demo/particle_demo/shaders/particle.vert", "src/demo/particle_demo/shaders/particle_bb.geom", "src/demo/particle_demo/shaders/particle.frag");
	particle_emitter_ = new GPUParticleEmitter(*particle_root_, *particle_computer_, *particle_drawer_);
	
	std::vector<Particle> initial_particle_state2;
	for (unsigned int i = 0; i < 1; ++i)
	{
		float x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 30.0f;
		float z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 30.0f;
		x = 0;
		z = -10;
		SceneNode* dummy_node = new SceneNode(*scene_manager_, particle_root_, glm::translate(glm::mat4(1.0f), glm::vec3(x, 0.0f, z)));
		SceneLeafModel* dummy_cube = new SceneLeafModel(*dummy_node, NULL, *underwater_volvano_model, *SceneNode::bright_material_, BasicShadowShader::getShader(), false, false);

		Particle particle(glm::vec3(x, 0.0f, z), glm::vec3(0, 0, 0), 0, 0);
		initial_particle_state2.push_back(particle);
	}
	
	particle_computer2_ = new GPUParticleComputerShader("src/demo/pandora/shaders/resources/ParticleInWater.vert", "src/demo/pandora/shaders/resources/ParticleCalculator.geom", initial_particle_state2, 0.0025f, 1.0f);
	particle_drawer2_ = new GPUParticleDrawShader(*particle_computer2_, *smoke_texture_, "src/demo/pandora/shaders/resources/particle.vert", "src/demo/pandora/shaders/resources/particle_bb.geom", "src/demo/pandora/shaders/resources/SmokeParticle.frag");
	particle_emitter2_ = new GPUParticleEmitter(*particle_root_, *particle_computer2_, *particle_drawer2_);
	
	// Add the camera system.
	camera_node_ = new FreeMovingCamera(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_node_);

	// Add a cube at the particle despenser's location.
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	MaterialLightProperty* concrete_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* concrete_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* concrete_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* concrete_emmisive = new MaterialLightProperty(1.0f, 0.0f, 0.0f, 1.0f);

	Material* material = new Material(*concrete_ambient, *concrete_diffuse, *concrete_specular, *concrete_emmisive);
	material->add2DTexture(*grass_texture);

	Cube* cube = new Cube(0.1f, 0.1f, 0.1f);
	SceneLeafModel* cube_leaf = new SceneLeafModel(*particle_root_, NULL, *cube, *material, BasicShadowShader::getShader(), false, false);
	
	SceneLeafModel* cube_leaf_root = new SceneLeafModel(scene_manager_->getRoot(), NULL, *cube, *material, BasicShadowShader::getShader(), false, false);

	return true;
}

bool ParticleTest::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
}

GLuint ParticleTest::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	glfwSetMousePos(width_ / 2, height_ / 2);
	return 0;
}

void ParticleTest::tick(float dt)
{
	total_time_ += dt;
	if (glfwGetKey('Y') == GLFW_PRESS)
	{
		particle_computer_->enableParticleSpawn(true);
		particle_computer2_->enableParticleSpawn(true);
	}

	if (glfwGetKey('N') == GLFW_PRESS)
	{
		particle_computer_->enableParticleSpawn(false);
		particle_computer2_->enableParticleSpawn(false);
	}

//	particle_root_->setTransformation(glm::translate(particle_root_->getLocalTransformation(), glm::vec3(dt / 10.0f, 0.0f, 0.0f)), true);
//	particle_computer_->updateParticlePosition(particle_root_->getGlobalLocation());
}

void ParticleTest::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_node_->onResize(width, height);
}
