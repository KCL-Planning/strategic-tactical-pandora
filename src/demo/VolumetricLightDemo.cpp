#include "VolumetricLightDemo.h"

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
#include "../shapes/Cylinder.h"
#include "../shapes/SkyBox.h"
#include "../shapes/Line.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"
#include "../core/renderer/ShadowRenderer.h"

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
#include "../core/shaders/MergeFBOShader.h"
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

#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"

#include "../core/loaders/PortalLevelFormatLoader.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

#include "shooter/ArmedPlayer.h"
#include "volumetric/LightVolumeShape.h"
#include "volumetric/ShadowVolumeShader.h"

#include "../core/loaders/AssimpLoader.h"
#include "../core/entities/Monster.h"

#include "../core/ai/pathfinding/NavMeshAStar.h"
#include "../core/ai/pathfinding/NavigationMesh.h"
#include "../core/ai/pathfinding/ConvexNavigationArea.h"

VolumetricLightDemo::VolumetricLightDemo(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool VolumetricLightDemo::init(int argc, char** argv)
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
	
	PortalLevelFormatLoader* level_loader = new PortalLevelFormatLoader();
	SceneNode* level_node_ = level_loader->importLevel("data/models/levels/volumetric_lighting_test/test1.plf", *wfl_material_, BasicShadowShader::getShader(), *scene_manager_, *terrain_node_);
	if (level_node_ == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the level!", "Error", MB_OK);
#endif
		std::cout << "Could not load the level!" << std::endl;
		exit(1);
	}
	
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.3f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);
	
	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, TerrainShader::getShader(), false, false);

	glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 5.2f, -9.25f));
	m = glm::rotate(m, -160.0f, glm::vec3(0, 1, 0));
	m = glm::rotate(m, -35.0f, glm::vec3(1, 0, 0));
	
	PointLight* point_light = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f, 1024);
	//SceneNode* light_node = new SceneNode(*scene_manager_, &region->getSceneNode(),  m);
	light_node_ = new SceneNode(*scene_manager_, &scene_manager_->getRoot(),  m);
	SceneLeafLight* light_leaf = new SceneLeafLight(*light_node_, NULL, *point_light);
	
	// Make it volumetric :).
	volumetric_light_point_ = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f, 256, GL_NONE, GL_NONE, GL_NONE);
	volumetric_light_leaf_ = new SceneLeafLight(*light_node_, NULL, *volumetric_light_point_);
	LightVolumeShape* lvs = new LightVolumeShape(*scene_manager_, *volumetric_light_point_);
	
	light_volume_leaf_ = new SceneLeafModel(*light_node_, NULL, *lvs, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
	lvs->setLeafNode(*light_volume_leaf_);

	// Adding a massive cube to block the light.
	//Cube* cube = new Cube(5, 5, 5);
	Cylinder* cube = new Cylinder(2.0f, 0.5f, 16);
	SceneNode* cube_node = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(10, 8, -25)));
	SceneLeafModel* cube_model = new SceneLeafModel(*cube_node, NULL, *cube, *wfl_material_, BasicShadowShader::getShader(), false, false);
	
	SceneNode* cube_node2 = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(0, 8, -25)));
	SceneLeafModel* cube_model2 = new SceneLeafModel(*cube_node2, NULL, *cube, *SceneNode::bright_material_, BasicShadowShader::getShader(), false, false);
	
	// The player.
	//player_ = new ArmedPlayer(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(10.0f, 30.0f, 0.0f)), 1.9f, *terrain_node_, *terrain_, *scene_manager_, *grass_texture);

	// Initialise the camera:
	camera_ = new FreeMovingCamera(*scene_manager_, terrain_node_, glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 60.0f);
	scene_manager_->addUpdateableEntity(*camera_);

	shadow_renderer_ = new ShadowRenderer(*scene_manager_, 512, GL_BACK, GL_NONE, GL_NONE);
	
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

	// Create a seperate framebuffer for the post processing.
	texture_ = new Texture(GL_TEXTURE_2D);
	//glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1024, 768, 0, GL_RGBA, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glBindTexture(GL_TEXTURE_2D, 0);

	return true;
}

bool VolumetricLightDemo::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
}


void VolumetricLightDemo::tick(float dt)
{
	//light_node_->setTransformation(glm::translate(light_node_->getLocalTransformation(), glm::vec3(dt, 0, 0)));
}

GLuint VolumetricLightDemo::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	glfwSetMousePos(width_ / 2, height_ / 2);
	// Get the depth and texture attachment for postprocessing.
	// The texture is used to later merge the volumetric light texture with the texture that has the scene rendered.
	//glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_id);
	//GLint texture_id;
	//glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &texture_id);
	
	//GLint depth_id;
	//glGetFramebufferAttachmentParameteriv(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME, &depth_id);
	
	// Render the scene from the camera's point of view, we use this depth texture to cull the light such that it does not shine through
	// solid objects.
	shadow_renderer_->render(*camera_);

	glm::vec3 camera_location = camera_->getLocation();
	glm::mat4 view_matrix = camera_->getViewMatrix();
	glm::mat4 perspective_matrix = camera_->getPerspectiveMatrix();
	
	std::vector<const SceneLeafLight*> active_lights;
	active_lights.push_back(volumetric_light_leaf_);

	// Render the shadow from the light's perspective.
	volumetric_light_point_->preRender(volumetric_light_leaf_->getParent()->getCompleteTransformation());

	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_DEPTH_CLAMP);

	// Enable additive blending.
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	// Render the light volume in this frame buffer.
	ShadowVolumeShader& shader = ShadowVolumeShader::getShader();
	shader.initialise(*light_volume_leaf_, view_matrix, light_volume_leaf_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf_, camera_->getNearPlane(), camera_->getFarPlane(), shadow_renderer_->getTexture());
	light_volume_leaf_->draw(view_matrix, perspective_matrix, active_lights, NULL);
	
	//ss_ << "end...";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_CLAMP);
	
	// Merge the images of the volumetric light step with the main image.
	MergeFBOShader& merge_shader = MergeFBOShader::getShader();
	merge_shader.postProcess(color_texture, *texture_, dt);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	
	return merge_shader.getFrameBufferId();	
}

void VolumetricLightDemo::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_->onResize(width, height);
	//glBindTexture(GL_TEXTURE_2D, texture_->getTextureId());
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_2D, texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	MergeFBOShader& merge_shader = MergeFBOShader::getShader();
	merge_shader.onResize(width, height);
}
