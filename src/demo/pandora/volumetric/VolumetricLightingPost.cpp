#include "VolumetricLightingPost.h"

#include <memory>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include <dpengine/loaders/targa.h>
#include <dpengine/entities/camera/Camera.h>
#include <dpengine/entities/camera/DynamicCamera.h>
#include <dpengine/entities/camera/FreeMovingCamera.h>
#include <dpengine/shapes/terrain.h>
#include <dpengine/shapes/Water.h>
#include <dpengine/shapes/Tree.h>
#include <dpengine/shapes/sphere.h>
#include <dpengine/shapes/Piramid.h>
#include <dpengine/shapes/Cube.h>
#include <dpengine/shapes/Cylinder.h>
#include <dpengine/shapes/SkyBox.h>
#include <dpengine/shapes/Line.h>
#include <dpengine/light/SpotLight.h>
#include <dpengine/light/Light.h>
#include <dpengine/renderer/ShadowRenderer.h>
#include <dpengine/renderer/CameraRenderer.h>
#include <dpengine/renderer/Window.h>

#include <dpengine/scene/SceneLeafLight.h>
#include <dpengine/scene/SceneLeafModel.h>
#include <dpengine/scene/SceneManager.h>
#include <dpengine/scene/SceneNode.h>
#include <dpengine/scene/SkyBoxLeaf.h>
#include <dpengine/scene/portal/Region.h>
#include <dpengine/shapes/terrain.h>
#include <dpengine/scene/Material.h>
#include <dpengine/shaders/TerrainShader.h>
#include <dpengine/shaders/BasicShadowShader.h>
#include <dpengine/shaders/WaterShader.h>
#include <dpengine/shaders/SkyBoxShader.h>
#include <dpengine/shaders/ShadowShader.h>
#include <dpengine/shaders/AnimatedShadowShader.h>
#include <dpengine/shaders/LineShader.h>
#include <dpengine/shaders/MergeFBOShader.h>
#include <dpengine/scene/frustum/SphereCheck.h>

#include <dpengine/texture/Texture.h>
#include <dpengine/texture/TargaTexture.h>

#include <dpengine/loaders/PortalLevelFormatLoader.h>
#include <dpengine/loaders/AssimpLoader.h>

#include "LightVolumeShape.h"
#include "ShadowVolumeShader.h"


VolumetricLightingPost::VolumetricLightingPost(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode& parent, std::shared_ptr<DreadedPE::Material> terrain_material)
	: scene_manager_(&scene_manager)
{
	DreadedPE::Window* window = DreadedPE::Window::getActiveWindow();
	window->addListener(*this);

	// Create a simple light.
	DreadedPE::SpotLight* point_light = new DreadedPE::SpotLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f, 1024);
	new DreadedPE::SceneLeafLight(parent, NULL, *point_light);
	
	// Make it volumetric :).
	volumetric_light_point_ = new DreadedPE::SpotLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f, 256, GL_NONE, GL_NONE, GL_NONE);
	volumetric_light_leaf_ = new DreadedPE::SceneLeafLight(parent, NULL, *volumetric_light_point_);
	std::shared_ptr<LightVolumeShape> lvs(std::make_shared<LightVolumeShape>(*scene_manager_, *volumetric_light_point_));
	
	light_volume_leaf_ = new DreadedPE::SceneLeafModel(parent, NULL, lvs, terrain_material, ShadowVolumeShader::getShader(), false, true, DreadedPE::COLLISION, DreadedPE::ShadowRenderer::NO_SHADOW);
	lvs->setLeafNode(*light_volume_leaf_);

	
	shadow_renderer_ = new DreadedPE::ShadowRenderer(*scene_manager_, 512, GL_BACK, GL_NONE, GL_NONE);

	// Create a seperate framebuffer for the post processing.
	texture_ = new DreadedPE::Texture(GL_TEXTURE_2D);
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
}

GLuint VolumetricLightingPost::postProcess(DreadedPE::Texture& color_texture, DreadedPE::Texture& depth_texture, float dt)
{
	// Render the scene from the camera's point of view, we use this depth texture to cull the light such that it does not shine through
	// solid objects.
	DreadedPE::Camera& camera = DreadedPE::CameraRenderer::getActiveCameraRenderer().getCamera();
	shadow_renderer_->render(camera);

	//glm::vec3 camera_location = camera.getLocation();
	glm::mat4 view_matrix = camera.getViewMatrix();
	glm::mat4 perspective_matrix = camera.getPerspectiveMatrix();
	
	std::vector<const DreadedPE::SceneLeafLight*> active_lights;
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
	shader.initialise(*light_volume_leaf_, view_matrix, light_volume_leaf_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf_, camera.getNearPlane(), camera.getFarPlane(), shadow_renderer_->getTexture());
	light_volume_leaf_->draw(view_matrix, perspective_matrix, active_lights, NULL);
	
	//ss_ << "end...";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_CLAMP);
	
	// Merge the images of the volumetric light step with the main image.
	DreadedPE::MergeFBOShader& merge_shader = DreadedPE::MergeFBOShader::getShader();
	merge_shader.postProcess(color_texture, *texture_, dt);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	
	return merge_shader.getFrameBufferId();	
}

void VolumetricLightingPost::windowResized(DreadedPE::Window& window, int width, int height)
{
	glBindTexture(GL_TEXTURE_2D, texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	DreadedPE::MergeFBOShader& merge_shader = DreadedPE::MergeFBOShader::getShader();
	merge_shader.onResize(width, height);
}
