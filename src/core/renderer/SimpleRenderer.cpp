#include <sstream>
#include <algorithm>

#include <GL/glew.h>
#include <GL/glfw.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "SimpleRenderer.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneManager.h"
#include "../scene/SceneLeafLight.h"
#include "../scene/SceneLeafModel.h"
#include "../scene/RenderableSceneLeaf.h"
#include "../entities/camera/Camera.h"
#include "../light/PointLight.h"
#include "../entities/Entity.h"
#include "../shaders/ShadowShader.h"
#include "../scene/frustum/Frustum.h"
#include "../scene/portal/Region.h"
#include "../shaders/LightShader.h"
#include "../texture/Texture.h"
//#include "../particles/ParticleSystem.h"

#ifdef _WIN32
#include <windows.h>
#endif

//#define HORROR_GAME_ENABLE_DEBUG

SimpleRenderer::SimpleRenderer(SceneManager& scene_manager)
	: sky_box_(NULL), scene_manager_(&scene_manager), show_collision_boxes_(false)
{
	setRenderMode(GL_FILL);
	frustum_ = new Frustum(glm::mat4(1.0f));
	initialise();
}

SimpleRenderer::~SimpleRenderer()
{
	
}

void SimpleRenderer::initialise()
{
	color_texture_ = new Texture(GL_TEXTURE_2D);
	depth_texture_ = new Texture(GL_TEXTURE_2D);
	
	// Initialise the off screen buffer we render to. This allows for post processing effects.
	//glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, color_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 768, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	//glGenTextures(1, &depth_id_);
	glBindTexture(GL_TEXTURE_2D, depth_texture_->getTextureId());
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, 1024, 768, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH32F_STENCIL8, 1024, 768, 0, GL_DEPTH_STENCIL, GL_FLOAT_32_UNSIGNED_INT_24_8_REV, NULL);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH32F_STENCIL8, 1024, 768, 0, GL_DEPTH_STENCIL, GL_FLOAT_32_UNSIGNED_INT_24_8_REV, NULL);

	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_texture_->getTextureId(), 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, depth_texture_->getTextureId(), 0);
	//glFramebufferTexture(GL_FRAMEBUFFER, GL_STENCIL_ATTACHMENT, depth_id_, 0);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		switch (glCheckFramebufferStatus(GL_FRAMEBUFFER))
		{
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete attachment!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Missing attachment!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_UNSUPPORTED:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Unsupported framebuffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Wrong dimensions!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_INVALID_ENUM:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Wrong enum!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete draw buffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete formats!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_COUNT_ARB:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete layer count!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete layer targets!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete multisample!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Incomplete read buffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		default:
#ifdef _WIN32
			MessageBox(NULL, "Framebuffer not complete!!! Unknown!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		}
		std::cerr << "Failed to construct the simple renderer" << std::endl;
		exit(1);
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SimpleRenderer::render(const FrustumCaster& cam)
{
	//glPolygonMode( GL_FRONT_AND_BACK, render_mode_);
	culled_objects_ = 0;
	rendered_objects_ = 0;
	pre_rendered_objects_ = 0;
	active_entities_.clear();
	active_lights_.clear();
	//ss_.str(std::string());

	double_models_ = 0;
	double_lights_ = 0;
	
	glm::vec3 camera_location = cam.getLocation();
	glm::mat4 view_matrix = cam.getViewMatrix();
	glm::mat4 perspective_matrix = cam.getPerspectiveMatrix();
	frustum_->setFrustum(perspective_matrix * view_matrix);

#ifdef HORROR_GAME_ENABLE_DEBUG
	// New rendering method.
	ss_.str(std::string());
#endif
	
	//Region* region = scene_manager_->getRoot().findRegion(cam.getPosition());
	Region* region = getRegionToRenderFrom(camera_location, *scene_manager_);
	if (region != NULL)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "(" << cam.getPosition().x << "," << cam.getPosition().y << "," << cam.getPosition().z << ") ";
		ss_ << region->getName() << "|";
#endif
//		std::cout << "Camera in region: " << region->getName() << std::endl;
		std::vector<const Portal*> processed_portals;
		region->preRender(*frustum_, camera_location, *this, true, pre_rendered_objects_, 0, processed_portals, ss_);
		for (std::vector<SceneNode*>::const_iterator ci = scene_manager_->getPlayers().begin(); ci != scene_manager_->getPlayers().end(); ++ci)
		{
			(*ci)->preRender(*frustum_, camera_location, *this, true, pre_rendered_objects_);
		}
		ss_ << std::endl;
		//std::cout << "." << std::endl;
		//std::cout << ss_.str() << std::endl;
//		std::cout << "Done rendering!"  << std::endl;
#ifdef _WIN32
		OutputDebugString(ss_.str().c_str());
		ss_.str(std::string());
#endif
	}
	// If we cannot find a region we fall back on the true and tested... Although this
	// is more a debug feature and should be removed in future versions of this rendering
	// engine.
	else
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "Default rendering...";
#endif
		// Update the animations before rendering.
		SceneNode& root = scene_manager_->getRoot();
		//root.preRender(*frustum_, cam.getPosition(), *this, true, pre_rendered_objects_);
		root.preRender(*frustum_, camera_location, *this, true, pre_rendered_objects_);
	}

	glCullFace(GL_BACK);
	// Visit all the leaf nodes and render them. We need to gather all the light sources which are of interest first.
	
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw the skybox (if any).
	if (sky_box_ != NULL)
	{
		glDisable(GL_DEPTH_TEST);
		sky_box_->draw(view_matrix, perspective_matrix, active_lights_, NULL);
		glEnable(GL_DEPTH_TEST);
	}


	// Sort lights such that there are no more than MAX_LIGHT lights being provided to the shaders.
	while (active_lights_.size() > LightShader::MAX_LIGHTS_)
	{
		std::vector<const SceneLeafLight*>::iterator i;
		float max_distance = -1;
		// Find the light that is furthest away from the camera.
		for (std::vector<const SceneLeafLight*>::iterator ci = active_lights_.begin(); ci != active_lights_.end(); ++ci)
		{
			//float distance = glm::length((*ci)->getParent()->getGlobalLocation() - cam.getPosition());
			float distance = glm::length((*ci)->getParent()->getGlobalLocation() - camera_location);
			if (distance > max_distance)
			{
				i = ci;
				max_distance = distance;
			}
		}

		active_lights_.erase(i);
	}
	
	//std::cout << "#lights: " << active_lights_.size() << std::endl;

	bool cull_face_enabled = true;
	glEnable(GL_CULL_FACE);

	// Two pass rendering, first for all objects which are not transparent and then for all objects which are.
	for (std::vector<const RenderableSceneLeaf*>::const_iterator ci = active_entities_.begin(); ci != active_entities_.end(); ++ci)
	{
		const RenderableSceneLeaf* leaf = *ci;
		++rendered_objects_;

		if (!leaf->isTransparent())
		{
			if (leaf->isDoubleSided())// && cull_face_enabled)
			{
				glDisable(GL_CULL_FACE);
				cull_face_enabled = false;
			}
			else// if (!cull_face_enabled)
			{
				glEnable(GL_CULL_FACE);
				cull_face_enabled = true;
			}
			leaf->draw(view_matrix, perspective_matrix, active_lights_, NULL);
		}
	}
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	// Render the particles.
	for (std::vector<const RenderableSceneLeaf*>::const_iterator ci = active_entities_.begin(); ci != active_entities_.end(); ++ci)
	{
		const RenderableSceneLeaf* leaf = *ci;
		if (leaf->isTransparent())
		{
			if (leaf->isDoubleSided())// && cull_face_enabled)
			{
				glDisable(GL_CULL_FACE);
				cull_face_enabled = false;
			}
			else// if (!cull_face_enabled)
			{
				glEnable(GL_CULL_FACE);
				cull_face_enabled = true;
			}
			leaf->draw(view_matrix, perspective_matrix, active_lights_, NULL);
		}
	}
	glDisable(GL_BLEND);
	
	if (double_lights_ != 0 || double_models_ != 0)
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "Double lights: " << double_lights_ << ". Double models: " << double_models_ << std::endl;
		OutputDebugString(ss.str().c_str());
#else
		std::cout << "Double lights: " << double_lights_ << ". Double models: " << double_models_ << std::endl;
#endif
	}
	
	//ss_ << "end...";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SimpleRenderer::visit(const SceneLeafLight& light)
{
	if (std::find(active_lights_.begin(), active_lights_.end(), &light) != active_lights_.end())
	{
		++double_lights_;
	//	return;
	}

	light.getLight().setId(active_lights_.size());
	active_lights_.push_back(&light);
}

void SimpleRenderer::visit(const RenderableSceneLeaf& model)
{
	if (std::find(active_entities_.begin(), active_entities_.end(), &model) != active_entities_.end())
	{
		++double_models_;
	//	return;
	}

	if (model.getType() == SKYBOX)
	{
		sky_box_ = &model;
	}
	
	else if (model.getType() == COLLISION && show_collision_boxes_)
	{
		active_entities_.push_back(&model);
	}
	else if (model.getType() != COLLISION && !show_collision_boxes_)
	{
		active_entities_.push_back(&model);
	}
}

void SimpleRenderer::onResize(int width, int height)
{
	glBindTexture(GL_TEXTURE_2D, color_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, NULL);

	glBindTexture(GL_TEXTURE_2D, depth_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH32F_STENCIL8, width, height, 0, GL_DEPTH_STENCIL, GL_FLOAT_32_UNSIGNED_INT_24_8_REV, NULL);
}
