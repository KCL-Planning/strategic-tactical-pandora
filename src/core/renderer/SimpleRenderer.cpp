#include <sstream>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/renderer/SimpleRenderer.h"
#include "dpengine/renderer/Window.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneLeafLight.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/RenderableSceneLeaf.h"
#include "dpengine/entities/camera/Camera.h"
#include "dpengine/light/PointLight.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/shaders/ShadowShader.h"
#include "dpengine/scene/frustum/Frustum.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/shaders/LightShader.h"
#include "dpengine/texture/Texture.h"

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

namespace DreadedPE
{
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
	delete frustum_;
}

void SimpleRenderer::initialise()
{
	color_texture_ = new Texture(GL_TEXTURE_2D);
	depth_texture_ = new Texture(GL_TEXTURE_2D);

	int width;
	int height;
	Window::getActiveWindow()->getSize(width, height);
	
	// Initialise the off screen buffer we render to. This allows for post processing effects.
	glBindTexture(GL_TEXTURE_2D, color_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glBindTexture(GL_TEXTURE_2D, depth_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH32F_STENCIL8, width, height, 0, GL_DEPTH_STENCIL, GL_FLOAT_32_UNSIGNED_INT_24_8_REV, NULL);
	
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_texture_->getTextureId(), 0);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, depth_texture_->getTextureId(), 0);

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
	culled_objects_ = 0;
	rendered_objects_ = 0;
	pre_rendered_objects_ = 0;
	active_entities_.clear();
	active_lights_.clear();
	sky_box_ = NULL;

	double_models_ = 0;
	double_lights_ = 0;
	
	glm::vec3 camera_location = cam.getLocation();
	glm::mat4 view_matrix = cam.getViewMatrix();
	glm::mat4 perspective_matrix = cam.getPerspectiveMatrix();
	frustum_->setFrustum(perspective_matrix * view_matrix);

#ifdef HORROR_GAME_ENABLE_DEBUG
	std::stringstream ss_;
#endif
	
	Region* region = getRegionToRenderFrom(camera_location, *scene_manager_);
	if (region != NULL)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "******************* RENDER *********************" << std::endl;
		ss_ << "Camera: (" << cam.getLocation().x << "," << cam.getLocation().y << "," << cam.getLocation().z << ")" << std::endl;
		OutputDebugString(ss_.str().c_str());
		ss_.str(std::string());
#endif
		std::vector<const Portal*> processed_portals;
		region->preRender(*frustum_, camera_location, *this, true, pre_rendered_objects_, 0, processed_portals);
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
		root.preRender(*frustum_, camera_location, *this, true, pre_rendered_objects_);
	}

	glCullFace(GL_BACK);
	// Visit all the leaf nodes and render them. We need to gather all the light sources which are of interest first.
	
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw the skybox first (if any).
	if (sky_box_ != NULL)
	{
		glDisable(GL_DEPTH_TEST);
		sky_box_->draw(view_matrix, perspective_matrix, active_lights_, NULL);
		glEnable(GL_DEPTH_TEST);
	}

	//bool cull_face_enabled = true;
	glEnable(GL_CULL_FACE);

	// Two pass rendering, first for all objects which are not transparent and then for all objects which are.
	for (std::vector<const RenderableSceneLeaf*>::const_iterator ci = active_entities_.begin(); ci != active_entities_.end(); ++ci)
	{
		const RenderableSceneLeaf* leaf = *ci;
		++rendered_objects_;
		
		if (!leaf->isTransparent())
		{
			// Find the closest lights.
			std::vector<const SceneLeafLight*> closest_lights;
			findClosestLights(active_lights_, *leaf, closest_lights);

			if (leaf->isDoubleSided())// && cull_face_enabled)
			{
				glDisable(GL_CULL_FACE);
				//cull_face_enabled = false;
			}
			else// if (!cull_face_enabled)
			{
				glEnable(GL_CULL_FACE);
				//cull_face_enabled = true;
			}
			//leaf->draw(view_matrix, perspective_matrix, active_lights_, NULL);
			leaf->draw(view_matrix, perspective_matrix, closest_lights, NULL);
		}
	}
	
	ShaderInterface::allRender();
	
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	// Render the particles.
	for (std::vector<const RenderableSceneLeaf*>::const_iterator ci = active_entities_.begin(); ci != active_entities_.end(); ++ci)
	{
		const RenderableSceneLeaf* leaf = *ci;

		if (leaf->isTransparent())
		{
			// Find the closest lights.
			std::vector<const SceneLeafLight*> closest_lights;
			findClosestLights(active_lights_, *leaf, closest_lights);

			if (leaf->isDoubleSided())// && cull_face_enabled)
			{
				glDisable(GL_CULL_FACE);
				//cull_face_enabled = false;
			}
			else// if (!cull_face_enabled)
			{
				glEnable(GL_CULL_FACE);
				//cull_face_enabled = true;
			}
			//leaf->draw(view_matrix, perspective_matrix, active_lights_, NULL);
			leaf->draw(view_matrix, perspective_matrix, closest_lights, NULL);
		}
	}
	ShaderInterface::allRender();
	glDisable(GL_BLEND);
	
	/*
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
	*/
	//ss_ << "end...";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void SimpleRenderer::findClosestLights(const std::vector<const SceneLeafLight*>& active_lights, const SceneLeaf& leaf, std::vector<const SceneLeafLight*>& closest_lights)
{
	// Find the closest lights.
	std::vector<float> light_distances;
	for (const SceneLeafLight* light : active_lights)
	{
		float distance = glm::distance(leaf.getParent()->getGlobalLocation(), light->getParent()->getGlobalLocation());
		// Check if this light is closer than others.
		if (closest_lights.size() < LightShader::MAX_LIGHTS_)
		{
			closest_lights.push_back(light);
			light_distances.push_back(distance);
		}
		else
		{
			float highest_distance = 0;
			unsigned int highest_distance_index = 0;
			for (unsigned int i = 0; i < closest_lights.size(); ++i)
			{
				if (highest_distance < light_distances[i])
				{
					highest_distance = light_distances[i];
					highest_distance_index = i;
				}
			}

			if (distance < highest_distance)
			{
				closest_lights.erase(closest_lights.begin() + highest_distance_index);
				light_distances.erase(light_distances.begin() + highest_distance_index);
				closest_lights.push_back(light);
				light_distances.push_back(distance);
			}
		}
	}
}

void SimpleRenderer::visit(const SceneLeafLight& light)
{
	if (std::find(active_lights_.begin(), active_lights_.end(), &light) != active_lights_.end())
	{
		++double_lights_;
		return;
	}

	//light.getLight().setId(active_lights_.size());
	active_lights_.push_back(&light);
}

void SimpleRenderer::visit(const RenderableSceneLeaf& model)
{
	if (std::find(active_entities_.begin(), active_entities_.end(), &model) != active_entities_.end())
	{
		++double_models_;
		return;
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

};
