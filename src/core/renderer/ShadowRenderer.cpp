#include <GL/glew.h>
#include <GL/glfw.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "FrustumCaster.h"
#include "ShadowRenderer.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneManager.h"
#include "../scene/SceneLeafLight.h"
#include "../scene/SceneLeafModel.h"
#include "../light/PointLight.h"
#include "../entities/Entity.h"
#include "../shaders/ShadowShader.h"
#include "../shaders/CreateAnimatedShadowMapShader.h"
#include "../scene/frustum/Frustum.h"
#include "../scene/portal/Region.h"
#include "../texture/Texture.h"

ShadowRenderer::ShadowRenderer(SceneManager& scene_manager, unsigned int depth_texture_size, GLenum cull_mode, GLint texture_compare_mode, GLint texture_compare_function)
	: scene_manager_(&scene_manager), depth_texture_size_(depth_texture_size), cull_mode_(cull_mode), texture_compare_mode_(texture_compare_mode), texture_compare_function_(texture_compare_function)
{
	glClearDepth(1.0f);
	frustum_ = new Frustum(glm::mat4(1.0f));
	initialise();
}

ShadowRenderer::~ShadowRenderer()
{
	
}

void ShadowRenderer::initialise()
{
	depth_texture_ = new Texture(GL_TEXTURE_2D);
	
	// Initialise the texture to be used for shadow mapping.
	//glGenTextures(1, &texture_id_);
	//glBindTexture(GL_TEXTURE_2D, texture_id_);
	glBindTexture(GL_TEXTURE_2D, depth_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, depth_texture_size_, depth_texture_size_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	// Set up depth comparison mode.
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, texture_compare_mode_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, texture_compare_function_);
	// Set up wrapping mode.
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	// Set the texture id for the depth texture.
	//glActiveTexture(GL_TEXTURE0 + light_->getDepthTextureId());
	//glBindTexture(GL_TEXTURE_2D, texture_id_);

	// Create FBO to render depth into.
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	// Attach the depth texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_texture_->getTextureId(), 0);
	// Disable colour rendering.
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		switch (glCheckFramebufferStatus(GL_FRAMEBUFFER))
		{
		case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete attachment!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Missing attachment!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_UNSUPPORTED:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Unsupported framebuffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS_EXT:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Wrong dimensions!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_INVALID_ENUM:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Wrong enum!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete draw buffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_FORMATS_EXT:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete formats!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_COUNT_ARB:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete layer count!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete layer targets!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete multisample!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Incomplete read buffer!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		default:
#ifdef _WIN32
			MessageBox(NULL, "sFramebuffer not complete!!! Unknown!", "An error occurred", MB_ICONERROR | MB_OK);
#endif
			break;
		}
		std::cerr << "Failed to construct the shadow renderer" << std::endl;
		exit(1);
	}

	// Unbind.
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ShadowRenderer::render(const FrustumCaster& cam)
{
	//std::cout << "Shadow renderer";
	//ss_.str(std::string());
	//ss_ << "Shadow: " << cam.getPosition().x << " " << cam.getPosition().y << " " << cam.getPosition().z << "|";
	//ss_ << cam.getPitch() << " " << cam.getYaw() << " " << cam.getRoll() << "|";
	active_entities_.clear();

	// Gather all the leaf nodes.
	// TODO: Make this process quicker. The scene manager should know about all the leaf nodes.
	//scene_manager_->visitLeafs(*this);
	unsigned int pre_rendered_objects_ = 0;

	glCullFace(cull_mode_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glViewport(0, 0, depth_texture_size_, depth_texture_size_);

	glClear(GL_DEPTH_BUFFER_BIT);
	
	glm::vec3 camera_location = cam.getLocation();
	glm::mat4 view_matrix = cam.getViewMatrix();
	glm::mat4 perspective_matrix = cam.getPerspectiveMatrix();

	CreateAnimatedShadowMapShader& animated_shader = CreateAnimatedShadowMapShader::getShader();
	ShadowShader& shader = ShadowShader::getShader();
	frustum_->setFrustum(perspective_matrix * view_matrix);
	
	// New rendering method.
	Region* region = Region::findRegionGlobal(cam.getLocation());
	if (region != NULL)
	{
		std::stringstream ss;
		std::vector<const Portal*> processed_portals;
		region->preRender(*frustum_, cam.getLocation(), *this, false, pre_rendered_objects_, 0, processed_portals, ss);
		for (std::vector<SceneNode*>::const_iterator ci = scene_manager_->getPlayers().begin(); ci != scene_manager_->getPlayers().end(); ++ci)
		{
			(*ci)->preRender(*frustum_, cam.getLocation(), *this, false, pre_rendered_objects_);
		}
	}
	// If we cannot find a region we fall back on the true and tested... Although this
	// is more a debuf feature and should be removed in future versions of this rendering
	// engine.
	else
	{
		SceneNode& root = scene_manager_->getRoot();
		root.preRender(*frustum_, cam.getLocation(), *this, false, pre_rendered_objects_);
	}

	bool cull_face_enabled = true;
	glEnable(GL_CULL_FACE);
	for (std::vector<const RenderableSceneLeaf*>::const_iterator ci = active_entities_.begin(); ci != active_entities_.end(); ++ci)
	{
		const RenderableSceneLeaf* leaf = *ci;

		if (leaf->getShadowType() == ShadowRenderer::NO_SHADOW || !leaf->isInFrustum(*frustum_))
		{
			continue;
		}

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
		switch (leaf->getShadowType())
		{
		case ShadowRenderer::ANIMATED_SHADOW:
			leaf->draw(view_matrix, perspective_matrix, active_lights_, &animated_shader);
			break;
		case ShadowRenderer::STATIC_SHADOW:
			leaf->draw(view_matrix, perspective_matrix, active_lights_, &shader);
			break;
		}
	}
	/*
	if (active_entities_.size() == 0)
	{
#ifdef _WIN32
		MessageBox(NULL, "No entities to render for the shadow renderer!", "Error", MB_OK);
#endif
	}
	*/
	// Unbind.
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glBindTexture(GL_TEXTURE_2D, 0);

	// Reset viewport.
	int orgWidth, orgHeight;
	glfwGetWindowSize(&orgWidth, &orgHeight);
	glViewport(0, 0, orgWidth, orgHeight);
}

void ShadowRenderer::visit(const SceneLeafLight& light)
{

}

void ShadowRenderer::visit(const RenderableSceneLeaf& model)
{
	if (!model.isTransparent() && model.getType() == OBJECT)
	{
		active_entities_.push_back(&model);
	}
}

void ShadowRenderer::onResize(int width, int height)
{
	
}
