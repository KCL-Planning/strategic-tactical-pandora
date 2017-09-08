#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include "dpengine/renderer/GUIRenderer.h"
#include "dpengine/renderer/Window.h"
#include "dpengine/gui/GUIManager.h"

namespace DreadedPE
{

GUIRenderer::GUIRenderer(unsigned int fbo_id)
	: fbo_id_(fbo_id)
{
	Window::getActiveWindow()->getSize(width_, height_);
}

void GUIRenderer::render(const GUIManager& gui_manager)
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	
	//glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
	
	glm::mat4 perspective_matrix = glm::ortho(0.0f, (float)width_, 0.0f, (float)height_, -1.0f, 1.0f);

	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	gui_manager.draw(perspective_matrix);

	glDisable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void GUIRenderer::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
}

};
