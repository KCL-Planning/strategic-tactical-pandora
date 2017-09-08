#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

#include <sstream>
#include "dpengine/gui/GUIElement.h"
#include "dpengine/gui/Container.h"

namespace DreadedPE
{

GUIElement::GUIElement(const Theme& theme, float x, float y, float size_x, float size_y)
	: x_(x), y_(y), size_x_(size_x), size_y_(size_y), parent_(NULL), theme_(&theme), is_visible_(true), is_selected_(false), needs_update_(true)
{
	local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(x, y, 0.0f));
	texture_ = &theme.getTexture();
}

void GUIElement::setVisible(bool visible)
{
	if (is_visible_ == visible)
		return;
	is_visible_ = visible;
	if (parent_ != NULL)
	{
		parent_->updateBuffers();
	}
}

void GUIElement::update(float dt)
{
	if (needs_update_ || is_selected_)
	{
		updateTransformations();
		needs_update_ = false;
	}

	if (is_selected_)
	{
		markForUpdate();
	}
}

void GUIElement::updateTransformations()
{
	if (parent_ == NULL)
	{
		global_transformation_ = local_transformation_;
	}
	else
	{
		global_transformation_ = parent_->global_transformation_ * local_transformation_;
	}
}

void GUIElement::onResize(int width, int height)
{
	markForUpdate();
}

void GUIElement::setTextureUVMapping(const std::vector<glm::vec2>& texture)
{
	m_tex_coords_ = texture;
	markForUpdate();
}

void GUIElement::markForUpdate()
{
	needs_update_ = true;
	if (parent_ != NULL)
	{
		parent_->markForUpdate();
	}
}

};
