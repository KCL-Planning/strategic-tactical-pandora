#ifdef _WIN32
#include <Windows.h>
#endif

#include <sstream>
#include "GUIElement.h"
#include "Container.h"

GUIElement::GUIElement(const Theme& theme, float x, float y, float size_x, float size_y)
	: x_(x), y_(y), size_x_(size_x), size_y_(size_y), parent_(NULL), theme_(&theme), is_visible_(true)
{
	local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(x, y, 0.0f));
	texture_ = &theme.getTexture();
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
	
}

void GUIElement::setTextureUVMapping(const std::vector<glm::vec2>& texture)
{
	m_tex_coords_ = texture;
}
