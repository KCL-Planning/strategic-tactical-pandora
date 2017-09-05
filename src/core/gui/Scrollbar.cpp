#include "Scrollbar.h"

#include "GL/glfw.h"

#include "Container.h"
#include "ScrollbarUnit.h"
#include "themes/Theme.h"

#include "../shaders/GUIShader.h"

Scrollbar::Scrollbar(Theme& theme, Font& font, float x, float y, float size_x, float size_y, Container& scrollee, bool vertical)
	: Container(theme, font, x, y, size_x, size_y, "Scrollbar"), scrollee_(&scrollee), is_vertical_(vertical)
{
	int width, height;
	glfwGetWindowSize(&width, &height);

	// Setup the scroll bar itself.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));

	m_tex_coords_.insert(m_tex_coords_.end(), theme.getScrollbarTexture().begin(), theme.getScrollbarTexture().end());
	
	scrollbar_unit_ = new ScrollbarUnit(theme, x, y, size_x, size_y, *this);
	addElement(*scrollbar_unit_, 0, 0);
}

void Scrollbar::resizedEvent(const Container& container)
{
	// Change the size of the scrollbar and update the location of the unit.
	scrollbar_unit_->setContentSize(container.getContentSizeX(), container.getContentSizeY());
}
/*
void Scrollbar::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
}
*/