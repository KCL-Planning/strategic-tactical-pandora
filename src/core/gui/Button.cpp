#include <string>

#include <GL/glew.h>
#include <GL/glfw.h>

#include <glm/gtc/matrix_transform.hpp>

#include "fonts/Font.h"
#include "Button.h"
#include "../shaders/GUIShader.h"
#include "fonts/TexturedFont.h"
#include "Container.h"

//#include "FreeTypeFont.h"
#include "events/ButtonPressedListener.h"
#include "themes/Theme.h"

Button::Button(Theme& theme, float size_x, float size_y, const std::string& label, float font_size)
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), is_pressed_(false), font_size_(font_size)
{
	int width, height;
	glfwGetWindowSize(&width, &height);

	// Test data.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));

	complete_tex_coords_ = theme.getButtonTexture();
	m_tex_coords_.insert(m_tex_coords_.end(), theme.getButtonTexture().begin(), theme.getButtonTexture().begin() + 4);

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
}

Button::Button(Theme& theme, float size_x, float size_y, const std::string& label, float font_size, const std::vector<glm::vec2>& uv_mapping)
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), is_pressed_(false), font_size_(font_size)
{
	int width, height;
	glfwGetWindowSize(&width, &height);

	// Test data.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	complete_tex_coords_ = uv_mapping;
	m_tex_coords_.insert(m_tex_coords_.end(), uv_mapping.begin(), uv_mapping.begin() + 4);

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
}

GUIElement* Button::processMousePressEvent(int x, int y)
{
	if (is_pressed_)
	{
		return this;
	}
	is_pressed_ = true;
	
	m_tex_coords_.clear();
	m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin() + 4, complete_tex_coords_.end());
	
	if (parent_ != NULL)
	{
		parent_->updateBuffers();
	}

	for (std::vector<ButtonPressedListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
	{
		(*ci)->buttonPressed(*this);
	}

#ifdef _WIN32
	std::stringstream ss;
	ss << "Button pressed!" << label_ << std::endl;
	OutputDebugString(ss.str().c_str());
#else
	std::cout << "Button has been pressed!" << std::endl;
#endif
	return this;
}

void Button::processMouseReleasedEvent(int x, int y)
{
	if (!is_pressed_)
	{
		return;
	}
	is_pressed_ = false;

	// Make the button go 'down'.
	m_tex_coords_.clear();
	m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin(), complete_tex_coords_.begin() + 4);

	if (parent_ != NULL)
	{
		parent_->updateBuffers();
	}
}

void Button::addListener(ButtonPressedListener& listener)
{
	listeners_.push_back(&listener);
}

void Button::removeListener(ButtonPressedListener& listener)
{
	for (std::vector<ButtonPressedListener*>::iterator i = listeners_.begin(); i != listeners_.end(); ++i)
	{
		if (*i == &listener)
		{
			listeners_.erase(i);
			break;
		}
	}
}

void Button::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
}
