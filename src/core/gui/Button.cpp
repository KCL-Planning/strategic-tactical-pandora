#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <string>

#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/gui/fonts/Font.h"
#include "dpengine/gui/Button.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/gui/fonts/TexturedFont.h"
#include "dpengine/gui/Container.h"
#include "dpengine/renderer/Window.h"

//#include "FreeTypeFont.h"
#include "dpengine/gui/events/ButtonPressedListener.h"
#include "dpengine/gui/themes/Theme.h"

namespace DreadedPE
{

Button::Button(Theme& theme, float size_x, float size_y, const std::string& label, float font_size)
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), font_size_(font_size), is_pressed_(false)
{
	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

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
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), font_size_(font_size), is_pressed_(false)
{
	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

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
	markForUpdate();
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
	markForUpdate();
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
	markForUpdate();
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
}

};
