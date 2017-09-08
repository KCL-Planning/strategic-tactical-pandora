#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/gui/CheckBox.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/gui/themes/Theme.h"
#include "dpengine/gui/Container.h"
#include "dpengine/gui/events/CheckBoxChangeListener.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

CheckBox::CheckBox(Theme& theme, float size_x, float size_y)
	: GUIElement(theme, 0, 0, size_x, size_y), is_pressed_(false), mouse_is_down_(false)
{
	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

	// Test data.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	complete_tex_coords_= theme.getCheckBoxTexture();
	m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin(), complete_tex_coords_.begin() + 4);

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
}

void CheckBox::addStateChangeListener(CheckBoxChangeListener& listener)
{
	for (std::vector<CheckBoxChangeListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
	{
		if (*ci == &listener)
		{
			return;
		}
	}
	listeners_.push_back(&listener);
}

void CheckBox::removeStateChangeListener(CheckBoxChangeListener& listener)
{
	for (std::vector<CheckBoxChangeListener*>::iterator i = listeners_.begin(); i != listeners_.end(); ++i)
	{
		if (*i == &listener)
		{
			listeners_.erase(i);
			return;
		}
	}
}

GUIElement* CheckBox::processMousePressEvent(int x, int y)
{
	markForUpdate();
	if (mouse_is_down_)
	{
		return this;
	}
	mouse_is_down_ = true;
	
	is_pressed_ = !is_pressed_;

	if (is_pressed_)
	{
		m_tex_coords_.clear();
		m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin() + 4, complete_tex_coords_.begin() + 8);
	}
	else
	{
		m_tex_coords_.clear();
		m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin(), complete_tex_coords_.begin() + 4);
	}

	for (std::vector<CheckBoxChangeListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
	{
		(*ci)->checkBoxStateChanged(*this);
	}

	if (parent_ != NULL)
	{
		parent_->updateBuffers();
	}

	return this;
	
	/*
	for (std::vector<ButtonPressedListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
	{
		(*ci)->buttonPressed(*this);
	}
	*/
}

void CheckBox::setPressed(bool pressed)
{
	markForUpdate();
	is_pressed_ = pressed;
	if (is_pressed_)
	{
		m_tex_coords_.clear();
		m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin() + 4, complete_tex_coords_.begin() + 8);
	}
	else
	{
		m_tex_coords_.clear();
		m_tex_coords_.insert(m_tex_coords_.end(), complete_tex_coords_.begin(), complete_tex_coords_.begin() + 4);
	}
}

void CheckBox::processMouseReleasedEvent(int x, int y)
{
	markForUpdate();
	if (!mouse_is_down_)
	{
		return;
	}
	mouse_is_down_ = false;
}

void CheckBox::onResize(int width, int height)
{
	markForUpdate();
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
}

};
