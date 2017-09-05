#include <GL/glew.h>
#include <GL/glfw.h>

#include "../shaders/GUIShader.h"

#include "ScrollbarUnit.h"
#include "Scrollbar.h"
#include "Container.h"
#include "themes/Theme.h"

ScrollbarUnit::ScrollbarUnit(Theme& theme, float x, float y, float size_x, float size_y, Scrollbar& scrollbar)
	: GUIElement(theme, x, y, size_x, size_y), scrollbar_(&scrollbar), scroll_unit_size_(scrollbar.isVertical() ? size_y : size_x), is_grapped_(false), grap_location_(0), org_location_(0)
{
	//m_tex_coords_.insert(m_tex_coords_.end(), theme.getScrollbarUnitTexture().begin(), theme.getScrollbarUnitTexture().end());
	m_tex_coords_.insert(m_tex_coords_.end(), theme.getButtonTexture().begin(), theme.getButtonTexture().end());
	
	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
	
	setContentSize(size_x, size_y);
}

GUIElement* ScrollbarUnit::processMousePressEvent(int x, int y)
{
	// Check if the frame was clicked.
	if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS &&
	    x >= global_transformation_[3][0] && x <= global_transformation_[3][0] + scroll_unit_size_ &&
	    y >= -global_transformation_[3][1] && y <= -global_transformation_[3][1] + scroll_unit_size_)
	{
		if (!is_grapped_)
		{
			if (scrollbar_->isVertical())
			{
				grap_location_ = y;
				org_location_ = local_transformation_[3][1];
			}
			else
			{
				grap_location_ = x;
				org_location_ = local_transformation_[3][0];
			}
			is_grapped_ = true;
			//std::cout << "Grapped at: " << x << "." << std::endl;

			if (parent_ != NULL)
			{
				parent_->updateBuffers();
			}
		}
		return this;
	}
	return NULL;
}

void ScrollbarUnit::processMouseReleasedEvent(int x, int y)
{
	is_grapped_ = false;
}

void ScrollbarUnit::setContentSize(float size_x, float size_y)
{
	m_vertices_.clear();
	
	int width, height;
	glfwGetWindowSize(&width, &height);
	
	if (scrollbar_->isVertical())
	{
		// Determine the size of the scroll unit.
		float size = std::max(10.0f, size_y_ / scrollbar_->getScrollee().getContentSizeY());
		
		// If the scrollbar is smaller than the container we do not need to
		// render anything, whoohoo! Then why did you put a scrollbar here, dumbass???
		if (size >= 1.0f)
		{
			size = 1.0f;
		}
		
		scroll_unit_size_ = size * 10;
		
		m_vertices_.push_back(glm::vec3(0, height, 0));
		m_vertices_.push_back(glm::vec3(size_x, height, 0));
		m_vertices_.push_back(glm::vec3(0, height - scroll_unit_size_, 0));
		m_vertices_.push_back(glm::vec3(size_x, height - scroll_unit_size_, 0));
		
		// Update the location of the scroll unit.
		float d = scrollbar_->getScrollee().getLocalY() / scrollbar_->getScrollee().getContentSizeY();
		local_transformation_[3][1] = size_y_ * d;
	}
	else
	{
		// Determine the size of the scroll unit.
		float size = std::max(10.0f, size_x_ / scrollbar_->getScrollee().getContentSizeX());
		
		// If the scrollbar is smaller than the container we do not need to
		// render anything, whoohoo! Then why did you put a scrollbar here, dumbass???
		if (size >= 1.0f)
		{
			size = 1.0f;
		}
		
		scroll_unit_size_ = size * 10;
		
		m_vertices_.push_back(glm::vec3(0, height, 0));
		m_vertices_.push_back(glm::vec3(scroll_unit_size_, height, 0));
		m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
		m_vertices_.push_back(glm::vec3(scroll_unit_size_, height - size_y, 0));
		
		// Update the location of the scroll unit.
		float d = scrollbar_->getScrollee().getLocalX() / scrollbar_->getScrollee().getContentSizeX();
		local_transformation_[3][0] = size_x_ * d;
	}
	
	// Determine the new location of the scroll unit.
	if (parent_ != NULL)
	{
		parent_->updateBuffers();
	}
}

void ScrollbarUnit::update(float dt)
{
	int mouse_x, mouse_y;
	glfwGetMousePos(&mouse_x, &mouse_y);
	
	// Move the container relative to the scroll unit :).
	if (is_grapped_)
	{
		//std::cout << "Grapped!" << std::endl;
		if (scrollbar_->isVertical())
		{
			local_transformation_[3][1] = org_location_ - (mouse_y - grap_location_);
			if (local_transformation_[3][1] > 0)
			{
				local_transformation_[3][1] = 0;
			}
			else if (local_transformation_[3][1] < -size_y_ + scroll_unit_size_)
			{
				local_transformation_[3][1] = -size_y_ + scroll_unit_size_;
			}
			// Update the local transformation of the container.
			Container& scrollee = scrollbar_->getScrollee();
			scrollee.setPosition(scrollee.getLocalX(), -(local_transformation_[3][1] / (size_y_ - scroll_unit_size_)) * (scrollee.getContentSizeY() - getHeight()));
		}
		else
		{
			local_transformation_[3][0] = org_location_ + (mouse_x - grap_location_);
			if (local_transformation_[3][0] < 0)
			{
				local_transformation_[3][0] = 0;
			}
			else if (local_transformation_[3][0] > size_x_ - scroll_unit_size_)
			{
				local_transformation_[3][0] = size_x_ - scroll_unit_size_;
			}
			
			// Update the local transformation of the container.
			Container& scrollee = scrollbar_->getScrollee();
			scrollee.setPosition(-(local_transformation_[3][0] / (size_x_ - scroll_unit_size_)) * (scrollee.getContentSizeX() - getWidth()), scrollee.getLocalY());
		}
		
		// Update global transformation immediately.
		scrollbar_->getScrollee().updateTransformations();
		if (parent_ != NULL)
		{
			parent_->updateBuffers();
		}
	}

	GUIElement::update(dt);
}

void ScrollbarUnit::onResize(int width, int height)
{
	m_vertices_.clear();
	if (scrollbar_->isVertical())
	{
		m_vertices_.push_back(glm::vec3(0, height, 0));
		m_vertices_.push_back(glm::vec3(size_x_, height, 0));
		m_vertices_.push_back(glm::vec3(0, height - scroll_unit_size_, 0));
		m_vertices_.push_back(glm::vec3(size_x_, height - scroll_unit_size_, 0));
		size_y_ = height;
	}
	else
	{
		m_vertices_.push_back(glm::vec3(0, height, 0));
		m_vertices_.push_back(glm::vec3(scroll_unit_size_, height, 0));
		m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
		m_vertices_.push_back(glm::vec3(scroll_unit_size_, height - size_y_, 0));
		size_x_ = width;
	}
}
