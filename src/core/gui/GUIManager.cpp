#include <algorithm>
#include <GL/glew.h>
#include <GL/glfw.h>

#include "GUIManager.h"

#include "Frame.h"

GUIManager* GUIManager::gui_manager_ = NULL;

GUIManager::GUIManager()
	: picked_gui_element_(NULL)
{

}

GUIManager& GUIManager::getInstance()
{
	if (gui_manager_ == NULL)
	{
		gui_manager_ = new GUIManager();
	}
	return *gui_manager_;
}

Frame& GUIManager::createFrame(Theme& theme, Font& font, float x, float y, float size_x, float size_y)
{
	Frame* frame = new Frame(theme, font, x, y, size_x, size_y);
	active_frames_.push_back(frame);
	return *frame;
}

void GUIManager::deleteFrame(Container& frame)
{
	for (std::vector<Container*>::iterator i = active_frames_.begin(); i != active_frames_.end(); ++i)
	{
		if (*i == &frame)
		{
			inactive_frames_.push_back(&frame);
			break;
		}
	}
}

void GUIManager::addFrame(Container& frame)
{
	active_frames_.push_back(&frame);
}

void GUIManager::draw(const glm::mat4& perspective_matrix) const
{
	for (std::vector<Container*>::const_iterator i = active_frames_.begin(); i != active_frames_.end(); ++i)
	{
		(*i)->draw(perspective_matrix, 0);
	}
}

void GUIManager::update(float dt)
{
	// First process
	int mouse_x, mouse_y;
	glfwGetMousePos(&mouse_x, &mouse_y);

	if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		if (picked_gui_element_ == NULL)
		{
			for (std::vector<Container*>::reverse_iterator ri = active_frames_.rbegin(); ri != active_frames_.rend(); ++ri)
			{
				Container* frame = *ri;

				// Check if the mouse falls within this frame.
				if (frame->getGlobalX() < mouse_x && frame->getGlobalX() + frame->getWidth() > mouse_x &&
				    frame->getGlobalY() < mouse_y && frame->getGlobalY() + frame->getHeight() > mouse_y)
				{
					GUIElement* element = frame->processMousePressEvent(mouse_x, mouse_y);

					if (element != NULL)
					{
						picked_gui_element_ = element;
					}

					// Push this frame to the back of the list (so it is rendered in front of all the other frames and processed first).
					active_frames_.erase((ri + 1).base());
					active_frames_.push_back(frame);

					break;
				}
			}
		}
		else
		{
			picked_gui_element_->processMousePressEvent(mouse_x, mouse_y);
		}
	}

	else if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE && picked_gui_element_ != NULL)
	{
		if (picked_gui_element_ != NULL)
		{
			picked_gui_element_->processMouseReleasedEvent(mouse_x, mouse_y);
			picked_gui_element_ = NULL;
		}
	}


	for (std::vector<Container*>::iterator i = inactive_frames_.begin(); i != inactive_frames_.end(); ++i)
	{
		std::vector<Container*>::iterator find_i = std::find(active_frames_.begin(), active_frames_.end(), *i);
		if (find_i != active_frames_.end())
		{
			active_frames_.erase(find_i);
		}
		delete *i;
	}
	inactive_frames_.clear();

	for (std::vector<Container*>::reverse_iterator i = active_frames_.rbegin(); i != active_frames_.rend(); ++i)
	{
		(*i)->update(dt);
	}
}

void GUIManager::onResize(int width, int height)
{
	for (std::vector<Container*>::const_iterator i = active_frames_.begin(); i != active_frames_.end(); ++i)
	{
		(*i)->onResize(width, height);
	}
}
