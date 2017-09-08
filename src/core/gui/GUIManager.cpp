#include <algorithm>

#include "dpengine/gui/GUIManager.h"
#include "dpengine/gui/Frame.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

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

void GUIManager::draw(const glm::mat4& perspective_matrix) const
{
	for (std::vector<Container*>::const_iterator i = active_frames_.begin(); i != active_frames_.end(); ++i)
	{
		(*i)->draw(perspective_matrix, 0);
	}
}

void GUIManager::tick(float dt)
{
	Window* window = Window::getActiveWindow();
	double mouse_x, mouse_y;
	window->getMouseCursor(mouse_x, mouse_y);

	if (window->isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
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

	else if (!window->isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT) && picked_gui_element_ != NULL)
	{
		if (picked_gui_element_ != NULL)
		{
			picked_gui_element_->processMouseReleasedEvent(mouse_x, mouse_y);
			picked_gui_element_ = NULL;
		}
	}


	for (Container* c : frames_to_remove_)
	{
		std::vector<Container*>::iterator find_i = std::find(active_frames_.begin(), active_frames_.end(), c);
		if (find_i != active_frames_.end())
		{
			if (picked_gui_element_ != NULL && (*find_i)->containsElement(*picked_gui_element_))
			{
				picked_gui_element_ = NULL;
			}
			active_frames_.erase(find_i);
			delete c;
		}
	}
	frames_to_remove_.clear();

	active_frames_.insert(active_frames_.end(), frames_to_add_.begin(), frames_to_add_.end());
	frames_to_add_.clear();

	for (Container* c : active_frames_)
	{
		if (c->needsUpdate())
		{
			c->update(dt);
		}
	}
}

void GUIManager::onResize(int width, int height)
{
	for (std::vector<Container*>::const_iterator i = active_frames_.begin(); i != active_frames_.end(); ++i)
	{
		(*i)->onResize(width, height);
	}
}

};
