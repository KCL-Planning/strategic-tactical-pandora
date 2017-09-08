#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

#include "dpengine/gui/Frame.h"

#include "dpengine/gui/fonts/Font.h"
#include "dpengine/gui/themes/Theme.h"
#include "dpengine/shaders/GUIShader.h"
#include "dpengine/gui/GUIManager.h"
#include "dpengine/renderer/Window.h"
#include "dpengine/gui/Button.h"

namespace DreadedPE
{

Frame::Frame(Theme& theme, Font& font, float x, float y, float size_x, float size_y)
	: Container(theme, font, x, y, size_x, size_y, false), font_(&font), org_size_x(size_x), org_size_y_(size_y), is_minimized_(false), frame_grapped_(false), grap_location_x_(0), grap_location_y_(0), org_location_x_(x), org_location_y_(y)
{
	menu_bar_ = new Container(theme, font.clone(), x, y, size_x, 20, false);
	content_pane_ = new Container(theme, font.clone(), x, y, size_x, size_y - 20, false);

	Container::addElement(*menu_bar_, 0.0f, 0.0f);
	Container::addElement(*content_pane_, 0.0f, -20.0f);

	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

	local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(x, -y, 0.0f));

	m_tex_coords_ = theme.getFrameTexture();

	// Add a button in the top right of the frame.
	close_button_ = new Button(theme, 10, 10, "", 12, theme.getCloseButtonTexture());
	menu_bar_->addElement(*close_button_, size_x - 15, -5);

	std::vector<glm::vec2> resize_texture = theme.getMinimiseTexture();
	resize_texture.insert(resize_texture.end(), theme.getMaximiseTexture().begin(), theme.getMaximiseTexture().end());

	resize_button_ = new Button(theme, 10, 10, "", 12, resize_texture);
	menu_bar_->addElement(*resize_button_, size_x - 30, -5);
	
	close_button_->addListener(*this);
	resize_button_->addListener(*this);
}

GUIElement* Frame::processMousePressEvent(int mouse_x, int mouse_y)
{
	markForUpdate();
	if (!frame_grapped_)
	{
		GUIElement* activated_element = Container::processMousePressEvent(mouse_x, mouse_y);
		if (activated_element != NULL)
		{
			return activated_element;
		}
	}

	// Check if the frame was clicked.
	if (mouse_x >= global_transformation_[3][0] && mouse_x <= global_transformation_[3][0] + size_x_ &&
	    mouse_y >= -global_transformation_[3][1] && mouse_y <= -global_transformation_[3][1] + 20)
	{
		if (!frame_grapped_)
		{
			grap_location_x_ = mouse_x;
			grap_location_y_ = mouse_y;
			frame_grapped_ = true;

			org_location_x_ = local_transformation_[3][0];
			org_location_y_ = local_transformation_[3][1];
		}
	}

	// Move the frame :).
	if (frame_grapped_)
	{
		Window* window = Window::getActiveWindow();
		int width, height;
		window->getSize(width, height);
		
		float diff_x = grap_location_x_ - mouse_x;
		float diff_y = grap_location_y_ - mouse_y;

		local_transformation_[3][0] = org_location_x_ - diff_x;
		local_transformation_[3][1] = org_location_y_ + diff_y;

		// Update global transformation immediately.
		updateTransformations();
		return this;
	}

	return NULL;
}

void Frame::processMouseReleasedEvent(int mouse_x, int mouse_y)
{
	markForUpdate();
	Window* window = Window::getActiveWindow();
	
	if (!window->isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
	{
		frame_grapped_ = false;
		org_location_x_ = local_transformation_[3][0];
		org_location_y_ = local_transformation_[3][1];
	}
}

void Frame::draw(const glm::mat4& perspective_matrix, int level) const
{
	// Set the clear value.
	glClearStencil(0);
	glClear(GL_STENCIL_BUFFER_BIT);
	
	// Use the stensil buffer to make sure we don't render anything outside the container.
	glEnable(GL_STENCIL_TEST);
	
	if (!is_minimized_)
	{
		Container::draw(perspective_matrix, 1);
	}
	else
	{
		menu_bar_->draw(perspective_matrix, 1);
	}

	glDisable(GL_STENCIL_TEST);
}

void Frame::update(float dt)
{
	if (needs_update_ || is_selected_)
	{
		m_tex_coords_ = theme_->getFrameTexture();
		if (!is_minimized_)
		{
			Container::update(dt);
		}
		else
		{
			menu_bar_->update(dt);
		}
	}
}

void Frame::buttonPressed(const Button& source)
{
	markForUpdate();
	if (&source == close_button_)
	{
		// Close the frame.
		GUIManager& gui_manager = GUIManager::getInstance();
		gui_manager.deleteFrame(*this);
	}
	else if (&source == resize_button_)
	{
		if (!is_minimized_)
		{
			size_y_ = 20;
			is_minimized_ = true;
			content_pane_->setVisible(false);
		}
		else
		{
			size_y_ = org_size_y_;
			is_minimized_ = false;
			content_pane_->setVisible(true);
		}
		
		Window* window = Window::getActiveWindow();
		int width, height;
		window->getSize(width, height);

		m_vertices_[2].y = height - size_y_;
		m_vertices_[3].y = height - size_y_;
		updateBuffers();
	}
}

void Frame::addElement(GUIElement& child, float x, float y)
{
	content_pane_->addElement(child, x, y);
	markForUpdate();
}

void Frame::removeElement(GUIElement& child)
{
	content_pane_->removeElement(child);
	markForUpdate();
}

void Frame::addElement(Container& child, float x, float y)
{
	content_pane_->addElement(child, x, y);
	markForUpdate();
}

void Frame::removeElement(Container& child)
{
	content_pane_->removeElement(child);
	markForUpdate();
}

};
