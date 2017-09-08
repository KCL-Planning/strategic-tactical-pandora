#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/gui/Label.h"
#include "dpengine/shaders/GUIShader.h"

//#include "FreeTypeFont.h"
#include "dpengine/gui/events/ButtonPressedListener.h"
#include "dpengine/gui/themes/Theme.h"
#include "dpengine/gui/fonts/Font.h"
#include "dpengine/renderer/Window.h"

namespace DreadedPE
{

Label::Label(const Theme& theme, float size_x, float size_y, const std::string& label, float font_size)
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), font_size_(font_size)
{
	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

	// Test data.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	m_tex_coords_ = theme.getLabelTexture();

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
}

Label::Label(const Theme& theme, float size_x, float size_y, const std::string& label, float font_size, const std::vector<glm::vec2>& uv_mapping)
	: FontRenderingGUIElement(theme, 0, 0, size_x, size_y), label_(label), font_size_(font_size)
{
	Window* window = Window::getActiveWindow();
	int width, height;
	window->getSize(width, height);

	// Test data.
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	m_tex_coords_ = uv_mapping;

	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
}

void Label::setLabel(const std::string& label)
{
	label_ = label;
	need_to_update_font_ = true;
	markForUpdate();
}

void Label::onResize(int width, int height)
{
	m_vertices_.clear();
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y_, 0));
	m_vertices_.push_back(glm::vec3(size_x_, height - size_y_, 0));
	markForUpdate();
}

};
