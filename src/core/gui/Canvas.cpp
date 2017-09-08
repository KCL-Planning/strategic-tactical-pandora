#include "dpengine/gui/Canvas.h"

namespace DreadedPE
{

Canvas::Canvas(const Theme& theme, Font& font, int x, int y, int size_x, int size_y, Texture& texture)
	: Container(theme, font, x, y, size_x, size_y, true)
{
	setTexture(texture);
	m_tex_coords_.clear();
	
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));
}

};
