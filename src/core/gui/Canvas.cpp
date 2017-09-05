#include "Canvas.h"

#include <GL/glfw.h>

Canvas::Canvas(const Theme& theme, Font& font, int x, int y, int size_x, int size_y, Texture& texture)
	: Container(theme, font, x, y, size_x, size_y, false)
{
	setTexture(texture);
	//setTextureUVMapping(theme.getMaximiseTexture());
	
	//m_tex_coords_ = theme.getMaximiseTexture();
	m_tex_coords_.clear();
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(1, 1));
	/*
	glBindBuffer(GL_ARRAY_BUFFER, tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * combined_tex_coords_.size(), &combined_tex_coords_[0], GL_STATIC_DRAW);
	
	combined_tex_coords_ = m_tex_coords_;
	glBindBuffer(GL_ARRAY_BUFFER, combined_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * combined_tex_coords_.size(), &combined_tex_coords_[0], GL_DYNAMIC_DRAW);
	*/
	//updateBuffers();
	
	
	/*
	int width, height;
	glfwGetWindowSize(&width, &height);
	
	m_vertices_.push_back(glm::vec3(0, height, 0));
	m_vertices_.push_back(glm::vec3(size_x, height, 0));
	m_vertices_.push_back(glm::vec3(0, height - size_y, 0));
	m_vertices_.push_back(glm::vec3(size_x, height - size_y, 0));
	
	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(2);

	m_indices_.push_back(1);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
	

	*/
}
