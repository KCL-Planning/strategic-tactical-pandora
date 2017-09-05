#include <iostream>

#include "TextBanner.h"


#include "../core/texture/Texture.h"

TextBanner::TextBanner(const std::string& text, const Texture& text_texture, float font_size)
	: text_(text)
{
	for (unsigned int i = 0; i < text.size(); ++i)
	{
		setUVMapping(text.at(i));
		
		m_vertices_.push_back(glm::vec3(font_size * i, 0.0f, 0.0f));
		m_vertices_.push_back(glm::vec3(font_size * i + font_size, 0.0f, 0.0f));
		m_vertices_.push_back(glm::vec3(font_size * i + font_size, font_size, 0.0f));
		m_vertices_.push_back(glm::vec3(font_size * i, font_size, 0.0f));
		
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		m_normals_.push_back(glm::vec3(0, 0, -1));
		
		m_indices_.push_back(i * 4);
		m_indices_.push_back(i * 4 + 3);
		m_indices_.push_back(i * 4 + 1);
		
		m_indices_.push_back(i * 4 + 1);
		m_indices_.push_back(i * 4 + 3);
		m_indices_.push_back(i * 4 + 2);
	}
	
	// Bind all this to OpenGL.
	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// To draw the upper part of the piramid.
	glGenBuffers(1, &m_index_buffer_);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_); //Bind the vertex buffer
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Generate the buffer for the texture coordinates.
	glGenBuffers(1, &m_tex_coord_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the normals.
	glGenBuffers(1, &m_normal_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);
}

void TextBanner::setUVMapping(const char& ch)
{
	float cell_size = 1.0f / 6.0f;
	
	int cell_number = ch;
	// Is it a number?
	if (ch >= 48 && ch <= 57)
	{
		cell_number -= (48 - 26);
	}
	// Is it a literal?
	else if ((ch >= 65 && ch <= 90) || (ch >= 97 && ch <= 122))
	{
		cell_number -= (ch <= 90 ? 65 : 97);
	}
	// Space
	else if (ch == 32)
	{
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
	}
	else
	{
		std::cerr << "Unknown character: " << ch << std::endl;
		// Defaults 'unknown' character texture.
		m_tex_coords_.push_back(glm::vec2(0.0f, 1.0f));
		m_tex_coords_.push_back(glm::vec2(1.0f, 1.0f));
		m_tex_coords_.push_back(glm::vec2(1.0f, 0.0f));
		m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
		return;
	}
	
	int row_nr = 5 - (cell_number / 6);
	int column_nr = cell_number % 6;
	
//	std::cout << ch << "[" << (int)ch << "]1 -> (" << column_nr << ", " << row_nr << ")" << std::endl;
	
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size, row_nr * cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size + cell_size, row_nr * cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size + cell_size, row_nr * cell_size + cell_size));
	m_tex_coords_.push_back(glm::vec2(column_nr * cell_size, row_nr * cell_size + cell_size));
}
