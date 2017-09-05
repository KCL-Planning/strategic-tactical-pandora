#include <iostream>
#include <stdlib.h> 

#include "Font.h"

Font::Font(Texture& texture)
	: buffers_need_updating_(false), font_texture_(&texture)
{
	glGenBuffers(1, &m_vertex_buffer_);
	glGenBuffers(1, &m_index_buffer_);
	glGenBuffers(1, &m_tex_coord_buffer_);
	glGenBuffers(1, &m_normal_buffer_);
}

void Font::appendString(const std::vector<glm::vec3>& m_vertices,
	                const std::vector<glm::vec2>& m_tex_coords,
	                const std::vector<GLuint>& m_indices,
	                const std::vector<glm::vec3>& m_normals)
{
	//std::cout << "[Font::appendString]" << std::endl;
	m_vertices_.insert(m_vertices_.end(), m_vertices.begin(), m_vertices.end());
	m_tex_coords_.insert(m_tex_coords_.end(), m_tex_coords.begin(), m_tex_coords.end());
	m_indices_.insert(m_indices_.end(), m_indices.begin(), m_indices.end());
	m_normals_.insert(m_normals_.end(), m_normals.begin(), m_normals.end());
	
	markBuffersForUpdate();
}

void Font::clearText()
{
	m_vertices_.clear();
	m_tex_coords_.clear();
	m_indices_.clear();
	m_normals_.clear();

	markBuffersForUpdate();
}

void Font::finaliseBuffers()
{
	//std::cout << "[Font::finaliseBuffers]" << std::endl;
	if (buffers_need_updating_ && m_vertices_.size() > 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW);

		glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);

		buffers_need_updating_ = false;
	}
}
