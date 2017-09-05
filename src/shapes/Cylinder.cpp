#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "GL/glew.h"

#include "Cylinder.h"
#include "../core/entities/camera/Camera.h"
#include "../core/light/Light.h"

Cylinder::Cylinder(float height, float width, int sections)
{
	float current_angle = 0;
	for (int current_section = 0; current_section < sections; ++current_section, current_angle += 360.0f / sections)
	{
		glm::vec3 point(0, height / 2.0f, width / 2.0f);
		point = glm::rotate(point, current_angle, glm::vec3(0, 1, 0));
		
		glm::vec3 point2(0, -height / 2.0f, width / 2.0f);
		point2 = glm::rotate(point2, current_angle, glm::vec3(0, 1, 0));
		
		m_vertices_.push_back(point);
		m_vertices_.push_back(point2);
		
		glm::vec3 normal(0, 0, 1.0f);
		normal = glm::rotate(normal, current_angle, glm::vec3(0, 1, 0));
		m_normals_.push_back(glm::normalize(normal));
		m_normals_.push_back(glm::normalize(normal));
		
		m_tex_coords_.push_back(glm::vec2(1.0f / sections, 0));
		m_tex_coords_.push_back(glm::vec2(1.0f / sections, 1));
		
		m_indices_.push_back((1 + 2 * current_section) % (sections * 2));
		m_indices_.push_back((3 + 2 * current_section) % (sections * 2));
		m_indices_.push_back((0 + 2 * current_section) % (sections * 2));

		m_indices_.push_back((2 + 2 * current_section) % (sections * 2));
		m_indices_.push_back((0 + 2 * current_section) % (sections * 2));
		m_indices_.push_back((3 + 2 * current_section) % (sections * 2));
	}
	
	// Get the top and bottom bit.
	current_angle = 0;
	for (int current_section = 0; current_section < sections; ++current_section, current_angle += 360.0f / sections)
	{
		glm::vec3 point(0, height / 2.0f, width / 2.0f);
		point = glm::rotate(point, current_angle, glm::vec3(0, 1, 0));
		
		glm::vec3 point2(0, -height / 2.0f, width / 2.0f);
		point2 = glm::rotate(point2, current_angle, glm::vec3(0, 1, 0));
		
		m_vertices_.push_back(point);
		m_vertices_.push_back(point2);
		
		m_normals_.push_back(glm::vec3(0, 1, 0));
		m_normals_.push_back(glm::vec3(0, -1, 0));
		
		m_tex_coords_.push_back(glm::vec2(1.0f / sections, 0));
		m_tex_coords_.push_back(glm::vec2(1.0f / sections, 1));
		
		if (current_section > 1)
		{
			m_indices_.push_back(0);
			m_indices_.push_back(current_section * 2 - 2);
			m_indices_.push_back(current_section * 2);
		
			m_indices_.push_back(1);
			m_indices_.push_back(current_section * 2 + 1);
			m_indices_.push_back(current_section * 2 - 1);
		}
	}
	
	for (std::vector<GLuint>::const_iterator ci = m_indices_.begin(); ci != m_indices_.end(); ++ci)
	{
		std::cout << *ci << ", ";
	}
	std::cout << std::endl;

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
