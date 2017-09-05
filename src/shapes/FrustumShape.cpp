#include "FrustumShape.h"

#include <glm/glm.hpp>

#include "GL/glew.h"

#include "Cube.h"
#include "../core/entities/camera/Camera.h"
#include "../core/light/Light.h"

FrustumShape::FrustumShape(float close_plane, float far_plane, float close_x, float close_y, float far_x, float far_y)
{
	glm::vec3 close_left_bottom(-close_x / 2.0f, -close_y / 2.0f, close_plane);
	glm::vec3 close_right_bottom(close_x / 2.0f, -close_y / 2.0f, close_plane);
	glm::vec3 close_left_top(-close_x / 2.0f, close_y / 2.0f, close_plane);
	glm::vec3 close_right_top(close_x / 2.0f, close_y / 2.0f, close_plane);
	
	glm::vec3 far_left_bottom(-far_x / 2.0f, -far_y / 2.0f, -far_plane);
	glm::vec3 far_right_bottom(far_x / 2.0f, -far_y / 2.0f, -far_plane);
	glm::vec3 far_left_top(-far_x / 2.0f, far_y / 2.0f, -far_plane);
	glm::vec3 far_right_top(far_x / 2.0f, far_y / 2.0f, -far_plane);
	
	// Close pane.
	m_vertices_.push_back(close_left_top);
	m_vertices_.push_back(close_left_bottom);
	m_vertices_.push_back(close_right_bottom);
	m_vertices_.push_back(close_right_top);

	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(0);
	m_indices_.push_back(3);
	m_indices_.push_back(1);

	m_indices_.push_back(1);
	m_indices_.push_back(3);
	m_indices_.push_back(2);

	// Far pane.
	m_vertices_.push_back(far_right_top);
	m_vertices_.push_back(far_right_bottom);
	m_vertices_.push_back(far_left_bottom);
	m_vertices_.push_back(far_left_top);

	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(4 + 1);
	m_indices_.push_back(4 + 3);
	m_indices_.push_back(4 + 0);
	
	m_indices_.push_back(4 + 2);
	m_indices_.push_back(4 + 3);
	m_indices_.push_back(4 + 1);
	
	// Bottom of the pane.
	m_vertices_.push_back(close_left_bottom);
	m_vertices_.push_back(far_left_bottom);
	m_vertices_.push_back(far_right_bottom);
	m_vertices_.push_back(close_right_bottom);

	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(8 + 0);
	m_indices_.push_back(8 + 3);
	m_indices_.push_back(8 + 2);
	
	m_indices_.push_back(8 + 0);
	m_indices_.push_back(8 + 2);
	m_indices_.push_back(8 + 1);

	// Top of the pane.
	m_vertices_.push_back(close_right_top);
	m_vertices_.push_back(far_right_top);
	m_vertices_.push_back(far_left_top);
	m_vertices_.push_back(close_left_top);

	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(12 + 2);
	m_indices_.push_back(12 + 3);
	m_indices_.push_back(12 + 0);
	
	m_indices_.push_back(12 + 1);
	m_indices_.push_back(12 + 2);
	m_indices_.push_back(12 + 0);
	
	// Left side of the pane.
	m_vertices_.push_back(close_left_bottom);
	m_vertices_.push_back(close_left_top);
	m_vertices_.push_back(far_left_top);
	m_vertices_.push_back(far_left_bottom);
	
	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(16 + 2);
	m_indices_.push_back(16 + 1);
	m_indices_.push_back(16 + 0);
	
	m_indices_.push_back(16 + 3);
	m_indices_.push_back(16 + 2);
	m_indices_.push_back(16 + 0);

	// Right side of the pane.
	m_vertices_.push_back(far_right_bottom);
	m_vertices_.push_back(far_right_top);
	m_vertices_.push_back(close_right_top);
	m_vertices_.push_back(close_right_bottom);
	
	m_tex_coords_.push_back(glm::vec2(1, 1));
	m_tex_coords_.push_back(glm::vec2(0, 1));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_tex_coords_.push_back(glm::vec2(1, 0));

	m_indices_.push_back(20 + 0);
	m_indices_.push_back(20 + 1);
	m_indices_.push_back(20 + 2);
	
	m_indices_.push_back(20 + 0);
	m_indices_.push_back(20 + 2);
	m_indices_.push_back(20 + 3);

	/// Front
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(close_left_bottom - close_left_top, close_right_top - close_left_top));
	/// Back
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(far_right_top - far_right_bottom, far_left_bottom - far_right_bottom));
	/// Bottom
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(far_left_bottom - far_right_bottom, close_right_bottom - far_right_bottom));
	/// Top
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(close_left_top - far_left_top, far_right_top - far_left_top));
	/// Left.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(far_left_top - far_left_bottom, close_left_bottom - far_left_bottom));
	/// Right.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(far_right_bottom - close_right_bottom, close_right_top - close_right_bottom));
	
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
