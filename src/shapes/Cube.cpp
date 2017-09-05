#include <glm/glm.hpp>

#include "GL/glew.h"

#include "Cube.h"
#include "../core/entities/camera/Camera.h"
#include "../core/light/Light.h"

Cube::Cube(float size)
{
	initialise(size, size, size);
}

Cube::Cube(float width, float height, float depth)
{
	initialise(width, height, depth);
}

Cube::Cube(const glm::vec3& bottom_left_away, 
           const glm::vec3& bottom_right_away, 
           const glm::vec3& top_left_away, 
           const glm::vec3& top_right_away, 
           const glm::vec3& bottom_left_close, 
           const glm::vec3& bottom_right_close, 
           const glm::vec3& top_left_close, 
           const glm::vec3& top_right_close)
{
	// Top of the cube.
	m_vertices_.push_back(top_right_away);
	m_vertices_.push_back(top_left_away);
	m_vertices_.push_back(top_left_close);
	m_vertices_.push_back(top_right_close);

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

	// Bottom of the cube.
	m_vertices_.push_back(bottom_right_away);
	m_vertices_.push_back(bottom_left_away);
	m_vertices_.push_back(bottom_left_close);
	m_vertices_.push_back(bottom_right_close);

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
	
	// Front of the cube.
	m_vertices_.push_back(top_right_close);
	m_vertices_.push_back(top_left_close);
	m_vertices_.push_back(bottom_left_close);
	m_vertices_.push_back(bottom_right_close);

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

	// Back of the cube.
	m_vertices_.push_back(top_right_away);
	m_vertices_.push_back(top_left_away);
	m_vertices_.push_back(bottom_left_away);
	m_vertices_.push_back(bottom_right_away);

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
	
	// Left side of the cube.
	m_vertices_.push_back(top_left_away);
	m_vertices_.push_back(bottom_left_away);
	m_vertices_.push_back(bottom_left_close);
	m_vertices_.push_back(top_left_close);

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

	// Right side of the cube.
	m_vertices_.push_back(top_right_away);
	m_vertices_.push_back(bottom_right_away);
	m_vertices_.push_back(bottom_right_close);
	m_vertices_.push_back(top_right_close);

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

	// Normals.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(top_right_away - top_left_away, top_left_close - top_left_away));
		//m_normals_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(bottom_left_close - bottom_left_away, bottom_right_away - bottom_left_away));
		//m_normals_.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	// Front
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(bottom_left_close - top_left_close, top_right_close - top_left_close));
		//m_normals_.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	// Back
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(top_right_away - top_left_away, bottom_left_away - top_left_away));
		//m_normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	// Right.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(bottom_right_close - top_right_close, top_right_away - top_right_close));
	// Left.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::cross(top_left_away - top_left_close, bottom_left_close - top_left_close));
	
	// Bind all this to OpenGL.
	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	//glGenBuffers(1, &m_colour_buffer_);
    //glBindBuffer(GL_ARRAY_BUFFER, m_colour_buffer_); //Bind the vertex buffer
    //glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_colours_.size(), &m_colours_[0], GL_STATIC_DRAW); //Send the data to OpenGL
	
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

void Cube::initialise(float width, float height, float depth)
{
	// Top of the cube.
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, -depth / 2));

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

	// Bottom of the cube.
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, -depth / 2));

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
	
	// Back of the cube.
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, -depth / 2));

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

	// Front of the cube.
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, depth / 2));

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
	
	// Left side of the cube.
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, -height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(-width / 2, height / 2, -depth / 2));

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

	// Right side of the cube.
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, -height / 2, -depth / 2));
	m_vertices_.push_back(glm::vec3(width / 2, height / 2, -depth / 2));

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

	// Normals.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(0.0f, 1.0f, 0.0f));
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(0.0f, -1.0f, 0.0f));
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(0.0f, 0.0f, -1.0f));
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(0.0f, 0.0f, 1.0f));
	// Right.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(-1.0f, 0.0f, 0.0f));
	// Left.
	for (unsigned int i = 0; i < 4; ++i)
		m_normals_.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
	
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
