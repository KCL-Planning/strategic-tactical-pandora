#include "Shape.h"

Shape::Shape(const std::vector<glm::vec3>& m_vertices,
	     const std::vector<glm::vec2>& m_tex_coords,
	     const std::vector<GLuint>& m_indices,
	     const std::vector<glm::vec3>& m_normals)
{
	m_vertices_.insert(m_vertices_.end(), m_vertices.begin(), m_vertices.end());
	m_tex_coords_.insert(m_tex_coords_.end(), m_tex_coords.begin(), m_tex_coords.end());
	m_indices_.insert(m_indices_.end(), m_indices.begin(), m_indices.end());
	m_normals_.insert(m_normals_.end(), m_normals.begin(), m_normals.end());

	// Initialise default values for the bone ids and weights.
	m_bone_ids_.resize(m_vertices_.size(), glm::ivec4(0, 0, 0, 0));
	m_bone_weights_.resize(m_vertices_.size(), glm::vec4(0.0f, 0.0f, 0.0f, 0.0f));
	
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

	// Generate the buffers to store the bone ids.
	glGenBuffers(1, &m_bone_ids_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_ids_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLint) * 4 * m_bone_ids_.size(), &m_bone_ids_[0], GL_STATIC_DRAW);

	// Generate the buffers to store the weights of the bones.
	glGenBuffers(1, &m_bone_weight_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_bone_weight_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 4 * m_bone_weights_.size(), &m_bone_weights_[0], GL_STATIC_DRAW);
}


void Shape::setVertexBuffer(const std::vector<glm::vec3>& v)
{
	if (v.empty()) return;
	m_vertices_ = v;
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_DYNAMIC_DRAW); //Send the data to OpenGL
}

void Shape::setTexCoords(const std::vector<glm::vec2>& c)
{
	if (c.empty()) return;
	m_tex_coords_ = c;
	glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_DYNAMIC_DRAW); //Send the data to OpenGL
}

void Shape::prepare(float dt)
{

}

void Shape::render()
{
	//Bind the index array
	// NOTE: Should be done by the shader now using Vertex Array Objects (VAOs).
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, getIndexBufferId());

	//Draw the triangles
	glDrawElements(GL_TRIANGLES, getIndices().size(), GL_UNSIGNED_INT, 0);
}
