#include "Piramid.h"
#include "../core/entities/camera/Camera.h"
#include "GL/glew.h"
#include "../core/light/Light.h"

Piramid::Piramid(float height, float width, float length)
{
	// All the vertices.
    m_vertices_.push_back(glm::vec3(0.0f, 0.0f, height));
	m_vertices_.push_back(glm::vec3(width / 2, length / 2, 0.0f));
	m_vertices_.push_back(glm::vec3(-width / 2, length / 2, 0.0f));
	m_vertices_.push_back(glm::vec3(width / 2, -length / 2, 0.0f));
	m_vertices_.push_back(glm::vec3(-width / 2, -length / 2, 0.0f));

	// Top of the piramid.
	m_indices_.push_back(0);
	m_indices_.push_back(2);
	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(4);
	m_indices_.push_back(2);
	m_indices_.push_back(0);
	m_indices_.push_back(3);
	m_indices_.push_back(4);
	m_indices_.push_back(0);
	m_indices_.push_back(1);
	m_indices_.push_back(3);

	// Bottom of the piramid.
	m_indices_.push_back(1);
	m_indices_.push_back(3);
	m_indices_.push_back(2);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
	m_indices_.push_back(4);
	
	// Calculate the normal vectors for the triangle fan.
	m_normals_.resize(m_vertices_.size());
	std::vector<unsigned int> shareCount;
	shareCount.resize(m_vertices_.size());

	for (unsigned int i = 0; i < shareCount.size(); ++i)
	{
		shareCount[i] = 0;
	}

	for (unsigned int i = 0; i < m_normals_.size(); ++i)
	{
		m_normals_[i] = glm::vec3(0.0f, 0.0f, 0.0f);
	}

	for (unsigned int i = 1; i < m_indices_.size() - 1; ++i)
	{
		float Ax = m_vertices_[m_indices_[i] * 3].x - m_vertices_[m_indices_[0] * 3].x;
		float Ay = m_vertices_[m_indices_[i] * 3].y - m_vertices_[m_indices_[0] * 3].y;
		float Az = m_vertices_[m_indices_[i] * 3].z - m_vertices_[m_indices_[0] * 3].z;
		glm::vec3 A(Ax, Ay, Az);

		float Bx = m_vertices_[m_indices_[i + 1] * 3].x - m_vertices_[m_indices_[0] * 3].x;
		float By = m_vertices_[m_indices_[i + 1] * 3].y - m_vertices_[m_indices_[0] * 3].y;
		float Bz = m_vertices_[m_indices_[i + 1] * 3].z - m_vertices_[m_indices_[0] * 3].z;
		glm::vec3 B(Bx, By, Bz);

		glm::vec3 C = glm::cross(A, B);
		C = glm::normalize(C);

		for (unsigned int j = 0; j < 2; ++j)
		{
			m_normals_[m_indices_[0] * 3].x += C.x;
			m_normals_[m_indices_[0] * 3].y += C.y;
			m_normals_[m_indices_[0] * 3].z += C.z;
			shareCount[m_indices_[0]] = shareCount[m_indices_[0]] + 1;

			m_normals_[m_indices_[i + j] * 3].x += C.x;
			m_normals_[m_indices_[i + j] * 3].y += C.y;
			m_normals_[m_indices_[i + j] * 3].z += C.z;
			shareCount[m_indices_[i + j]] = shareCount[m_indices_[i + j]] + 1;
		}
	}

	for (unsigned int i = 0; i < m_vertices_.size(); ++i)
    {
		glm::vec3 n(m_normals_[i].x / shareCount[i], m_normals_[i].y / shareCount[i], m_normals_[i].z / shareCount[i]);
        n = glm::normalize(n);
		m_normals_[i].x = n.x;
		m_normals_[i].y = n.y;
		m_normals_[i].z = n.z;
    }

	m_indices_.push_back(0);
	m_indices_.push_back(2);
	m_indices_.push_back(1);
	m_indices_.push_back(0);
	m_indices_.push_back(4);
	m_indices_.push_back(2);
	m_indices_.push_back(0);
	m_indices_.push_back(3);
	m_indices_.push_back(4);
	m_indices_.push_back(0);
	m_indices_.push_back(1);
	m_indices_.push_back(3);

	// Bottom of the piramid.
	m_indices_.push_back(1);
	m_indices_.push_back(3);
	m_indices_.push_back(2);
	m_indices_.push_back(2);
	m_indices_.push_back(3);
	m_indices_.push_back(4);


	// The texture coordinates.
	// TODO: Sort these out.
	m_tex_coords_.push_back(glm::vec2(0.5f, 1.0f));
	m_tex_coords_.push_back(glm::vec2(0.0f, 0.0f));
	m_tex_coords_.push_back(glm::vec2(0.25f, 0.0f));
	m_tex_coords_.push_back(glm::vec2(0.5f, 0.0f));
	m_tex_coords_.push_back(glm::vec2(0.75f, 0.0f));
	m_tex_coords_.push_back(glm::vec2(1.0f, 0.0f));

	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

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
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW);
}
