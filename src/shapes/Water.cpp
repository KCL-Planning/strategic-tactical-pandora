#include "Water.h"


Water::Water(unsigned int width, unsigned int height, float depth)
	: width_(width), height_(height), depth_(depth)
{
	total_time_ = 0.0f;

	// Create the vertices for the water.
	//m_vertices_water.resize(width * height * 3);
	for (unsigned int z = 0; z < height; ++z)
	{
		for (unsigned int x = 0; x < width; ++x)
		{
			m_vertices_.push_back(glm::vec3(x - (float)(width) / 2, depth, z - (float)(height) / 2));
		}
	}

	glGenBuffers(1, &m_vertex_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	for (unsigned int z = 0; z < height; ++z)
	{
		for (unsigned int x = 0; x < width; ++x)
		{
			m_wave_effect_.push_back(0);
		}
	}
	glGenBuffers(1, &wave_effect_index_);
    glBindBuffer(GL_ARRAY_BUFFER, wave_effect_index_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_wave_effect_.size(), &m_wave_effect_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Create the indices, make sure they are stored counter clockwise!
	// (z + 1) * width + x     (z + 1) * width + x + 1
	//          *---------------*
	//          |            /  |
	//          |         /     |
	//          |      /        |
	//          |  /            |
	//          *---------------*
	//    z * width + x    z * width + x + 1
	for (unsigned int z = 0; z < height - 1; ++z)
	{
		for (unsigned int x = 0; x < width - 1; ++x)
		{
			m_indices_.push_back(z * width + x);
			m_indices_.push_back((z + 1) * width + x + 1);
			m_indices_.push_back((z + 1) * width + x);
			
			m_indices_.push_back(z * width + x);
			m_indices_.push_back(z * width + x + 1);
			m_indices_.push_back((z + 1) * width + x + 1);
		}
	}

	glGenBuffers(1, &m_index_buffer_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_index_buffer_); //Bind the vertex buffer
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * m_indices_.size(), &m_indices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Create the normal vectors, they all point up :D.
	for (unsigned int z = 0; z < height; ++z)
	{
		for (unsigned int x = 0; x < width; ++x)
		{
			m_normals_.push_back(glm::vec3(0.0, 1.0, 0.0));
		}
	}

	glGenBuffers(1, &m_normal_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * m_normals_.size(), &m_normals_[0], GL_STATIC_DRAW); //Send the data to OpenGL

	// Create the texture coordinates.
	for (unsigned int z = 0; z < height; ++z)
	{
		for (unsigned int x = 0; x < width; ++x)
		{
			float s = (float(x) / float(width)) * 8.0f;
            float t = (float(z) / float(height)) * 8.0f;
			m_tex_coords_.push_back(glm::vec2(s, t));
		}
	}

	glGenBuffers(1, &m_tex_coord_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 2 * m_tex_coords_.size(), &m_tex_coords_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}

void Water::progressWaves(float dt)
{
	total_time_ += dt;
	for (unsigned int i = 0; i < width_; ++i)
	{
		for (unsigned int j = 0; j < height_; ++j)
		{
			m_wave_effect_[i * width_ + j] = (sin(total_time_ + i) + cos(total_time_ + j));
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, wave_effect_index_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_wave_effect_.size(), &m_wave_effect_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}
