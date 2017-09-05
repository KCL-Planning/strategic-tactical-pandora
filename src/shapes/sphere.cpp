#include <cmath>
#include "sphere.h"
#include "../core/entities/camera/Camera.h"
#include "../core/light/Light.h"

Sphere::Sphere(int slices, int stacks, float radius)
{
    const float PI = 3.14159f;

    m_stacks = stacks;
    m_slices = slices;

    float x = 0.0f, y = 0.0f, z = 0.0f, s = 0.0f, t = 0.0f;

    float PIOverStacks = PI / float(stacks);
    float PI2OverSlices = 2.0f * PI / float(slices);
    for (int stack = 0; stack < stacks; ++stack) 
    {
        unsigned size = m_vertices_.size();

        float Phi = float(stack) * PIOverStacks;
        float CosP = cosf(Phi);
        float SinP = sinf(Phi);
        for (int slice = 0; slice < slices; ++slice)
        {
            float Theta = float(slice) * PI2OverSlices;
            x = radius * cosf(Theta) * SinP;
            y = radius * sinf(Theta) * SinP;
            z = radius * CosP;
            s = 1.0f - (float)slice / (float)slices;
            t = (float)stack / (float)stacks;
            
            m_tex_coords_.push_back(glm::vec2(s, t));
            m_vertices_.push_back(glm::vec3(x, y, z));
            glm::vec3 n1(x, y, z);
            n1 = glm::normalize(n1);
            m_normals_.push_back(n1);

            float nextPhi = float(stack + 1) * PIOverStacks;
            float nextCosP = cosf(nextPhi);
            float nextSinP = sinf(nextPhi);
            x = radius * cosf(Theta) * nextSinP;
            y = radius * sinf(Theta) * nextSinP;
            z = radius * nextCosP;
            s = 1.0f - (float)slice / (float)slices;
            t = (float)(stack + 1.0f) / (float)stacks;

            m_tex_coords_.push_back(glm::vec2(s, t));
            m_vertices_.push_back(glm::vec3(x, y, z));
            
            glm::vec3 n2(x, y, z);
            n2 = glm::normalize(n2);
            m_normals_.push_back(n2);
        }

        m_vertices_.push_back(m_vertices_[size]);
        m_tex_coords_.push_back(m_tex_coords_[size]);
        m_normals_.push_back(m_normals_[size]);

        m_vertices_.push_back(m_vertices_[size+1]);
        m_tex_coords_.push_back(m_tex_coords_[size+1]);
        m_normals_.push_back(m_normals_[size+1]);
        
    }

    glGenBuffers(1, &m_vertex_buffer_); //Generate a buffer for the vertices
    glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_vertices_.size() * 3, &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

    glGenBuffers(1, &m_tex_coord_buffer_);
    glBindBuffer(GL_ARRAY_BUFFER, m_tex_coord_buffer_); //Bind the tex coord buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_tex_coords_.size() * 2, &m_tex_coords_[0], GL_STATIC_DRAW); //Send the data to OpenGL

    glGenBuffers(1, &m_normal_buffer_); //Generate a buffer for the vertices
    glBindBuffer(GL_ARRAY_BUFFER, m_normal_buffer_); //Bind the vertex buffer
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * m_normals_.size() * 3, &m_normals_[0], GL_STATIC_DRAW); //Send the data to OpenGL
}

void Sphere::render()
{
	int verticesPerStrip = (m_slices + 1) * 2;
	int start = 0;
	for(int stack = 0; stack < m_stacks; ++stack)
	{
		glDrawArrays(GL_TRIANGLE_STRIP, start, (m_slices * 2) + 2);
		start += verticesPerStrip;
	}
}
