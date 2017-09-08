#include "LightVolumeShape.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <glm/glm.hpp>

#include <dpengine/light/SpotLight.h>
#include <dpengine/renderer/ShadowRenderer.h>
#include "ShadowVolumeShader.h"

LightVolumeShape::LightVolumeShape(DreadedPE::SceneManager& scene_manager, DreadedPE::SpotLight& point_light)
	: point_light_(&point_light)
{
	// Create a vertex for each pixel in the shadow map + 1 for the location of where the camera is.
	// We set the pixels up as they are projected on the near plane, that way we only need to change
	// the length of these verteces in the vertex shader.
	float total_angle = 2 * point_light.getAngle();
	float shadow_map_dimension = point_light.getShadowMapDimension();
	
	float angle_per_pixel = total_angle / shadow_map_dimension;
	
	float length_per_pixel = tan(angle_per_pixel * M_PI / 180.0f) * point_light.getClosePlane();

	glm::vec4 direction(0.0f, 0.0f, -1.0f, 0.0f);

	// Add a vertex that will be the 'tip' of the piramid and corresponds to the location of the light source.
	m_vertices_.push_back(glm::vec3(0, 0, 0));
	m_tex_coords_.push_back(glm::vec2(0, 0));
	m_normals_.push_back(glm::vec3(0, 0, 0));
	
	
	// Add the vertexes for the depth mesh.
	for (float y = 0; y < shadow_map_dimension; ++y)
	{
		for (float x = 0; x < shadow_map_dimension; ++x)
		{
			float y_offset = (y - shadow_map_dimension / 2.0f) * length_per_pixel;
			float x_offset = (x - shadow_map_dimension / 2.0f) * length_per_pixel;
			
			glm::vec4 point(x_offset*2, y_offset*2, -point_light.getClosePlane()*2, 1.0f);
			
			m_vertices_.push_back(glm::vec3(point));
			m_tex_coords_.push_back(glm::vec2(x, y));
			m_normals_.push_back(glm::vec3(0, 1, 0));
		}
	}
	
	// Tip of the piramid.
	for (unsigned int y = 0; y < point_light.getShadowMapDimension() - 1; ++y)
	{
		m_indices_.push_back(0); m_indices_.push_back(y + 1); m_indices_.push_back(y + 2);
		m_indices_.push_back(0); m_indices_.push_back((point_light.getShadowMapDimension() - 1) * point_light.getShadowMapDimension() + y + 2); m_indices_.push_back((point_light.getShadowMapDimension() - 1) * point_light.getShadowMapDimension() + y + 1);
		m_indices_.push_back(0);  m_indices_.push_back(point_light.getShadowMapDimension() * (y + 1) + 1); m_indices_.push_back(point_light.getShadowMapDimension() * y + 1);
		m_indices_.push_back(0); m_indices_.push_back(point_light.getShadowMapDimension() * (y + 1)); m_indices_.push_back(point_light.getShadowMapDimension() * (y + 2)); 
	}

	// The rest.
	for (unsigned int y = 0; y < point_light.getShadowMapDimension() - 1; ++y)
	{
		for (unsigned int x = 0; x < point_light.getShadowMapDimension() - 1; ++x)
		{
			m_indices_.push_back(1 + x + point_light.getShadowMapDimension() * (y + 1));
			m_indices_.push_back(2 + x + point_light.getShadowMapDimension() * (y + 1));
			m_indices_.push_back(1 + x + point_light.getShadowMapDimension() * y);

			m_indices_.push_back(2 + x + point_light.getShadowMapDimension() * (y + 1));
			m_indices_.push_back(2 + x + point_light.getShadowMapDimension() * y);
			m_indices_.push_back(1 + x + point_light.getShadowMapDimension() * y);
		}
	}

	// Bind all this to OpenGL.
	// Generate buffers.
	glGenBuffers(1, &m_vertex_buffer_);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertex_buffer_); //Bind the vertex buffer
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * 3 * m_vertices_.size(), &m_vertices_[0], GL_STATIC_DRAW); //Send the data to OpenGL

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
