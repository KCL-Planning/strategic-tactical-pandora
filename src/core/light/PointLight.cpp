#include "GL/glew.h"
#include "GL/glfw.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>

#include <fstream>
#include <iostream>

#include "../renderer/ShadowRenderer.h"
#include "PointLight.h"
#include "../entities/camera/Camera.h"

PointLight::PointLight(SceneManager& scene_manager, float light_angle, const glm::vec3& ambient, const glm::vec3& diffuse, const glm::vec3& specular, float constant_attenuation, float linear_attenuation, float quadratic_attenuation, float close_plane, float far_plane, GLuint shadow_map_dimension, GLenum cull_mode, GLint texture_compare_mode, GLint texture_compare_function)
	: Light(SPOTLIGHT, ambient, diffuse, specular, constant_attenuation, linear_attenuation, quadratic_attenuation, light_angle, close_plane, far_plane, shadow_map_dimension)
{
	shadow_renderer_ = new ShadowRenderer(scene_manager, shadow_map_dimension, cull_mode, texture_compare_mode, texture_compare_function);
}

PointLight::~PointLight()
{
	delete shadow_renderer_;
}

glm::mat4 PointLight::getPerspectiveMatrix() const
{
	return glm::perspective(light_angle_ * 2, 1.0f, close_plane_, far_plane_);
}

void PointLight::preRender(const glm::mat4& transition)
{
	Light::preRender(transition);

	shadow_renderer_->render(*this);

	glm::mat4 shadow_perspective_matrix = glm::perspective(light_angle_ * 2, 1.0f, close_plane_, far_plane_);

	glm::mat4 shadow_scale_matrix = glm::translate(glm::mat4(1.0), glm::vec3(0.5f, 0.5f, 0.5f));
	shadow_scale_matrix = glm::scale(shadow_scale_matrix, glm::vec3(0.5f, 0.5f, 0.5f));

	shadow_matrix_ = shadow_scale_matrix * shadow_perspective_matrix * getViewMatrix();
	
	// Create the mesh used for geometric lighting.
	//outputDepthBufferToFile("depth_data.txt");
	//exit(0);
}


void PointLight::outputDepthBufferToFile(const std::string& file_name) const
{
	glBindFramebuffer(GL_FRAMEBUFFER, shadow_renderer_->getFramebufferId());
	std::vector<float> depth_data;
	depth_data.resize(shadow_map_dimension_ * shadow_map_dimension_);
	glReadPixels(0, 0, shadow_map_dimension_, shadow_map_dimension_, GL_DEPTH_COMPONENT, GL_FLOAT, &depth_data[0]);
	
	// Write these values to a file.
	std::ofstream myfile;
	myfile.open(file_name.c_str());
	
	float z_far = far_plane_;
	float z_near = close_plane_;
	float a = z_far / (z_far - z_near);
	float b = z_far * z_near / (z_near - z_far);
	
	unsigned int i = 0;
	for (std::vector<float>::const_iterator ci = depth_data.begin(); ci != depth_data.end(); ++ci)
	{
		float value = *ci;
		
		value = b / (value - a);
		
		std::stringstream ss;
		ss << value;
		std::string s(ss.str());
		myfile << s << " ";
		if (++i == shadow_map_dimension_)
		{
			myfile << "\n";
			i = 0;
		}
	}
	myfile.close();
	
	// Unbind buffer.
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}
