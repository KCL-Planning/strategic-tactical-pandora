#include "ShadowVolumeShader.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "LightVolumeShape.h"
#include <dpengine/scene/Material.h>
#include <dpengine/scene/SceneLeafModel.h>
#include <dpengine/shapes/Shape.h>
#include <dpengine/scene/SceneNode.h>
#include <dpengine/scene/SceneLeafLight.h>
#include <dpengine/light/PointLight.h>
#include <dpengine/texture/Texture.h>

ShadowVolumeShader* ShadowVolumeShader::shader_ = NULL;

ShadowVolumeShader::ShadowVolumeShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader), depth_not_initialised_(true), m_depth_buffer_(0)
{

}

void ShadowVolumeShader::prepareToRender(const DreadedPE::SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const DreadedPE::SceneLeafLight*>& lights)
{
	
}

void ShadowVolumeShader::initialise(const DreadedPE::SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const DreadedPE::SceneLeafLight& light, float camera_near_plane, float camera_far_plane, DreadedPE::Texture& camera_depth_texture)
{
	if (depth_not_initialised_)
	{
		loadDepth(light.getLight());
		glGenBuffers(1, &m_cos_buffer_);
		glBindBuffer(GL_ARRAY_BUFFER, m_cos_buffer_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *  m_cos_.size(), &m_cos_[0], GL_STATIC_DRAW);
		depth_not_initialised_ = false;
	}

	std::map<const DreadedPE::Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(model_node.getShape().get());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, m_cos_buffer_);
		glVertexAttribPointer((GLint)1, 1, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model_node.getShape()->getIndexBufferId());
		shape_to_vbo_[model_node.getShape().get()] = vbo_index;
		model_node.getShape()->addDestructionListener(*this);
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	if (last_used_shader_ != this)
	{
		bindShader();
	}
	
	glm::mat4 model_view_matrix = view_matrix * model_matrix;

	glm::mat4 shadow_scale_matrix = glm::translate(glm::mat4(1.0), glm::vec3(0.5f, 0.5f, 0.5f));
	shadow_scale_matrix = glm::scale(shadow_scale_matrix, glm::vec3(0.5f, 0.5f, 0.5f));
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(shadow_matrix_loc_, 1, false, glm::value_ptr(shadow_scale_matrix));
	glUniformMatrix4fv(view_matrix_loc_, 1, false, glm::value_ptr(view_matrix));
	glUniformMatrix4fv(model_matrix_loc_, 1, false, glm::value_ptr(model_matrix));
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniformMatrix4fv(light_shadow_matrix_loc_, 1, false, glm::value_ptr(light.getLight().getShadowMatrix()));

	//assert (model_node.getMaterial()->get1DTextures().size() == 0);
	assert (model_node.getMaterial()->get2DTextures().size() == 1);
		
	glUniform1i(camera_depth_texture_loc_, camera_depth_texture.getActiveTextureId());
	glUniform1i(depth_texture_loc_, light.getLight().getShadowRendererTextureID());
	glUniform1f(z_near_loc_, light.getLight().getClosePlane());
	glUniform1f(z_far_loc_, light.getLight().getFarPlane());
	glUniform3f(light_pos_loc_, light.getLight().getLocation().x, light.getLight().getLocation().y, light.getLight().getLocation().z);
	glUniform3f(light_colour_loc_, light.getLight().getAmbient().r, light.getLight().getAmbient().g, light.getLight().getAmbient().b);
	glUniform1f(camera_near_plane_loc_, camera_near_plane);
	glUniform1f(camera_far_plane_loc_, camera_far_plane);
	
	glDrawElements(model_node.getShape()->getRenderingMode(), model_node.getShape()->getIndices().size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

ShadowVolumeShader& ShadowVolumeShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new ShadowVolumeShader("shaders/ShadowVolume.vert", "shaders/ShadowVolume.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cout << "Failed to get the shadow volume shader!" << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_cos");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		// Cache the locations of all the uniform variables.
		shader_->shadow_matrix_loc_ = shader_->getUniformLocation("shadow_matrix");
		shader_->model_matrix_loc_ = shader_->getUniformLocation("model_matrix");
		shader_->modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		shader_->projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		shader_->light_shadow_matrix_loc_ = shader_->getUniformLocation("light_shadow_matrix");
		shader_->depth_texture_loc_ = shader_->getUniformLocation("depth_texture");
		shader_->camera_depth_texture_loc_ = shader_->getUniformLocation("camera_depth_texture");
		shader_->z_near_loc_ = shader_->getUniformLocation("z_near");
		shader_->z_far_loc_ = shader_->getUniformLocation("z_far");
		shader_->camera_near_plane_loc_ = shader_->getUniformLocation("camera_near");
		shader_->camera_far_plane_loc_ = shader_->getUniformLocation("camera_far");
		shader_->light_pos_loc_ = shader_->getUniformLocation("light_pos");
		shader_->light_colour_loc_ = shader_->getUniformLocation("light_colour");
	}
	return *shader_;
}

void ShadowVolumeShader::loadDepth(const DreadedPE::Light& light)
{
	float near_plane = light.getClosePlane();

	// We precompute the cos value of every pixel on the screen. This allows us the take account of the perspective
	// matrix when calculating the depth value of every pixel from the light's perspective.
	float total_angle = 2 * light.getAngle();
	float shadow_map_dimension = light.getShadowMapDimension();
	
	float angle_per_pixel = total_angle / shadow_map_dimension;
	float length_per_pixel = tan(angle_per_pixel * M_PI / 180.0f) * near_plane;

	m_cos_.push_back(0);

	// Add the vertexes for the depth mesh.
	for (float y = 0; y < shadow_map_dimension; ++y)
	{
		for (float x = 0; x < shadow_map_dimension; ++x)
		{
			float y_offset = (y - shadow_map_dimension / 2.0f) * length_per_pixel;
			float x_offset = (x - shadow_map_dimension / 2.0f) * length_per_pixel;

			float length = sqrt(x_offset * x_offset + y_offset * y_offset + near_plane * near_plane);
			m_cos_.push_back(near_plane / length);
		}
	}
}
