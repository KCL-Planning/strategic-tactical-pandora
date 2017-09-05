#include <glm/gtc/type_ptr.hpp>

#include "SkyBoxShader.h"
#include "../scene/Material.h"
#include "../scene/SceneLeafModel.h"
#include "../texture/Texture.h"
#include "../../shapes/Shape.h"

SkyBoxShader* SkyBoxShader::shader_ = NULL;

SkyBoxShader::SkyBoxShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{

}

void SkyBoxShader::initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{

}

void SkyBoxShader::initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(&model_node.getModel());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		//Bind the vertex array and set the vertex pointer to point at it
		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model_node.getModel().getIndexBufferId());
		shape_to_vbo_[&model_node.getModel()] = vbo_index;
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	if (last_used_shader_ != this)
	{
		bindShader();
/*	
		glEnableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);
*/
	}

	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));

	assert (model_node.getMaterial().get1DTextures().size() == 0);
	assert (model_node.getMaterial().get2DTextures().size() == 0);
	assert (model_node.getMaterial().getCubeTextures().size() == 1);

	//glActiveTexture(GL_TEXTURE20);
	//glBindTexture(GL_TEXTURE_CUBE_MAP, model_node.getMaterial().getCubeTextures()[0]->getTextureId());
	//glUniform1i(texture0_loc_, 20);
	glUniform1i(texture0_loc_, model_node.getMaterial().getCubeTextures()[0]->getActiveTextureId());
/*
	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
*/

	model_node.getModel().render();

	glBindVertexArray(0);
}

SkyBoxShader& SkyBoxShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new SkyBoxShader("shaders/skybox.vert", "shaders/skybox.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cerr << "Failed to initialise the sky boxshader." << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		// Cache the locations of all the uniform variables.
		shader_->modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		shader_->projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		shader_->texture0_loc_ = shader_->getUniformLocation("texture0");
	}
	return *shader_;
}
