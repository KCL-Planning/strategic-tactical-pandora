#include "ShadowShader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "../scene/SceneLeafModel.h"
#include "../../shapes/Shape.h"
#include "../scene/Material.h"
#include "../models/Bone.h"
#include "../texture/Texture.h"

ShadowShader* ShadowShader::shader_ = NULL;

ShadowShader::ShadowShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{

}

ShadowShader& ShadowShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new ShadowShader("shaders/CreateShadowMap.vert", "shaders/CreateShadowMap.frag");
		if (!shader_->initialize())
		{
			std::cerr << "Failed to initialise the shadow shader." << std::endl;
			exit(1);
		}
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		shader_->projectiomodelview_matrix_loc_ = shader_->getUniformLocation("modelviewprojection_matrix");
		shader_->texture0_loc_ = shader_->getUniformLocation("texture0");
	}
	return *shader_;
}

void ShadowShader::initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	if (model_node.getMaterial().get2DTextures().size() == 0)
	{
		return;
	}

	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(&model_node.getModel());
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

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

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
		glEnableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);
*/
	}

	glm::mat4 projection_model_view = projection_matrix * view_matrix * model_matrix;
	glUniformMatrix4fv(projectiomodelview_matrix_loc_, 1, false, glm::value_ptr(projection_model_view));

	//glActiveTexture(GL_TEXTURE0);
	//glBindTexture(GL_TEXTURE_2D, model_node.getMaterial().get2DTextures()[0]);
	//glUniform1i(texture0_loc_, 0);
	glUniform1i(texture0_loc_, model_node.getMaterial().get2DTextures()[0]->getActiveTextureId());
/*
	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getTexCoordBufferId());
	glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);
*/

	model_node.getModel().render();

	glBindVertexArray(0);
}