#include "dpengine/shaders/CreateAnimatedShadowMapShader.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/shapes/Shape.h"
#include "dpengine/scene/Material.h"
#include "dpengine/models/Bone.h"
#include "dpengine/texture/Texture.h"

namespace DreadedPE
{

CreateAnimatedShadowMapShader* CreateAnimatedShadowMapShader::shader_ = NULL;
GLuint CreateAnimatedShadowMapShader::projectiomodelview_matrix_loc_ = 0;
GLuint CreateAnimatedShadowMapShader::texture0_loc_ = 0;
GLuint CreateAnimatedShadowMapShader::bone_matrix_loc_[CreateAnimatedShadowMapShader::MAX_BONES_] = {};

CreateAnimatedShadowMapShader::CreateAnimatedShadowMapShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, fragment_shader)
{

}

CreateAnimatedShadowMapShader& CreateAnimatedShadowMapShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new CreateAnimatedShadowMapShader("shaders/CreateAnimatedShadowMap.vert", "shaders/CreateShadowMap.frag");
		if (!shader_->initialize())
		{
			std::cout << "Failed to get the animated shadow shader!" << std::endl;
			exit(1);
		}
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");
		shader_->bindAttrib(2, "a_bones_id");
		shader_->bindAttrib(3, "a_bone_weights");
		shader_->bindAttrib(4, "a_bones_id2");
		shader_->bindAttrib(5, "a_bone_weights2");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();

		projectiomodelview_matrix_loc_ = shader_->getUniformLocation("modelviewprojection_matrix");
		texture0_loc_ = shader_->getUniformLocation("texture0");

		for (unsigned int i = 0; i < MAX_BONES_; ++i)
		{
			std::stringstream ss;
			ss << "bone_matrix[" << i << "]";
			bone_matrix_loc_[i] = shader_->getUniformLocation(ss.str());
		}
	}
	return *shader_;
}

void CreateAnimatedShadowMapShader::prepareToRender(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	if (model_node.getMaterial()->get2DTextures().size() == 0)
	{
		return;
	}

	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(model_node.getShape().get());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glEnableVertexAttribArray(3);
		glEnableVertexAttribArray(4);
		glEnableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneIdsBufferId());
		glVertexAttribIPointer((GLint)2, 4, GL_INT, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneWeightsBufferId());
		glVertexAttribPointer((GLint)3, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneIds2BufferId());
		glVertexAttribIPointer((GLint)4, 4, GL_INT, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneWeights2BufferId());
		glVertexAttribPointer((GLint)5, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model_node.getShape()->getIndexBufferId());
		shape_to_vbo_[model_node.getShape().get()] = vbo_index;
		model_node.getShape()->addDestructionListener(*this);
	}
	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	// Check if this shape has been rendered using this shader before.
	if (last_used_shader_ != this)
	{
		bindShader();
	}

	glm::mat4 projection_model_view = projection_matrix * view_matrix * model_matrix;
	
	glUniformMatrix4fv(projectiomodelview_matrix_loc_, 1, false, glm::value_ptr(projection_model_view));

	glUniform1i(texture0_loc_, model_node.getMaterial()->get2DTextures()[0]->getActiveTextureId());

	unsigned int i = 0;
	for (; i < model_node.getShape()->getBones().size(); ++i)
	{
		glUniformMatrix4fv(bone_matrix_loc_[i], 1, false, glm::value_ptr(model_node.getShape()->getBones()[i]->getFinalTransformation()));
	}
	for (; i < MAX_BONES_; ++i)
	{
		glUniformMatrix4fv(bone_matrix_loc_[i], 1, false, glm::value_ptr(glm::mat4(1.0f)));
	}
	glDrawElements(model_node.getShape()->getRenderingMode(), model_node.getShape()->getIndices().size(), GL_UNSIGNED_INT, 0);

	glBindVertexArray(0);
}

};
