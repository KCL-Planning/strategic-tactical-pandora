#include "dpengine/shaders/AnimatedShadowShader.h"

//#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dpengine/light/Light.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shapes/terrain.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/models/Bone.h"
#include "dpengine/models/BoneNode.h"
#include "dpengine/texture/Texture.h"

namespace DreadedPE
{

AnimatedShadowShader* AnimatedShadowShader::shader_ = NULL;
GLuint AnimatedShadowShader::bone_matrix_loc_[AnimatedShadowShader::MAX_BONES_] = {};

AnimatedShadowShader::AnimatedShadowShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: BasicShadowShader(vertex_shader, fragment_shader)
{

}

void AnimatedShadowShader::prepareToRender(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
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
		glEnableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getNormalBufferId());
		glVertexAttribPointer((GLint)2, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneIdsBufferId());
		glVertexAttribIPointer((GLint)3, 4, GL_INT, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneWeightsBufferId());
		glVertexAttribPointer((GLint)4, 4, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneIds2BufferId());
		glVertexAttribIPointer((GLint)5, 4, GL_INT, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getShape()->getBoneWeights2BufferId());
		glVertexAttribPointer((GLint)6, 4, GL_FLOAT, GL_FALSE, 0, 0);

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
	//glm::mat4 normal_matrix = glm::inverseTranspose(model_view_matrix);
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(model_matrix_loc_, 1, false, glm::value_ptr(model_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniformMatrix4fv(view_matrix_loc_, 1, false, glm::value_ptr(view_matrix));
	
	assert (model_node.getMaterial()->get1DTextures().size() == 0);
	assert (model_node.getMaterial()->get2DTextures().size() == 1);

	glUniform1i(texture0_loc_, model_node.getMaterial()->get2DTextures()[0]->getActiveTextureId());

	const MaterialLightProperty& material_ambient = model_node.getMaterial()->getAmbient();
	const MaterialLightProperty& material_diffuse = model_node.getMaterial()->getDiffuse();
	const MaterialLightProperty& material_specular = model_node.getMaterial()->getSpecular();
	const MaterialLightProperty& material_emissive = model_node.getMaterial()->getEmissive();
	
	glUniform4f(material_ambient_loc_, material_ambient.red_, material_ambient.green_, material_ambient.blue_, material_ambient.alpha_);
	glUniform4f(material_diffuse_loc_, material_diffuse.red_, material_diffuse.green_, material_diffuse.blue_, material_diffuse.alpha_);
	glUniform4f(material_specular_loc_, material_specular.red_, material_specular.green_, material_specular.blue_, material_specular.alpha_);
	glUniform4f(material_emissive_loc_, material_emissive.red_, material_emissive.green_, material_emissive.blue_, material_emissive.alpha_);
	glUniform1f(transparency_loc_, model_node.getMaterial()->getTransparency());

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

AnimatedShadowShader& AnimatedShadowShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new AnimatedShadowShader("shaders/animated-shadow.vert", "shaders/basic-shadow.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cerr << "Failed to initialise the animated shadow shader." << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_TexCoord0");
		shader_->bindAttrib(2, "a_Normal");
		shader_->bindAttrib(3, "a_bones_id");
		shader_->bindAttrib(4, "a_bone_weights");
		shader_->bindAttrib(5, "a_bones_id2");
		shader_->bindAttrib(6, "a_bone_weights2");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();
		shader_->resolveUniformNames();

		// Cache the locations of all the uniform variables.
		shader_->modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		shader_->model_matrix_loc_ = shader_->getUniformLocation("model_matrix");
		shader_->projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		shader_->view_matrix_loc_ = shader_->getUniformLocation("view_matrix");
		shader_->texture0_loc_ = shader_->getUniformLocation("texture0");
		shader_->material_ambient_loc_ = shader_->getUniformLocation("material_ambient");
		shader_->material_diffuse_loc_ = shader_->getUniformLocation("material_diffuse");
		shader_->material_specular_loc_ = shader_->getUniformLocation("material_specular");
		shader_->material_emissive_loc_ = shader_->getUniformLocation("material_emissive");
		
		for (unsigned int i = 0; i < MAX_BONES_; ++i)
		{
			std::stringstream ss;
			ss << "bone_matrix[" << i << "]";
			bone_matrix_loc_[i] = shader_->getUniformLocation(ss.str());
		}
		
	}
	return *shader_;
}

};
