#include "InstanceShader.h"

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "../../core/scene/SceneLeafModel.h"
#include "../../core/scene/Material.h"
#include "../../shapes/Shape.h"

#include "../../core/models/Bone.h"

#include "../../core/texture/Texture.h"

#include "InstanceRenderedShape.h"
#include "SceneLeafInstanced.h"

InstanceShader* InstanceShader::shader_ = NULL;

InstanceShader::InstanceShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: LightShader(vertex_shader, fragment_shader)
{

}

void InstanceShader::initialise(const SceneLeafInstanced& instanced_leaf, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
/*
	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(&instanced_leaf.getInstancedShape());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);
*/
/*
		//Bind the vertex array and set the vertex pointer to point at it
		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getModelMatrixBufferId());
		for (unsigned int i = 0; i < 4; ++i)
		{
			glEnableVertexAttribArray(1 + i);
			glVertexAttribPointer(1 + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (const GLvoid*)(sizeof(GLfloat) * i * 4));
			glVertexAttribDivisor(1 + i, 1);
		}
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getIndexBufferId());
*/
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);

		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getNormalBufferId());
		glVertexAttribPointer((GLint)2, 3, GL_FLOAT, GL_FALSE, 0, 0);
		
		glBindBuffer(GL_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getModelMatrixBufferId());
		for (unsigned int i = 0; i < 4; ++i)
		{
			glEnableVertexAttribArray(3 + i);
			glVertexAttribPointer(3 + i, 4, GL_FLOAT, GL_FALSE, sizeof(glm::mat4), (const GLvoid*)(sizeof(GLfloat) * i * 4));
			glVertexAttribDivisor(3 + i, 1);
		}

		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, instanced_leaf.getInstancedShape().getIndexBufferId());
		
	/*
	}

	else
	{
		glBindVertexArray((*mapped_i).second);
	}

	if (last_used_shader_ != this)*/
	{
		bindShader();
	} 
	
	LightShader::initialise(instanced_leaf, view_matrix, model_matrix, projection_matrix, lights);
	
	// TODO Normal matrix is false.
	glm::mat4 normal_matrix = glm::inverseTranspose(view_matrix);
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniformMatrix4fv(view_matrix_loc_, 1, false, glm::value_ptr(view_matrix));
	
	assert (instanced_leaf.getMaterial().get1DTextures().size() == 0);
	assert (instanced_leaf.getMaterial().get2DTextures().size() == 1);

	glUniform1i(texture0_loc_, instanced_leaf.getMaterial().get2DTextures()[0]->getActiveTextureId());

	const MaterialLightProperty& material_ambient = instanced_leaf.getMaterial().getAmbient();
	const MaterialLightProperty& material_diffuse = instanced_leaf.getMaterial().getDiffuse();
	const MaterialLightProperty& material_specular = instanced_leaf.getMaterial().getSpecular();
	const MaterialLightProperty& material_emissive = instanced_leaf.getMaterial().getEmissive();
	
	glUniform4f(material_ambient_loc_, material_ambient.red_, material_ambient.green_, material_ambient.blue_, material_ambient.alpha_);
	glUniform4f(material_diffuse_loc_, material_diffuse.red_, material_diffuse.green_, material_diffuse.blue_, material_diffuse.alpha_);
	glUniform4f(material_specular_loc_, material_specular.red_, material_specular.green_, material_specular.blue_, material_specular.alpha_);
	glUniform4f(material_emissive_loc_, material_emissive.red_, material_emissive.green_, material_emissive.blue_, material_emissive.alpha_);
	glUniform1f(transparency_loc_, instanced_leaf.getMaterial().getTransparency());

	instanced_leaf.getInstancedShape().render();

	glBindVertexArray(0);
}


InstanceShader& InstanceShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new InstanceShader("src/demo/instance_rendering/instanced.vert", "src/demo/instance_rendering/instanced.frag");
		
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
		shader_->bindAttrib(3, "model_matrix");

		//Re link the program
		shader_->linkProgram();
		shader_->bindShader();
		shader_->resolveUniformNames();

		// Cache the locations of all the uniform variables.
		shader_->projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		shader_->view_matrix_loc_ = shader_->getUniformLocation("view_matrix");
		shader_->texture0_loc_ = shader_->getUniformLocation("texture0");
		shader_->material_ambient_loc_ = shader_->getUniformLocation("material_ambient");
		shader_->material_diffuse_loc_ = shader_->getUniformLocation("material_diffuse");
		shader_->material_specular_loc_ = shader_->getUniformLocation("material_specular");
		shader_->material_emissive_loc_ = shader_->getUniformLocation("material_emissive");
	}
	return *shader_;
}
