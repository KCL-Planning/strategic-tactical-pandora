#include "TerrainShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../light/Light.h"
#include "../scene/Material.h"
#include "../../shapes/terrain.h"
#include "../scene/SceneLeafModel.h"
#include "../texture/Texture.h"

TerrainShader* TerrainShader::terrain_shader_ = NULL;

TerrainShader::TerrainShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: LightShader(vertex_shader, fragment_shader)
{

}

void TerrainShader::initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	std::map<const Shape*, GLuint>::iterator mapped_i = shape_to_vbo_.find(&model_node.getModel());
	GLuint vbo_index;
	if (mapped_i == shape_to_vbo_.end())
	{
		glGenVertexArrays(1, &vbo_index);
		glBindVertexArray(vbo_index);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
		glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getTexCoordBufferId());
		glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getNormalBufferId());
		glVertexAttribPointer((GLint)2, 3, GL_FLOAT, GL_FALSE, 0, 0);

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
		glEnableVertexAttribArray(2);
		glDisableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);
*/
	}

	LightShader::initialise(model_node, view_matrix, model_matrix, projection_matrix, lights);
	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	glm::mat4 normal_matrix = glm::inverseTranspose(model_view_matrix);

	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(model_matrix_loc_, 1, false, glm::value_ptr(model_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniformMatrix4fv(view_matrix_loc_, 1, false, glm::value_ptr( view_matrix));

	assert (model_node.getMaterial().get1DTextures().size() == 1);
	assert (model_node.getMaterial().get2DTextures().size() == 1);

	//glActiveTexture(GL_TEXTURE10);
	//glBindTexture(GL_TEXTURE_2D, model_node.getMaterial().get2DTextures()[0]);
	//glUniform1i(texture0_loc_, 10);
	glUniform1i(texture0_loc_, model_node.getMaterial().get2DTextures()[0]->getActiveTextureId());

	//glActiveTexture(GL_TEXTURE11);
	//glBindTexture(GL_TEXTURE_1D, model_node.getMaterial().get1DTextures()[0]);
	//glUniform1i(texture1_loc_, 11);
	glUniform1i(texture1_loc_, model_node.getMaterial().get1DTextures()[0]->getActiveTextureId());

	const MaterialLightProperty& material_ambient = model_node.getMaterial().getAmbient();
	const MaterialLightProperty& material_diffuse = model_node.getMaterial().getDiffuse();
	const MaterialLightProperty& material_specular = model_node.getMaterial().getSpecular();
	const MaterialLightProperty& material_emissive = model_node.getMaterial().getEmissive();

	glUniform4f(material_ambient_loc_, material_ambient.red_, material_ambient.green_, material_ambient.blue_, material_ambient.alpha_);
	glUniform4f(material_diffuse_loc_, material_diffuse.red_, material_diffuse.green_, material_diffuse.blue_, material_diffuse.alpha_);
	glUniform4f(material_specular_loc_, material_specular.red_, material_specular.green_, material_specular.blue_, material_specular.alpha_);
	glUniform4f(material_emissive_loc_, material_emissive.red_, material_emissive.green_, material_emissive.blue_, material_emissive.alpha_);
/*
	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getTexCoordBufferId());
	glVertexAttribPointer((GLint)1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getNormalBufferId());
	glVertexAttribPointer((GLint)2, 3, GL_FLOAT, GL_FALSE, 0, 0);
*/
	model_node.getModel().render();
	glBindVertexArray(0);
}

TerrainShader& TerrainShader::getShader()
{
	if (terrain_shader_ == NULL)
	{
		terrain_shader_ = new TerrainShader("shaders/basic-shadow.vert", "shaders/terrain.frag");
		
		// Load the shader.
		if (!terrain_shader_->initialize())
		{
			std::cerr << "Failed to initialise the terrain shader." << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		terrain_shader_->bindAttrib(0, "a_Vertex");
		terrain_shader_->bindAttrib(1, "a_TexCoord0");
		terrain_shader_->bindAttrib(2, "a_Normal");

		//Re link the program
		terrain_shader_->linkProgram();
		terrain_shader_->resolveUniformNames();
		terrain_shader_->bindShader();

		terrain_shader_->modelview_matrix_loc_ = terrain_shader_->getUniformLocation("modelview_matrix");
		terrain_shader_->model_matrix_loc_ = terrain_shader_->getUniformLocation("model_matrix");
		terrain_shader_->projection_matrix_loc_ = terrain_shader_->getUniformLocation("projection_matrix");
		terrain_shader_->view_matrix_loc_ = terrain_shader_->getUniformLocation("view_matrix");
		terrain_shader_->texture0_loc_ = terrain_shader_->getUniformLocation("texture0");
		terrain_shader_->texture1_loc_ = terrain_shader_->getUniformLocation("texture1");
		terrain_shader_->material_ambient_loc_ = terrain_shader_->getUniformLocation("material_ambient");
		terrain_shader_->material_diffuse_loc_ = terrain_shader_->getUniformLocation("material_diffuse");
		terrain_shader_->material_specular_loc_ = terrain_shader_->getUniformLocation("material_specular");
		terrain_shader_->material_emissive_loc_ = terrain_shader_->getUniformLocation("material_emissive");
	}
	return *terrain_shader_;
}