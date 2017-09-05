#include "WaterShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../light/Light.h"
#include "../scene/Material.h"
#include "../../shapes/Water.h"
#include "../scene/SceneLeafModel.h"
#include "../texture/Texture.h"

WaterShader* WaterShader::shader_ = NULL;
GLuint WaterShader::modelview_matrix_loc_ = 0;
GLuint WaterShader::model_matrix_loc_ = 0;
GLuint WaterShader::projection_matrix_loc_ = 0;
GLuint WaterShader::view_matrix_loc_ = 0;
GLuint WaterShader::texture0_loc_ = 0;
GLuint WaterShader::material_ambient_loc_ = 0;
GLuint WaterShader::material_diffuse_loc_ = 0;
GLuint WaterShader::material_specular_loc_ = 0;
GLuint WaterShader::material_emissive_loc_ = 0;

WaterShader::WaterShader(const std::string& vertex_shader, const std::string& fragment_shader)
	: LightShader(vertex_shader, fragment_shader)
{

}

void WaterShader::initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights)
{
	bindShader();
	LightShader::initialise(model_node, view_matrix, model_matrix, projection_matrix, lights);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);
	glDisableVertexAttribArray(7);

	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	glm::mat4 normal_matrix = glm::inverseTranspose(model_view_matrix);
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(model_matrix_loc_, 1, false, glm::value_ptr(model_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniformMatrix4fv(view_matrix_loc_, 1, false, glm::value_ptr(view_matrix));

	assert (model_node.getMaterial().get1DTextures().size() == 0);
	assert (model_node.getMaterial().get2DTextures().size() == 1);

	//glActiveTexture(GL_TEXTURE2);
	//glBindTexture(GL_TEXTURE_2D, model_node.getMaterial().get2DTextures()[0]);
	//glUniform1i(texture0_loc_, 2);
	glUniform1i(texture0_loc_, model_node.getMaterial().get2DTextures()[0]->getActiveTextureId());

	const MaterialLightProperty& material_ambient = model_node.getMaterial().getAmbient();
	const MaterialLightProperty& material_diffuse = model_node.getMaterial().getDiffuse();
	const MaterialLightProperty& material_specular = model_node.getMaterial().getSpecular();
	const MaterialLightProperty& material_emissive = model_node.getMaterial().getEmissive();
	
	glUniform4f(material_ambient_loc_, material_ambient.red_, material_ambient.green_, material_ambient.blue_, material_ambient.alpha_);
	glUniform4f(material_diffuse_loc_, material_diffuse.red_, material_diffuse.green_, material_diffuse.blue_, material_diffuse.alpha_);
	glUniform4f(material_specular_loc_, material_specular.red_, material_specular.green_, material_specular.blue_, material_specular.alpha_);
	glUniform4f(material_emissive_loc_, material_emissive.red_, material_emissive.green_, material_emissive.blue_, material_emissive.alpha_);

	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getVertexBufferId());
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	// TODO: Can make generic by altering the vertex information instead of using the shader to update the positions of the vertices.
	const Water* water_entity = static_cast<const Water*>(&model_node.getModel());
	glBindBuffer(GL_ARRAY_BUFFER, water_entity->getWaveHeightsBufferId());
	glVertexAttribPointer((GLint)1, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getTexCoordBufferId());
	glVertexAttribPointer((GLint)2, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, model_node.getModel().getNormalBufferId());
	glVertexAttribPointer((GLint)3, 3, GL_FLOAT, GL_FALSE, 0, 0);
	}

WaterShader& WaterShader::getShader()
{
	if (shader_ == NULL)
	{
		shader_ = new WaterShader("shaders/water.vert", "shaders/water.frag");
		//shader_ = new WaterShader("shaders/water.vert", "shaders/basic-shadow.frag");
		//shader_ = new WaterShader("shaders/basic-shadow.vert", "shaders/basic-shadow.frag");
		
		// Load the shader.
		if (!shader_->initialize())
		{
			std::cerr << "Failed to initialise the water shader." << std::endl;
			exit(1);
		}
	
		//Bind the attribute locations
		shader_->bindAttrib(0, "a_Vertex");
		shader_->bindAttrib(1, "a_WaveEffect");
		shader_->bindAttrib(2, "a_TexCoord0");
		shader_->bindAttrib(3, "a_Normal");

		//Re link the program
		shader_->linkProgram();
		shader_->resolveUniformNames();
		shader_->bindShader();

		// Cache the locations of all the uniform variables.
		modelview_matrix_loc_ = shader_->getUniformLocation("modelview_matrix");
		model_matrix_loc_ = shader_->getUniformLocation("model_matrix");
		projection_matrix_loc_ = shader_->getUniformLocation("projection_matrix");
		view_matrix_loc_ = shader_->getUniformLocation("view_matrix");
		texture0_loc_ = shader_->getUniformLocation("texture0");

		material_ambient_loc_ = shader_->getUniformLocation("material_ambient");
		material_diffuse_loc_ = shader_->getUniformLocation("material_diffuse");
		material_specular_loc_ = shader_->getUniformLocation("material_specular");
		material_emissive_loc_ = shader_->getUniformLocation("material_emissive");
	}
	return *shader_;
}