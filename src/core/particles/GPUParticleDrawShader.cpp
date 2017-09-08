#include "dpengine/particles/GPUParticleDrawShader.h"
#include "dpengine/particles/GPUParticleComputerShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "dpengine/particles/GPUParticleEmitter.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/texture/Texture.h"

namespace DreadedPE
{

GPUParticleDrawShader::GPUParticleDrawShader(GPUParticleComputerShader& computer_shader, const Texture& texture, const std::string& vertex_shader, const std::string& geometry_shader, const std::string& fragment_shader)
	: GLSLProgram(vertex_shader, geometry_shader, fragment_shader), texture_(&texture), computer_shader_(&computer_shader)
{
	init();
}

void GPUParticleDrawShader::init()
{
	// Load the shader.
	if (!initialize())
	{
		std::cout << "Failed to get the particle shader!" << std::endl;
		exit(1);
	}

	//Bind the attribute locations
	bindAttrib(0, "position");
	bindAttrib(1, "velocity");
	bindAttrib(2, "lifetime");
	bindAttrib(3, "type");

	//Re link the program
	linkProgram();
	bindShader();

	// Cache the locations of all the uniform variables.
	modelview_matrix_loc_ = getUniformLocation("modelview_matrix");
	projection_matrix_loc_ = getUniformLocation("projection_matrix");
	camera_loc_ = getUniformLocation("camera_location");

	texture0_loc_ = getUniformLocation("texture0");

	linkProgram();
	bindShader();
}

void GPUParticleDrawShader::drawParticles(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, const glm::vec3& camera_location)
{	
	if (last_used_shader_ != this)
	{
		bindShader();
	
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
		glEnableVertexAttribArray(3);
		glDisableVertexAttribArray(4);
		glDisableVertexAttribArray(5);
		glDisableVertexAttribArray(6);
		glDisableVertexAttribArray(7);
	}

	glm::mat4 model_view_matrix = view_matrix * model_matrix;
	
	//Send the modelview and projection matrices to the shaders
	glUniformMatrix4fv(modelview_matrix_loc_, 1, false, glm::value_ptr(model_view_matrix));
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));
	glUniform3f(camera_loc_, camera_location.x, camera_location.y, camera_location.z);
	glUniformMatrix4fv(projection_matrix_loc_, 1, false, glm::value_ptr(projection_matrix));

	glUniform1i(texture0_loc_, texture_->getActiveTextureId());
		
	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, computer_shader_->getPositionsBufferId());
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    
	glBindBuffer(GL_ARRAY_BUFFER, computer_shader_->getVelocitiesBufferId());
	glVertexAttribPointer((GLint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, computer_shader_->getLifetimesBufferId());
	glVertexAttribPointer((GLint)2, 1, GL_FLOAT, GL_FALSE, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, computer_shader_->getTypesBufferId());
	glVertexAttribPointer((GLint)3, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glDrawTransformFeedback(GL_POINTS, computer_shader_->getTransformFeedbackBufferId());
}

};
