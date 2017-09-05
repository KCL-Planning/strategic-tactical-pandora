#include "ParticleComputerShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../particles/ParticleEmitter.h"
#include "../scene/SceneLeafModel.h"

const GLchar* ParticleComputerShader::vars[] = { "tf_position", "tf_velocity", "tf_lifetime" };

/*
GLuint ParticleComputerShader::delta_loc_ = 0;

GLuint ParticleComputerShader::particle_tf_id_ = 0;
GLuint ParticleComputerShader::buffer_tf_id_[3] = { 0, 0, 0 };
GLuint ParticleComputerShader::buffer_point_positions_id_ = 0;
GLuint ParticleComputerShader::buffer_point_velocities_id_ = 0;
GLuint ParticleComputerShader::buffer_point_lifetime_id_ = 0;
*/

ParticleComputerShader::ParticleComputerShader(ParticleEmitter& particle_system)
	: GLSLProgram("shaders/ParticleCalculator.vert"), particle_system_(&particle_system)
{
	init();
}

ParticleComputerShader::ParticleComputerShader(const std::string& vertex_shader, ParticleEmitter& particle_system)
	: GLSLProgram(vertex_shader), particle_system_(&particle_system)
{
	init();
}

void ParticleComputerShader::init()
{
	// Load the shader.
        if (!initialize())
        {
            std::cout << "Failed to get the particle in vector field shader!" << std::endl;
            exit(1);
        }
	
	//Bind the attribute locations
	bindAttrib(0, "position");
	bindAttrib(1, "velocity");
	bindAttrib(2, "lifetime");

	//Re link the program
	linkProgram();
	bindShader();

	// Cache the locations of all the uniform variables.
	delta_loc_ = getUniformLocation("delta");

	glGenTransformFeedbacks(1, &particle_tf_id_);
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, particle_tf_id_);

	// Create a pair of buffers to store all the point data.
	glGenBuffers(1, &buffer_point_positions_id_);
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * particle_system_->getMaxNrParticles(), NULL, GL_DYNAMIC_COPY);

	glGenBuffers(1, &buffer_point_velocities_id_);
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_velocities_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * particle_system_->getMaxNrParticles(), NULL, GL_DYNAMIC_COPY);

	glGenBuffers(1, &buffer_point_lifetime_id_);
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_lifetime_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * particle_system_->getMaxNrParticles(), NULL, GL_DYNAMIC_COPY);

	// Create a pair of buffers to store all the feedback data.
	glGenBuffers(3, buffer_tf_id_);

	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, particle_tf_id_);
	for (int i = 0; i < 3; ++i)
	{
		// Bind the buffer.
		glBindBuffer(GL_TRANSFORM_FEEDBACK_BUFFER, buffer_tf_id_[i]);

		if (i == 2)
		{
			// Allocate space for it.
			glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER,
				sizeof(float) * particle_system_->getMaxNrParticles(),
				NULL,
				GL_DYNAMIC_COPY);
		}
		else
		{
			// Allocate space for it.
			glBufferData(GL_TRANSFORM_FEEDBACK_BUFFER,
				sizeof(float) * 3 * particle_system_->getMaxNrParticles(),
				NULL,
				GL_DYNAMIC_COPY);
		}

		// Bind this buffer to an indexed buffer binding point.
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, i, buffer_tf_id_[i]);
	}

	// Setup the varyings, this will tell OpenGL what variables we want to capture
	// in our TF buffer.
	glTransformFeedbackVaryings(m_programID, 3, ParticleComputerShader::vars, GL_SEPARATE_ATTRIBS);
	linkProgram();
	bindShader();
}

void ParticleComputerShader::initialise(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, float dt)
{
	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, particle_tf_id_);

	// Copy the particle values to the buffers.
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * particle_system_->getMaxNrParticles(), particle_system_->getPositions(), GL_DYNAMIC_COPY);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_velocities_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * particle_system_->getMaxNrParticles(), particle_system_->getVelocities(), GL_DYNAMIC_COPY);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_lifetime_id_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * particle_system_->getMaxNrParticles(), particle_system_->getLifeTimes(), GL_DYNAMIC_COPY);

	bindShader();
	
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);
	glDisableVertexAttribArray(7);

	//Send the modelview and projection matrices to the shaders
	glUniform1f(delta_loc_, dt);

	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_);
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_velocities_id_);
	glVertexAttribPointer((GLint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_lifetime_id_);
	glVertexAttribPointer((GLint)2, 1, GL_FLOAT, GL_FALSE, 0, 0);
}

