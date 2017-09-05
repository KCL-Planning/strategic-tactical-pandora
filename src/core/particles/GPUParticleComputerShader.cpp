#include "GPUParticleComputerShader.h"

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../../core/scene/SceneLeafModel.h"
#include "../../core/texture/RandomTexture.h"

#include "Particle.h"

const GLchar* GPUParticleComputerShader::vars[] = { "gposition", "gvelocity", "glifetime", "gtype" };
const unsigned int GPUParticleComputerShader::MAX_PARTICLES = 10000;

GPUParticleComputerShader::GPUParticleComputerShader(const std::string& vertex_shader, const std::string& geometry_shader, float spawn_rate, float spawn_lifetime)
	: GLSLProgram(vertex_shader, geometry_shader, true), read_buffer_id_(0), write_buffer_id_(1), has_been_updated_at_least_once_(false), spawn_enabled_(true), spawn_rate_(spawn_rate), spawn_lifetime_(spawn_lifetime), initial_particles_(1), total_time_(0)
{
	std::vector<Particle> initial_state;
	init(initial_state);
}

GPUParticleComputerShader::GPUParticleComputerShader(const std::string& vertex_shader, const std::string& geometry_shader, const std::vector<Particle>& initial_state, float spawn_rate, float spawn_lifetime)
	: GLSLProgram(vertex_shader, geometry_shader, true), read_buffer_id_(0), write_buffer_id_(1), has_been_updated_at_least_once_(false), spawn_enabled_(true), spawn_rate_(spawn_rate), spawn_lifetime_(spawn_lifetime), initial_particles_(initial_state.size()), total_time_(0)
{
	init(initial_state);
}

void GPUParticleComputerShader::init(const std::vector<Particle>& initial_state)
{
	// Load the shader.
    if (!initialize())
    {
        std::cout << "Failed to get the particle computer shader!" << std::endl;
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
	delta_loc_ = getUniformLocation("delta");
	random_texture_loc_ = getUniformLocation("random_texture");
	spawn_enabled_loc_ = getUniformLocation("spawn_enabled");
	total_time_loc_ = getUniformLocation("total_time");
	spawn_rate_loc_ = getUniformLocation("spawn_rate");
	spawn_lifetime_loc_ = getUniformLocation("spawn_lifetime");
	spawn_rate_loc_ = getUniformLocation("spawn_rate");
	spawn_lifetime_loc_ = getUniformLocation("spawn_lifetime");

	// Generate the vertex buffers to store the transform feedback ids and buffer ids that store all the information regarding
	// all the particles.
	glGenTransformFeedbacks(2, &particle_tf_id_[0]);
	glGenBuffers(2, &buffer_point_positions_id_[0]);
	glGenBuffers(2, &buffer_point_velocities_id_[0]);
	glGenBuffers(2, &buffer_point_lifetime_id_[0]);
	glGenBuffers(2, &buffer_point_type_id_[0]);

	// Initialise the buffers. We create the particles such that the 0th particle is the emitter that spawn new particles when 
	// necessary. We could do this by checking the primative ID but use this for now to be on the safe side.
	for (unsigned int i = 0; i < 2; ++i)
	{
		glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, particle_tf_id_[i]);

		// Create a buffer object and bind it to the 0th TFB index. This is for the locations of the particles.
		float positions[3 * MAX_PARTICLES];
		memset(&positions[0], 0, sizeof(float) * 3 * MAX_PARTICLES);

		for (unsigned int i = 0; i < initial_state.size(); ++i)
		{
			positions[i * 3] = initial_state[i].position_.x;
			positions[i * 3 + 1] = initial_state[i].position_.y;
			positions[i * 3 + 2] = initial_state[i].position_.z;
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_[i]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * MAX_PARTICLES, &positions[0], GL_DYNAMIC_COPY);
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 0, buffer_point_positions_id_[i]);

		// Create a buffer object and bind it to the 1st TFB index. This is for the velocity of the particles.
		float velocities[3 * MAX_PARTICLES];
		memset(&velocities[0], 0, sizeof(float) * 3 * MAX_PARTICLES);

		for (unsigned int i = 0; i < initial_state.size(); ++i)
		{
			velocities[i * 3] = initial_state[i].velocity_.x;
			velocities[i * 3 + 1] = initial_state[i].velocity_.y;
			velocities[i * 3 + 2] = initial_state[i].velocity_.z;
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffer_point_velocities_id_[i]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * MAX_PARTICLES, &velocities[0], GL_DYNAMIC_COPY);
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 1, buffer_point_velocities_id_[i]);

		// Create a buffer object and bind it to the 2nd TFB index. This is for the lifetime of the particles.
		float lifetime[MAX_PARTICLES];
		memset(&lifetime[0], 0, sizeof(float) * MAX_PARTICLES);

		for (unsigned int i = 0; i < initial_state.size(); ++i)
		{
			lifetime[i] = initial_state[i].lifetime_;
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffer_point_lifetime_id_[i]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * MAX_PARTICLES, &lifetime[0], GL_DYNAMIC_COPY);
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 2, buffer_point_lifetime_id_[i]);

		// Create a buffer object and bind it to the 3rd TFB index. This is for the size of the particles.
		float types[MAX_PARTICLES];
		memset(&types[0], 0, sizeof(float) * MAX_PARTICLES);

		for (unsigned int i = 0; i < initial_state.size(); ++i)
		{
			types[i] = initial_state[i].type_;
		}

		glBindBuffer(GL_ARRAY_BUFFER, buffer_point_type_id_[i]);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * MAX_PARTICLES, &types[0], GL_DYNAMIC_COPY);
		glBindBufferBase(GL_TRANSFORM_FEEDBACK_BUFFER, 3, buffer_point_type_id_[i]);
	}

	// Setup the varyings, this will tell OpenGL what variables we want to capture
	// in our TF buffer.
	glTransformFeedbackVaryings(m_programID, 4, GPUParticleComputerShader::vars, GL_SEPARATE_ATTRIBS);
	linkProgram();
	bindShader();

	random_texture_ = new RandomTexture(1024);
}

void GPUParticleComputerShader::updateParticles(float dt)
{
	total_time_ += dt;
	glEnable(GL_RASTERIZER_DISCARD);

	bindShader();
	
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);
	glDisableVertexAttribArray(7);

	glBindTransformFeedback(GL_TRANSFORM_FEEDBACK, particle_tf_id_[write_buffer_id_]);

	//Send the modelview and projection matrices to the shaders
	glUniform1f(delta_loc_, dt);
	glUniform1i(random_texture_loc_, random_texture_->getActiveTextureId());
	glUniform1f(new_particles_, 10);
	glUniform1i(spawn_enabled_loc_, spawn_enabled_);
	glUniform1f(spawn_rate_loc_, spawn_rate_);
	glUniform1f(spawn_lifetime_loc_, spawn_lifetime_);
	glUniform1f(total_time_loc_, total_time_);

	//Bind the vertex array and set the vertex pointer to point at it
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_[read_buffer_id_]);
	glVertexAttribPointer((GLint)0, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_velocities_id_[read_buffer_id_]);
	glVertexAttribPointer((GLint)1, 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_lifetime_id_[read_buffer_id_]);
	glVertexAttribPointer((GLint)2, 1, GL_FLOAT, GL_FALSE, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_type_id_[read_buffer_id_]);
	glVertexAttribPointer((GLint)3, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glBeginTransformFeedback(GL_POINTS);

	// If this is the first time that we render these particles we need to do so manually. After the first time
	// we can use the transform feedback renderer to do the rendering for us.
	if (!has_been_updated_at_least_once_)
	{
		glDrawArrays(GL_POINTS, 0, initial_particles_);
		has_been_updated_at_least_once_ = true;
	}
	else
	{
/*
		// Lets also check how many primitives are actually rendered here...
		GLuint q;
		glGenQueries(1, &q);
		glBeginQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN, q);
*/
		glDrawTransformFeedback(GL_POINTS, particle_tf_id_[read_buffer_id_]);
/*
		glEndQuery(GL_TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN);

		// Retrieve the results.
		GLint is_ready = GL_FALSE;
		GLint primitives;
		while (is_ready != GL_TRUE)
		{
			glGetQueryObjectiv(q, GL_QUERY_RESULT_AVAILABLE, &is_ready);
		}
		glGetQueryObjectiv(q, GL_QUERY_RESULT, &primitives);
		std::stringstream ss;

		ss << "Rendered: " << primitives << " primitives using glDrawTransformFeedback!" << std::endl;
		OutputDebugString(ss.str().c_str());

		if (primitives == 0)
		{
			primitives = 0;
		}

		if (primitives > 1)
		{
			primitives = 0;
		}
*/
	}

	glEndTransformFeedback();
	
	glDisable(GL_RASTERIZER_DISCARD);

	// Swap the buffers before drawing.
	read_buffer_id_ = (read_buffer_id_ + 1) % 2;
	write_buffer_id_ = (write_buffer_id_ + 1) % 2;
}

void GPUParticleComputerShader::updateParticlePosition(const glm::vec3& new_position)
{
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_[read_buffer_id_]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * 3, &new_position);
}

void GPUParticleComputerShader::updateParticlePositions(const std::vector<glm::vec3>& new_positions)
{
	glBindBuffer(GL_ARRAY_BUFFER, buffer_point_positions_id_[read_buffer_id_]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(GLfloat) * 3 * new_positions.size(), &new_positions[0]);
}
