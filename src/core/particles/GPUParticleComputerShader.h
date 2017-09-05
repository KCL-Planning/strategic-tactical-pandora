#ifndef DEMO_PARTICLE__DEMO_GPU_PARTICLE_COMPUTER_SHADER_H
#define DEMO_PARTICLE__DEMO_GPU_PARTICLE_COMPUTER_SHADER_H

#include "../shaders/glslshader.h"
#include "../shaders/ShaderInterface.h"

class Particle;
class Texture;

/**
 * This shader is used in conjunction with a geometric shader the updates and reads back the 
 * parameters of each particle.
 * 
 * Each particle emitter must have their own shader instance because we need to read back 
 * the values and if multiple emitters use the same shader than these values will be overwritten.
 */
class GPUParticleComputerShader : public GLSLProgram
{
public:
	/**
	 * Create the shader that calculates the new attributes of all the particles.
	 * @param vertex_shader The path to the vertex shader.
	 * @param geometry_shader The path to the geometry shader.
	 */
	GPUParticleComputerShader(const std::string& vertex_shader, const std::string& geometry_shader, float spawn_rate, float spawn_lifetime);
	
	GPUParticleComputerShader(const std::string& vertex_shader, const std::string& geometry_shader, const std::vector<Particle>& initial_state, float spawn_rate, float spawn_lifetime);
	
	void updateParticles(float dt);

	/**
	 * Enabled / disable the spawn of new particles.
	 * @param spawn_enabled If set to true new particles are allowed to spawn, otherwise only the existing particles
	 * will be processed and the spawn of new ones are enabled. This effect differs from particle to particle shaders
	 * and some might ignore it completely. Default = true.
	 */
	void enableParticleSpawn(bool spawn_enabled) { spawn_enabled_ = spawn_enabled; }

	void updateParticlePosition(const glm::vec3& new_position);
	void updateParticlePositions(const std::vector<glm::vec3>& new_positions);

	GLuint getTransformFeedbackBufferId() const { return particle_tf_id_[read_buffer_id_]; }

	GLuint getPositionsBufferId() const { return buffer_point_positions_id_[read_buffer_id_]; }
	GLuint getVelocitiesBufferId() const { return buffer_point_velocities_id_[read_buffer_id_]; }
	GLuint getLifetimesBufferId() const { return buffer_point_lifetime_id_[read_buffer_id_]; }
	GLuint getTypesBufferId() const { return buffer_point_type_id_[read_buffer_id_]; }

protected:
	GLuint delta_loc_, new_particles_;
	
private:
	void init(const std::vector<Particle>& initial_state);

	static const unsigned int MAX_PARTICLES;

	// The ids of the two transform feedback objects.
	GLuint particle_tf_id_[2];

	// The ids of the buffers that contain all the information regarding particles.
	GLuint buffer_point_positions_id_[2];
	GLuint buffer_point_velocities_id_[2];
	GLuint buffer_point_lifetime_id_[2];
	GLuint buffer_point_type_id_[2];

	// Keep track on which buffer is active. At any one time one of them will be used for reading and the 
	// other one will be used for writing.
	GLuint read_buffer_id_;
	GLuint write_buffer_id_;

	// Keep track whether the particles have been updated at least once. The first time we need to manually
	// render them. The subsequent times we can use the transform feedback draw to do this for us.
	bool has_been_updated_at_least_once_;

	// Locations of all the uniform variables in the shader.
	GLuint random_texture_loc_, spawn_enabled_loc_, total_time_loc_, spawn_rate_loc_, spawn_lifetime_loc_;

	// Keep track whether new particles are allowed to spawn.
	bool spawn_enabled_;

	// Store the spawn and life times of the particles.
	float spawn_rate_, spawn_lifetime_;

	// In order to introduce randomness in the shader we use a 1D texture that contains random numbers.
	Texture* random_texture_;

	// Store the number of initial particles, this is relevant for the initial draw command that initialises transform feedback.
	unsigned int initial_particles_;

	// We keep track of the total time spend for the benefit of the random generation texture used in the geometric shader.
	float total_time_;

	enum PARTICLE_TYPE { INACTIVE, SPAWNER, NORMAL_SMOKE };

	static const GLchar* vars[];
};

#endif
