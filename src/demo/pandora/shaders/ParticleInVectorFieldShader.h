#ifndef PANDORA_SHADERS_PARTICLE_IN_VECTOR_FIELD_SHADER_H
#define PANDORA_SHADERS_PARTICLE_IN_VECTOR_FIELD_SHADER_H

class ParticleEmitter;

#include "dpengine/shaders/glslshader.h"
#include "dpengine/shaders/ShaderInterface.h"

/**
 * This shader is used in conjunction with a geometric shader the updates and reads back the 
 * parameters of each particle.
 * 
 * Each particle emitter must have their own shader instance because we need to read back 
 * the values and if multiple emitters use the same shader than these values will be overwritten.
 */
class ParticleInVectorFieldShader : public DreadedPE::GLSLProgram
{
public:
	ParticleInVectorFieldShader(ParticleEmitter& particle_system);
	ParticleInVectorFieldShader(const std::string& vertex_shader, ParticleEmitter& particle_system);
	
	/**
	 * We do not render scene leaf lights...
	 */
	void initialise(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const DreadedPE::SceneLeafLight*>& lights, float dt);

	GLuint getTFPositions() { return buffer_tf_id_[0]; }
	GLuint getTFVelocities() { return buffer_tf_id_[1]; }
	GLuint getTFLifeTimes() { return buffer_tf_id_[2]; }

	GLuint getPositionsBufferId() { return buffer_point_positions_id_; }
	GLuint getVelocitiesBufferId() { return buffer_point_velocities_id_; }
	GLuint getLifeTimeBufferId() { return buffer_point_lifetime_id_; }

protected:
	GLuint delta_loc_;
	
private:
	void init();
	
	ParticleEmitter* particle_system_;

	GLuint particle_tf_id_;
	GLuint buffer_tf_id_[3];
	GLuint buffer_point_positions_id_;
	GLuint buffer_point_velocities_id_;
	GLuint buffer_point_lifetime_id_;

	static const GLchar* vars[];
};

#endif
