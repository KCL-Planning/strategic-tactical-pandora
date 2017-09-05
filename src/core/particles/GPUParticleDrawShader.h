#ifndef DEMO_PARTICLE__DEMO_GPU_PARTICLE_SHADER_H
#define DEMO_PARTICLE__DEMO_GPU_PARTICLE_SHADER_H

#include <GL/glew.h>
#include <string>
#include "../../core/shaders/glslshader.h"
#include "../../core/shaders/ShaderInterface.h"

class GPUParticleComputerShader;
class Texture;

class GPUParticleDrawShader : public GLSLProgram, public ShaderInterface
{
public:
	GPUParticleDrawShader(GPUParticleComputerShader& computer_shader, const Texture& texture, const std::string& vertex_shader, const std::string& geometry_shader, const std::string& fragment_shader);
	//GPUParticleDrawShader(GPUParticleComputerShader& computer_shader, const std::string& particle_fragment_shader, const Texture& texture);
	
	void drawParticles(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, const glm::vec3& camera_location);
	
	// We ignore the draw commands from the interface as we never use them.
	void initialise(const SceneLeafLight& light_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights) {}
	void initialise(const SceneLeafModel& model_node, const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights) {}

	GPUParticleComputerShader& getComputeShader() const { return *computer_shader_; }


protected:
	GLuint modelview_matrix_loc_, projection_matrix_loc_, camera_loc_, texture0_loc_;
	
private:
	void init();

	const Texture* texture_;

	GPUParticleComputerShader* computer_shader_;
};

#endif
