#ifndef DEMO_PARTICLE__DEMO_GPU_PARTICLE_EMITTER_H
#define DEMO_PARTICLE__DEMO_GPU_PARTICLE_EMITTER_H

#include <vector>

#include <GL/glew.h>
#include <glm/glm.hpp>

#include "../scene/RenderableSceneLeaf.h"

class GPUParticleComputerShader;
class GPUParticleDrawShader;
class Texture;

/**
 * An entity that can emit particles.
 */
class GPUParticleEmitter : public RenderableSceneLeaf
{
public:
	GPUParticleEmitter(SceneNode& scene_node, GPUParticleComputerShader& compute_shader, GPUParticleDrawShader& draw_shader);
	
	void accept(SceneVisitor& visitor) const;
	
	void preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights);

	void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const;
	
	void prepare(float dt);
	//Texture& getTexture() const { return *texture_; }
	
	/**
	 * TODO: Properly implement this.
	 */
	void initialiseFrustrumChecker();
private:
	/**
	 * We update the locations of the particles on the GPU.
	 */
	void updateParticles() const;
	
	GPUParticleComputerShader* compute_shader_; // The geometry shader that update the locations of the particles on the GPU.
	GPUParticleDrawShader* draw_shader_; // The geometry shader that update the locations of the particles on the GPU.

	//Texture* texture_; // The texture used to render the particles.
	float dt_;         // The dt given by the last time 'prepare' was called.

	glm::vec3 camera_position_;              // The location of where the particles are being rendered from.
};

#endif
