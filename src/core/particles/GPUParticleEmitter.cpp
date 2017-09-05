#include "GPUParticleEmitter.h"

#include "../texture/Texture.h"
#include "../renderer/ShadowRenderer.h"

#include "GPUParticleComputerShader.h"
#include "GPUParticleDrawShader.h"

GPUParticleEmitter::GPUParticleEmitter(SceneNode& scene_node, GPUParticleComputerShader& compute_shader, GPUParticleDrawShader& draw_shader)
	: RenderableSceneLeaf(scene_node, true, true, OBJECT, ShadowRenderer::STATIC_SHADOW), compute_shader_(&compute_shader), draw_shader_(&draw_shader), dt_(0), camera_position_(glm::vec3(0, 0, 0))
{

}

void GPUParticleEmitter::prepare(float dt)
{
	dt_ = dt;
}
	
void GPUParticleEmitter::accept(SceneVisitor& visitor) const
{
	visitor.visit(*this);
}

void GPUParticleEmitter::initialiseFrustrumChecker()
{
	frustum_checker_ = NULL;
}

void GPUParticleEmitter::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights)
{
	// TODO: Check if the particles need to be rendered.
	
	// Save the location of the camera so we make all the particles face the camera whilst drawing.
	
	camera_position_ = camera_position;
	renderer.visit(*this);
}

void GPUParticleEmitter::draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader) const
{
	compute_shader_->updateParticles(dt_);
	draw_shader_->drawParticles(view_matrix, glm::mat4(1.0f), projection_matrix, lights, camera_position_);
}
