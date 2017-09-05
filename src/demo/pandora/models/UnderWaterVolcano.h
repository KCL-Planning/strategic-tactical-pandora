#ifndef PANDORA_MODELS_UNDER_WATER_VOLCANO_H
#define PANDORA_MODELS_UNDER_WATER_VOLCANO_H

#include <vector>
#include <glm/glm.hpp>

#include "../../../core/entities/Entity.h"

class Texture;
class GPUParticleEmitter;

/**
 * A model that emits particles into the water that are affected by its vector field. This simulates
 * the current of the water and visualises the underwater currents.
 */
class UnderWaterVolcano : public Entity
{
public:
	UnderWaterVolcano(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::vector<glm::vec3>& under_water_volcano_locations);
	
	void prepare(float dt);
private:
	float total_time_;
	
	Texture* smoke_texture_; // The texture of the particles.
	GPUParticleEmitter* particle_emitter_; // The particle system.
};

#endif
