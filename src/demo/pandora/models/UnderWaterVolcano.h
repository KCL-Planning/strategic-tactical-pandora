#ifndef PANDORA_MODELS_UNDER_WATER_VOLCANO_H
#define PANDORA_MODELS_UNDER_WATER_VOLCANO_H

#include <vector>
#include <glm/glm.hpp>

#include "dpengine/entities/Entity.h"

namespace DreadedPE
{
	class Texture;
	class GPUParticleEmitter;
};

/**
 * A model that emits particles into the water that are affected by its vector field. This simulates
 * the current of the water and visualises the underwater currents.
 */
class UnderWaterVolcano : public DreadedPE::Entity
{
public:
	UnderWaterVolcano(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::vector<glm::vec3>& under_water_volcano_locations);
	
	void prepare(float dt);
private:
	float total_time_;
	
	DreadedPE::Texture* smoke_texture_; // The texture of the particles.
	DreadedPE::GPUParticleEmitter* particle_emitter_; // The particle system.
};

#endif
