#include "UnderWaterVolcano.h"

#include <iostream>
#include <stdlib.h>

#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/entities/Entity.h"
#include "../../../core/particles/GPUParticleEmitter.h"
#include "../../../core/particles/GPUParticleComputerShader.h"
#include "../../../core/particles/GPUParticleDrawShader.h"
#include "../../../core/particles/Particle.h"
#include "../../../core/texture/Texture.h"
#include "../../../core/texture/TargaTexture.h"
#include "../../../core/shaders/BasicShadowShader.h"

#include "../../../shapes/Cube.h"

UnderWaterVolcano::UnderWaterVolcano(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const std::vector<glm::vec3>& under_water_volcano_locations)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "UnderWaterVolcano"), total_time_(0)
{
	//updateTransformations();
	// Set the initial particle state.
	std::vector<Particle> initial_particle_state;

	Cube* underwater_volvano_model = new Cube(0.1f, 0.1f, 0.1f);

	for (std::vector<glm::vec3>::const_iterator ci = under_water_volcano_locations.begin(); ci != under_water_volcano_locations.end(); ++ci)
	{
		SceneNode* dummy_node = new SceneNode(scene_manager, this, glm::translate(glm::mat4(1.0f), *ci));
		SceneLeafModel* dummy_cube = new SceneLeafModel(*dummy_node, NULL, *underwater_volvano_model, *SceneNode::bright_material_, BasicShadowShader::getShader(), false, false);

		Particle particle(*ci, glm::vec3(0, 0, 0), 0, 0);
		initial_particle_state.push_back(particle);
	}

	smoke_texture_ = TargaTexture::loadTexture("data/textures/smoke_particle.tga");
	GPUParticleComputerShader* computer_shader = new GPUParticleComputerShader("src/demo/pandora/shaders/resources/ParticleInVectorField.vert", "src/demo/pandora/shaders/resources/ParticleCalculator.geom", initial_particle_state, 0.05f, 4.0f);
	GPUParticleDrawShader* draw_shader = new GPUParticleDrawShader(*computer_shader, *smoke_texture_, "src/demo/pandora/shaders/resources/particle.vert", "src/demo/pandora/shaders/resources/particle_bb.geom", "src/demo/pandora/shaders/resources/SmokeParticle.frag");
	particle_emitter_ = new GPUParticleEmitter(*this, *computer_shader, *draw_shader);
}

void UnderWaterVolcano::prepare(float dt)
{
	total_time_ += dt;
	/*
	// Generate some water bubbles at the propeller.
	if (total_time_ > 0.5f)
	{
		total_time_ = 0;
		
		// Generate some particles :D.
		glm::vec3 particle_direction(0, 1, 0);

		// Generate an axis system where 'particle_direction' is one of the axis.
		glm::vec3 axis_y = glm::normalize(glm::vec3(-particle_direction.y / particle_direction.x, 1, 0));
		glm::vec3 axis_z = glm::cross(particle_direction, axis_y);
		for (int i = 0; i < 25; ++i)
		{
			float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float s = (1 - r * r) * (rand() % 2 == 0 ? -1 : 1);
			r *= (rand() % 2 == 0 ? -1 : 1);

			float scale = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			r *= scale * scale * 0.6;
			s *= scale * scale * 0.7;

			float size = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 12.0f;
			float noisex = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float noisey = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			float noisez = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
			particle_emitter_->addParticle(getGlobalLocation() + glm::vec3(noisex * 0.1, 0, noisez * 0.1), glm::vec3(noisex / 4, noisey / 2 + 0.25f, noisez / 4), size, size * 2);
			//particle_emitter_->addParticle(glm::vec3(0, 0, 5), glm::vec3(0, 1, 0), 12.0f);
		}
	}
	*/
	SceneNode::prepare(dt);
}
