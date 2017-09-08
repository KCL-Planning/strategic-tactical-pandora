#include "UnderWaterVolcano.h"

#include <iostream>
#include <stdlib.h>
#include <memory>
#include <glm/gtc/matrix_transform.hpp>

#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/particles/GPUParticleEmitter.h"
#include "dpengine/particles/GPUParticleComputerShader.h"
#include "dpengine/particles/GPUParticleDrawShader.h"
#include "dpengine/particles/Particle.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/shaders/BasicShadowShader.h"

#include "dpengine/shapes/Cube.h"

UnderWaterVolcano::UnderWaterVolcano(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, const std::vector<glm::vec3>& under_water_volcano_locations)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, "UnderWaterVolcano"), total_time_(0)
{
	//updateTransformations();
	// Set the initial particle state.
	std::vector<DreadedPE::Particle> initial_particle_state;

	//std::shared_ptr<DreadedPE::Cube> underwater_volcano_model(std::make_shared<DreadedPE::Cube>(0.1f, 0.1f, 0.1f));

	for (std::vector<glm::vec3>::const_iterator ci = under_water_volcano_locations.begin(); ci != under_water_volcano_locations.end(); ++ci)
	{
		//DreadedPE::SceneNode* dummy_node = new DreadedPE::SceneNode(scene_manager, this, glm::translate(glm::mat4(1.0f), *ci));
		//DreadedPE::SceneLeafModel* dummy_cube = new DreadedPE::SceneLeafModel(*dummy_node, NULL, underwater_volcano_model, *DreadedPE::SceneNode::bright_material_, DreadedPE::BasicShadowShader::getShader(), false, false);

		DreadedPE::Particle particle(*ci, glm::vec3(0, 0, 0), 0, 0);
		initial_particle_state.push_back(particle);
	}

	smoke_texture_ = DreadedPE::TargaTexture::loadTexture("data/textures/smoke_particle.tga");
	DreadedPE::GPUParticleComputerShader* computer_shader = new DreadedPE::GPUParticleComputerShader("src/demo/pandora/shaders/resources/ParticleInVectorField.vert", "src/demo/pandora/shaders/resources/ParticleCalculator.geom", initial_particle_state, 0.05f, 4.0f);
	DreadedPE::GPUParticleDrawShader* draw_shader = new DreadedPE::GPUParticleDrawShader(*computer_shader, *smoke_texture_, "src/demo/pandora/shaders/resources/particle.vert", "src/demo/pandora/shaders/resources/particle_bb.geom", "src/demo/pandora/shaders/resources/SmokeParticle.frag");
	particle_emitter_ = new DreadedPE::GPUParticleEmitter(*this, *computer_shader, *draw_shader);
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
	DreadedPE::SceneNode::prepare(dt);
}
