#ifndef DEMO_PARTICLE_TEST_H
#define DEMO_PARTICLE_TEST_H

#include "GL/glew.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "ApplicationInterface.h"

class SceneManager;
class Frustrum;
class Entity;
class Texture;

class GPUParticleEmitter;
class GPUParticleDrawShader;
class GPUParticleComputerShader;

class ParticleTest : public ApplicationInterface
{
public:
	ParticleTest(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	Camera& getCamera() const { return *camera_node_; }

	void onResize(int width, int height);
private:
	Entity* particle_root_;
	Camera* camera_node_;
	SceneManager* scene_manager_;

	Texture* smoke_texture_;

	GPUParticleEmitter* particle_emitter_;
	GPUParticleDrawShader* particle_drawer_;
	GPUParticleComputerShader* particle_computer_;

	GPUParticleEmitter* particle_emitter2_;
	GPUParticleDrawShader* particle_drawer2_;
	GPUParticleComputerShader* particle_computer2_;

	float total_time_;
	int width_, height_;
};

#endif
