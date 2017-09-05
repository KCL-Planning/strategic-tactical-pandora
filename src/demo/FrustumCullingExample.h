#ifndef DEMO_FRUSTUM_CULLING_EXAMPLE_H
#define DEMO_FRUSTUM_CULLING_EXAMPLE_H

#include "GL/glew.h"

class SceneManager;
class Player;
class Frustrum;
class Entity;
class Material;
class Cube;
class Camera;

class FrustumCullingExample
{
public:
	FrustumCullingExample(SceneManager& scene_manager);

    bool init(int argc, char** argv);

	bool postInit() { return true; }

	void tick(float dt) { }

	const Camera& getCamera() const { return *camera_; }

	Player& getPlayer() const { return *player_; }
	//Entity& getStair() const { return *stair_entity_; }
	const Entity& getStepEntity() const { return *stair_step_entity_; }
private:

	void generateFloor(Entity& parent);
	void generateWall(Entity& parent);

	Material* terrain_material_,* concrete_material_;

	Cube* stair_step_, *floor_, *stair_;

	Player* player_;
	Camera* camera_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	GLuint textures[15];

	Entity* stair_step_entity_;

	//Entity* stair_entity_;
};

#endif
