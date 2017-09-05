#ifndef DEMO_LOADER_EXAMPLE_H
#define DEMO_LOADER_EXAMPLE_H

#include "GL/glew.h"

class AnimatedModel;
class SceneManager;
class Player;
class Frustrum;
class Entity;
class Material;
class Cube;

class LoaderExample
{
public:
	LoaderExample(SceneManager& scene_manager);

    bool init();

	Player& getPlayer() const { return *player_; }
	//Entity& getStair() const { return *stair_entity_; }
	const Entity& getStepEntity() const { return *stair_step_entity_; }

	const AnimatedModel& getModel() const { return *snake_; }
private:

	void generateFloor(Entity& parent);
	void generateWall(Entity& parent);

	Material* terrain_material_,* concrete_material_;

	Cube* stair_step_, *floor_, *stair_;

	Player* player_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	GLuint textures[15];

	Entity* stair_step_entity_;

	AnimatedModel* snake_;

	//Entity* stair_entity_;
};

#endif
