#ifndef DEMO_REGION_TEST_REGION_WORLD_H
#define DEMO_REGION_TEST_REGION_WORLD_H

#include <map>

#include "../ApplicationInterface.h"

class SceneLeafModel;
class Camera;
class Entity;
class Region;
class SceneManager;
class SceneNode;
class Texture;
class Terrain;
class PhysicsCube;

class RegionWorld : public ApplicationInterface
{
public:
	RegionWorld(SceneManager& scene_manager);
	
	bool init(int argc, char** argv);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	Camera& getCamera() const { return *camera_; }

	void onResize(int width, int height);
private:
	Terrain* terrain_;
	Entity* terrain_node_;
	SceneManager* scene_manager_;
	int width_, height_;
	Camera* camera_;
	PhysicsCube* cube_;

	Entity* movable_node_;
	
	std::map<Region*, SceneLeafModel*> region_to_visible_model_;
};

#endif
