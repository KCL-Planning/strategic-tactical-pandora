#ifndef DEMO_FALLING_BLOCK_H
#define DEMO_FALLING_BLOCK_H

#include "GL/glew.h"
#include "ApplicationInterface.h"

class AnimatedModel;
class NavMeshAStar;
class SceneManager;
class Player;
class Frustrum;
class Entity;
class Material;
class Cube;
class Terrain;
class Monster;
class SceneNode;
class NavigationMesh;
class Camera;
class DynamicCamera;
class SceneLeafModel;
class Texture;
class FPSLabel;
class PhysicsCube;

class FallingBlock : public ApplicationInterface
{
public:
	FallingBlock(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	void tick(float dt);

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);
	
	Camera& getCamera() const { return *camera_; }
	void onResize(int width, int height);
private:
	Camera* camera_;
	PhysicsCube* cube_;
	Entity* terrain_node_;
	Entity* rotating_cube_entity_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	Terrain* terrain_;
	FPSLabel* fps_label_;
	
	int width_, height_;
};

#endif
