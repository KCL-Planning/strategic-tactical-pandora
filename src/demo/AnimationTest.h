#ifndef DEMO_ANIMATION_TEST_H
#define DEMO_ANIMATION_TEST_H

#include <GL/glew.h>

#include "ApplicationInterface.h"

class AnimatedModel;
class Camera;
class Entity;
class FPSLabel;
class Monster;
class Player;
class SceneManager;
class SceneNode;
class Texture;

class AnimationTest : public ApplicationInterface
{
public:
	AnimationTest(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	void tick(float dt);
	
	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);
	
	Camera& getCamera() const { return *camera_; }
	
	void onResize(int width, int height);

private:
	SceneManager* scene_manager_;
	Camera* camera_;
	Player* player_;
	
	FPSLabel* fps_label_;
	Entity* terrain_node_;
	Entity* dragon_node_;
	
	Monster* monster_;
	AnimatedModel* dragon_;
	
	int width_, height_;
};

#endif
