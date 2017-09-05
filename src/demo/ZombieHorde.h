/**
 * A demo to test whether we can render many animated models all going after a player.
 */
#ifndef DEMO_ZOMBIE_HORDE_H
#define DEMO_ZOMBIE_HORDE_H

#include <GL/glew.h>
#include "ApplicationInterface.h"

class SceneManager;
class Texture;
class Camera;
class FPSLabel;
class InstanceRenderedShape;
class InstanceRenderedSceneNode;

class ZombieHorde : public ApplicationInterface
{
public:
	ZombieHorde(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	Camera& getCamera() const { return *camera_node_; }

	void onResize(int width, int height);
private:
	Camera* camera_node_;
	SceneManager* scene_manager_;

	InstanceRenderedShape* single_instance_;
	InstanceRenderedSceneNode* instance_rendered_node_;

	FPSLabel* fps_label_;
	int width_, height_;
};

#endif
