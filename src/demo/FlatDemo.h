#ifndef DEMO_FLAT_DEMO_H
#define DEMO_FLAT_DEMO_H

#include "GL/glew.h"
#include "ApplicationInterface.h"

class SceneManager;
class Player;
class ArmedPlayer;
class Frustrum;
class Entity;
class Material;
class Cube;
class Camera;
class Texture;
class FPSLabel;

class FlatDemo : public ApplicationInterface
{
public:
	FlatDemo(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();
	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	ArmedPlayer& getPlayer() const { return *player_; }
	//Entity& getStair() const { return *stair_entity_; }

	Camera& getCamera() const { return *camera_; }

	void onResize(int width, int height);
private:

	void generateFloor(Entity& parent);
	void generateWall(Entity& parent);

	Material* terrain_material_,* concrete_material_;

	Cube* stair_step_, *floor_, *stair_;

	ArmedPlayer* player_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	GLuint textures[15];

	Camera* camera_;
	FPSLabel* fps_label_;
	int width_, height_;
	//Entity* stair_entity_;
};

#endif
