#ifndef _TEST_SCENE_H
#define _TEST_SCENE_H

#include <vector>
#include "GL/glew.h"

#include "../core/shaders/LightShader.h"
#include "../shapes/Cube.h"
#include "../shapes/terrain.h"

class Camera;
class Light;
class Sphere;
class Tree;
class Reflection;
class Water;
class Cube;
class LightManager;
class GodRays;
//class FreeTypeFont;
class SceneManager;
class SimpleRenderer;
class Player;
class Bridge;
class Frustrum;
class SceneLeaf;
class SphereCheck;

class TestScene 
{
public:
	TestScene(SceneManager& scene_manager);

	bool init(int argc, char** argv);

	Camera& getCamera() const { return *camera_; }

	Player& getPlayer() const { return *player_; }

	const SceneLeaf& getSceneLeaf() const { return *leaf_; }

	const SphereCheck& getSphereCheck() const { return *sphere_check_; }

	void tick(float dt) { }

	bool postInit() { return true; }

private:

	Player* player_;
	Camera* camera_;

	SceneManager* scene_manager_;

	bool checkError(const std::string& description) const;
	GLuint textures[15];
	LightShader* m_GLSLProgram;

	// Font.
//	FreeTypeFont* font;
	Frustrum* frustrum_;

	SceneLeaf* leaf_;

	SphereCheck* sphere_check_;
};

#endif
