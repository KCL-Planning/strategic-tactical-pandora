#ifndef DEMO_GUI_TESTER_H
#define DEMO_GUI_TESTER_H

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
class PointLight;
class SceneLeafLight;
class ShadowRenderer;
class Texture;
class FPSLabel;

class GUITester : public ApplicationInterface
{
public:
	GUITester(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	void tick(float dt);

	/**
	 * Apply any post processing effects.
	 * @param framebuffer_id The framebuffer id prior to post processing.
	 * @param dt The time elapsed since the last frame had rendered.
	 * @return The framebuffer ID that should be used for the final result. The default one is 0.
	 */
	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	Player& getPlayer() const { return *player_; }

	Camera& getCamera() const { return *camera_; }

	void onResize(int width, int height);
private:

	Material* terrain_material_,* concrete_material_;

	Entity* terrain_node_;

	SceneNode* light_node_;

	Player* player_;
	Camera* camera_;

	ShadowRenderer* shadow_renderer_;

	SceneManager* scene_manager_;
	Frustrum* frustrum_;

	Terrain* terrain_;

	SceneLeafModel* light_volume_leaf_;

	PointLight* volumetric_light_point_;
	SceneLeafLight* volumetric_light_leaf_;

	GLuint fbo_id_;//, texture_id_;
	Texture* texture_;

	FPSLabel* fps_label_;
};

#endif
