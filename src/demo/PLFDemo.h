#ifndef DEMO_PLF_DEMO_H
#define DEMO_PLF_DEMO_H

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
class Label;

class PLFDemo : public ApplicationInterface
{
public:
	PLFDemo(SceneManager& scene_manager);

	bool init(int argc, char** argv);
	bool postInit();

	void tick(float dt);

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);
	
	Player& getPlayer() const { return *player_; }
	//Entity& getStair() const { return *stair_entity_; }
	const Entity& getStepEntity() const { return *stair_step_entity_; }

	Camera& getCamera() const { return *camera_; }

	const AnimatedModel& getModel() const { return *snake_; }

	const Monster& getMonster() const { return *monster_; }
	void onResize(int width, int height);
private:

	void generateFloor(Entity& parent);
	void generateWall(Entity& parent);

	Material* terrain_material_,* concrete_material_;

	Entity* terrain_node_;

	Cube* stair_step_, *floor_, *stair_;

	Player* player_;
	Camera* camera_;

	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	GLuint textures[15];

	Entity* stair_step_entity_;

	AnimatedModel* snake_;
	Terrain* terrain_;

	Monster* monster_;
	SceneNode* level_node_;
	NavigationMesh* navigation_mesh_;

	NavMeshAStar* pathfinder_;

	SceneLeafModel* waypoint_visualiser_;
	FPSLabel* fps_label_;
	
	Label* debug_label_;
	
	int width_, height_;
	//Entity* stair_entity_;
};

#endif
