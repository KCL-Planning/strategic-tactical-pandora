#ifndef DEMO_PANDORA_PILLAR_EXPERIMENT_H
#define DEMO_PANDORA_PILLAR_EXPERIMENT_H

#ifndef _WIN32
#include <ros/ros.h>
#endif

#include "GL/glew.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "ApplicationInterface.h"

class ActionLabel;
class ActionController;
class OntologyInterface;
class AnimatedModel;
class Button;
class SceneManager;
class AUV;
class Frustrum;
class Entity;
class Material;
class Cube;
class Terrain;
class SceneNode;
class HeightMap;
class Label;
class Odometry;
class PlanningGUI;
class ValvePanel;
class OctomapBuilder;
class RRT;
class PlanLine;
class Texture;
class FPSLabel;
class Pillar;
class Sonar;
class CausticTexture;
class Font;
class Theme;

class PandoraPillarExperiment : public ApplicationInterface
{
public:
	PandoraPillarExperiment(SceneManager& scene_manager);

	bool init(int argc, char** argv);//, bool use_hwu_ontology = false);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	//AUV& getPlayer() const { return *auv_; }
	const Entity& getStepEntity() const { return *stair_step_entity_; }
	//Camera& getCamera() const { return camera_node_->getCamera(); }
	Camera& getCamera() const { return *camera_node_; }
	Camera& getCameraNode() const { return *camera_node_; }

	void onResize(int width, int height);
private:

	void generateFloor(Entity& parent);
	void generateWall(Entity& parent);

	Material* terrain_material_,* concrete_material_;

	HeightMap* terrain_node_;

	Cube* stair_step_, *floor_, *stair_;
	DynamicCamera* camera_node_;

	//Player* player_;
	AUV* auv1_;
	AUV* auv2_;
	SceneNode* stable_platform1_;
	SceneNode* stable_platform2_;

	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	GLuint textures[15];

	Entity* stair_step_entity_;

	Terrain* terrain_;

	//SceneNode* level_node_;
	
	ValvePanel* value_panel_;
	
	ShadowRenderer* shadow_renderer_;

	//Button* start_rrt_;
	//Button* stop_rrt_;

	//Label* label_;
	PlanningGUI* planning_gui_;
	
	CausticTexture* caustic_texture_;
#ifndef _WIN32
	// ROS related stuff.
	ros::NodeHandle* ros_node_;
	Odometry* auv_odometry1_;
	Odometry* sonar_odometry1_;
	ActionController* action_controller1_;
	
	Odometry* auv_odometry2_;
	Odometry* sonar_odometry2_;
	ActionController* action_controller2_;
	
	OntologyInterface* ontology_;
	
	OctomapBuilder* octomap_;
#endif	
	PointLight* volumetric_light_point_;
	SceneLeafLight* volumetric_light_leaf_;
	SceneLeafModel* light_volume_leaf_;
	
	PointLight* volumetric_light_point2_;
	SceneLeafLight* volumetric_light_leaf2_;
	SceneLeafModel* light_volume_leaf2_;
#ifndef _WIN32
	RRT* rrt_;
#endif	
	PlanLine* pl_;
	bool pillars_enabled_;

	GLuint fbo_id_;//, texture_id_;
	Texture* post_processing_texture_;

	FPSLabel* fps_label_;
	ActionLabel* status_label_;
	
	Sonar* sonar1_;
	Sonar* sonar2_;
	
	Font* font_;
	Theme* theme_;
};

#endif
