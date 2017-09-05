#ifndef DEMO_PANDORA_OPPORTUNITY_GENERATOR_H
#define DEMO_PANDORA_OPPORTUNITY_GENERATOR_H

#ifndef _WIN32
#include <ros/ros.h>
#endif

#include "GL/glew.h"
#include "../../core/entities/camera/DynamicCamera.h"
#include "../ApplicationInterface.h"

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
class Sonar;
class CausticTexture;
class MissionSite;
class Mission;

class OpportunityGenerator : public ApplicationInterface
{
public:
	OpportunityGenerator(SceneManager& scene_manager);

	bool init(int argc, char** argv);//, bool use_hwu_ontology = false);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	AUV& getPlayer() const { return *auv_; }
	Camera& getCamera() const { return *camera_node_; }
	Camera& getCameraNode() const { return *camera_node_; }

	void onResize(int width, int height);
private:

	Mission& createInspectionSite(MissionSite& mission_site, unsigned int nr_structures);
	Mission& createValveTurningSite(MissionSite& mission_site, unsigned int valves, Texture& valve_texture, float valve_deadline);
	Mission& createChainFollowingSite(MissionSite& mission_site);
	Material* terrain_material_,* concrete_material_;

	HeightMap* terrain_node_;

	Camera* camera_node_;

	AUV* auv_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	
	Terrain* terrain_;
	
	ShadowRenderer* shadow_renderer_;
	PlanningGUI* planning_gui_;

	CausticTexture* caustic_texture_;
#ifndef _WIN32
	// ROS related stuff.
	ros::NodeHandle* ros_node_;
	Odometry* auv_odometry_;
	Odometry* sonar_odometry_;
	OntologyInterface* ontology_;
	ActionController* action_controller_;
	OctomapBuilder* octomap_;
#endif	
	PointLight* volumetric_light_point_;
	SceneLeafLight* volumetric_light_leaf_;
	SceneLeafModel* light_volume_leaf_;
	Sonar* sonar_;
#ifndef _WIN32
	RRT* rrt_;
#endif	
//	PlanLine* pl_;

	GLuint fbo_id_;//, texture_id_;
	Texture* post_processing_texture_;
	
	std::vector<glm::vec3> structure_locations_;

	FPSLabel* fps_label_;
	int width_, height_;
};

#endif
