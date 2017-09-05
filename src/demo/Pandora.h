#ifndef PANDORA_DEMO_H
#define PANDORA_DEMO_H

#ifndef _WIN32
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif

#include "GL/glew.h"
#include <yaml-cpp/yaml.h>

#include "../core/entities/camera/DynamicCamera.h"
#include "ApplicationInterface.h"

class Container;
class ROSTimer;
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
class StrategicPlanGUIElement;
class ValvePanel;
class OctomapBuilder;
class RRT;
class PlanLine;
class Texture;
class FPSLabel;
class Sonar;
class CausticTexture;

class Pandora : public ApplicationInterface
{
public:
	Pandora(SceneManager& scene_manager);

	bool init(int argc, char** argv);//, bool use_hwu_ontology = false);
	bool postInit();

	GLuint postProcess(Texture& color_texture, Texture& depth_texture, float dt);

	void tick(float dt);

	AUV& getPlayer() const { return *auv_; }
	Camera& getCamera() const { return *camera_node_; }
	Camera& getCameraNode() const { return *camera_node_; }

	void onResize(int width, int height);
	
	/**
	 * Receive the status of the planner.
	 */
	void receivePlannerStatus(const std_msgs::StringConstPtr& msg);
private:
	
	bool getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision);

	Material* terrain_material_,* concrete_material_;

	HeightMap* terrain_node_;

	Camera* camera_node_;

	AUV* auv_;
	//AUV* auv2_;
	SceneManager* scene_manager_;
	Frustrum* frustrum_;
	
	Terrain* terrain_;
	
	ShadowRenderer* shadow_renderer_;
	PlanningGUI* planning_gui_;
	StrategicPlanGUIElement* strategic_planning_gui_;

	CausticTexture* caustic_texture_;
#ifndef _WIN32
	// ROS related stuff.
	ros::NodeHandle* ros_node_;
	Odometry* auv_odometry_;
	Odometry* sonar_odometry_;
	OntologyInterface* ontology_;
	ActionController* action_controller_;
	//ActionController* action_controller2_;
	OctomapBuilder* octomap_;
#endif	
	PointLight* volumetric_light_point_;
	SceneLeafLight* volumetric_light_leaf_;
	SceneLeafModel* light_volume_leaf_;
	/*
	PointLight* volumetric_light_point2_;
	SceneLeafLight* volumetric_light_leaf2_;
	SceneLeafModel* light_volume_leaf2_;
	*/
	Sonar* sonar_;
#ifndef _WIN32
	RRT* rrt_;
#endif

	GLuint fbo_id_;
	Texture* post_processing_texture_;

	FPSLabel* fps_label_;
	int width_, height_;
	
	Label* time_label_;
	Label* planning_status_label_;
	
	ROSTimer* ros_timer_;
	Container* time_container_;
	Container* planning_status_container_;
	
	ros::Subscriber plan_status_sub_;
};

#endif
