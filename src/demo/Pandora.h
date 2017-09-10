#ifndef PANDORA_DEMO_H
#define PANDORA_DEMO_H

#ifndef _WIN32
#include <ros/ros.h>
#include <std_msgs/String.h>
#endif

#include "GL/glew.h"
#include <yaml-cpp/yaml.h>

#include "dpengine/entities/camera/DynamicCamera.h"
#include "dpengine/game/GameComponent.h"

namespace DreadedPE
{
	class Container;
	class Button;
	class SceneManager;
	class Frustrum;
	class Entity;
	class Material;
	class Cube;
	class Terrain;
	class SceneNode;
	class Label;
	class Texture;
	class Camera;
	class SpotLight;
	class SceneLeafLight;
	class SceneLeafModel;
};

class HeightMap;
class FPSLabel;
class ROSTimer;
class ActionController;
class OntologyInterface;
class AnimatedModel;
class AUV;
class Odometry;
class PlanningGUI;
class StrategicPlanGUIElement;
class ValvePanel;
class OctomapBuilder;
class RRT;
class PlanLine;
class Sonar;
class CausticTexture;

class Pandora : public DreadedPE::GameComponent
{
public:
	Pandora(DreadedPE::SceneManager& scene_manager);

	bool init(int argc, char** argv);//, bool use_hwu_ontology = false);
	bool postInit();

	GLuint postProcess(DreadedPE::Texture& color_texture, DreadedPE::Texture& depth_texture, float dt);

	void tick(float dt);
	void prepareForRendering(float p);

	AUV& getPlayer() const { return *auv_; }
	DreadedPE::Camera& getCamera() const { return *camera_node_; }

	void onResize(int width, int height);
	
	/**
	 * Receive the status of the planner.
	 */
	void receivePlannerStatus(const std_msgs::StringConstPtr& msg);
private:
	
	bool getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision);

	HeightMap* terrain_node_;

	DreadedPE::Camera* camera_node_;

	AUV* auv_;
	//AUV* auv2_;
	DreadedPE::SceneManager* scene_manager_;
	DreadedPE::Frustrum* frustrum_;
	
	//DreadedPE::Terrain* terrain_;
	
	DreadedPE::ShadowRenderer* shadow_renderer_;
	PlanningGUI* planning_gui_;
	StrategicPlanGUIElement* strategic_planning_gui_;

	CausticTexture* caustic_texture_;

	// ROS related stuff.
	ros::NodeHandle* ros_node_;
	Odometry* auv_odometry_;
	Odometry* sonar_odometry_;
	OntologyInterface* ontology_;
	ActionController* action_controller_;
	//ActionController* action_controller2_;
	OctomapBuilder* octomap_;

	DreadedPE::SpotLight* volumetric_light_point_;
	DreadedPE::SceneLeafLight* volumetric_light_leaf_;
	DreadedPE::SceneLeafModel* light_volume_leaf_;
	/*
	SpotLight* volumetric_light_point2_;
	SceneLeafLight* volumetric_light_leaf2_;
	SceneLeafModel* light_volume_leaf2_;
	*/
	Sonar* sonar_;

	RRT* rrt_;

	GLuint fbo_id_;
	DreadedPE::Texture* post_processing_texture_;

	FPSLabel* fps_label_;
	int width_, height_;
	
	DreadedPE::Label* time_label_;
	DreadedPE::Label* planning_status_label_;
	
	ROSTimer* ros_timer_;
	DreadedPE::Container* time_container_;
	DreadedPE::Container* planning_status_container_;
	
	ros::Subscriber plan_status_sub_;
	
	bool is_planning_;
	float last_update_p_;
};

#endif

