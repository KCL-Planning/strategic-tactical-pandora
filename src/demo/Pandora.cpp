#include "Pandora.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>

#ifndef _WIN32
#include <ros/ros.h>
#endif

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include "../core/scene/frustum/Frustum.h"
#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../shapes/FrustumShape.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"
#include "../core/light/DirectedLight.h"

#include "../core/scene/SceneLeafLight.h"
#include "../core/scene/SceneLeafModel.h"
#include "../core/scene/SceneManager.h"
#include "../core/scene/SceneNode.h"
#include "../core/scene/SkyBoxLeaf.h"
#include "../core/scene/portal/Region.h"
#include "../shapes/terrain.h"
#include "../core/scene/Material.h"
#include "../core/shaders/TerrainShader.h"
#include "../core/shaders/BasicShadowShader.h"
#include "../core/shaders/ToonShader.h"
#include "../core/shaders/WaterShader.h"
#include "../core/shaders/SkyBoxShader.h"
#include "../core/shaders/ShadowShader.h"
#include "../core/shaders/AnimatedShadowShader.h"
#include "../core/shaders/LineShader.h"
#include "../core/shaders/MergeFBOShader.h"
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "pandora/AUV.h"
#include "pandora/Propeller.h"

#ifndef _WIN32
#include "pandora/ontology/OctomapBuilder.h"
#endif

#include "../core/collision/BoxCollision.h"
#include "../core/collision/CollisionInfo.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "../core/entities/camera/FreeMovingCamera.h"

#include "../core/entities/behaviours/RotateBehaviour.h"

#include "../core/loaders/WavefrontLoader.h"

#include "../core/loaders/AssimpLoader.h"

#include "../core/texture/Texture.h"
#include "../core/texture/TargaTexture.h"
#include "../core/texture/FreeImageLoader.h"

#include "../core/loaders/PortalLevelFormatLoader.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

#include "shooter/ArmedPlayer.h"

#ifndef _WIN32
#include "pandora/gui/PlanVisualiser.h"
#include "pandora/gui/StrategicPlanVisualiser.h"
#include "pandora/gui/WaypointLabeler.h"
#include "pandora/gui/BillBoard.h"
#endif

#include "../core/loaders/AssimpLoader.h"

#include "../core/entities/behaviours/HoverBehaviour.h"
#include "../core/entities/behaviours/MoveBehaviour.h"
#include "../core/entities/HeightMap.h"
#include "pandora/ontology/ValveGoal.h"

#ifndef _WIN32
// ROS stuff.
#include "pandora/ontology/Ontology.h"
//#include "pandora/ontology/HWUOntology.h"
#include "pandora/ontology/Pose.h"
#include "pandora/ontology/InspectionPoint.h"
#include "pandora/ontology/ChainGoal.h"
#include "pandora/controllers/ActionController.h"

#include "pandora/editor/XMLLevelLoader.h"

// Test the RRT code.
#include "pandora/RRT.h"
#include "pandora/sensors/Sonar.h"
#include "pandora/sensors/SliceSonar.h"
#include "pandora/sensors/Odometry.h"
#include "pandora/controllers/FollowWaypointController.h"
//#include "pandora/controllers/ObserveController.h"
#endif

// GUI stuff.
#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "pandora/gui/PlanLine.h"
#include "pandora/gui/PlanningGUI.h"
#include "pandora/gui/StrategicPlanGUIElement.h"

#include "pandora/structures/ValvePanel.h"
#include "pandora/structures/Valve.h"
#include "pandora/structures/Chain.h"
#include "pandora/structures/Pillar.h"
#include "pandora/level/MissionSite.h"
#include "pandora/level/Mission.h"
#include "pandora/structures/PipeNetwork.h"
//#include "pandora/structures/SmallManifold.h"
#include "pandora/structures/RechargeStation.h"

// Volumetric light.
#include "volumetric/LightVolumeShape.h"
#include "volumetric/ShadowVolumeShader.h"

// Sea life.
#include "pandora/models/Shark.h"
#include "pandora/models/Seal.h"
#include "pandora/models/UnderWaterVolcano.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Label.h"
#include "../core/gui/Container.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "gui_demo/FPSLabel.h"

#include "pandora/shaders/CausticShader.h"
#include "pandora/shaders/CausticTerrainShader.h"
#include "pandora/shaders/CausticTexture.h"

#include "pandora/gui/EditorTool.h"
#include "pandora/ROSTimer.h"

Pandora::Pandora(SceneManager& scene_manager)
#ifndef _WIN32
	: scene_manager_(&scene_manager), ros_node_(NULL)
#else
	: scene_manager_(&scene_manager)
#endif
{
	srand (time(NULL));
}

void Pandora::receivePlannerStatus(const std_msgs::StringConstPtr& msg)
{
	planning_status_label_->setLabel(msg->data);
}

bool Pandora::getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision)
{
	// Transform the viewport coordinates to the normalised devise coordinates.
	float ndc_mouse_x = (2 * mouse_x) / camera_node_->getWidth() - 1.0f;
	float ndc_mouse_y = 1.0f - (2 * mouse_y) / camera_node_->getHeight();
	
	// Transform the normalised device coordinates into homogeneous clip coordinates.
	glm::vec4 hcc_ray(ndc_mouse_x, ndc_mouse_y, -1.0f, 1.0f);
	
	// Transform these into eye coordinates.
	glm::vec4 eye_ray = glm::inverse(camera_node_->getPerspectiveMatrix()) * hcc_ray;
	eye_ray.z = -1.0f;
	eye_ray.w = 0.0f;
	
	// Transform these into world coordinates.
	glm::vec3 world_coordinates = glm::vec3(glm::inverse(camera_node_->getViewMatrix()) * eye_ray);
	glm::vec3 direction = glm::normalize(world_coordinates);
	
	CollisionInfo collision_info;
	if (terrain_node_->doesCollide(*auv_, camera_node_->getGlobalLocation(), camera_node_->getGlobalLocation() + direction * 200.0f, collision_info))
	{
		for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
		{
			std::cout << "Got a collision at: (" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
			collision = *ci;
			return true;
		}
	}
	return false;
}

bool Pandora::init(int argc, char** argv)//, bool use_hwu_ontology)
{
#ifndef _WIN32
	//ros::init(argc, argv, "PlannerVisualisation");
	ros_node_ = new ros::NodeHandle();
#endif
	
	ros_timer_ = new ROSTimer(*ros_node_);
	
	std::string level = "data/pandora/levels/test_level.xml";
	for (int i = 0; i < argc; ++i)
	{
		std::string param(argv[i]);
		std::string name = param;
		std::string value;
		std::size_t splitter = param.find('=');
		
		if (splitter != std::string::npos)
		{
			name = param.substr(0, splitter);
			value = param.substr(splitter + 1);
			std::cout << name << "==" << value << std::endl;
		}
		
		if ("--level" == name)
		{
			level = value;
		}
	}
	
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2, 0.2, 0.4, 1.0f);

	glfwGetWindowSize(&width_, &height_);
	
	terrain_ = new Terrain();
	terrain_->createRandomHeightmap(65, -2.0f, 2.0f);

	terrain_node_ = new HeightMap(terrain_->getWidth(), terrain_->getWidth(), terrain_->getVertices()[1].x - terrain_->getVertices()[0].x, terrain_->getHeights(), *scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), OBSTACLE, "terrain");
	
	Texture* water_texture = TargaTexture::loadTexture("data/textures/waterbubble.tga");
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* sand_texture = TargaTexture::loadTexture("data/models/Pandora/misc/sand.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/underwater_height.tga");
	Texture* auv_texture = TargaTexture::loadTexture("data/models/Pandora/misc/auv_texture.tga");
	
	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.5f, 0.5f, 0.5f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	//terrain_material_->add2DTexture(*grass_texture);
	terrain_material_->add2DTexture(*sand_texture);

	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, CausticTerrainShader::getShader(), false, false);
	
	// The AUV.
	auv_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 3.0f, 0.0f)), *scene_manager_, *auv_texture, "auv0");
	scene_manager_->addUpdateableEntity(*auv_);
	
	//auv2_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(0.0f, 3.0f, 0.0f)), *scene_manager_, *grass_texture, "auv1");
	//scene_manager_->addUpdateableEntity(*auv2_);

	DirectedLight* sun = new DirectedLight(*scene_manager_, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 1.01, 0.15, 0.01);
	SceneNode* sun_node = new SceneNode(*scene_manager_, auv_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0, 20, 0)), -90.0f, glm::vec3(1, 0, 0)));
	SceneLeafLight* light_leaf = new SceneLeafLight(*sun_node, NULL, *sun);

	caustic_texture_ = new CausticTexture();

	CausticShader::initialiseSun(*sun, *caustic_texture_);
	CausticTerrainShader::initialiseSun(*sun, *caustic_texture_);
	
	octomap_ = new OctomapBuilder(*ros_node_, *auv_);
	ontology_ = new Ontology(*ros_node_, *octomap_);
	ontology_->addAUV(*auv_);
	
	XMLLevelLoader* level_loader = new XMLLevelLoader(*scene_manager_, *terrain_node_, *static_cast<Ontology*>(ontology_), *terrain_node_);
	level_loader->loadLevel(level);
	auv_->setTransformation(level_loader->getAUVLocation());

	MaterialLightProperty* ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(0, 0, 1, 0.1f);
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	
	rrt_ = new RRT(*ros_node_, *scene_manager_, *auv_, *terrain_node_, *octomap_, *ontology_, *material);
	ontology_->initialise(*rrt_);
	
	// Setup the GUI.
	MyGUITheme* theme = new MyGUITheme();
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");

	Font* font = new TexturedFont(*font_texture);
	
	//WaypointLabeler* wl = new WaypointLabeler(*scene_manager_, *rrt_, *terrain_node_);

	// Initialise the action controller that will receive all the messages from the planner.
	FollowWaypointController* controller_ = new FollowWaypointController(*scene_manager_, *auv_, *rrt_);
	action_controller_ = new ActionController(*scene_manager_, *ros_node_, *auv_, *terrain_node_, *controller_, *ontology_);
	
	//FollowWaypointController* controller2_ = new FollowWaypointController(*scene_manager_, *auv2_, *rrt_);
	//action_controller2_ = new ActionController(*scene_manager_, *ros_node_, *auv2_, *terrain_node_, *controller2_, *ontology_);

	// Setup the ROS odometry class.
	auv_odometry_ = new Odometry(*auv_);
	/*
	// Test under water vulcanos.
	std::vector<glm::vec3> under_water_volcano_locations;
	for (unsigned int i = 0; i < 50; ++i)
	{
		float x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 200.0f;
		float z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5f) * 200.0f;
		float uwv_height = terrain_->getHeight(x, z);
		under_water_volcano_locations.push_back(glm::vec3(x, uwv_height, z));
	}
	UnderWaterVolcano* under_Water_volcano = new UnderWaterVolcano(*scene_manager_, terrain_node_, glm::mat4(1.0f), under_water_volcano_locations);
	scene_manager_->addUpdateableEntity(*under_Water_volcano);
	*/
	// Create a node that ignores the rotation.
	SceneNode* stable_platform = new SceneNode(*scene_manager_, &auv_->getAUVNode(), glm::mat4(1.0f));
	stable_platform->ignoreRotations(true);
	
	{
		PointLight* point_light = new PointLight(*scene_manager_, 34, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, &auv_->getAUVModel(),  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);

		volumetric_light_point_ = new PointLight(*scene_manager_, 34, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f, 128, GL_NONE, GL_NONE, GL_NONE);
		volumetric_light_leaf_ = new SceneLeafLight(*light_node, NULL, *volumetric_light_point_);
		LightVolumeShape* lvs = new LightVolumeShape(*scene_manager_, *volumetric_light_point_);
		
		light_volume_leaf_ = new SceneLeafModel(*light_node, NULL, *lvs, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
		lvs->setLeafNode(*light_volume_leaf_);
		/*
		// AUV 2
		PointLight* point_light2 = new PointLight(*scene_manager_, 34, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f);
		SceneNode* light_node2 = new SceneNode(*scene_manager_, &auv2_->getAUVModel(),  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.25f)));
		SceneLeafLight* light_leaf2 = new SceneLeafLight(*light_node2, NULL, *point_light2);
		
		volumetric_light_point2_ = new PointLight(*scene_manager_, 34, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f, 128, GL_NONE, GL_NONE, GL_NONE);
		volumetric_light_leaf2_ = new SceneLeafLight(*light_node2, NULL, *volumetric_light_point2_);
		LightVolumeShape* lvs2 = new LightVolumeShape(*scene_manager_, *volumetric_light_point2_);
		
		light_volume_leaf2_ = new SceneLeafModel(*light_node2, NULL, *lvs2, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
		lvs2->setLeafNode(*light_volume_leaf2_);
		*/
	}
	
#ifndef _WIN32
	sonar_ = new Sonar(*ros_node_, *scene_manager_, auv_, *auv_, auv_->getName(), glm::mat4(1.0f), 0.1f, 60.0f, 30.0f);
	//FrustumShape* frustumShape = new FrustumShape(0.1f, 60.0f, tan((30.0f / 180.0f) * M_PI) * 0.1f, tan((30.0f / 180.0f) * M_PI) * 0.1f, tan((30.0f / 180.0f) * M_PI) * 60.0f, tan((30.0f / 180.0f) * M_PI) * 60.0f);
	//SceneLeafModel* scene_leaf_model = new SceneLeafModel(*sonar_, NULL, *frustumShape, *terrain_material_, BasicShadowShader::getShader(), true, false);
#endif

	// Add the camera system.
	//camera_node_ = new DynamicCamera(*auv_, *scene_manager_, stable_platform, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 2.0f)), 90.0f, 1024, 768, 0.1f, 300.0f);
	camera_node_ = new FreeMovingCamera(*scene_manager_, terrain_node_, glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_node_);
	
	PlanVisualiser* pv = new PlanVisualiser(*ros_node_, *auv_, *ontology_, *terrain_node_, *scene_manager_, *theme, *font, *camera_node_);	action_controller_->addListener(*pv);
	
	StrategicPlanVisualiser* spv = new StrategicPlanVisualiser(*ros_node_, *auv_, *ontology_, *terrain_node_, *scene_manager_, *theme, *font, *camera_node_);
	action_controller_->addListener(*spv);
	
	GUIManager& gui_manager = GUIManager::getInstance();
	std::vector<glm::vec2> uv_mapping;
	uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
	uv_mapping.push_back(glm::vec2(1.0f, 1.0f));
	uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
	uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
	BillBoard* bb_auv = new BillBoard(*theme, *font, *auv_, *camera_node_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
	bb_auv->setVisible(false);
	gui_manager.addFrame(*bb_auv);
	auv_->setBillBoard(*bb_auv);
	
	/*
	BillBoard* bb_auv2 = new BillBoard(*theme, *font, *auv2_, *camera_node_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
	bb_auv2->setVisible(false);
	gui_manager.addFrame(*bb_auv2);
	auv2_->setBillBoard(*bb_auv2);
	*/
	
	strategic_planning_gui_ = new StrategicPlanGUIElement(*ros_node_, *theme, *font, 0, 0, width_, 60, 1);
	gui_manager.addFrame(*strategic_planning_gui_);
	
	planning_gui_ = new PlanningGUI(*ros_node_, *theme, *font, 0, 0, width_, 60, 15);
	planning_gui_->addPlanLine(*auv_);
	action_controller_->addListener(*planning_gui_);
	gui_manager.addFrame(*planning_gui_);
	
	MaterialLightProperty* inspection_point_material_ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* inspection_point_material_diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* inspection_point_material_specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* inspection_point_material_emmisive = new MaterialLightProperty(1, 1, 1, 1.0f);
	Material* inspection_point_material = new Material(*inspection_point_material_ambient, *inspection_point_material_diffuse, *inspection_point_material_specular, *inspection_point_material_emmisive);
	inspection_point_material->add2DTexture(*grass_texture);
	
	MaterialLightProperty* view_point_material_emmisive = new MaterialLightProperty(1, 0, 1, 1.0f);
	Material* view_point_material = new Material(*inspection_point_material_ambient, *inspection_point_material_diffuse, *inspection_point_material_specular, *view_point_material_emmisive);
	view_point_material->add2DTexture(*grass_texture);
	
	// Create some shapes to denote the inspection points.
	std::vector<InspectionPoint*> inspection_points;
	if (ontology_->getInspectionPoints(inspection_points))
	{
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
		{
			const InspectionPoint* ip = *ci;
			std::cout << ip->getPose().x_ << " " << ip->getPose().y_ << " " << ip->getPose().z_ << std::endl;
				
			Sphere* sphere = new Sphere(10, 10, 0.1);
			SceneNode* sphere_node = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(ip->getPose().x_, ip->getPose().y_, ip->getPose().z_)));
			SceneLeafModel* sphere_model = new SceneLeafModel(*sphere_node, NULL, *sphere, *inspection_point_material, BasicShadowShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW);
			
			Sphere* sphere2 = new Sphere(10, 10, 0.2);
			SceneNode* sphere_node2 = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(ip->getVisiblePoint().x, ip->getVisiblePoint().y, ip->getVisiblePoint().z)));
			SceneLeafModel* sphere_model2 = new SceneLeafModel(*sphere_node2, NULL, *sphere2, *view_point_material, BasicShadowShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW);
		}
	}
//#endif
	shadow_renderer_ = new ShadowRenderer(*scene_manager_, 512, GL_BACK, GL_NONE, GL_NONE);
	
	// Create a seperate framebuffer for the post processing.
	post_processing_texture_ = new Texture(GL_TEXTURE_2D);
	
	//glGenTextures(1, &texture_id_);
	glBindTexture(GL_TEXTURE_2D, post_processing_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1024, 768, 0, GL_RGBA, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glGenFramebuffers(1, &fbo_id_);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);

	// Attach the rgb texture to it.
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, post_processing_texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	EditorTool* et = new EditorTool(*scene_manager_, *ontology_, *theme, font->clone(), 100, 100, *camera_node_, *terrain_node_);
	
	// Setup the GUI for the FPS.
	Container* fps_container = new Container(*theme, font->clone(), 10, 10, 120, 20, false);
	Label* fps_label = new Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FPSLabel(*fps_label);

	gui_manager.addFrame(*fps_container);
	
	time_container_ = new Container(*theme, font->clone(), width_ -  160, 10, 160, 20, false);
	time_label_ = new Label(*theme, 160, 20, "0d 0h 0m 0s", 12);
	time_container_->addElement(*time_label_, 0, -20);
	gui_manager.addFrame(*time_container_);
	
	planning_status_container_ = new Container(*theme, font->clone(), width_ / 2.0f - 300, 10, 200, 20, false);
	planning_status_label_ = new Label(*theme, 600, 20, "IDLE", 12);
	planning_status_container_->addElement(*planning_status_label_, 0, -20);
	gui_manager.addFrame(*planning_status_container_);
	
	plan_status_sub_ = ros_node_->subscribe("/planning_system/state", 1, &Pandora::receivePlannerStatus, this);
	
	return true;
}

bool Pandora::postInit()
{
#ifndef _WIN32
	//PipeNetwork* pipe_network = new PipeNetwork(*scene_manager_, terrain_node_, ontology_->getMissionSites(), 1.0f);
#endif
	
	glfwGetWindowSize(&width_, &height_);
	//return true;
	// Lets add some sharks :)
	//AssimpLoader* loader = new AssimpLoader();
	std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > shark_properties = AssimpLoader::LoadModel(*scene_manager_, "data/models/Pandora/sea creatures/shark.dae");
/*
	Texture* diffuse_texture = (*(*shark_properties.second)[aiTextureType_DIFFUSE])[0];
	MaterialLightProperty* ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* diffuse = new MaterialLightProperty(1.0f, 1.0f, 1.0f, 1.0f);
	MaterialLightProperty* specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* emmisive = new MaterialLightProperty(0.4f, 0.4f, 0.4f, 1.0f);

	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	material->add2DTexture(*diffuse_texture);
*/
	// Load the texture for the shark.
	Texture* shark_texture = TargaTexture::loadTexture("data/models/Pandora/sea creatures/shark.tga");
	
	MaterialLightProperty* shark_ambient = new MaterialLightProperty(0.2, 0.2, 0.2, 1.0);
	MaterialLightProperty* shark_diffuse = new MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	MaterialLightProperty* shark_specular = new MaterialLightProperty(0.01, 0.01, 0.01, 1.0);
	MaterialLightProperty* shark_emissive = new MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	
	Material* shark_material = new Material(*shark_ambient, *shark_diffuse, *shark_specular, *shark_emissive);
	shark_material->add2DTexture(*shark_texture);
	
	
	Shark* shark = new Shark(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(20.0f, 13.25f, 0.0f)), *scene_manager_);
	SceneLeafModel* chair_leaf_node = new SceneLeafModel(*shark, NULL, *shark_properties.first, *shark_material, AnimatedShadowShader::getShader(), false, false);
	chair_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);

	shark->init(*shark_material, BasicShadowShader::getShader());
	
	std::vector<glm::vec3> waypoints1;
	waypoints1.push_back(glm::vec3(10, 15, 25));
	waypoints1.push_back(glm::vec3(-40, 6, 34));
	waypoints1.push_back(glm::vec3(15, 30, -15));
	shark->setWaypoints(waypoints1);
	
	Shark* shark2 = new Shark(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(-20.0f, 45.25f, 0.0f)), *scene_manager_);
	SceneLeafModel* chair_leaf_node2 = new SceneLeafModel(*shark2, NULL, *shark_properties.first, *shark_material, AnimatedShadowShader::getShader(), false, false);
	chair_leaf_node2->setShadowType(ShadowRenderer::ANIMATED_SHADOW);
	
	shark2->init(*shark_material, BasicShadowShader::getShader());
	
	std::vector<glm::vec3> waypoints2;
	waypoints2.push_back(glm::vec3(1, 9, 9));
	waypoints2.push_back(glm::vec3(-15, 7, 45));
	waypoints2.push_back(glm::vec3(50, 30, 5));
	shark2->setWaypoints(waypoints2);
	/*
	Shark* shark3 = new Shark(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(20.0f, 7.25f, -30.0f)), *scene_manager_);
	SceneLeafModel* chair_leaf_node3 = new SceneLeafModel(*shark3, NULL, *shark_properties.first, *shark_material, AnimatedShadowShader::getShader(), false, false);
	chair_leaf_node3->setShadowType(ShadowRenderer::ANIMATED_SHADOW);
	
	std::vector<glm::vec3> waypoints3;
	waypoints3.push_back(glm::vec3(23, 15, -19));
	waypoints3.push_back(glm::vec3(55, 5, -25));
	waypoints3.push_back(glm::vec3(0, 25, 35));
	shark3->setWaypoints(waypoints3);
	*/
	scene_manager_->addUpdateableEntity(*shark);
	//scene_manager_->addUpdateableEntity(*shark2);
	//scene_manager_->addUpdateableEntity(*shark3);
	
	shark_properties.first->setAnimation(*shark_properties.first->getAnimations()[0]);
/*
	// Load the seal model1
	std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > seal_properties = AssimpLoader::LoadModel(*scene_manager_, "data/models/Pandora/sea creatures/swimforward.dae");
	
	Texture* texture = FreeImageLoader::loadTexture("data/models/Pandora/sea creatures/RawTexture/DefaultMaterial_Base_Color.png");
	//Texture* seal_diffuse_texture = (*(*seal_properties.second)[aiTextureType_DIFFUSE])[0];
	MaterialLightProperty* seal_ambient = new MaterialLightProperty(1.0f, 1.0f, 1.0f, 1.0f);
	MaterialLightProperty* seal_diffuse = new MaterialLightProperty(1.0f, 1.0f, 1.0f, 1.0f);
	MaterialLightProperty* seal_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* seal_emmisive = new MaterialLightProperty(1.0f, 1.0f, 1.0f, 1.0f);

	Material* seal_material = new Material(*seal_ambient, *seal_diffuse, *seal_specular, *seal_emmisive);
	seal_material->add2DTexture(*texture);

	Seal* seal = new Seal(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(6.0f, 0.0f, 0.0f)), *scene_manager_);
	SceneLeafModel* seal_leaf_node = new SceneLeafModel(*seal, NULL, *seal_properties.first, *seal_material, AnimatedShadowShader::getShader(), false, false);
	seal_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);
	
	scene_manager_->addUpdateableEntity(*seal);
	
	seal_properties.first->setAnimation(*seal_properties.first->getAnimations()[0]);
	seal->init(*seal_material, BasicShadowShader::getShader());
*/

/*
	Cube* test_cube = new Cube(0.2f, 0.2f, 0.2f);
	// Test the collisions against the chain.
	for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
	{
		MissionSite* ms = *ci;
		for (std::vector<Chain*>::const_iterator ci = ms->getChains().begin(); ci != ms->getChains().end(); ++ci)
		{
			Chain* chain = *ci;
			
			// Create a grid.
			for (float y = -20; y < 0; y += 0.2f)
			{
				for (float x = -10; x < 10; x += 0.2f)
				{
					if (chain->doesCollide(*chain, glm::vec3(x, 2.0f, y), glm::vec3(x, terrain_node_->getHeight(x, y), y), 0.01f))
					{
						SceneNode* sn = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(x, terrain_node_->getHeight(x, y), y)));
						SceneLeafModel* slm= new SceneLeafModel(*sn, NULL, *test_cube, *shark_material, BasicShadowShader::getShader(), false, false);
					}
				}
			}
		}
	}
*/
	return true;
}

GLuint Pandora::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	fps_label_->frameRendered();
	
	if (!auv_->isLightOn())
	{
		return 0;
	}

	// Render the scene from the camera's point of view, we use this depth texture to cull the light such that it does not shine through
	// solid objects.
	shadow_renderer_->render(*camera_node_);

	glm::vec3 camera_location = camera_node_->getLocation();
	glm::mat4 view_matrix = camera_node_->getViewMatrix();
	glm::mat4 perspective_matrix = camera_node_->getPerspectiveMatrix();
	
	std::vector<const SceneLeafLight*> active_lights;
	active_lights.push_back(volumetric_light_leaf_);

	// Render the shadow from the light's perspective.
	volumetric_light_point_->preRender(volumetric_light_leaf_->getParent()->getCompleteTransformation());
	//volumetric_light_point2_->preRender(volumetric_light_leaf2_->getParent()->getCompleteTransformation());
	
	glClearColor(0.0, 0.0, 0.0, 0.0f);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_DEPTH_CLAMP);

	// Enable additive blending.
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	ShadowVolumeShader& shader = ShadowVolumeShader::getShader();
	shader.initialise(*light_volume_leaf_, view_matrix, light_volume_leaf_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf_, camera_node_->getNearPlane(), camera_node_->getFarPlane(), shadow_renderer_->getTexture());
	light_volume_leaf_->draw(view_matrix, perspective_matrix, active_lights, NULL);
	
	/*
	active_lights.clear();
	active_lights.push_back(volumetric_light_leaf2_);
	
	shader.initialise(*light_volume_leaf2_, view_matrix, light_volume_leaf2_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf2_, camera_node_->getNearPlane(), camera_node_->getFarPlane(), shadow_renderer_->getTexture());
	light_volume_leaf2_->draw(view_matrix, perspective_matrix, active_lights, NULL);
	*/
	
	//ss_ << "end...";
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_CLAMP);

	// Merge the images of the volumetric light step with the main image.
	MergeFBOShader& merge_shader = MergeFBOShader::getShader();
	merge_shader.postProcess(color_texture, *post_processing_texture_, dt);

	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2, 0.2, 0.4, 1.0f);
	
	return merge_shader.getFrameBufferId();
}

void Pandora::tick(float dt)
{
	ros_timer_->update(dt);

	int nr_seconds = ros_timer_->getTime();
	std::stringstream ss;
	ss << "Time " << nr_seconds << "s";
	time_label_->setLabel(ss.str());
	
	caustic_texture_->update(dt);
#ifndef _WIN32
	ros::spinOnce();
	// Publish the odometry information of the UAV.
	auv_odometry_->update(dt);
	//sonar_odometry_->update(dt);
	
	// Update the actions.
	action_controller_->update(dt);
	//action_controller2_->update(dt);

	// Update ROS.
	ros::spinOnce();
	Frustum frustum(sonar_->getPerspectiveMatrix() * sonar_->getViewMatrix());
	for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
		{
			Chain* chain = *ci;
			if (chain->getFrustumChecker().isInsideFrustum(frustum))// && glm::distance(chain->getGlobalLocation(), auv_->getGlobalLocation()) < 20.0f)
			{
				if (!chain->hasBeenObserved())
				{
					std::vector<glm::vec2> uv_mapping;
					uv_mapping.push_back(glm::vec2(0.0f, 0.75f));
					uv_mapping.push_back(glm::vec2(0.25f, 0.75f));
					uv_mapping.push_back(glm::vec2(0.0f, 0.5f));
					uv_mapping.push_back(glm::vec2(0.25f, 0.5f));
					auv_->setBillBoardUVs(uv_mapping);
				}
				chain->setObserved();
			}
		}
	}
#endif
/*
	if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		int mouse_x, mouse_y;
		glfwGetMousePos(&mouse_x, &mouse_y);
		
		glm::vec3 collision;
		if (getPointOnSeaBed(mouse_x, mouse_y, collision))
		{
			std::cout << "Collision at the seabed at: (" << collision.x << ", " << collision.y << ", " << collision.z << ")" << std::endl;
		}
		
		Entity* entity = scene_manager_->pickEntity(getCamera(), mouse_x, mouse_y);
		if (entity != NULL)
		{
			std::cout << "Selected: " << entity->getName() << std::endl;
		}
	}
*/
}

void Pandora::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_node_->onResize(width, height);
	
	glBindTexture(GL_TEXTURE_2D, post_processing_texture_->getTextureId());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, NULL);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, post_processing_texture_->getTextureId(), 0);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	MergeFBOShader& merge_shader = MergeFBOShader::getShader();
	merge_shader.onResize(width, height);
	
	planning_gui_->onResize(width, height);
	strategic_planning_gui_->onResize(width, height);
	time_container_->setPosition(width - 160, 10);
	planning_status_container_->setPosition(width / 2.0f - 300, 10);
	planning_status_container_->onResize(width, height);
}