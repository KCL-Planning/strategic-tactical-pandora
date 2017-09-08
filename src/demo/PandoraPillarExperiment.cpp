#include "PandoraPillarExperiment.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>

#ifndef _WIN32
#include <ros/ros.h>
#endif

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"
#include "../core/light/DirectedLight.h"

#include "../core/scene/frustum/Frustum.h"

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

#include "../core/collision/ConvexPolygon.h"
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

#include "../core/loaders/PortalLevelFormatLoader.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

#include "shooter/ArmedPlayer.h"

#ifndef _WIN32
#include "pandora/gui/PlanVisualiser.h"
#include "pandora/gui/WaypointLabeler.h"
#include "pandora/gui/BillBoard.h"
#endif

#include "../core/loaders/AssimpLoader.h"

#include "../core/entities/behaviours/HoverBehaviour.h"
#include "../core/entities/behaviours/MoveBehaviour.h"
#include "../core/entities/HeightMap.h"
#include "pandora/ontology/ValveGoal.h"

#include "pandora/ontology/InspectionPoint.h"

#ifndef _WIN32
// ROS stuff.
#include "pandora/ontology/Ontology.h"
//#include "pandora/ontology/HWUOntology.h"
#include "pandora/ontology/Pose.h"
#include "pandora/controllers/ActionController.h"

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

#include "pandora/structures/ValvePanel.h"
#include "pandora/structures/Valve.h"
#include "pandora/structures/Pillar.h"

// Volumetric light.
#include "volumetric/LightVolumeShape.h"
#include "volumetric/ShadowVolumeShader.h"

// Sea life.
#include "pandora/models/Shark.h"
#include "pandora/models/Seal.h"
#include "pandora/models/UnderWaterVolcano.h"
#include "pandora/level/MissionSite.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Label.h"
#include "../core/gui/Container.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "gui_demo/FPSLabel.h"

#include "pandora/shaders/CausticShader.h"
#include "pandora/shaders/CausticTerrainShader.h"
#include "pandora/shaders/CausticTexture.h"

#include "pandora/gui/ActionLabel.h"

PandoraPillarExperiment::PandoraPillarExperiment(SceneManager& scene_manager)
#ifndef _WIN32
	: scene_manager_(&scene_manager), ros_node_(NULL)
#else
	: scene_manager_(&scene_manager)
#endif
{
	srand (time(NULL));
}

bool PandoraPillarExperiment::init(int argc, char** argv)//, bool use_hwu_ontology)
{
#ifndef _WIN32
	ros::init(argc, argv, "PlannerVisualisation");
	ros_node_ = new ros::NodeHandle();
#endif
	pillars_enabled_ = true;
	unsigned int nr_pillars = 3;
	unsigned int seed = time(NULL);
	float min_distance = 10.0f;
	float area_size = 40.0f;
	
	for (unsigned int i = 1; i < argc; ++i)
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
		if (param == "--no-pillars")
		{
			pillars_enabled_ = false;
			std::cout << "Pillars are disabled." << std::endl;
		}
		else if (name == "--seed")
		{
			seed = ::atoi(value.c_str());
			std::cout << "Random generator initalised with seed: " << value << std::endl;
		}
		else if (name == "--npillars")
		{
			nr_pillars = ::atoi(value.c_str());
			std::cout << "Number of pillars is " << nr_pillars << "." << std::endl;
		}
		else if (name == "--min-distance")
		{
			min_distance = ::atof(value.c_str());
			std::cout << "The minimal distance between pillars and AUVs is " << min_distance << "." << std::endl;
		}
		else if (name == "--area-size")
		{
			area_size = ::atof(value.c_str());
			std::cout << "The size of the area is now " << area_size << "." << std::endl;
		}
		else
		{
			if (name == "--help" || name == "-h")
			{
				std::cout << "Welcome weary traveler, stay a while and listen." << std::endl;
			}
			else
			{
				std::cout << "Unknown option: " << param << "." << std::endl;
			}
			std::cout << "Usage: ./bin/visualiser " << std::endl;
			std::cout << "\t--no-pillars           default=false      // To disconnect the inspection points from the pillars. Pillars can still be detected." << std::endl;
			std::cout << "\t--seed={int}           default=time(null) // This sets the seed for the random generator. I give you my pledge that pillars will be set in the same location given the same seed." << std::endl;
			std::cout << "\t--npillars={int}       default=3          // Sets the number of pillars that need to be generated." << std::endl;
			std::cout << "\t--min-distance={float} default=10.0       // Sets the minimal distance between objects." << std::endl;
			std::cout << "\t--area-size={float}    default=40.0       // Sets the dimensions of the area of operation." << std::endl;
			exit(1);
		}
	}

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2, 0.2, 0.4, 1.0f);

	terrain_ = new Terrain();
	terrain_->createRandomHeightmap(65, -2.0f, 2.0f);

	terrain_node_ = new HeightMap(terrain_->getWidth(), terrain_->getWidth(), terrain_->getVertices()[1].x - terrain_->getVertices()[0].x, terrain_->getHeights(), *scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), OBSTACLE, "terrain");
	
	MissionSite* mission_site = new MissionSite(*scene_manager_, terrain_node_, glm::mat4(1.0f));
	
	Texture* water_texture = TargaTexture::loadTexture("data/textures/waterbubble.tga");
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/underwater_height.tga");
	
	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.5f, 0.5f, 0.5f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);
	
	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, CausticTerrainShader::getShader(), false, false);
	
	// The AUV.
	auv1_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(4.2f, 3.0f, -3.0f)), *scene_manager_, *grass_texture, "auv0");
	scene_manager_->addUpdateableEntity(*auv1_);
	
	auv2_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(-12.2f, 3.0f, 19.0f)), *scene_manager_, *grass_texture, "auv1");
	scene_manager_->addUpdateableEntity(*auv2_);
	
	Texture* pillar_texture = TargaTexture::loadTexture("data/models/Pandora/misc/damaged_beacon.tga");
	
	
	srand (seed);
	for (unsigned int i = 0; i < nr_pillars; ++i)
	{
		bool valid_pillar_location = false;
		glm::vec3 pillar_location;
		while (!valid_pillar_location)
		{
			pillar_location.x = ((float)rand() / (float)RAND_MAX) * area_size - area_size / 2.0f;
			pillar_location.y = -1.0f;
			pillar_location.z = ((float)rand() / (float)RAND_MAX) * area_size - area_size / 2.0f;
			
			// Check that this location is not too close to other pillars and not too close to the auvs.
			valid_pillar_location = true;
			for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
			{
				Pillar* existing_pillar = *ci;
				if (glm::distance(existing_pillar->getLocalLocation(), pillar_location) < min_distance)
				{
					valid_pillar_location = false;
					break;
				}
			}
			/*
			if (glm::distance(pillar_location, auv1_->getLocalLocation()) < min_distance ||
			    glm::distance(pillar_location, auv2_->getLocalLocation()) < min_distance)
			{
				valid_pillar_location = false;
			}
			*/
		}
		//Pillar* pillar = new Pillar(i, *mission_site, *scene_manager_, mission_site, glm::translate(glm::mat4(1.0f), pillar_location), "data/models/Pandora/Pillar.plf", *pillar_texture);
		std::stringstream pillar_name_ss;
		pillar_name_ss << "Pillar" << i;
		Pillar* pillar = new Pillar(pillar_name_ss.str(), *mission_site, *scene_manager_, mission_site, glm::translate(glm::mat4(1.0f), pillar_location), "data/models/Pandora/misc/damaged_beacon.plf", *pillar_texture);
		mission_site->addPillar(*pillar);
		
		Sphere* sphere = new Sphere(10, 10, 0.1);
		SceneNode* sphere_node = new SceneNode(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), pillar_location));
		SceneLeafModel* sphere_model = new SceneLeafModel(*sphere_node, NULL, *sphere, *terrain_material_, BasicShadowShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW);

	}

#ifndef _WIN32
	octomap_ = new OctomapBuilder(*ros_node_, *auv1_, *auv2_);
	ontology_ = new Ontology(*ros_node_, *octomap_);
	ontology_->addAUV(*auv1_);
	ontology_->addAUV(*auv2_);

#endif
	
	for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
	{
		InspectionPoint* ip = new InspectionPoint(Pose((*ci)->getLocalLocation().x, 5.0f, (*ci)->getLocalLocation().z - 1.0f, 0, 0), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip);
		
		InspectionPoint* ip2 = new InspectionPoint(Pose((*ci)->getLocalLocation().x - 1.0f, 5.0f, (*ci)->getLocalLocation().z, 0, 90), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip2);

		InspectionPoint* ip3 = new InspectionPoint(Pose((*ci)->getLocalLocation().x, 5.0f, (*ci)->getLocalLocation().z + 1.0f, 0, 180), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip3);

		InspectionPoint* ip4 = new InspectionPoint(Pose((*ci)->getLocalLocation().x + 1.0f, 5.0f, (*ci)->getLocalLocation().z, 0, 270), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip4);
		
		InspectionPoint* ip5 = new InspectionPoint(Pose((*ci)->getLocalLocation().x, 10.0f, (*ci)->getLocalLocation().z - 1.0f, 0, 0), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip5);

		InspectionPoint* ip6 = new InspectionPoint(Pose((*ci)->getLocalLocation().x - 1.0f, 10.0f, (*ci)->getLocalLocation().z, 0, 90), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip6);

		InspectionPoint* ip7 = new InspectionPoint(Pose((*ci)->getLocalLocation().x, 10.0f, (*ci)->getLocalLocation().z + 1.0f, 0, 180), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip7);

		InspectionPoint* ip8 = new InspectionPoint(Pose((*ci)->getLocalLocation().x + 1.0f, 10.0f, (*ci)->getLocalLocation().z, 0, 270), pillars_enabled_ ? *ci : NULL);
		mission_site->addInspectionPoint(*ip8);
		
		if (pillars_enabled_)
		{
			(*ci)->addInspectionPoint(*ip);
			(*ci)->addInspectionPoint(*ip2);
			(*ci)->addInspectionPoint(*ip3);
			(*ci)->addInspectionPoint(*ip4);
			(*ci)->addInspectionPoint(*ip5);
			(*ci)->addInspectionPoint(*ip6);
			(*ci)->addInspectionPoint(*ip7);
			(*ci)->addInspectionPoint(*ip8);
		}
	}
#ifndef _WIN32
	//ontology_->addValvePanel(*value_panel_);
	//ontology_->addValveGoal(*valve, vg);
	
	// Make sure to call this AFTER all the properties have been set for each mission site!
	ontology_->addMissionSite(*mission_site);

	MaterialLightProperty* ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(0, 0, 1, 0.1f);
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	
	std::vector<Entity*> entities;
	entities.push_back(auv1_);
	entities.push_back(auv2_);
	
	rrt_ = new RRT(*ros_node_, *scene_manager_, entities, *terrain_node_, *octomap_, *ontology_, *material);
	
	ontology_->initialise(*rrt_);
	
	//WaypointLabeler* wl = new WaypointLabeler(*scene_manager_, *rrt_, *terrain_node_);

	// Setup the GUI.
	theme_ = new MyGUITheme();

	//planning_gui_ = new PlanningGUI(*theme, *auv_, *rrt_);
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");

	font_ = new TexturedFont(*font_texture);
	
	// Setup the GUI for the FPS.
	GUIManager& gui_manager = GUIManager::getInstance();
	
	Container* fps_container = new Container(*theme_, font_->clone(), 10, 10, 120, 20, false);
	Label* fps_label = new Label(*theme_, 120, 15, "", 12);
	fps_container->addElement(*fps_label, 0, -10);
	fps_label_ = new FPSLabel(*fps_label);

	planning_msgs::ActionDispatch action;
	Container* status_container = new Container(*theme_, font_->clone(), 300, 10, 400, 40, false);
	status_label_ = new ActionLabel(*theme_, font_->clone(), glm::vec4(1.0f, 0.0f, 0.0f, 0.5f), 400, 30, "All systems go", action);
	status_container->addElement(*status_label_, 0, -10);

	gui_manager.addFrame(*fps_container);
//	gui_manager.addFrame(*status_container);
	
	// Initialise the action controller that will receive all the messages from the planner.
	FollowWaypointController* controller1 = new FollowWaypointController(*scene_manager_, *auv1_, *rrt_);
	action_controller1_ = new ActionController(*scene_manager_, *ros_node_, *auv1_, *terrain_node_, *controller1, *ontology_, status_label_);
	
	FollowWaypointController* controller2 = new FollowWaypointController(*scene_manager_, *auv2_, *rrt_);
	action_controller2_ = new ActionController(*scene_manager_, *ros_node_, *auv2_, *terrain_node_, *controller2, *ontology_, status_label_);

	// Setup the ROS odometry class.
	auv_odometry1_ = new Odometry(*auv1_);
	auv_odometry2_ = new Odometry(*auv2_);
#endif
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
	stable_platform1_ = new SceneNode(*scene_manager_, &auv1_->getAUVNode(), glm::mat4(1.0f));
	stable_platform1_->ignoreRotations(true);

	stable_platform2_ = new SceneNode(*scene_manager_, &auv2_->getAUVNode(), glm::mat4(1.0f));
	stable_platform2_->ignoreRotations(true);
	
	// Add some lighting.
	{
		// AUV1
		PointLight* point_light = new PointLight(*scene_manager_, 34, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, &auv1_->getAUVModel(),  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);

		volumetric_light_point_ = new PointLight(*scene_manager_, 34, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f, 128, GL_NONE, GL_NONE, GL_NONE);
		volumetric_light_leaf_ = new SceneLeafLight(*light_node, NULL, *volumetric_light_point_);
		LightVolumeShape* lvs = new LightVolumeShape(*scene_manager_, *volumetric_light_point_);
		
		light_volume_leaf_ = new SceneLeafModel(*light_node, NULL, *lvs, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
		lvs->setLeafNode(*light_volume_leaf_);
	}
	{
		// AUV2
		PointLight* point_light = new PointLight(*scene_manager_, 34, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, &auv2_->getAUVModel(),  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);

		volumetric_light_point2_ = new PointLight(*scene_manager_, 34, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f, 128, GL_NONE, GL_NONE, GL_NONE);
		volumetric_light_leaf2_ = new SceneLeafLight(*light_node, NULL, *volumetric_light_point2_);
		LightVolumeShape* lvs = new LightVolumeShape(*scene_manager_, *volumetric_light_point2_);
		
		light_volume_leaf2_ = new SceneLeafModel(*light_node, NULL, *lvs, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
		lvs->setLeafNode(*light_volume_leaf2_);
	}
#ifndef _WIN32
	sonar1_ = new Sonar(*ros_node_, *scene_manager_, &auv1_->getAUVModel(), *auv1_, auv1_->getName(), glm::translate(glm::mat4(1.0f), glm::vec3(0.00f, 0.0f, -2.25f)), 0.1f, 60.0f, 30.0f);
	sonar2_ = new Sonar(*ros_node_, *scene_manager_, &auv2_->getAUVModel(), *auv2_, auv2_->getName(), glm::translate(glm::mat4(1.0f), glm::vec3(0.00f, 0.0f, -2.25f)), 0.1f, 60.0f, 30.0f);
#endif
	//SliceSonar* slice_sonar = new SliceSonar(*ros_node_, *scene_manager_, 9, &auv_->getAUVNode(), glm::translate(glm::mat4(1.0f), glm::vec3(0.00f, 0.0f, -2.25f)), 0.1f, 60.0f, 30.0f);
		
	//sonar_odometry_ = new Odometry(*slice_sonar);
	
	// Add the camera system.
	camera_node_ = new DynamicCamera(*auv1_, *scene_manager_, stable_platform1_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 2.0f)), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_node_);
	
	std::vector<glm::vec2> uv_mapping;
	uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
	uv_mapping.push_back(glm::vec2(1.0f, 1.0f));
	uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
	uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
	BillBoard* bb_auv1 = new BillBoard(*theme_, *font_, *auv1_, *camera_node_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
	bb_auv1->setVisible(false);
	gui_manager.addFrame(*bb_auv1);
	auv1_->setBillBoard(*bb_auv1);
	
	BillBoard* bb_auv2 = new BillBoard(*theme_, *font_, *auv2_, *camera_node_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
	bb_auv2->setVisible(false);
	gui_manager.addFrame(*bb_auv2);
	auv2_->setBillBoard(*bb_auv2);
	
	
	DirectedLight* sun = new DirectedLight(*scene_manager_, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 1.01, 0.15, 0.01);
	SceneNode* sun_node = new SceneNode(*scene_manager_, terrain_node_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0, 20, 0)), -90.0f, glm::vec3(1, 0, 0)));
	SceneLeafLight* light_leaf = new SceneLeafLight(*sun_node, NULL, *sun);

	caustic_texture_ = new CausticTexture();

	CausticShader::initialiseSun(*sun, *caustic_texture_);
	CausticTerrainShader::initialiseSun(*sun, *caustic_texture_);
	
	PlanVisualiser* pv1 = new PlanVisualiser(*ros_node_, *auv1_, *ontology_, *terrain_node_, *scene_manager_, *theme_, *font_, *camera_node_);
	action_controller1_->addListener(*pv1);
	action_controller2_->addListener(*pv1);
	PlanVisualiser* pv2 = new PlanVisualiser(*ros_node_, *auv2_, *ontology_, *terrain_node_, *scene_manager_, *theme_, *font_, *camera_node_);
	action_controller1_->addListener(*pv2);
	action_controller2_->addListener(*pv2);
	
#ifdef _WIN32
	//pl_ = new PlanLine(*theme, *font, 200, 30, 650, 150);
#else
	
	pl_ = new PlanLine(*ros_node_, *theme_, *font_, 200, 30, 200, 650);
	action_controller1_->addListener(*pl_);
	action_controller2_->addListener(*pl_);
	
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
			SceneLeafModel* sphere_model = new SceneLeafModel(*sphere_node, NULL, *sphere, *terrain_material_, BasicShadowShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW);
		}
	}
	
#endif
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
	
	// Test! :)
	//BillBoard* bb_auv1 = new BillBoard(*theme_, font_->clone(), *auv1_, *camera_node_, glm::vec3(0, 1, 0), 50, 30, theme_->getMaximiseTexture());
	//BillBoard* bb_auv2 = new BillBoard(*theme_, font_->clone(), *auv2_, *camera_node_, glm::vec3(0, 1, 0), 50, 30, theme_->getMaximiseTexture());
	//gui_manager.addFrame(*bb_auv1);
	//gui_manager.addFrame(*bb_auv2);
	
	return true;
}

bool PandoraPillarExperiment::postInit()
{
	return true;
}

GLuint PandoraPillarExperiment::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	GLuint merged_frame_buffer = 0;
	fps_label_->frameRendered();
	if (auv1_->isLightOn() || auv2_->isLightOn())
	{
		// Render the scene from the camera's point of view, we use this depth texture to cull the light such that it does not shine through
		// solid objects.
		shadow_renderer_->render(*camera_node_);

		glm::vec3 camera_location = camera_node_->getLocation();
		glm::mat4 view_matrix = camera_node_->getViewMatrix();
		glm::mat4 perspective_matrix = camera_node_->getPerspectiveMatrix();
		
		std::vector<const SceneLeafLight*> active_lights;
		if (auv1_->isLightOn())
		{
			active_lights.push_back(volumetric_light_leaf_);
			volumetric_light_point_->preRender(volumetric_light_leaf_->getParent()->getCompleteTransformation());
		}
		if (auv2_->isLightOn())
		{
			volumetric_light_point2_->preRender(volumetric_light_leaf2_->getParent()->getCompleteTransformation());
		}
		
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

		if (auv1_->isLightOn())
		{
			// Render the light volume in this frame buffer.
			shader.initialise(*light_volume_leaf_, view_matrix, light_volume_leaf_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf_, camera_node_->getNearPlane(), camera_node_->getFarPlane(), shadow_renderer_->getTexture());
			light_volume_leaf_->draw(view_matrix, perspective_matrix, active_lights, NULL);
		}
		
		if (auv2_->isLightOn())
		{
			// For the second light.
			active_lights.clear();
			active_lights.push_back(volumetric_light_leaf2_);

			// Render the shadow from the light's perspective.
			

			// Render the light volume in this frame buffer.
			shader.initialise(*light_volume_leaf2_, view_matrix, light_volume_leaf2_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf2_, camera_node_->getNearPlane(), camera_node_->getFarPlane(), shadow_renderer_->getTexture());
			light_volume_leaf2_->draw(view_matrix, perspective_matrix, active_lights, NULL);
		}
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
		merged_frame_buffer = merge_shader.getFrameBufferId() ;
	}
	
	// Draw any additional GUI elements.
	
	
	
	return merged_frame_buffer;
}

void PandoraPillarExperiment::tick(float dt)
{
	caustic_texture_->update(dt);
	ros::spinOnce();
	// Publish the odometry information of the UAV.
	auv_odometry1_->update(dt);
	auv_odometry2_->update(dt);
	
	// Update the actions.
	action_controller1_->update(dt);
	action_controller2_->update(dt);
	
	// Update ROS.
	ros::spinOnce();
	
	Frustum frustum1(sonar1_->getPerspectiveMatrix() * sonar1_->getViewMatrix());
	Frustum frustum2(sonar2_->getPerspectiveMatrix() * sonar2_->getViewMatrix());
	
	if (pillars_enabled_)
	{
		//Frustum frustum(camera_node_->getPerspectiveMatrix() * camera_node_->getViewMatrix());
		std::stringstream ss;
		ss << "Visible pillars ";
		for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
			{
				Pillar* pillar = *ci;

				if (pillar->getFrustumChecker().isInsideFrustum(frustum1) && !pillar->hasBeenObserved())
				{
					std::vector<glm::vec2> uv_mapping;
					uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
					uv_mapping.push_back(glm::vec2(1.0f, 1.0f));
					uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
					uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
					auv1_->setBillBoardUVs(uv_mapping);
				}
				else if (pillar->getFrustumChecker().isInsideFrustum(frustum2) && !pillar->hasBeenObserved())
				{
					//SceneNode* dummy_node = new SceneNode(*scene_manager_, NULL, glm::translate(glm::mat4(1.0f), auv2_->getGlobalLocation()));
					std::vector<glm::vec2> uv_mapping;
					uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
					uv_mapping.push_back(glm::vec2(1.0f, 1.0f));
					uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
					uv_mapping.push_back(glm::vec2(1.0f, 0.75f));
					//BillBoard* bb = new BillBoard(*theme_, *font_, *dummy_node, *camera_node_, glm::vec3(0, 1, 0), 50, 50, uv_mapping);
					//GUIManager& gui_manager = GUIManager::getInstance();
					//gui_manager.addFrame(*bb);
					//auv2_->setBillBoard(*bb);
					auv2_->setBillBoardUVs(uv_mapping);
				}
				
				if (pillar->getFrustumChecker().isInsideFrustum(frustum1) ||
				pillar->getFrustumChecker().isInsideFrustum(frustum2))
				{
					pillar->setObserved();
					ss << pillar->getName() << " ";
				}
			}
		}
		//status_label_->setLabel(ss.str());
	}
	else
	{
		//status_label_->setLabel("Pillar detection disabled");
	}

	if (glfwGetKey('1') == GLFW_PRESS)
	{
		camera_node_->setTargetToFollow(*auv1_, stable_platform1_);
	}
	if (glfwGetKey('2') == GLFW_PRESS)
	{
		camera_node_->setTargetToFollow(*auv2_, stable_platform2_);
	}
	
	if (glfwGetKey('Z') == GLFW_PRESS)
	{
		std::cout << "Create an RRT" << std::endl;
		std::vector<glm::vec3> begin;
		begin.push_back(auv1_->getGlobalLocation());
		//begin.push_back(auv2_->getGlobalLocation());
		
		std::vector<bool> connectivity(2, false);
		
		std::set<Waypoint*> waypoints;
		/*
		for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
			{
				Pillar* pillar = *ci;
				for (std::vector<InspectionPoint*>::const_iterator ci = pillar->getInspectionPoint().begin(); ci != pillar->getInspectionPoint().end(); ++ci)
				{
					InspectionPoint* inspection_point = *ci;
					std::stringstream ss;
					ss << "wp_inspectionpoint" << inspection_point->getId();
					Waypoint* waypoint = new Waypoint(ss.str(), inspection_point->getVisiblePoint(), connectivity);
					waypoints.insert(waypoint);
				}
			}
		}
		*/
		std::vector<bool> resultsz(2, false);
		waypoints.insert(new Waypoint("test", auv1_->getLocalLocation() + glm::vec3(0.0f, 0.0f, -8.0f), resultsz));
		rrt_->createRRT(begin, waypoints, -100, 100, -20, 20, -100, 100);
		for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
		{
			std::cout << "p: (" << (*ci)->position_.x << ", " << (*ci)->position_.y << ", " << (*ci)->position_.z << ")" << std::endl;
		}
	}
	if (glfwGetKey('Q') == GLFW_PRESS)
	{
		rrt_->setVisible(false);
	}
	if (glfwGetKey('W') == GLFW_PRESS)
	{
		rrt_->setVisible(true);
	}
}

void PandoraPillarExperiment::onResize(int width, int height)
{
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
}
