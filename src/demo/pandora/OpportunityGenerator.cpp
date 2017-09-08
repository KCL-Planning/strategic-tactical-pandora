#include "OpportunityGenerator.h"

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

#include "dpengine/scene/frustum/Frustum.h"
#include "dpengine/loaders/targa.h"
#include "dpengine/entities/camera/Camera.h"
#include "dpengine/shapes/terrain.h"
#include "dpengine/shapes/Water.h"
#include "dpengine/shapes/Tree.h"
#include "dpengine/shapes/sphere.h"
#include "dpengine/shapes/Piramid.h"
#include "dpengine/shapes/Cube.h"
#include "dpengine/shapes/SkyBox.h"
#include "dpengine/shapes/FrustumShape.h"
#include "dpengine/light/PointLight.h"
#include "dpengine/light/Light.h"
#include "dpengine/light/DirectedLight.h"

#include "dpengine/scene/SceneLeafLight.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SkyBoxLeaf.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/shapes/terrain.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/TerrainShader.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/shaders/ToonShader.h"
#include "dpengine/shaders/WaterShader.h"
#include "dpengine/shaders/SkyBoxShader.h"
#include "dpengine/shaders/ShadowShader.h"
#include "dpengine/shaders/AnimatedShadowShader.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/shaders/MergeFBOShader.h"
#include "dpengine/animation/LinearAnimation.h"
#include "dpengine/animation/BouncingBox.h"
#include "dpengine/entities/WavingWater.h"
#include "AUV.h"
#include "Propeller.h"

#ifndef _WIN32
#include "ontology/OctomapBuilder.h"
#endif

#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/collision/CollisionInfo.h"
#include "dpengine/entities/Lever.h"
#include "dpengine/entities/Bridge.h"
#include "dpengine/scene/frustum/SphereCheck.h"
#include "dpengine/entities/camera/DynamicCamera.h"
#include "dpengine/entities/camera/FreeMovingCamera.h"

#include "dpengine/entities/behaviours/RotateBehaviour.h"

#include "dpengine/loaders/WavefrontLoader.h"

#include "dpengine/loaders/AssimpLoader.h"

#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"
#include "dpengine/texture/FreeImageLoader.h"

#include "dpengine/loaders/PortalLevelFormatLoader.h"

#include "dpengine/models/AnimatedModel.h"

#ifndef _WIN32
#include "gui/PlanVisualiser.h"
#include "gui/StrategicPlanVisualiser.h"
#include "gui/WaypointLabeler.h"
#include "gui/BillBoard.h"
#endif

#include "dpengine/loaders/AssimpLoader.h"

#include "dpengine/entities/behaviours/HoverBehaviour.h"
#include "dpengine/entities/behaviours/MoveBehaviour.h"
#include "dpengine/entities/HeightMap.h"
#include "ontology/ValveGoal.h"

#ifndef _WIN32
// ROS stuff.
#include "ontology/Ontology.h"
//#include "ontology/HWUOntology.h"
#include "ontology/Pose.h"
#include "ontology/InspectionPoint.h"
#include "ontology/ChainGoal.h"
#include "controllers/ActionController.h"

// Test the RRT code.
#include "RRT.h"
#include "sensors/Sonar.h"
#include "sensors/SliceSonar.h"
#include "sensors/Odometry.h"
#include "controllers/FollowWaypointController.h"
//#include "controllers/ObserveController.h"
#endif

// GUI stuff.
#include "dpengine/gui/themes/MyGUITheme.h"
#include "dpengine/gui/fonts/TexturedFont.h"
#include "gui/PlanLine.h"
#include "gui/PlanningGUI.h"

#include "structures/ValvePanel.h"
#include "structures/Valve.h"
#include "structures/Chain.h"
#include "structures/Pillar.h"
#include "level/MissionSite.h"
#include "level/Mission.h"
#include "structures/PipeNetwork.h"
#include "structures/SmallManifold.h"

// Volumetric light.
#include "../volumetric/LightVolumeShape.h"
#include "../volumetric/ShadowVolumeShader.h"

// Sea life.
#include "models/Shark.h"
#include "models/Seal.h"
#include "models/UnderWaterVolcano.h"

#include "dpengine/gui/themes/MyGUITheme.h"
#include "dpengine/gui/Label.h"
#include "dpengine/gui/Container.h"
#include "dpengine/gui/GUIManager.h"
#include "dpengine/gui/fonts/TexturedFont.h"
#include "../gui_demo/FPSLabel.h"

#include "shaders/CausticShader.h"
#include "shaders/CausticTerrainShader.h"
#include "shaders/CausticTexture.h"

OpportunityGenerator::OpportunityGenerator(DreadedPE::SceneManager& scene_manager)
#ifndef _WIN32
	: scene_manager_(&scene_manager), ros_node_(NULL)
#else
	: scene_manager_(&scene_manager)
#endif
{
	srand (time(NULL));
}

bool OpportunityGenerator::init(int argc, char** argv)//, bool use_hwu_ontology)
{
#ifndef _WIN32
	ros::init(argc, argv, "PlannerVisualisation");
	ros_node_ = new ros::NodeHandle();
#endif
	
	float valve_turning_deadline_1 = 0;
	float valve_turning_deadline_2 = 0;
	unsigned int nr_inspection_points_1 = 0;
	unsigned int nr_inspection_points_2 = 0;
	unsigned int nr_valves1 = 0;
	unsigned int nr_valves2 = 0;
	unsigned int nr_chains = 0;
	
	int seed = 0;
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
		if (name == "--valve-deadline1")
		{
			valve_turning_deadline_1 = ::atof(value.c_str());
		}
		else if (name == "--valve-deadline2")
		{
			valve_turning_deadline_2 = ::atof(value.c_str());
		}
		else if (name == "--nr-inspection-points1")
		{
			nr_inspection_points_1 = ::atoi(value.c_str());
			std::cout << "The number of structures for inspection site #1 = " << nr_inspection_points_1 << std::endl;
		}
		else if (name == "--nr-inspection-points2")
		{
			nr_inspection_points_2 = ::atoi(value.c_str());
			std::cout << "The number of structures for inspection site #2 = " << nr_inspection_points_2 << std::endl;
		}
		else if (name == "--seed")
		{
			seed = ::atoi(value.c_str());
			std::cout << "Random generator initalised with seed: " << value << std::endl;
		}
		else if (name == "--nr-valves1")
		{
			nr_valves1 = ::atoi(value.c_str());
			std::cout << "Number of valves for valve site #1 = " << nr_valves1 << std::endl;
		}
		else if (name == "--nr-valves2")
		{
			nr_valves2 = ::atoi(value.c_str());
			std::cout << "Number of valves for valve site #2 = " << nr_valves2 << std::endl;
		}
		else if (name == "--nr-chains")
		{
			nr_chains = ::atoi(value.c_str());
 			std::cout << "Number of chains are " << nr_chains << std::endl;
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
			std::cout << "\t--valve-deadline1       default=500        // Deadline for the first valve." << std::endl;
			std::cout << "\t--valve-deadline2       default=500        // Deadline for the second valve." << std::endl;
			std::cout << "\t--nr-valves1            default=0          // The number of valves for the first valve mission." << std::endl;
			std::cout << "\t--nr-valves2            default=0          // The number of valves for the second valve mission." << std::endl;
			std::cout << "\t--nr-inspection-points1 default=0          // The number of inspection points for the first inspection mission." << std::endl;
			std::cout << "\t--nr-inspection-points2 default=0          // The number of inspection points for the second inspection mission." << std::endl;
			std::cout << "\t--nr-chain              default=0          // The number of chains to be inspected." << std::endl;
			std::cout << "\t--seed={int}            default=time(null) // This sets the seed for the random generator. I give you my pledge that pillars will be set in the same location given the same seed." << std::endl;
			exit(1);
		}
	}
	
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2, 0.2, 0.4, 1.0f);

	glfwGetWindowSize(&width_, &height_);
	
	terrain_ = new DreadedPE::Terrain();
	terrain_->createRandomHeightmap(65, -2.0f, 2.0f);

	terrain_node_ = new DreadedPE::HeightMap(terrain_->getWidth(), terrain_->getWidth(), terrain_->getVertices()[1].x - terrain_->getVertices()[0].x, terrain_->getHeights(), *scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0f), OBSTACLE, "terrain");
	
	DreadedPE::Texture* water_texture = DreadedPE::TargaTexture::loadTexture("data/textures/waterbubble.tga");
	DreadedPE::Texture* grass_texture = DreadedPE::TargaTexture::loadTexture("data/textures/grass.tga");
	DreadedPE::Texture* height_texture = DreadedPE::TargaTexture::loadTexture("data/textures/underwater_height.tga");
	
	// Initialise a terrain to render.
	DreadedPE::MaterialLightProperty* terrain_ambient = new DreadedPE::MaterialLightProperty(0.7f, 0.7f, 0.7f, 1.0f);
	DreadedPE::MaterialLightProperty* terrain_diffuse = new DreadedPE::MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	DreadedPE::MaterialLightProperty* terrain_specular = new DreadedPE::MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	DreadedPE::MaterialLightProperty* terrain_emmisive = new DreadedPE::MaterialLightProperty(0.5f, 0.5f, 0.5f, 1.0f);

	terrain_material_ = new DreadedPE::Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);

	DreadedPE::SceneLeafModel* terrain_leaf_node = new DreadedPE::SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, CausticTerrainShader::getShader(), false, false);
	
	// The AUV.
	//auv_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(4.2f, 7.0f, 13.0f)), *scene_manager_, *grass_texture, "auv0");
	auv_ = new AUV(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(50.0, 3.0f, 30.0f)), *scene_manager_, *grass_texture, "auv0");
	scene_manager_->addUpdateableEntity(*auv_);

	DreadedPE::DirectedLight* sun = new DreadedPE::DirectedLight(*scene_manager_, glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), 1.01, 0.15, 0.01);
	SceneNode* sun_node = new DreadedPE::SceneNode(*scene_manager_, auv_, glm::rotate(glm::translate(glm::mat4(1.0f), glm::vec3(0, 20, 0)), glm::radians(-90.0f), glm::vec3(1, 0, 0)));
	DreadedPE::SceneLeafLight* light_leaf = new DreadedPE::SceneLeafLight(*sun_node, NULL, *sun);

	caustic_texture_ = new CausticTexture();

	CausticShader::initialiseSun(*sun, *caustic_texture_);
	CausticTerrainShader::initialiseSun(*sun, *caustic_texture_);
	
	// Initialise random generator.
	srand(seed);
	
	/**
	 * Create the missions.
	 */
	MissionSite* mission_site = new MissionSite(*scene_manager_, terrain_node_, glm::mat4(1.0f));
	Mission* inspection_mission1 = NULL;
	Mission* inspection_mission2 = NULL;
	if (nr_inspection_points_1 > 0)
	{
		inspection_mission1 = &createInspectionSite(*mission_site, nr_inspection_points_1);
	}
	if (nr_inspection_points_2 > 0)
	{
		inspection_mission2 = &createInspectionSite(*mission_site, nr_inspection_points_2);
	}
	
	/**
	 * Create the combined inspection missions.
	 */
	if (inspection_mission1 != NULL && inspection_mission2 != NULL)
	{
		Mission* combined_inspection_mission = new Mission(*mission_site);
		for (std::vector<Goal*>::const_iterator ci = inspection_mission1->getGoals().begin(); ci != inspection_mission1->getGoals().end(); ++ci)
		{
			combined_inspection_mission->addGoal(**ci);
		}
		for (std::vector<Goal*>::const_iterator ci = inspection_mission2->getGoals().begin(); ci != inspection_mission2->getGoals().end(); ++ci)
		{
			combined_inspection_mission->addGoal(**ci);
		}
		mission_site->addMission(*combined_inspection_mission);
	}
	
	/**
	 * Create the valve panels and their goals.
	 */
	Mission* valve_turning_mission1 = NULL;
	Mission* valve_turning_mission2 = NULL;
	if (valve_turning_deadline_1 != 0)
	{
		valve_turning_mission1 = &createValveTurningSite(*mission_site, nr_valves1, *grass_texture, valve_turning_deadline_1);
	}
	if (valve_turning_deadline_2 != 0)
	{
		valve_turning_mission2 = &createValveTurningSite(*mission_site, nr_valves2, *grass_texture, valve_turning_deadline_2);
	}
	
	/**
	 * Create the combined valve turning missions.
	 */
	if (valve_turning_mission1 != NULL && valve_turning_mission2 != NULL)
	{
		Mission* combined_valve_turning_mission = new Mission(*mission_site);
		for (std::vector<Goal*>::const_iterator ci = valve_turning_mission1->getGoals().begin(); ci != valve_turning_mission1->getGoals().end(); ++ci)
		{
			combined_valve_turning_mission->addGoal(**ci);
		}
		for (std::vector<Goal*>::const_iterator ci = valve_turning_mission2->getGoals().begin(); ci != valve_turning_mission2->getGoals().end(); ++ci)
		{
			combined_valve_turning_mission->addGoal(**ci);
		}
		mission_site->addMission(*combined_valve_turning_mission);
	}
	
	/**
	 * Combine valve turning and the inspection missions.
	 */
	if (valve_turning_mission1 != NULL && inspection_mission1 != NULL)
	{
		Mission* combined_valve_turning_mission = new Mission(*mission_site);
		for (std::vector<Goal*>::const_iterator ci = valve_turning_mission1->getGoals().begin(); ci != valve_turning_mission1->getGoals().end(); ++ci)
		{
			combined_valve_turning_mission->addGoal(**ci);
		}
		for (std::vector<Goal*>::const_iterator ci = inspection_mission1->getGoals().begin(); ci != inspection_mission1->getGoals().end(); ++ci)
		{
			combined_valve_turning_mission->addGoal(**ci);
		}
		mission_site->addMission(*combined_valve_turning_mission);
	}
	
	/**
	 * Create the chain missions.
	 */
	if (nr_chains > 0)
	{
		Mission& chain_mission = createChainFollowingSite(*mission_site);
		
		// Combine with any other mission.
		Mission* combined_mission = new Mission(*mission_site);
		for (std::vector<Goal*>::const_iterator ci = chain_mission.getGoals().begin(); ci != chain_mission.getGoals().end(); ++ci)
		{
			combined_mission->addGoal(**ci);
		}
		
		if (valve_turning_mission1 != NULL)
		{
			for (std::vector<Goal*>::const_iterator ci = valve_turning_mission1->getGoals().begin(); ci != valve_turning_mission1->getGoals().end(); ++ci)
			{
				combined_mission->addGoal(**ci);
			}
		}
		
		if (valve_turning_mission2 != NULL)
		{
			for (std::vector<Goal*>::const_iterator ci = valve_turning_mission2->getGoals().begin(); ci != valve_turning_mission2->getGoals().end(); ++ci)
			{
				combined_mission->addGoal(**ci);
			}
		}
		
		if (inspection_mission1 != NULL)
		{
			for (std::vector<Goal*>::const_iterator ci = inspection_mission1->getGoals().begin(); ci != inspection_mission1->getGoals().end(); ++ci)
			{
				combined_mission->addGoal(**ci);
			}
		}
		
		if (inspection_mission2 != NULL)
		{
			for (std::vector<Goal*>::const_iterator ci = inspection_mission2->getGoals().begin(); ci != inspection_mission2->getGoals().end(); ++ci)
			{
				combined_mission->addGoal(**ci);
			}
		}
		mission_site->addMission(*combined_mission);
	}
	
	/**
	 * Initiate the octomap and ontology.
	 */
	octomap_ = new OctomapBuilder(*ros_node_, *auv_);
	ontology_ = new Ontology(*ros_node_, *octomap_);
	ontology_->addAUV(*auv_);
	ontology_->addMissionSite(*mission_site);
	
	
	DreadedPE::MaterialLightProperty* ambient = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* diffuse = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* specular = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* emmisive = new DreadedPE::MaterialLightProperty(0, 0, 1, 0.1f);
	DreadedPE::Material* material = new DreadedPE::Material(*ambient, *diffuse, *specular, *emmisive);
	
	rrt_ = new RRT(*ros_node_, *scene_manager_, *auv_, *terrain_node_, *octomap_, *ontology_, *material);
	ontology_->initialise(*rrt_);
	
	// Setup the GUI.
	DreadedPE::MyGUITheme* theme = new DreadedPE::MyGUITheme();
	DreadedPE::Texture* font_texture = DreadedPE::TargaTexture::loadTexture("data/textures/fonts/test_font.tga");

	DreadedPE::Font* font = new DreadedPE::TexturedFont(*font_texture);
	
	//WaypointLabeler* wl = new WaypointLabeler(*scene_manager_, *rrt_, *terrain_node_);

	// Initialise the action controller that will receive all the messages from the planner.
	FollowWaypointController* controller_ = new FollowWaypointController(*scene_manager_, *auv_, *rrt_);
	action_controller_ = new ActionController(*scene_manager_, *ros_node_, *auv_, *terrain_node_, *controller_, *ontology_);

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
	DreadedPE::SceneNode* stable_platform = new DreadedPE::SceneNode(*scene_manager_, &auv_->getAUVNode(), glm::mat4(1.0f));
	stable_platform->ignoreRotations(true);
	
	{
		DreadedPE::PointLight* point_light = new DreadedPE::PointLight(*scene_manager_, 34, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f);
		DreadedPE::SceneNode* light_node = new DreadedPE::SceneNode(*scene_manager_, &auv_->getAUVModel(),  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -1.25f)));
		DreadedPE::SceneLeafLight* light_leaf = new DreadedPE::SceneLeafLight(*light_node, NULL, *point_light);

		volumetric_light_point_ = new DreadedPE::PointLight(*scene_manager_, 34, glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 10.0f, 128, GL_NONE, GL_NONE, GL_NONE);
		volumetric_light_leaf_ = new DreadedPE::SceneLeafLight(*light_node, NULL, *volumetric_light_point_);
		DreadedPE::LightVolumeShape* lvs = new DreadedPE::LightVolumeShape(*scene_manager_, *volumetric_light_point_);
		
		light_volume_leaf_ = new DreadedPE::SceneLeafModel(*light_node, NULL, *lvs, *terrain_material_, ShadowVolumeShader::getShader(), false, true, COLLISION, ShadowRenderer::NO_SHADOW);
		lvs->setLeafNode(*light_volume_leaf_);
	}
	
#ifndef _WIN32
	sonar_ = new Sonar(*ros_node_, *scene_manager_, auv_, *auv_, auv_->getName(), glm::mat4(1.0f), 0.1f, 60.0f, 30.0f);
	//FrustumShape* frustumShape = new FrustumShape(0.1f, 60.0f, tan((30.0f / 180.0f) * M_PI) * 0.1f, tan((30.0f / 180.0f) * M_PI) * 0.1f, tan((30.0f / 180.0f) * M_PI) * 60.0f, tan((30.0f / 180.0f) * M_PI) * 60.0f);
	//SceneLeafModel* scene_leaf_model = new SceneLeafModel(*sonar_, NULL, *frustumShape, *terrain_material_, BasicShadowShader::getShader(), true, false);
#endif

	// Add the camera system.
	camera_node_ = new DreadedPE::DynamicCamera(*auv_, *scene_manager_, stable_platform, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.0f, 2.0f)), 90.0f, 1024, 768, 0.1f, 300.0f);
	//camera_node_ = new FreeMovingCamera(*scene_manager_, terrain_node_, glm::mat4(1.0f), 90.0f, 1024, 768, 0.1f, 300.0f);
	scene_manager_->addUpdateableEntity(*camera_node_);
	
	PlanVisualiser* pv = new PlanVisualiser(*ros_node_, *auv_, *ontology_, *terrain_node_, *scene_manager_, *theme, *font, *camera_node_);
	action_controller_->addListener(*pv);
	
	StrategicPlanVisualiser* spv = new StrategicPlanVisualiser(*ros_node_, *auv_, *ontology_, *terrain_node_, *scene_manager_, *theme, *font, *camera_node_);
	action_controller_->addListener(*spv);
	
	DreadedPE::GUIManager& gui_manager = DreadedPE::GUIManager::getInstance();
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
#ifdef _WIN32
	pl_ = new PlanLine(*theme, *font, 200, 30, 200, 650);
#else
	pl_ = new PlanLine(*ros_node_, *theme, *font, 0, -700, 1024, 60);
	//pl_ = new PlanLine(*ros_node_, *theme, *font, 0, 200, width_, 200);
	*/
	planning_gui_ = new PlanningGUI(*ros_node_, *theme, *font, 0, 0, width_, height_, 15);
	planning_gui_->addPlanLine(*auv_);
	action_controller_->addListener(*planning_gui_);
	gui_manager.addFrame(*planning_gui_);
	
	DreadedPE::MaterialLightProperty* inspection_point_material_ambient = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* inspection_point_material_diffuse = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* inspection_point_material_specular = new DreadedPE::MaterialLightProperty(0, 0, 0, 0);
	DreadedPE::MaterialLightProperty* inspection_point_material_emmisive = new DreadedPE::MaterialLightProperty(1, 1, 1, 1.0f);
	DreadedPE::Material* inspection_point_material = new DreadedPE::Material(*inspection_point_material_ambient, *inspection_point_material_diffuse, *inspection_point_material_specular, *inspection_point_material_emmisive);
	inspection_point_material->add2DTexture(*grass_texture);
	
	DreadedPE::MaterialLightProperty* view_point_material_emmisive = new DreadedPE::MaterialLightProperty(1, 0, 1, 1.0f);
	DreadedPE::Material* view_point_material = new DreadedPE::Material(*inspection_point_material_ambient, *inspection_point_material_diffuse, *inspection_point_material_specular, *view_point_material_emmisive);
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
	shadow_renderer_ = new DreadedPE::ShadowRenderer(*scene_manager_, 512, GL_BACK, GL_NONE, GL_NONE);
	
	// Create a seperate framebuffer for the post processing.
	post_processing_texture_ = new DreadedPE::Texture(GL_TEXTURE_2D);
	
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

	// Setup the GUI for the FPS.
	DreadedPE::Container* fps_container = new DreadedPE::Container(*theme, font->clone(), 10, 10, 120, 20, false);
	DreadedPE::Label* fps_label = new DreadedPE::Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FDreadedPE::PSLabel(*fps_label);

	gui_manager.addFrame(*fps_container);
	return true;
}

bool OpportunityGenerator::postInit()
{
#ifndef _WIN32
	//PipeNetwork* pipe_network = new PipeNetwork(*scene_manager_, terrain_node_, ontology_->getMissionSites(), 1.0f);
#endif
	
	glfwGetWindowSize(&width_, &height_);
	//return true;
	// Lets add some sharks :)
	//AssimpLoader* loader = new AssimpLoader();
	std::pair<DreadedPE::AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > shark_properties = DreadedPE::AssimpLoader::LoadModel(*scene_manager_, "data/models/Pandora/sea creatures/shark.dae");
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
	DreadedPE::Texture* shark_texture = DreadedPE::TargaTexture::loadTexture("data/models/Pandora/sea creatures/shark.tga");
	
	DreadedPE::MaterialLightProperty* shark_ambient = new DreadedPE::MaterialLightProperty(0.2, 0.2, 0.2, 1.0);
	DreadedPE::MaterialLightProperty* shark_diffuse = new DreadedPE::MaterialLightProperty(0.8, 0.8, 0.8, 1.0);
	DreadedPE::MaterialLightProperty* shark_specular = new DreadedPE::MaterialLightProperty(0.01, 0.01, 0.01, 1.0);
	DreadedPE::MaterialLightProperty* shark_emissive = new DreadedPE::MaterialLightProperty(0.6, 0.6, 0.6, 1.0);
	
	Material* shark_material = new Material(*shark_ambient, *shark_diffuse, *shark_specular, *shark_emissive);
	shark_material->add2DTexture(*shark_texture);
	
	
	Shark* shark = new Shark(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(20.0f, 13.25f, 0.0f)), *scene_manager_);
	DreadedPE::SceneLeafModel* chair_leaf_node = new DreadedPE::SceneLeafModel(*shark, NULL, *shark_properties.first, *shark_material, AnimatedShadowShader::getShader(), false, false);
	chair_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);

	shark->init(*shark_material, DreadedPE::BasicShadowShader::getShader());
	
	std::vector<glm::vec3> waypoints1;
	waypoints1.push_back(glm::vec3(10, 15, 25));
	waypoints1.push_back(glm::vec3(-40, 6, 34));
	waypoints1.push_back(glm::vec3(15, 30, -15));
	shark->setWaypoints(waypoints1);
	
	Shark* shark2 = new Shark(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(-20.0f, 45.25f, 0.0f)), *scene_manager_);
	DreadedPE::SceneLeafModel* chair_leaf_node2 = new DreadedPE::SceneLeafModel(*shark2, NULL, *shark_properties.first, *shark_material, AnimatedShadowShader::getShader(), false, false);
	chair_leaf_node2->setShadowType(DreadedPE::ShadowRenderer::ANIMATED_SHADOW);
	
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

GLuint OpportunityGenerator::postProcess(DreadedPE::Texture& color_texture, DreadedPE::Texture& depth_texture, float dt)
{
	//glfwSetMousePos(width_ / 2, height_ / 2);
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

	glClearColor(0.0, 0.0, 0.0, 0.0f);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo_id_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_DEPTH_CLAMP);

	// Enable additive blending.
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE, GL_ONE);

	DreadedPE::ShadowVolumeShader& shader = DreadedPE::ShadowVolumeShader::getShader();
	shader.initialise(*light_volume_leaf_, view_matrix, light_volume_leaf_->getParent()->getCompleteTransformation(), perspective_matrix, *volumetric_light_leaf_, camera_node_->getNearPlane(), camera_node_->getFarPlane(), shadow_renderer_->getTexture());
	light_volume_leaf_->draw(view_matrix, perspective_matrix, active_lights, NULL);
	
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

void OpportunityGenerator::tick(float dt)
{
	caustic_texture_->update(dt);
#ifndef _WIN32
	ros::spinOnce();
	// Publish the odometry information of the UAV.
	auv_odometry_->update(dt);
	//sonar_odometry_->update(dt);
	
	// Update the actions.
	action_controller_->update(dt);
	
	// Update ROS.
	ros::spinOnce();
	DreadedPE::Frustum frustum(sonar_->getPerspectiveMatrix() * sonar_->getViewMatrix());
	for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
		{
			Chain* chain = *ci;
			if (chain->getFrustumChecker().isInsideFrustum(frustum) && glm::distance(chain->getGlobalLocation(), auv_->getGlobalLocation()) < 20.0f)
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
}

void OpportunityGenerator::onResize(int width, int height)
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
}

Mission& OpportunityGenerator::createInspectionSite(MissionSite& mission_site, unsigned int nr_inspection_points)
{
	std::vector<Structure*> structures_of_inspection_site_1;
	for (unsigned int i = 0; i < nr_inspection_points; i += 2)
	{
		glm::vec3 structure_location;
		bool is_collision_free = false;
		while (!is_collision_free)
		{
			structure_location.x = (0.5f - (float)rand() / (float)RAND_MAX) * 100;
			structure_location.y = 0;
			structure_location.z = (0.5f - (float)rand() / (float)RAND_MAX) * 100;
			
			// Check it is not too close to other structures.
			bool new_location_is_collision_free = true;
			for (std::vector<glm::vec3>::const_iterator ci = structure_locations_.begin(); ci != structure_locations_.end(); ++ci)
			{
				const glm::vec3& location = *ci;
				if (glm::distance(location, structure_location) < 20)
				{
					new_location_is_collision_free = false;
					break;
				}
			}
			
			if (new_location_is_collision_free)
			{
				is_collision_free = true;
				structure_locations_.push_back(structure_location);
			}
		}
		
		SmallManifold* small_manifold = new SmallManifold(*scene_manager_, &mission_site, mission_site, glm::translate(glm::mat4(1.0f), structure_location));
		mission_site.addStructure(*small_manifold);
		structures_of_inspection_site_1.push_back(small_manifold);
	}
	
	// Create an inspection mission for this.
	Mission* inspection_mission = new Mission(mission_site);
	for (std::vector<Structure*>::const_iterator ci = structures_of_inspection_site_1.begin(); ci != structures_of_inspection_site_1.end(); ++ci)
	{
		Structure* structure = *ci;
		for (std::vector<InspectionGoal*>::const_iterator ci = structure->getInspectionGoals().begin(); ci != structure->getInspectionGoals().end(); ++ci)
		{
			inspection_mission->addGoal(**ci);
			
			if (nr_inspection_points == inspection_mission->getGoals().size())
			{
				break;
			}
		}
		
		if (nr_inspection_points == inspection_mission->getGoals().size())
		{
			break;
		}
	}
	//mission_site.addMission(*inspection_mission);
	return *inspection_mission;
}

Mission& OpportunityGenerator::createValveTurningSite(MissionSite& mission_site, unsigned int valves, Texture& valve_texture, float valve_deadline)
{
	Mission* valve_turning_mission = new Mission(mission_site);
	
	glm::vec3 centre_structure;
	centre_structure.x = (0.5f - (float)rand() / (float)RAND_MAX) * 100;
	centre_structure.y = 2;
	centre_structure.z = (0.5f - (float)rand() / (float)RAND_MAX) * 100;
	
	for (unsigned int i = 0; i < valves; ++i)
	{
		glm::vec3 structure_location;
		bool is_collision_free = false;
		while (!is_collision_free)
		{
			structure_location.x = (0.5f - (float)rand() / (float)RAND_MAX) * 50;
			structure_location.y = 2;
			structure_location.z = (0.5f - (float)rand() / (float)RAND_MAX) * 50;
			structure_location += centre_structure;
			
			// Check it is not too close to other structures.
			bool new_location_is_collision_free = true;
			for (std::vector<glm::vec3>::const_iterator ci = structure_locations_.begin(); ci != structure_locations_.end(); ++ci)
			{
				const glm::vec3& location = *ci;
				if (glm::distance(location, structure_location) < 5)
				{
					new_location_is_collision_free = false;
					break;
				}
			}
			
			if (new_location_is_collision_free)
			{
				is_collision_free = true;
				structure_locations_.push_back(structure_location);
			}
		}
		
		ValvePanel* valve_panel = new ValvePanel(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), structure_location), "Valve Panel", valve_texture);
		Valve* valve = new Valve(*valve_panel, *scene_manager_, valve_panel, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)), "Valve Panel", valve_texture);
		ValveGoal* vg = new ValveGoal(mission_site, *valve, 1, valve_deadline, M_PI / 2.0f);
		valve->addValveGoal(*vg);
		valve_panel->addValve(*valve);
		scene_manager_->addUpdateableEntity(*valve);
		valve_turning_mission->addGoal(*vg);
		mission_site.addPanel(*valve_panel);
	}
	//mission_site.addMission(*valve_turning_mission);
	return *valve_turning_mission;
}

Mission& OpportunityGenerator::createChainFollowingSite(MissionSite& mission_site)
{
	glm::vec3 structure_location;
	bool is_collision_free = false;
	while (!is_collision_free)
	{
		structure_location.x = (0.5f - (float)rand() / (float)RAND_MAX) * 50;
		structure_location.y = 2;
		structure_location.z = (0.5f - (float)rand() / (float)RAND_MAX) * 50;
		
		// Check it is not too close to other structures.
		bool new_location_is_collision_free = true;
		for (std::vector<glm::vec3>::const_iterator ci = structure_locations_.begin(); ci != structure_locations_.end(); ++ci)
		{
			const glm::vec3& location = *ci;
			if (glm::distance(location, structure_location) < 5)
			{
				new_location_is_collision_free = false;
				break;
			}
		}
		
		if (new_location_is_collision_free)
		{
			is_collision_free = true;
			structure_locations_.push_back(structure_location);
		}
	}

	Mission* chain_following_mission = new Mission(mission_site);
	Chain* chain = new Chain(mission_site, *scene_manager_, &mission_site, *terrain_node_, glm::translate(glm::mat4(1.0f), structure_location), "");
	chain_following_mission->addGoal(chain->getGoal());
	mission_site.addChain(*chain);
	chain->setObserved();
	//mission_site.addMission(*chain_following_mission);
	
	return *chain_following_mission;
}
