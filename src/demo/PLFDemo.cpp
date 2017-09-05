#include "PLFDemo.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <fstream>

#include <glm/gtc/matrix_transform.hpp> 
#include <GL/glew.h>

#include "../core/loaders/targa.h"
#include "../core/entities/camera/Camera.h"
#include "../core/entities/camera/DynamicCamera.h"
#include "../shapes/terrain.h"
#include "../shapes/Water.h"
#include "../shapes/Tree.h"
#include "../shapes/sphere.h"
#include "../shapes/Piramid.h"
#include "../shapes/Cube.h"
#include "../shapes/SkyBox.h"
#include "../shapes/Line.h"
#include "../core/light/PointLight.h"
#include "../core/light/Light.h"

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
#include "../core/animation/LinearAnimation.h"
#include "../core/animation/BouncingBox.h"
#include "../core/entities/WavingWater.h"
#include "../core/entities/Player.h"
#include "../core/collision/BoxCollision.h"
#include "../core/entities/Lever.h"
#include "../core/entities/Bridge.h"
#include "../core/scene/frustum/SphereCheck.h"

#include "../core/entities/behaviours/RotateBehaviour.h"

#include "../core/loaders/WavefrontLoader.h"

#include "../core/loaders/AssimpLoader.h"

#include "../core/texture/TargaTexture.h"
#include "../core/texture/Texture.h"

#include "../core/loaders/PortalLevelFormatLoader.h"

#include "flat/Wall.h"

#include "../core/models/AnimatedModel.h"

#include "shooter/ArmedPlayer.h"

#include "../core/loaders/AssimpLoader.h"
#include "../core/entities/Monster.h"

#include "../core/ai/pathfinding/NavMeshAStar.h"
#include "../core/ai/pathfinding/NavigationMesh.h"
#include "../core/ai/pathfinding/ConvexNavigationArea.h"

#include "../core/gui/themes/MyGUITheme.h"
#include "../core/gui/Label.h"
#include "../core/gui/Container.h"
#include "../core/gui/GUIManager.h"
#include "../core/gui/fonts/TexturedFont.h"
#include "gui_demo/FPSLabel.h"

PLFDemo::PLFDemo(SceneManager& scene_manager)
	: scene_manager_(&scene_manager)
{
	srand (time(NULL));
}

bool PLFDemo::init(int argc, char** argv)
{
#ifdef _WIN32
	if (!GLEW_VERSION_4_3)
	{
		std::cerr << "Right OpenGL version is not suppored!" << std::endl;
		std::ofstream file("example_init_errors.txt");
		file << "VBOs are not supported by your graphics card." << std::endl;
		file.close();
        	std::cerr << "VBOs are not supported by your graphics card" << std::endl;
	        MessageBox(NULL, "VBOs are not supported by your graphics card!", "An error occurred", MB_ICONERROR | MB_OK);
        	return false;
	}
	std::cout << "Right version is supported! :D" << std::endl;
#endif
	glEnable(GL_DEPTH_TEST);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

	glEnableVertexAttribArray(0); // Vertex.
	glEnableVertexAttribArray(1); // Colour.
	glEnableVertexAttribArray(2); // Texture.
	glEnableVertexAttribArray(3); // Normals.
	glEnableVertexAttribArray(4); // Bone weights.
	glEnableVertexAttribArray(5); // Bone ids.

	terrain_ = new Terrain();
	if (!terrain_->createHeightmap(129, -0.02f))
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the terrain heightmap", "Error", MB_OK);
#endif
		std::cout << "Could not load the terrain heightmap" << std::endl;
		return false;
	}

	terrain_node_ = new Entity(*scene_manager_, &scene_manager_->getRoot(), glm::mat4(1.0), OBSTACLE, "terrain");

	
	Texture* wfl_texture = TargaTexture::loadTexture("data/models/levels/modular/atlas.tga");
	
	// Initialise the texture to use.
	MaterialLightProperty* wfl_ambient = new MaterialLightProperty(0.4f, 0.4f, 0.4f, 1.0f);
	MaterialLightProperty* wfl_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* wfl_specular = new MaterialLightProperty(0.3f, 0.3f, 0.3f, 1.0f);
	MaterialLightProperty* wfl_emmisive = new MaterialLightProperty(0.6f, 0.6f, 0.6f, 1.0f);

	Material* wfl_material_ = new Material(*wfl_ambient, *wfl_diffuse, *wfl_specular, *wfl_emmisive);
	wfl_material_->add2DTexture(*wfl_texture);
	
	PortalLevelFormatLoader* level_loader = new PortalLevelFormatLoader();
	//level_node_ = level_loader->importLevel("data/models/levels/modular/basic_elements6.plf", *scene_manager_, *terrain_node_);
	level_node_ = level_loader->importLevel("data/models/levels/PathFindingTests/Test1.plf", *wfl_material_, BasicShadowShader::getShader(), *scene_manager_, *terrain_node_);
	if (level_node_ == NULL)
	{
#ifdef _WIN32
		MessageBox(NULL, "Could not load the level!", "Error", MB_OK);
#endif
		std::cout << "Could not load the level!" << std::endl;
		exit(1);
	}

	SceneNode* connectivity_node = new SceneNode(*scene_manager_, level_node_, glm::mat4(1.0f));
	
	
	navigation_mesh_ = level_loader->createNavigationMesh(scene_manager_);
	
	std::vector<glm::vec3> waypoints;
//	NavMeshAStar* pathfinder = new NavMeshAStar(navigation_mesh_->getAreas());
	
	Texture* grass_texture = TargaTexture::loadTexture("data/textures/grass.tga");
	Texture* height_texture = TargaTexture::loadTexture("data/textures/height.tga");
	
	MaterialLightProperty* bright_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
	MaterialLightProperty* bright_emmisive = new MaterialLightProperty(1.0, 0.2, 1.0, 1.0);

	Material* bright_material_ = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
	bright_material_->add2DTexture(*grass_texture);
	
	glLineWidth(3.0f);
	glEnable(GL_LINE_SMOOTH);

	// Visualise the connectivity between the different navigation areas.
	std::vector<glm::vec3> connection_points;
	/*
	{
	//std::vector<glm::vec3> vertices;
	//std::vector<GLuint> indices;
	//std::vector<glm::vec3> normals;
	//std::vector<glm::vec2> texture_coordinates;
	
	Line* connectivity_lines = new Line(false);
	for (std::vector<ConvexNavigationArea>::const_iterator ci = navigation_mesh_->getAreas().begin(); ci != navigation_mesh_->getAreas().end(); ++ci)
	{
		const ConvexNavigationArea& cna = *ci;
		for (std::vector<const CNA_Adjacent*>::const_iterator ci = cna.getAdjacentAreas().begin(); ci != cna.getAdjacentAreas().end(); ++ci)
		{
			const CNA_Adjacent* adjacent = *ci;
			connection_points.push_back(adjacent->adjacent1_->getCentre() + glm::vec3(0.0f, 1.0f, 0.0f));
			connection_points.push_back(adjacent->adjacent2_->getCentre() + glm::vec3(0.0f, 1.0f, 0.0f));
			
			connection_points.push_back((adjacent->p1_ + adjacent->p2_) / 2.0f + glm::vec3(0.0f, 1.0f, 0.0f));
			connection_points.push_back(adjacent->adjacent1_->getCentre() + glm::vec3(0.0f, 1.0f, 0.0f));

			connection_points.push_back((adjacent->p1_ + adjacent->p2_) / 2.0f + glm::vec3(0.0f, 1.0f, 0.0f));
			connection_points.push_back(adjacent->adjacent2_->getCentre() + glm::vec3(0.0f, 1.0f, 0.0f));
		}

		connection_points.push_back(cna.getCentre());
		connection_points.push_back(cna.getCentre() + glm::vec3(0.0f, 1.0f, 0.0f));
	}
	connectivity_lines->setVertexBuffer(connection_points);

	SceneLeafModel* connectivity = new SceneLeafModel(*connectivity_node, NULL, *connectivity_lines, *bright_material_, LineShader::getShader(), false, false);
	}
	
	
	MaterialLightProperty* bright_emmisive2 = new MaterialLightProperty(0.0, 1.0, 0.1, 1.0);
	Material* bright_material2 = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive2);
	bright_material2->add2DTexture(*grass_texture);

	{
	Line* connectivity_lines2 = new Line(false);
	connection_points.clear();
	std::cout << "Show " << navigation_mesh_->getAreas().size() << " arenas." << std::endl;
	for (std::vector<ConvexNavigationArea>::const_iterator ci = navigation_mesh_->getAreas().begin(); ci != navigation_mesh_->getAreas().end(); ++ci)
	{
		const ConvexNavigationArea& cna = *ci;
		
		std::cout << "- " << cna << std::endl;
		//for (std::vector<glm::vec3>::const_iterator ci = cna.getPoints().begin(); ci != cna.getPoints().end(); ++ci)
		for (unsigned int i = 0; i < cna.getPoints().size(); ++i)
		{
			const glm::vec3& point = cna.getPoints()[i];
			connection_points.push_back(cna.getPoints()[i] + glm::vec3(0.0f, 1.0f, 0.0f));
			connection_points.push_back(cna.getPoints()[(i + 1) % cna.getPoints().size()] + glm::vec3(0.0f, 1.0f, 0.0f));
		}
	}
	connectivity_lines2->setVertexBuffer(connection_points);
	SceneLeafModel* connectivity = new SceneLeafModel(*connectivity_node, NULL, *connectivity_lines2, *bright_material2, LineShader::getShader(), false, false);
	}
	*/
	scene_manager_->addPlayer(*connectivity_node);	

	// Initialise a terrain to render.
	MaterialLightProperty* terrain_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* terrain_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* terrain_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* terrain_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	terrain_material_ = new Material(*terrain_ambient, *terrain_diffuse, *terrain_specular, *terrain_emmisive);
	terrain_material_->add1DTexture(*height_texture);
	terrain_material_->add2DTexture(*grass_texture);

	SceneLeafModel* terrain_leaf_node = new SceneLeafModel(*terrain_node_, NULL, *terrain_, *terrain_material_, TerrainShader::getShader(), false, false);
	
	// The player.
	player_ = new ArmedPlayer(terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(10.0f, 30.0f, 0.0f)), 1.9f, *terrain_node_, *terrain_, *scene_manager_, *grass_texture);
	scene_manager_->addPlayer(*player_);
	scene_manager_->addUpdateableEntity(*player_);

	SceneNode* stable_platform = new SceneNode(*scene_manager_, player_, glm::mat4(1.0f));
	stable_platform->ignoreRotations(true);

	// Represent the player by a cube.
	Cube* player_shape = new Cube(0.25f, 1.7f, 0.25f);
	SceneLeafModel* player_model = new SceneLeafModel(*stable_platform, NULL, *player_shape, *bright_material_, BasicShadowShader::getShader(), false, false);

	// Initialise the camera:
	camera_ = new Camera(*scene_manager_, player_, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.7f, 2.0f)), 90.0f, 1024, 768, 0.1f, 60.0f);
	
	// Add some lighting.
	{
		PointLight* point_light = new PointLight(*scene_manager_, 35.0f, glm::vec3(0.6f, 0.6f, 0.6f), glm::vec3(0.9f, 0.9f, 0.9f), glm::vec3(0.2f, 0.2f, 0.2f), 1.0f, 0.3f, 0.15f, 0.1f, 60.0f);
		SceneNode* light_node = new SceneNode(*scene_manager_, player_,  glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.2f, -0.25f)));
		SceneLeafLight* light_leaf = new SceneLeafLight(*light_node, NULL, *point_light);
	}

	Cube* rotating_cube = new Cube(2, 0.5, 2);
	Entity* rotating_cube_entity = new Entity(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0), glm::vec3(50.0f, 1.0f, 0.0f)), OBSTACLE, "Rotating Cube");
	BoxCollision* stair_step_bc = new BoxCollision(*rotating_cube_entity, 2.0f, 0.5f, 2.0f);
	rotating_cube_entity->addCollision(*stair_step_bc);
	Behaviour* behaviour = new RotateBehaviour(*rotating_cube_entity, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0), 0, 180, 90, 10);
	rotating_cube_entity->addBehaviour(*behaviour);
	SceneLeafModel* rotating_cube_leaf = new SceneLeafModel(*rotating_cube_entity, NULL, *rotating_cube, *bright_material_, BasicShadowShader::getShader(), false, false);
	scene_manager_->addUpdateableEntity(*rotating_cube_entity);
	scene_manager_->addPlayer(*rotating_cube_entity);
	
	// Initialise the sky box to render.
	MaterialLightProperty* skybox_ambient = new MaterialLightProperty(0.2f, 0.2f, 0.2f, 1.0f);
	MaterialLightProperty* skybox_diffuse = new MaterialLightProperty(0.8f, 0.8f, 0.8f, 1.0f);
	MaterialLightProperty* skybox_specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
	MaterialLightProperty* skybox_emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

	Material* skybox_material = new Material(*skybox_ambient, *skybox_diffuse, *skybox_specular, *skybox_emmisive);
	Texture* skybox_texture = TargaTexture::loadTexture("data/textures/skybox/sp3left.tga", "data/textures/skybox/sp3right.tga", "data/textures/skybox/sp3top.tga", "data/textures/skybox/sp3bot.tga", "data/textures/skybox/sp3back.tga", "data/textures/skybox/sp3front.tga");
	skybox_material->addCubeTexture(*skybox_texture);

	SkyBox* inverted_cube = new SkyBox(5.0f);
	SkyBoxLeaf* sky_box = new SkyBoxLeaf(*camera_, *inverted_cube, SkyBoxShader::getShader(), *skybox_material);

	MaterialLightProperty* bright_emmisive3 = new MaterialLightProperty(0.3, 0.3, 1.0, 1.0);

	Material* bright_material3 = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive3);
	bright_material3->add2DTexture(*grass_texture);

	// Add a component to render the lines that result from the waypoint generator.
	SceneNode* visualise_node = new SceneNode(*scene_manager_, NULL, glm::mat4(1.0f));
	waypoint_visualiser_ = new SceneLeafModel(*visualise_node, NULL, *(new Line()), *bright_material3, LineShader::getShader(), false, false);
	scene_manager_->addPlayer(*visualise_node);

	// Setup the GUI for the FPS.
	MyGUITheme* theme = new MyGUITheme();
	Texture* font_texture = TargaTexture::loadTexture("data/textures/fonts/test_font.tga");
	
	Font* font = new TexturedFont(*font_texture);
	GUIManager& gui_manager = GUIManager::getInstance();
	
	Container* fps_container = new Container(*theme, font->clone(), 10, 10, 120, 20, false);
	Label* fps_label = new Label(*theme, 120, 20, "", 12);
	fps_container->addElement(*fps_label, 0, -20);
	fps_label_ = new FPSLabel(*fps_label);

	gui_manager.addFrame(*fps_container);
	
	debug_label_ = new Label(*theme, 300, 50, "", 16);
	Container* debug_container = new Container(*theme, font->clone(), 200, 10, 700, 50, true);
	debug_container->addElement(*debug_label_, 0, -40);
	gui_manager.addFrame(*debug_container);

	return true;
}

bool PLFDemo::postInit()
{
	glfwGetWindowSize(&width_, &height_);
	return true;
	pathfinder_ = new NavMeshAStar(navigation_mesh_->getAreas());
	

	std::cout << "Start loading the monster model..." << std::endl;
	AssimpLoader* loader = new AssimpLoader();
	std::pair<AnimatedModel*, std::map<aiTextureType, std::vector<Texture*>* >* > monster_properties = loader->LoadModel(*scene_manager_, "data/models/grunt_beast/grunt beast2.dae");
	//for (unsigned int i = 0; i < 10; ++i)
	{
		//Region* region = scene_manager_->getRoot().findRegion(glm::vec3(11.0f, 1.01f, 3.1f));
		Region* region = Region::findRegionGlobal(glm::vec3(-2.0f, 1.01f, 4.5f));
		if (region == NULL)
		{
#ifdef _WIN32
			MessageBox(NULL, "Could not find the region of the monster :(.", "Error", MB_OK);
#endif
			std::cout << "Could not find the region of the monster :(" << std::endl;
			return false;
		}
		//monster_ = new Monster(&region->getSceneNode(), glm::translate(glm::mat4(1.0), glm::vec3(11.0f, 1.01f, 3.1f)), *scene_manager_);
		monster_ = new Monster(&region->getSceneNode(), glm::translate(glm::mat4(1.0), glm::vec3(-2.0f, 1.01f, 4.5f)), *scene_manager_);
		
		Texture* diffuse_texture = (*(*monster_properties.second)[aiTextureType_DIFFUSE])[0];
		MaterialLightProperty* ambient = new MaterialLightProperty(0.1f, 0.1f, 0.1f, 1.0f);
		MaterialLightProperty* diffuse = new MaterialLightProperty(1.0f, 1.0f, 1.0f, 1.0f);
		MaterialLightProperty* specular = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);
		MaterialLightProperty* emmisive = new MaterialLightProperty(0.0f, 0.0f, 0.0f, 1.0f);

		Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
		material->add2DTexture(*diffuse_texture);
		
		SceneLeafModel* chair_leaf_node = new SceneLeafModel(*monster_, NULL, *monster_properties.first, *material, AnimatedShadowShader::getShader(), false, false);
		chair_leaf_node->setShadowType(ShadowRenderer::ANIMATED_SHADOW);

		monster_->init(*SceneNode::bright_material_, BasicShadowShader::getShader());
		scene_manager_->addUpdateableEntity(*monster_);
		monster_properties.first->setAnimation(*monster_properties.first->getAnimations()[0]);
		//monster_->setWaypoints(waypoints);
	}
	
	return true;
}

void PLFDemo::tick(float dt)
{
	std::stringstream ss;
	Region* player_region = player_->getRegion();
	if (player_region == NULL)
		ss << "Player is in NO region." << std::endl;
	else
		ss << "Player is in region: " << player_region->getName() << std::endl;
	debug_label_->setLabel(ss.str());
	/*
	for (std::vector<Region*>::const_iterator ci = Region::getRegions().begin(); ci != Region::getRegions().end(); ++ci)
	{
		Region* region = *ci;
		region->g
	}
	*/
	return;
	// Update the monster's waypoint every second.
	static float time_elapsed = 0;
	/*
	Region* player_region = player_->getRegion();
	if (player_region == NULL)
		std::cout << "Player is in NO region." << std::endl;
	else
		std::cout << "Player is in region: " << player_region->getName() << std::endl;
	*/
	time_elapsed += dt;
	if (time_elapsed > 1.0f)
	{
		time_elapsed -= 0.1f;
		
		const ConvexNavigationArea* area = pathfinder_->getArea(player_->getGlobalLocation());
		/*
		if (area != NULL)
		{
			std::cout << *area << std::endl;
		}
		else
		{
			std::cout << "Unknown location." << std::endl;
		}
		*/

		// Find out where the player is.
		std::vector<glm::vec3> waypoints;
		//bool found_path = pathfinder_->findPath(monster_->getGlobalLocation() + glm::vec3(0, 10, 0), player_->getGlobalLocation() + glm::vec3(0, 10, 0), waypoints);
		bool found_path = pathfinder_->findPath(monster_->getGlobalLocation() + glm::vec3(0, 2, 0), player_->getGlobalLocation() + glm::vec3(0, 1, 0), waypoints);
		
		if (waypoints.size() > 2)
		{
			for (std::vector<glm::vec3>::iterator ci = waypoints.begin() + 1; ci != waypoints.end() - 1; ++ci)
			{
				glm::vec3& w = *ci;
				w.y += 0.75f;
			}
		}
/*		
		if (waypoints.size() > 0)
		{
			//std::cout << "Set waypoints: " << waypoints.size() << std::endl;
			waypoint_visualiser_->getModel().setVertexBuffer(waypoints);
		}
*/
		monster_->setWaypoints(waypoints);
	}
}

GLuint PLFDemo::postProcess(Texture& color_texture, Texture& depth_texture, float dt)
{
	fps_label_->frameRendered();
	glfwSetMousePos(width_ / 2, height_ / 2);
	return 0;
}

void PLFDemo::onResize(int width, int height)
{
	width_ = width;
	height_ = height;
	camera_->onResize(width, height);
}
