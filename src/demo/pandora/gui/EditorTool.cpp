#include "EditorTool.h"

#include <GL/glfw.h>

#include "../../../core/gui/GUIManager.h"
#include "../../../core/gui/Button.h"
#include "../../../core/gui/Label.h"
#include "../../../core/entities/camera/Camera.h"
#include "../../../core/entities/HeightMap.h"
#include "../../../core/collision/CollisionInfo.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/scene/SceneLeaf.h"
#include "../../../core/scene/Material.h"

#include "../structures/Structure.h"
#include "../level/MissionSite.h"
#include "../level/Mission.h"
#include "../ontology/InspectionGoal.h"
#include "../ontology/InspectionPoint.h"
#include "../ontology/OntologyInterface.h"

std::string EditorTool::status_string_[] = { "IDLE", "REMOVE", "ADD PILLAR", "ADD MANIFOLD", "ADD VALVE PANEL" };

EditorTool::EditorTool(SceneManager& scene_manager, OntologyInterface& ontology, Theme& theme, Font& font, float x, float y, Camera& camera, HeightMap& height_map)
	: Frame(theme, font, x, y, 200, 400), scene_manager_(&scene_manager), ontology_(&ontology), camera_(&camera), height_map_(&height_map), selected_structure_(NULL), submitted_(false), current_mode_(IDLE)
{
	GUIManager& gui_manager = GUIManager::getInstance();
	gui_manager.addFrame(*this);
	
	selected_entity_label_ = new Label(theme, 180, 30, "Selected item", 12);
	submit_button_ = new Button(theme, 180, 30, "Submit", 12);
	remove_button_ = new Button(theme, 180, 30, "Remove", 12);
	add_pillar_button_ = new Button(theme, 180, 30, "Add Pillar", 12);
	add_manifold_button_ = new Button(theme, 180, 30, "Add Manifold", 12);
	add_valve_panel_button_ = new Button(theme, 180, 30, "Add Valve Panel", 12);
	
	addElement(*selected_entity_label_, 10, -10);
	addElement(*submit_button_, 10, -42);
	addElement(*remove_button_, 10, -84);
	addElement(*add_pillar_button_, 10, -126);
	addElement(*add_manifold_button_, 10, -168);
	addElement(*add_valve_panel_button_, 10, -210);
	
	submit_button_->addListener(*this);
	remove_button_->addListener(*this);
	add_pillar_button_->addListener(*this);
	add_manifold_button_->addListener(*this);
	add_valve_panel_button_->addListener(*this);
	
	mission_site_ = new MissionSite(*scene_manager_, &height_map, glm::mat4(1.0f), glm::vec3(0, 10, 0), ontology);
}

void EditorTool::buttonPressed(const Button& source)
{
	EDITOR_MODE new_mode = IDLE;
	if (&source == submit_button_ && !submitted_)
	{
		ontology_->removeMissionSite(*mission_site_);
		ontology_->addMissionSite(*mission_site_);
		submitted_ = true;
		std::cout << "Submit the mission site!" << std::endl;
	}
	else if (&source == add_pillar_button_)
	{
		new_mode = ADD_PILLAR;
	}
	else if (&source == add_manifold_button_)
	{
		new_mode = ADD_MANIFOLD;
	}
	else if (&source == add_valve_panel_button_)
	{
		new_mode = ADD_VALVE_PANEL;
	}
	else if (&source == remove_button_)
	{
		mission_site_->removeStructure(*selected_structure_);
		selected_structure_->destroy();
	}
	
	if (new_mode == current_mode_)
	{
		setCurrentMode(IDLE);
	}
	else
	{
		setCurrentMode(new_mode);
	}
 	Frame::buttonPressed(source);
}

void EditorTool::update(float dt)
{
	int mouse_x, mouse_y;
	glfwGetMousePos(&mouse_x, &mouse_y);
	
	if (glfwGetMouseButton(GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS &&
	    (getGlobalX() >= mouse_x || getGlobalX() + getWidth() <= mouse_x ||
	    getGlobalY() >= mouse_y || getGlobalY() + getHeight() <= mouse_y))
	{
		glm::vec3 collision;
		
		if (current_mode_ == ADD_MANIFOLD || 
		    current_mode_ == ADD_PILLAR ||
		    current_mode_ == ADD_VALVE_PANEL)
		{
			if (getPointOnSeaBed(mouse_x, mouse_y, collision))
			{
				std::cout << "Collision at the seabed at: (" << collision.x << ", " << collision.y << ", " << collision.z << ")" << std::endl;
				
				glm::mat4 transformation = glm::translate(glm::mat4(1.0f), collision);
				
				std::vector<InspectionPoint*> inspection_points;
				if (current_mode_ == ADD_MANIFOLD)
				{
					// Initialise the inspection points.
					InspectionPoint* ip1 = new InspectionPoint(Pose(collision.x + 4, collision.y + 2, collision.z, 0, -90), NULL);
					InspectionPoint* ip2 = new InspectionPoint(Pose(collision.x + 4, collision.y + 2, collision.z - 2, 0, -90), NULL);
					InspectionPoint* ip3 = new InspectionPoint(Pose(collision.x + 4, collision.y + 2, collision.z + 2, 0, -90), NULL);
					InspectionPoint* ip4 = new InspectionPoint(Pose(collision.x - 4, collision.y + 2, collision.z + 2, 0, 90), NULL);
					InspectionPoint* ip5 = new InspectionPoint(Pose(collision.x - 4, collision.y + 2, collision.z, 0, 90), NULL);
					InspectionPoint* ip6 = new InspectionPoint(Pose(collision.x - 4, collision.y + 2, collision.z - 2, 0, 90), NULL);
					InspectionPoint* ip7 = new InspectionPoint(Pose(collision.x - 0.5f, collision.y + 2, collision.z + 5, 0, 180), NULL);
					InspectionPoint* ip8 = new InspectionPoint(Pose(collision.x - 0.5f, collision.y + 2, collision.z - 5, 0, 0), NULL);
					
					inspection_points.push_back(ip1);
					inspection_points.push_back(ip2);
					inspection_points.push_back(ip3);
					inspection_points.push_back(ip4);
					inspection_points.push_back(ip5);
					inspection_points.push_back(ip6);
					inspection_points.push_back(ip7);
					inspection_points.push_back(ip8);
					
					std::stringstream ss;
					ss << "Structure" << mission_site_->getStructures().size();
					
					Structure* structure = new Structure(ss.str(), "data/models/Pandora/misc/small_manifold.plf", "data/models/Pandora/misc/atlas/ColourAtlas.tga", *scene_manager_, height_map_, *mission_site_, transformation, inspection_points);
					
					InspectionGoal* inspection_goal = new InspectionGoal(*structure);
					for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
					{
						InspectionPoint* ip = *ci;
						inspection_goal->addInspectionPoint(*ip);
					}
					
					
					Mission* mission = new Mission(*mission_site_);
					mission->addGoal(*inspection_goal);
					mission_site_->addMission(*mission);
					mission_site_->addStructure(*structure);
					setCurrentMode(IDLE);
				} 
				else if (current_mode_ == ADD_PILLAR)
				{
					// Initialise the inspection points.
					InspectionPoint* ip1 = new InspectionPoint(Pose(collision.x, collision.y + 5, collision.z - 1, 0, 0), NULL);
					InspectionPoint* ip2 = new InspectionPoint(Pose(collision.x - 1, collision.y + 5, collision.z, 0, 90), NULL);
					InspectionPoint* ip3 = new InspectionPoint(Pose(collision.x, collision.y + 5, collision.z, 0, 180), NULL);
					InspectionPoint* ip4 = new InspectionPoint(Pose(collision.x + 1, collision.y + 5, collision.z, 0, 270), NULL);
					InspectionPoint* ip5 = new InspectionPoint(Pose(collision.x, collision.y + 10, collision.z, 0, 0), NULL);
					InspectionPoint* ip6 = new InspectionPoint(Pose(collision.x - 1, collision.y + 10, collision.z, 0, 90), NULL);
					InspectionPoint* ip7 = new InspectionPoint(Pose(collision.x, collision.y + 10, collision.z, 0, 180), NULL);
					InspectionPoint* ip8 = new InspectionPoint(Pose(collision.x +1, collision.y + 10, collision.z, 0, 270), NULL);
					
					inspection_points.push_back(ip1);
					inspection_points.push_back(ip2);
					inspection_points.push_back(ip3);
					inspection_points.push_back(ip4);
					inspection_points.push_back(ip5);
					inspection_points.push_back(ip6);
					inspection_points.push_back(ip7);
					inspection_points.push_back(ip8);
					
					Structure* structure = new Structure("Manifold", "data/models/Pandora/misc/damaged_beacon.plf", "data/models/Pandora/misc/damaged_beacon.tga", *scene_manager_, height_map_, *mission_site_, transformation, inspection_points);
					
					InspectionGoal* inspection_goal = new InspectionGoal(*structure);
					for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points.begin(); ci != inspection_points.end(); ++ci)
					{
						InspectionPoint* ip = *ci;
						inspection_goal->addInspectionPoint(*ip);
					}
					
					Mission* mission = new Mission(*mission_site_);
					mission->addGoal(*inspection_goal);
					mission_site_->addMission(*mission);
					mission_site_->addStructure(*structure);
					setCurrentMode(IDLE);
				} 
				else if (current_mode_ == ADD_VALVE_PANEL)
				{
					Structure* structure = new Structure("Manifold", "data/models/Pandora/misc/valve_panel.plf", "data/models/Pandora/misc/atlas/ColourAtlas.tga", *scene_manager_, height_map_, *mission_site_, transformation, inspection_points);
					setCurrentMode(IDLE);
				}
			}
		}
		else
		{
			// Restore the previous materials.
			for (std::vector<std::pair<SceneLeafModel*, const Material*> >::const_iterator ci = org_materials_.begin(); ci != org_materials_.end(); ++ci)
			{
				(*ci).first->setMaterial(*(*ci).second);
			}
			org_materials_.clear();
			
			for (std::vector<Material*>::const_iterator ci = tmp_materials_.begin(); ci != tmp_materials_.end(); ++ci)
			{
				delete *ci;
			}
			tmp_materials_.clear();
			
			Entity* selected_entity = scene_manager_->pickEntity(*camera_, mouse_x, mouse_y);
			if (selected_entity != NULL)
			{
				selected_entity_label_->setLabel(selected_entity->getName());
				
				if (selected_entity->getParent() != NULL && selected_entity->getParent()->getParent() != NULL)
				{
					selected_structure_ = dynamic_cast<Structure*>(selected_entity->getParent()->getParent());
				}
				
				if (selected_structure_ != NULL)
				{
					for (std::vector<SceneLeaf*>::const_iterator ci = selected_entity->getLeafs().begin(); ci != selected_entity->getLeafs().end(); ++ci)
					{
						SceneLeaf* scene_leaf = *ci;
						SceneLeafModel* scene_leaf_model = dynamic_cast<SceneLeafModel*>(scene_leaf);
						if (scene_leaf_model == NULL)
						{
							continue;
						}
						
						MaterialLightProperty ambient(0, 0, 0, 1);
						MaterialLightProperty diffuse(0, 0, 0, 1);
						MaterialLightProperty specular(0, 0, 0, 1);
						MaterialLightProperty emissive(1.0f, 0.0f, 0.0f, 1.0f);
						
						Material* material = new Material(ambient, diffuse, specular, emissive);
						for (std::vector<Texture*>::const_iterator ci = scene_leaf_model->getMaterial().get1DTextures().begin(); ci != scene_leaf_model->getMaterial().get1DTextures().end(); ++ci)
						{
							material->add1DTexture(**ci);
						}
						
						for (std::vector<Texture*>::const_iterator ci = scene_leaf_model->getMaterial().get2DTextures().begin(); ci != scene_leaf_model->getMaterial().get2DTextures().end(); ++ci)
						{
							material->add2DTexture(**ci);
						}
						
						org_materials_.push_back(std::make_pair(scene_leaf_model, &scene_leaf_model->getMaterial()));
						tmp_materials_.push_back(material);
						scene_leaf_model->setMaterial(*material);
					}
				}
			}
			else 
			{
				selected_structure_ = NULL;
			}
		}
	}
	
	Frame::update(dt);
}

bool EditorTool::getPointOnSeaBed(int mouse_x, int mouse_y, glm::vec3& collision)
{
	// Transform the viewport coordinates to the normalised devise coordinates.
	float ndc_mouse_x = (2 * mouse_x) / camera_->getWidth() - 1.0f;
	float ndc_mouse_y = 1.0f - (2 * mouse_y) / camera_->getHeight();
	
	// Transform the normalised device coordinates into homogeneous clip coordinates.
	glm::vec4 hcc_ray(ndc_mouse_x, ndc_mouse_y, -1.0f, 1.0f);
	
	// Transform these into eye coordinates.
	glm::vec4 eye_ray = glm::inverse(camera_->getPerspectiveMatrix()) * hcc_ray;
	eye_ray.z = -1.0f;
	eye_ray.w = 0.0f;
	
	// Transform these into world coordinates.
	glm::vec3 world_coordinates = glm::vec3(glm::inverse(camera_->getViewMatrix()) * eye_ray);
	glm::vec3 direction = glm::normalize(world_coordinates);
	
	CollisionInfo collision_info;
	if (height_map_->doesCollide(*camera_, camera_->getGlobalLocation(), camera_->getGlobalLocation() + direction * 200.0f, collision_info))
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

void EditorTool::setCurrentMode(EDITOR_MODE mode)
{
	current_mode_ = mode;
	selected_entity_label_->setLabel(status_string_[current_mode_]);
}
