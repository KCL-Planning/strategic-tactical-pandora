#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "XMLLevelLoader.h"

#include "dpengine/scene/SceneManager.h"

#include "../ontology/InspectionPoint.h"
#include "../ontology/ValveGoal.h"
#include "../ontology/ChainGoal.h"
#include "../ontology/Ontology.h"

#include "../structures/Structure.h"
#include "../structures/Valve.h"
#include "../structures/Chain.h"
#include "../level/MissionSite.h"
#include "../level/Mission.h"

std::ostream& operator<<(std::ostream& os, const StructureDescription& sd)
{
	os << sd.name_ << "(" << sd.file_name_ << "); Can recharge? " << sd.can_recharge_ << std::endl;
	for (std::vector<Pose>::const_iterator ci = sd.inspection_points_.begin(); ci != sd.inspection_points_.end(); ++ci)
	{
		os << *ci << std::endl;
	}
	return os;
}


XMLLevelLoader::XMLLevelLoader(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode& root, Ontology& ontology, HeightMap& height_map)
	: scene_manager_(&scene_manager), root_(&root), ontology_(&ontology), height_map_(&height_map)
{
	
}

void XMLLevelLoader::loadLevel(const std::string& xml_file)
{
	auv_location_ = glm::translate(auv_location_, glm::vec3(0, 3, 0));
	pugi::xml_document doc;
	pugi::xml_parse_result result = doc.load_file(xml_file.c_str());
	
	if (result)
	{
		std::cout << "XML [" << xml_file << "] parsed without errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n\n";
	}
	else
	{
		std::cout << "XML [" << xml_file << "] parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n";
		std::cout << "Error description: " << result.description() << "\n";
		std::cout << "Error offset: " << result.offset << " (error at [..." << (xml_file.c_str() + result.offset) << "]\n\n";
	}
	
	const pugi::xml_node& root = *doc.root().begin();
	printXMLNode(root);
	
	// Load the structures.
	bool separate_inspection_goals = false;
	for (pugi::xml_node child_node = root.first_child(); child_node; child_node = child_node.next_sibling())
	{
		std::cout << "- " << child_node.name() << std::endl;
		if (std::string(child_node.name()) == "structures")
		{
			pugi::xml_attribute structure_name_attribute = child_node.attribute("separate_inspection_goals");
			if (structure_name_attribute != 0 && "true" == std::string(structure_name_attribute.as_string()))
			{
				separate_inspection_goals = true;
				std::cout << "Separate the inspection points!" << std::endl;
			}
			loadStructures(child_node);
		}
		
		if (std::string(child_node.name()) == "world")
		{
			std::cout << "-- LOAD THE WORLD!!!!!!!!!!!!!!!" << std::endl;
			loadWorld(child_node, separate_inspection_goals);
		}
	}
	/*
	// Load the world.
	std::cout << "Check out the world!" << std::endl;
	for (pugi::xml_node child_node = root.first_child(); child_node; child_node = child_node.next_sibling())
	{
		std::cout << "*** " << child_node.name() << std::endl;
		if (std::string(child_node.name()) == "world")
		{
			std::cout << "-- LOAD THE WORLD!!!!!!!!!!!!!!!" << std::endl;
			loadWorld(child_node);
		}
	}
	*/
}

void XMLLevelLoader::loadStructures(const pugi::xml_node& node) 
{
	std::cout << "[ XMLLevelLoader::loadStructures]" << std::endl;
	
	for (pugi::xml_node structure_node = node.first_child(); structure_node; structure_node = structure_node.next_sibling())
	{
		StructureDescription structure_description;
		std::string id = structure_node.attribute("id").as_string();
		if ("" == id)
		{
			std::cerr << "Missing the ID attribute for the node: " << node.name() << std::endl;
		}

		pugi::xml_attribute structure_name_attribute = structure_node.attribute("name");
		pugi::xml_attribute can_recharge_attribute = structure_node.attribute("can_recharge");
		pugi::xml_node structure_file_name_node = structure_node.child("filename");
		pugi::xml_node structure_texture_node = structure_node.child("texture");
		pugi::xml_node structure_inspection_points_node = structure_node.child("inspection_points");
		
		if (structure_name_attribute == 0)
		{
			std::cerr << "Could not find the name!" << std::endl;
		}
		if (structure_file_name_node == 0)
		{
			std::cerr << "Could not find the file name!" << std::endl;
		}
		if (structure_texture_node == 0)
		{
			std::cerr << "Could not find the texture!" << std::endl;
		}
		
		if (can_recharge_attribute != 0 && "true" == std::string(can_recharge_attribute.as_string()))
		{
			structure_description.can_recharge_ = true;
		}
		else
		{
			structure_description.can_recharge_ = false;
		}
		
		structure_description.name_ = structure_name_attribute.as_string();
		structure_description.file_name_ = structure_file_name_node.child_value();
		structure_description.texture_ = structure_texture_node.child_value();
		if (structure_inspection_points_node != 0)
		{
			std::cout << "Found inspection points!" << std::endl;
			for (pugi::xml_node inspection_point_node = structure_inspection_points_node.first_child(); inspection_point_node; inspection_point_node = inspection_point_node.next_sibling())
			{
				float x = inspection_point_node.attribute("x").as_float();
				float y = inspection_point_node.attribute("y").as_float();
				float z = inspection_point_node.attribute("z").as_float();
				float pitch = inspection_point_node.attribute("pitch").as_float();
				float yaw = inspection_point_node.attribute("yaw").as_float();
				
				Pose pose(x, y ,z, pitch, yaw);
				structure_description.inspection_points_.push_back(pose);
				std::cout << "* " << pose << std::endl;
			}
		}
		
		std::cout << "Loaded structure: [" << id << "] " << structure_description << std::endl;
		structure_mapping_[id] = structure_description;
	}
}

void XMLLevelLoader::loadWorld(const pugi::xml_node& node, bool separate_inspection_goals)
{
	std::cout << "[ XMLLevelLoader::loadWorld]" << std::endl;
	std::cout << "Process: " << node.name() << std::endl;
	for (pugi::xml_node mission_sites_node = node.first_child(); mission_sites_node; mission_sites_node = mission_sites_node.next_sibling())
	{
		std::cout << "Process: " << mission_sites_node.name() << " (expecting auv_location)" << std::endl;
		if ("auv_location" == std::string(mission_sites_node.name()))
		{
			std::cerr << "Processing auvs node name." << std::endl;
		
			float auv_location_x = mission_sites_node.attribute("x").as_float();
			float auv_location_y = mission_sites_node.attribute("y").as_float();
			float auv_location_z = mission_sites_node.attribute("z").as_float();
			float auv_location_pitch = mission_sites_node.attribute("pitch").as_float();
			float auv_location_yaw = mission_sites_node.attribute("yaw").as_float();
			Pose auv_location(auv_location_x, auv_location_y , auv_location_z, auv_location_pitch, auv_location_yaw);
			
			std::cout << "Structure location: " << auv_location << std::endl;
			
			auv_location_ = glm::mat4(1.0f);
			auv_location_ = glm::translate(auv_location_, glm::vec3(auv_location_x, auv_location_y, auv_location_z));
			auv_location_ = glm::rotate(auv_location_, glm::radians(auv_location_pitch), glm::vec3(1, 0, 0));
			auv_location_ = glm::rotate(auv_location_, glm::radians(auv_location_yaw), glm::vec3(0, 1, 0));
		}
		
		
		std::cout << "Process: " << mission_sites_node.name() << " (expecting mission_sites)" << std::endl;
		if ("mission_sites" != std::string(mission_sites_node.name()))
		{
			std::cerr << "Skipping unknown node name: " << mission_sites_node.name() << std::endl;
			continue;
		}
		
		for (pugi::xml_node mission_site_node = mission_sites_node.first_child(); mission_site_node; mission_site_node = mission_site_node.next_sibling())
		{
			std::cout << "Process: " << mission_site_node.name() << " (expecting mission_site)" << std::endl;
			if ("mission_site" != std::string(mission_site_node.name()))
			{
				std::cerr << "Skipping unknown node name: " << mission_site_node.name() << std::endl;
				continue;
			}
			
			pugi::xml_node location_node = mission_site_node.child("location");
			pugi::xml_node start_waypoint_node = mission_site_node.child("start_waypoint");
			
			if (location_node == 0 || start_waypoint_node == 0)
			{
				std::cerr << "Mission sites quire a location AND start_waypoint node." << std::endl;
				continue;
			}
			
			float location_x = location_node.attribute("x").as_float();
			float location_y = location_node.attribute("y").as_float();
			float location_z = location_node.attribute("z").as_float();
			float location_pitch = location_node.attribute("pitch").as_float();
			float location_yaw = location_node.attribute("yaw").as_float();
			Pose location(location_x, location_y , location_z, location_pitch, location_yaw);
			
			glm::mat4 mission_site_transformation(1.0f);
			mission_site_transformation = glm::translate(mission_site_transformation, glm::vec3(location_x, location_y, location_z));
			mission_site_transformation = glm::rotate(mission_site_transformation, glm::radians(location_pitch), glm::vec3(1, 0, 0));
			mission_site_transformation = glm::rotate(mission_site_transformation, glm::radians(location_yaw), glm::vec3(0, 1, 0));	
			
			float start_waypoint_x = start_waypoint_node.attribute("x").as_float();
			float start_waypoint_y = start_waypoint_node.attribute("y").as_float();
			float start_waypoint_z = start_waypoint_node.attribute("z").as_float();
			glm::vec3 start_waypoint(start_waypoint_x, start_waypoint_y, start_waypoint_z);
			
			glm::mat4 mission_site_location(1.0f);
			mission_site_location = glm::translate(mission_site_location, glm::vec3(location_x, location_y, location_z));
			mission_site_location = glm::rotate(mission_site_location, glm::radians(location_pitch), glm::vec3(1, 0, 0));
			mission_site_location = glm::rotate(mission_site_location, glm::radians(location_yaw), glm::vec3(0, 1, 0));
			
			MissionSite* mission_site = new MissionSite(*scene_manager_, root_, mission_site_location, start_waypoint, *ontology_);
			
			std::cout << "Location: " << location << std::endl;
			std::cout << "Start waypoint: (" << start_waypoint.x << ", " << start_waypoint.y << ", " << start_waypoint.z << ")" << std::endl;
			
			std::map<std::string, Structure*> structure_mappings;
			std::map<std::string, Valve*> valve_mappings;
			
			// Process the structures.
			pugi::xml_node structures_node = mission_site_node.child("structures");
			if (structures_node == 0)
			{
				std::cout << "No structures found!" << std::endl;
			}
			else
			{
				for (pugi::xml_node structure_node = structures_node.first_child(); structure_node; structure_node = structure_node.next_sibling())
				{
					std::cout << "Process: " << structure_node.name() << " (expecting structure)" << std::endl;
					if ("structure" != std::string(structure_node.name()))
					{
						std::cerr << "Skipping unknown node name: " << structure_node.name() << std::endl;
						continue;
					}
					
					pugi::xml_attribute ref_attribute = structure_node.attribute("ref");
					pugi::xml_attribute id_attribute = structure_node.attribute("id");
					
					if (ref_attribute == 0 || id_attribute == 0)
					{
						std::cerr << "Structures must have an id attribute AND a ref attribute!" << std::endl;
						continue;
					}
					
					std::string id = id_attribute.as_string();
					std::string ref = ref_attribute.as_string();
					
					std::cout << "Structure id: " << id << " reference: " << ref << std::endl;
					
					std::map<std::string, StructureDescription>::const_iterator structure_description_it = structure_mapping_.find(ref);
					if (structure_description_it == structure_mapping_.end())
					{
						std::cerr << "Could not find the structure description for: " << ref << std::endl;
						continue;
					}
					StructureDescription structure_description = structure_mapping_[ref];
					
					pugi::xml_node structure_location_node = structure_node.child("location");
					if (structure_location_node == 0)
					{
						std::cerr << "Structures must have a location node!" << std::endl;
						continue;
					}
					float structure_location_x = structure_location_node.attribute("x").as_float();
					float structure_location_y = structure_location_node.attribute("y").as_float();
					float structure_location_z = structure_location_node.attribute("z").as_float();
					float structure_location_pitch = structure_location_node.attribute("pitch").as_float();
					float structure_location_yaw = structure_location_node.attribute("yaw").as_float();
					Pose structure_location(structure_location_x, structure_location_y , structure_location_z, structure_location_pitch, structure_location_yaw);
					
					std::cout << "Structure location: " << structure_location << std::endl;
					
					glm::mat4 transformation(1.0f);
					transformation = glm::translate(transformation, glm::vec3(structure_location_x, structure_location_y, structure_location_z));
					transformation = glm::rotate(transformation, glm::radians(location_pitch), glm::vec3(1, 0, 0));
					transformation = glm::rotate(transformation, glm::radians(location_yaw), glm::vec3(0, 1, 0));
					
					transformation = mission_site_transformation * transformation;
					
					std::vector<InspectionPoint*> inspection_points;
					for (std::vector<Pose>::const_iterator ci = (*structure_description_it).second.inspection_points_.begin(); ci != (*structure_description_it).second.inspection_points_.end(); ++ci)
					{
						Pose pose = *ci;
						glm::vec4 point(pose.x_, pose.y_, pose.z_, 1.0f);
						point = transformation * point;
						pose.x_ = point.x;
						pose.y_ = point.y;
						pose.z_ = point.z;
						
						InspectionPoint* inspection_point = new InspectionPoint(pose);
						inspection_points.push_back(inspection_point);
					}
					Structure* structure = new Structure(id, structure_description.file_name_, structure_description.texture_, *scene_manager_, root_, *mission_site, transformation, inspection_points, separate_inspection_goals);
					structure->setCanRecharge(structure_description.can_recharge_);
					mission_site->addStructure(*structure);
					
					structure_mappings[id] = structure;
					pugi::xml_node valves_node = structure_node.child("valves");
					if (valves_node != 0)
					{
						std::cout << "Valves detected!" << std::endl;
						for (pugi::xml_node valve_node = valves_node.first_child(); valve_node; valve_node = valve_node.next_sibling())
						{
							if ("valve" != std::string(valve_node.name()))
							{
								std::cerr << "Skipping unknown node name: " << structure_node.name() << std::endl;
								continue;
							}
							
							pugi::xml_attribute valve_id_attribute = valve_node.attribute("id");
							if (valve_id_attribute == 0)
							{
								std::cerr << "Valves must have a valve id!" << std::endl;
								continue;
							}
							
							std::string valve_id = valve_id_attribute.as_string();
							
							std::cout << "Valve id: " << valve_id << std::endl;
							
							pugi::xml_node valve_location_node = valve_node.child("location");
							if (valve_location_node == 0)
							{
								std::cerr << "Valves must have a location node!" << std::endl;
								continue;
							}
							float valve_location_x = valve_location_node.attribute("x").as_float();
							float valve_location_y = valve_location_node.attribute("y").as_float();
							float valve_location_z = valve_location_node.attribute("z").as_float();
							float valve_location_pitch = valve_location_node.attribute("pitch").as_float();
							float valve_location_yaw = valve_location_node.attribute("yaw").as_float();
							Pose valve_location(valve_location_x, valve_location_y , valve_location_z, valve_location_pitch, valve_location_yaw);
							std::cout << "Valve location: " << valve_location << std::endl;
							
							std::map<std::string, Valve*>::const_iterator valve_it = valve_mappings.find(valve_id);
							if (valve_it != valve_mappings.end())
							{
								std::cerr << "A valve with the ID: " << valve_id << " already exists, ignoring this one!" << std::endl;
								continue;
							}
							
							glm::mat4 valve_matrix(1.0f);
							valve_matrix = glm::translate(valve_matrix, glm::vec3(valve_location_x, valve_location_y, valve_location_z));
							valve_matrix = glm::rotate(valve_matrix, glm::radians(valve_location_pitch), glm::vec3(1, 0, 0));
							valve_matrix = glm::rotate(valve_matrix, glm::radians(valve_location_yaw), glm::vec3(0, 1, 0));
							
							valve_matrix = valve_matrix;
							
							Valve* valve = new Valve(*structure, *scene_manager_, structure, valve_matrix, valve_id);
							structure->addValve(*valve);
							valve_mappings[valve_id] = valve;
						}
					}
				}
			}
			
			// Chains
			pugi::xml_node chains_node = mission_site_node.child("chains");
			if (chains_node == 0)
			{
				std::cout << "The site has no chains." << std::endl;
			}

			for (pugi::xml_node chain_node = chains_node.first_child(); chain_node; chain_node = chain_node.next_sibling())
			{
				std::cout << "Process: " << chain_node.name() << " (expecting chain)" << std::endl;
				if ("chain" != std::string(chain_node.name()))
				{
					std::cerr << "Skipping unknown node name: " << chain_node.name() << std::endl;
					continue;
				}
				
				pugi::xml_node chain_location_node = chain_node.child("location");
				if (chain_location_node == 0)
				{
					std::cerr << "The location of the chain is not specified!" << std::endl;
					continue;
				}
				float chain_location_x = chain_location_node.attribute("x").as_float();
				float chain_location_y = chain_location_node.attribute("y").as_float();
				float chain_location_z = chain_location_node.attribute("z").as_float();
				float chain_location_pitch = chain_location_node.attribute("pitch").as_float();
				float chain_location_yaw = chain_location_node.attribute("yaw").as_float();
				Pose chain_location(chain_location_x, chain_location_y , chain_location_z, chain_location_pitch, chain_location_yaw);
				std::cout << "Chain location: " << chain_location << std::endl;
				
				glm::mat4 chain_matrix = glm::translate(glm::mat4(1.0f), glm::vec3(chain_location_x, chain_location_y, chain_location_z));
				chain_matrix = glm::rotate(chain_matrix, glm::radians(chain_location_pitch), glm::vec3(1, 0, 0));
				chain_matrix = glm::rotate(chain_matrix, glm::radians(chain_location_yaw), glm::vec3(0, 1, 0));
				
				chain_matrix = chain_matrix;
				
				std::cout << "Absolute Chain location: (" << chain_matrix[3][0] << ", " << chain_matrix[3][1] << ", " << chain_matrix[3][2] << ")" << std::endl;
				
				pugi::xml_attribute chain_nr_links_attribute = chain_node.attribute("chain_links");
				if (chain_nr_links_attribute == 0)
				{
					std::cerr << "The number of chain links is not specified!" << std::endl;
					continue;
				}
				
				Chain* chain = new Chain(*mission_site, *scene_manager_, mission_site, *height_map_, chain_matrix, chain_nr_links_attribute.as_uint());
				mission_site->addChain(*chain);
				
				Mission* chain_following_mission = new Mission(*mission_site);
				chain_following_mission->addGoal(chain->getGoal());
				mission_site->addMission(*chain_following_mission);
				
				pugi::xml_attribute has_been_observed_attribute = chain_node.attribute("has_been_observed");
				if (has_been_observed_attribute != 0 && "true" == std::string(has_been_observed_attribute.as_string()))
				{
					chain->setObserved();
 				}
			}
			
			pugi::xml_node missions_node = mission_site_node.child("missions");
			if (missions_node == 0)
			{
				std::cout << "The site has no missions." << std::endl;
			}

			for (pugi::xml_node mission_node = missions_node.first_child(); mission_node; mission_node = mission_node.next_sibling())
			{
				std::cout << "Process: " << mission_node.name() << " (expecting mission)" << std::endl;
				if ("mission" != std::string(mission_node.name()))
				{
					std::cerr << "Skipping unknown node name: " << mission_node.name() << std::endl;
					continue;
				}
				Mission* mission = new Mission(*mission_site);
				mission_site->addMission(*mission);
				
				for (pugi::xml_node goals_node = mission_node.first_child(); goals_node; goals_node = goals_node.next_sibling())
				{
					std::cout << "Process: " << goals_node.name() << " (expecting goals)" << std::endl;
					if ("goals" != std::string(goals_node.name()))
					{
						std::cerr << "Skipping unknown node name: " << goals_node.name() << std::endl;
						continue;
					}
					
					for (pugi::xml_node goal_node = goals_node.first_child(); goal_node; goal_node = goal_node.next_sibling())
					{
						std::cout << "Process: " << goal_node.name() << " (expecting goal)" << std::endl;
						if ("goal" != std::string(goal_node.name()))
						{
							std::cerr << "Skipping unknown node name: " << goal_node.name() << std::endl;
							continue;
						}
						
						pugi::xml_node goal_type_node = goal_node.child("type");
						pugi::xml_node goal_structure_node = goal_node.child("structure");
						
						if (goal_type_node == 0 || goal_structure_node == 0)
						{
							std::cerr << "For every goal the nodes <type> AND <structure> must be declared." << std::endl;
							continue;
						}
						//Mission* mission = new Mission(*mission_site);
						//mission_site->addMission(*mission);
						
						std::string goal_type = goal_type_node.child_value();
						std::string goal_structure = goal_structure_node.child_value();
						
						std::map<std::string, Structure*>::const_iterator map_iterator = structure_mappings.find(goal_structure);
						if (map_iterator == structure_mappings.end())
						{
							std::cerr << "The structure " << goal_structure << " has not been defined in the XML document!" << std::endl;
							continue;
						}
						
						Structure* structure = (*map_iterator).second;
						
						if (goal_type == "valve_turning_mission")
						{
							pugi::xml_node goal_valve_node = goal_node.child("valve");
							pugi::xml_node goal_start_time_node = goal_node.child("start_time");
							pugi::xml_node goal_dead_line_node = goal_node.child("dead_line");
							pugi::xml_node goal_valve_angle_node = goal_node.child("valve_angle");
							
							if (goal_valve_node == 0 || goal_start_time_node == 0 || goal_dead_line_node == 0 || goal_valve_angle_node == 0)
							{
								std::cerr << "For valve_turning_missions the nodes <valve>, <start_time>, <dead_line>, AND <<valve_angle> must be declared." << std::endl;
								continue;
							}
							std::cout << "Create a valve turning mission!" << std::endl;
							std::string valve_id = goal_valve_node.child_value();
							float start_time = ::atof(goal_start_time_node.child_value());
							float dead_line = ::atof(goal_dead_line_node.child_value());
							float valve_angle = ::atof(goal_valve_angle_node.child_value());
							
							// Find the valve.
							std::map<std::string, Valve*>::const_iterator valve_it = valve_mappings.find(valve_id);
							if (valve_it == valve_mappings.end())
							{
								std::cerr << "Could not find the valve: " << valve_id << "." << std::endl;
								continue;
							}
							
							Valve* valve = (*valve_it).second;
							ValveGoal* valve_goal = new ValveGoal(*structure, *valve, start_time, dead_line, valve_angle);
							valve->addValveGoal(*valve_goal);
							std::cerr << "Valve goals: " << valve->getName() << ", " << start_time << ", " << dead_line << ", " << valve_angle << std::endl;
							
							if (separate_inspection_goals && mission->getGoals().size() > 0)
							{
								mission = new Mission(*mission_site);
								mission_site->addMission(*mission);
							}
							
							mission->addGoal(*valve_goal);
							
							scene_manager_->addUpdateableEntity(*valve);
						}
						else if (goal_type == "inspection_mission")
						{
							std::cout << "Create an inspection mission!" << std::endl;
							for (std::vector<InspectionGoal*>::const_iterator ci = structure->getInspectionGoals().begin(); ci != structure->getInspectionGoals().end(); ++ci)
							{
								mission = new Mission(*mission_site);
								mission_site->addMission(*mission);
								mission->addGoal(**ci);
							}
							
							/*
							InspectionGoal* inspection_goal = new InspectionGoal(*structure, structure->getInspectionPoints());
							if (separate_inspection_goals && mission->getGoals().size() > 0)
							{
								mission = new Mission(*mission_site);
								mission_site->addMission(*mission);
							}
							mission->addGoal(*inspection_goal);
							*/
						}
						else
						{
							std::cerr << "Unknown mission: " << goal_type << std::endl;
							continue;
						}
					}
				}
			}
			
			for (std::map<std::string, Structure*>::const_iterator ci = structure_mappings.begin(); ci != structure_mappings.end(); ++ci)
			{
				mission_site->addStructure(*(*ci).second);
			}
			ontology_->addMissionSite(*mission_site);
			std::cerr << "Add the mission site: " << mission_site->getId() << std::endl;
		}
	}
}

void XMLLevelLoader::printXMLNode(const pugi::xml_node& node, unsigned int depth) const
{
	for (unsigned int i = 0; i < depth; ++i)
	{
		std::cout << "\t";
	}
	std::cout << "<" << node.name();
	for (pugi::xml_attribute attr = node.first_attribute(); attr; attr = attr.next_attribute())
	{
		std::cout << " " << attr.name() << "=" << attr.value();
	}
	std::cout << ">";// << node.value() << "(Child value: " << node.child_value() << ")";
	const char* empty_string = PUGIXML_TEXT("");
	if (node.child_value() == empty_string && node.children().begin() != node.children().end())
	{
		std::cout << std::endl;
		
		// Lets load the structures first.
		for (pugi::xml_node tool = node.first_child(); tool; tool = tool.next_sibling())
		{
			printXMLNode(tool, depth + 1);
		}
		for (unsigned int i = 0; i < depth; ++i)
		{
			std::cout << "\t";
		}
	}
	else 
	{
		std::cout << node.child_value();
	}
	std::cout << "</" << node.name() << ">" << std::endl;
}
