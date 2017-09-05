#include "ProblemGenerator.h"

ProblemGenerator::ProblemGenerator(SceneManager& scene_manager, SceneNode& terrain_node)
	: scene_manager_(&scene_manager), terrain_node_(&terrain_node)
{
	
}

void ProblemGenerator::generateProblem(unsigned int nr_valves, unsigned int nr_chains, unsigned int nr_inspection_missions)
{
	
}

void ProblemGenerator::generateMissionSite(unsigned int nr_valves, unsigned int nr_chains, unsigned int nr_inspection_missions)
{
	MissionSite* mission_site = new MissionSite(*scene_manager_, terrain_node_, glm::scale(glm::translate(glm::mat4(1.0f), glm::vec3(-55, 0.25f, -35)), glm::vec3(1, 1, 1)), "data/models/Pandora/misc/small_manifold.plf");
	
	InspectionGoal* inspection_goal = new InspectionGoal(*mission_site);
	Mission* inspection_mission = new Mission(*mission_site);
	inspection_mission->addGoal(*inspection_goal);
	mission_site->addMission(*inspection_mission);

	InspectionPoint* ip1 = new InspectionPoint(Pose(mission_site->getLocalLocation().x + 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z, 0, -90), NULL);
	InspectionPoint* ip2 = new InspectionPoint(Pose(mission_site->getLocalLocation().x + 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z - 2, 0, -90), NULL);
	InspectionPoint* ip3 = new InspectionPoint(Pose(mission_site->getLocalLocation().x + 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z + 2, 0, -90), NULL);
	InspectionPoint* ip4 = new InspectionPoint(Pose(mission_site->getLocalLocation().x - 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z + 2, 0, 90), NULL);
	InspectionPoint* ip5 = new InspectionPoint(Pose(mission_site->getLocalLocation().x - 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z, 0, 90), NULL);
	InspectionPoint* ip6 = new InspectionPoint(Pose(mission_site->getLocalLocation().x - 4, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z - 2, 0, 90), NULL);
	InspectionPoint* ip7 = new InspectionPoint(Pose(mission_site->getLocalLocation().x - 0.5f, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z + 5, 0, 180), NULL);
	InspectionPoint* ip8 = new InspectionPoint(Pose(mission_site->getLocalLocation().x - 0.5f, mission_site->getLocalLocation().y + 2, mission_site->getLocalLocation().z - 5, 0, 0), NULL);
	
	mission_site->addInspectionPoint(*ip1);
	mission_site->addInspectionPoint(*ip2);
	mission_site->addInspectionPoint(*ip3);
	mission_site->addInspectionPoint(*ip4);
	mission_site->addInspectionPoint(*ip5);
	mission_site->addInspectionPoint(*ip6);
	mission_site->addInspectionPoint(*ip7);
	mission_site->addInspectionPoint(*ip8);
	
	inspection_goal->addInspectionPoint(*ip1);
	inspection_goal->addInspectionPoint(*ip2);
	inspection_goal->addInspectionPoint(*ip3);
	inspection_goal->addInspectionPoint(*ip4);
	inspection_goal->addInspectionPoint(*ip5);
	inspection_goal->addInspectionPoint(*ip6);
	inspection_goal->addInspectionPoint(*ip7);
	inspection_goal->addInspectionPoint(*ip8);
	
	// Add a valve panel.
	ValvePanel* value_panel = new ValvePanel(*scene_manager_, terrain_node_, glm::translate(glm::mat4(1.0f), glm::vec3(-5.0f, 5.0f, 0.0f) + glm::vec3(mission_site->getLocalLocation())), "Valve Panel", *grass_texture);
	Valve* valve = new Valve(*value_panel, *scene_manager_, value_panel, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 0.0f)), "Valve Panel", *grass_texture);
	ValveGoal* vg = new ValveGoal(*mission_site, *valve, 10, 1500, M_PI / 2.0f);
	valve->addValveGoal(*vg);
	value_panel->addValve(*valve);
	
	//Mission* valve_turning_mission = new Mission(*mission_site);
	//valve_turning_mission->addGoal(*vg);
	//mission_site->addMission(*valve_turning_mission);
	inspection_mission->addGoal(*vg);
	
	scene_manager_->addUpdateableEntity(*valve);
	
	mission_site->addPanel(*value_panel);
	
	ontology_->addMissionSite(*mission_site);
}
