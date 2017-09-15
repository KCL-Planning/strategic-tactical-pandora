#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

#include "VisualiserTimer.h"

namespace PandoraKCL {

	/**
	 * Generate a PDDL problem file, saving the result in dataPath/pandora_problem.pddl.
	 * This file is later read by the planner, and the result saved in dataPath/plan.pddl.
	 */
	void generatePDDLProblemFile(std::string &dataPath, bool useDeadlines, std::string &mission)
	{

		//int startTime = (int)(ros::WallTime::now().toSec() - missionStart);
		int startTime = (int)(VisualiserTimer::getTime() - missionStart);

		/*--------*/
		/* header */
		/*--------*/

		ROS_INFO("KCL: Generating PDDL problem file");
		std::ofstream pFile;
		pFile.open((dataPath + "pandora_problem_" + mission + ".pddl").c_str());

		pFile << "(define (problem pandora_mission_task)" << std::endl;
		pFile << "(:domain pandora_domain_persistent)" << std::endl;

		/* objects */

		pFile << "(:objects" << std::endl;

		// vehicles
		for(int i=0;i<auv_names.size();i++)
			pFile << auv_names[i] << " ";
		pFile << "- vehicle" << std::endl;

		// panels
		if(PandoraKCL::panelPositions.size()>0) {
			for (std::map<std::string,Point3DQuat>::iterator it=PandoraKCL::panelPositions.begin(); it!=PandoraKCL::panelPositions.end(); ++it)
				pFile << it->first << " ";
			pFile << "- panel" << std::endl;
		}

		// valves
		if(PandoraKCL::valveAngles.size()>0) {
			for (std::map<std::string,double>::iterator it=PandoraKCL::valveAngles.begin(); it!=PandoraKCL::valveAngles.end(); ++it)
				pFile << it->first << " ";
			pFile << "- valve" << std::endl;
		}

		// pillars
		if(PandoraKCL::pillars.size()>0) {
			for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::pillars.begin(); iit!=PandoraKCL::pillars.end(); ++iit)
				pFile << iit->first << " ";
			pFile << "- pillar" << std::endl;
		}

		// chains
		if(chain_positions.size()>0) {
			for (std::map<std::string,std::string>::iterator iit=chain_positions.begin(); iit!=chain_positions.end(); ++iit)
				pFile << iit->first << " ";
			pFile << "- chain" << std::endl;
		}

		// inspection points
		if(PandoraKCL::inspectionPoints.size()>0) {
			for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::inspectionPoints.begin(); iit!=PandoraKCL::inspectionPoints.end(); ++iit)
				pFile << iit->first << " ";
			pFile << "- inspectionpoint" << std::endl;
		}

		// waypoints
		for (std::map<std::string,Point3D>::iterator it=PandoraKCL::waypoints.begin(); it!=PandoraKCL::waypoints.end(); ++it)
			pFile << it->first << " ";
		pFile << "- Waypoint" << std::endl;;

		pFile << ")" << std::endl;

		/*---------------*/
		/* initial state */
		/*---------------*/

		pFile << "(:init" << std::endl;

		for(int i=0;i<auv_names.size();i++) {
			pFile << "(= (arm_calibration " << auv_names[i] << ") " << arm_calibration[auv_names[i]] << ")" << std::endl;
			pFile << "(not_illuminating " << auv_names[i] << ")" << std::endl;
		}

		// position
		if (waypoints.size()>0) {
			if(PandoraKCL::panelPositions.empty()) {
				for(int i=0;i<auv_names.size();i++)
					pFile << "(near " << auv_names[i] << " " << auv_starting_locations[auv_names[i]] << ")" << std::endl;
			} else {
				for(int i=0;i<auv_names.size();i++)
					pFile << "(at " << auv_names[i] << " " << auv_starting_locations[auv_names[i]] << ")" << std::endl;
			}
			pFile << std::endl;

			std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin();
			for (; wit!=PandoraKCL::waypoints.end(); ++wit) {
				bool occupied = false;
				for(int i=0;i<auv_names.size();i++) {
					if(wit->first==auv_starting_locations[auv_names[i]])
						occupied = true;
				}
				if(!occupied)
					pFile << "(waypoint_not_occupied " << wit->first << ")" << std::endl;
			}
			pFile << std::endl;
		}

		// valves
		for (std::map<std::string,std::string>::iterator it=PandoraKCL::valves.begin(); it!=PandoraKCL::valves.end(); ++it) {
			pFile << "(on " << it->first << " " << it->second << ")" << std::endl;
			pFile << "(= (valve_state " << it->first << ") " << valveAngles[it->first] << ")" << std::endl;
			pFile << "(= (valve_goal_completed " << it->first << ") 0)" << std::endl;
			if(useDeadlines) pFile << "(valve_blocked " << it->first << ")" << std::endl;
			else pFile << "(valve_free " << it->first << ")" << std::endl; 
			pFile << std::endl;
		}

		// valve time windows
		if(useDeadlines) {
			for (size_t i = 0; i<valveGoals.size(); i++) {
				std::string valveID = valveGoals[i].valve_id;
				
				std::cout << "Process the valve: " << valveID << std::endl;
				std::cout << (PandoraKCL::valves.find(valveID) !=  PandoraKCL::valves.end() ? "Valve is there, we are OK!" : "Could not find the valve!") << std::endl;
				std::cout << startTime << " < " << valveGoals[i].deadline << std::endl;
				std::cout << lastCompletedDeadline[valveID] << " < " << valveGoals[i].deadline << std::endl;
				
				if(PandoraKCL::valves.find(valveID) != PandoraKCL::valves.end()
						&& startTime < valveGoals[i].deadline
						&& lastCompletedDeadline[valveID] < valveGoals[i].deadline) {
					pFile << ";; VALVE TIME WINDOW" << std::endl;
					// free and angle
					double window_open = valveGoals[i].startTime - startTime;
					if(window_open < 1 && (valveGoals[i].deadline - startTime) > 1) window_open = 1;
					pFile << "(at " << window_open << " (not (valve_blocked " << valveID << ")))" << std::endl;
					pFile << "(at " << window_open << " (valve_free " << valveID << "))" << std::endl;
					pFile << "(at " << window_open << " (= (valve_goal " << valveID << ") " << valveGoals[i].valve_angle << "))" << std::endl;
					//blocked
					pFile << "(at " << (valveGoals[i].deadline - startTime) << " (not (valve_free " << valveID << ")))" << std::endl;
					pFile << "(at " << (valveGoals[i].deadline - startTime) << " (valve_blocked " << valveID << "))" << std::endl;
					pFile << std::endl;
				}
			}
		}

		// panel visibility and reachability
		for (std::map<std::string,std::string>::iterator pit=panelVisibility.begin(); pit!=panelVisibility.end(); ++pit) {
			for(int i=0;i<auv_names.size();i++)
				pFile << "(canexamine " << auv_names[i] << " " << pit->first << " " << pit->second << ")" << std::endl;
		}
		for (std::map<std::string,std::string>::iterator pit=panelReachability.begin(); pit!=panelReachability.end(); ++pit) {
			for(int i=0;i<auv_names.size();i++)
				pFile << "(canreach " << auv_names[i] << " " << pit->first << " " << pit->second << ")" << std::endl;
		}

		// pillars
		for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::pillars.begin(); iit!=PandoraKCL::pillars.end(); ++iit) {
			if(PandoraKCL::pillarsObserved.find(iit->first) != PandoraKCL::pillarsObserved.end()
					&& PandoraKCL::pillarsObserved[iit->first])
				pFile << "(observed_pillar " << iit->first << ")" << std::endl;
		}

		// pillar visibility
		for (std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin(); wit!=PandoraKCL::waypoints.end(); ++wit) {
		for (size_t i=0; i<pillarVisibility[wit->first].size(); i++) {
			if(PandoraKCL::pillars.find(pillarVisibility[wit->first][i]) != PandoraKCL::pillars.end()) {
				for(int j=0;j<auv_names.size();j++)
					pFile << "(cansee_pillar " << auv_names[j] << " " << wit->first << " " << pillarVisibility[wit->first][i] << ") " << std::endl;
			}
		}};

		// inspection points
		for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::inspectionPoints.begin(); iit!=PandoraKCL::inspectionPoints.end(); ++iit) {
			if(PandoraKCL::inspectionPointsInspected.find(iit->first) != PandoraKCL::inspectionPointsInspected.end()
				&& PandoraKCL::inspectionPointsInspected[iit->first])
				pFile << "(= (observed " << iit->first << ") 1)" << std::endl;
			else pFile << "(= (observed " << iit->first << ") 0)" << std::endl;
		}

		// inspection visibility
		for (std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin(); wit!=PandoraKCL::waypoints.end(); ++wit) {
		for (size_t i=0; i<inspectionPointVisibility[wit->first].size(); i++) {
			if(PandoraKCL::inspectionPoints.find(inspectionPointVisibility[wit->first][i]) != PandoraKCL::inspectionPoints.end()) {
				for(int j=0;j<auv_names.size();j++)
					pFile << "(cansee " << auv_names[j] << " " << wit->first << " " << inspectionPointVisibility[wit->first][i] << ") ";
				pFile << "(= (obs "<< inspectionPointVisibility[wit->first][i] << " " << wit->first << ") 1)" << std::endl;
			}
		}};

		// chain follow locations
		for (std::map<std::string,std::string>::iterator cit=chain_positions.begin(); cit!=chain_positions.end(); ++cit) {
			pFile << "(chainat " << cit->first << " " << cit->second << ")" << std::endl;
		}

		// Waypoints
		for(size_t i=0;i<edges.size();i++) {
			pFile << "(connected " << edges[i].start << " " << edges[i].end << ")  ";
			pFile << "(= (distance " << edges[i].start << " " << edges[i].end << ") " << computeDistance(waypoints[edges[i].start],waypoints[edges[i].end]) << ")" << std::endl;
		}

		pFile << ")" << std::endl;

		/*-------*/
		/* goals */
		/*-------*/

		pFile << "(:goal (and" << std::endl;

		// observe pillars
		if(PandoraKCL::panelPositions.size()<=0) {
			for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::pillars.begin(); iit!=PandoraKCL::pillars.end(); ++iit)
				pFile << "(observed_pillar " << iit->first << ")" << std::endl;
		}

		// observe inspection points that aren't part of pillars
		// if(PandoraKCL::panelPositions.size()<=0) { // Only in UdG, we are searching for the panel
			for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::inspectionPoints.begin(); iit!=PandoraKCL::inspectionPoints.end(); ++iit) {
				if(PandoraKCL::inpsectionPointPillar.find(iit->first) == PandoraKCL::inpsectionPointPillar.end()
						|| PandoraKCL::pillars.find(PandoraKCL::inpsectionPointPillar[iit->first]) == PandoraKCL::pillars.end())
					pFile << "(>= (observed " << iit->first << ") 1)" << std::endl;
			}
		// }

		// examine mysterious panels
		for (std::map<std::string,bool>::iterator it=PandoraKCL::panelExamined.begin(); it!=PandoraKCL::panelExamined.end(); ++it)
			if(!it->second) pFile << "(examined " << it->first << ")" << std::endl;

		// turn valves
		std::map<std::string,int> noValveGoals;
		for (std::map<std::string,std::string>::iterator it=PandoraKCL::valves.begin(); it!=PandoraKCL::valves.end(); ++it)
			noValveGoals[it->first] = 0;
		for (size_t i = 0; i<valveGoals.size(); i++) {
			std::string valveID = valveGoals[i].valve_id;
			if(!useDeadlines
					|| (PandoraKCL::valves.find(valveID) != PandoraKCL::valves.end()
						&& startTime < valveGoals[i].deadline
						&& lastCompletedDeadline[valveID] < valveGoals[i].deadline))
				noValveGoals[valveID]++;
		}
		for (std::map<std::string,std::string>::iterator it=PandoraKCL::valves.begin(); it!=PandoraKCL::valves.end(); ++it)
			if(noValveGoals[it->first] > 0 && valveBlocked[it->first] < 2)
				pFile << "(>= (valve_goal_completed " << it->first << ") " << noValveGoals[it->first] << ")" << std::endl;

		// follow chains
		for (std::map<std::string,bool>::iterator cit=chain_examined.begin(); cit!=chain_examined.end(); ++cit) {
			pFile << "(chain_examined " << cit->first << ")" << std::endl;
		}

		// get back to access point
		for(int i=0;i<auv_names.size();i++)
			pFile << "(near " << auv_names[i] << " " << structure_wp_names[mission_structure_map[mission]] << ")" << std::endl;

		pFile << ")))" << std::endl;

		pFile.close();
	}

} // close namespace
