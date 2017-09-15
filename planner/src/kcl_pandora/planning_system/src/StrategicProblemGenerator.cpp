#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

namespace PandoraKCL {

	/*----------*/
	/* planning */
	/*----------*/

	/* 
	 * Processes the strategic_plan.pddl into dispatchable actions.
	 */
	void prepareStrategicPlan(std::string dataPath) {

		// popf output
		std::ifstream planfile;
		planfile.open((dataPath + "plan_strategic.pddl").c_str());

		int curr, next;
		std::string line;
		std::vector<planning_msgs::ActionDispatch> potentialPlan;
		double planDuration = -1;
		double expectedPlanDuration = 0;

		while(!planfile.eof()) {

			getline(planfile, line);

			if (line.substr(0,6).compare("; Cost") == 0) {
				//; Cost: xxx.xxx
				expectedPlanDuration = atof(line.substr(8).c_str());
			} else if (line.substr(0,6).compare("; Time")!=0) {
				//consume useless lines
			} else {

				potentialPlan.clear();
				planDuration = 0;

				while(!planfile.eof() && line.compare("")!=0) {

					getline(planfile, line);
					if (line.length()<2)
						break;

					planning_msgs::ActionDispatch msg;

					// action ID doesn't matter
					msg.action_id = -1;

					// dispatchTime
					curr=line.find(":");
					double dispatchTime = (double)atof(line.substr(0,curr).c_str());
					msg.dispatch_time = dispatchTime;

					// name
					curr=line.find("(")+1;
					next=line.find(" ",curr);
					std::string name = line.substr(curr,next-curr).c_str();
					msg.name = name;

					// parameters
					std::vector<std::string> params;
					curr=next+5;
					next=line.find(")",curr);
					int at = curr;
					while(at < next) {
						int cc = line.find(" ",curr);
						int cc1 = line.find(")",curr);
						curr = cc<cc1?cc:cc1;
						std::string param = name_map[line.substr(at,curr-at)];
						params.push_back(param);
						++curr;
						at = curr;
					}
					if("do_hover" == msg.name) {
						// parameters (?v - vehicle ?from ?to - waypoint)
						std::string wp_id = params[1];
						msg.name = "goto_structure";
						diagnostic_msgs::KeyValue pair;
						pair.key = "structure";
						pair.value = wp_id;
						msg.parameters.push_back(pair);
					} else if("complete_mission" == msg.name) {
						// parameters (?v - vehicle ?m - mission ?wp - waypoint)
						std::string mission_id = params[0];
						msg.name = "complete_mission";
						diagnostic_msgs::KeyValue pair;
						pair.key = "mission";
						pair.value = mission_id;
						msg.parameters.push_back(pair);
					} else if("dock_auv" == msg.name) {
						// parameters (?v - vehicle ?wp - waypoint)
						msg.name = "dock_auv";
					} else if("undock_auv" == msg.name) {
						// parameters (?v - vehicle ?wp - waypoint)
						msg.name = "undock_auv";
					}

					// duration
					curr=line.find("[",curr)+1;
					next=line.find("]",curr);
					msg.duration = (double)atof(line.substr(curr,next-curr).c_str());

					potentialPlan.push_back(msg);

					// update plan duration
					curr=line.find(":");
					planDuration = msg.duration + atof(line.substr(0,curr).c_str());
				}

				if(fabs(planDuration - expectedPlanDuration) < 0.01 || true)  {
					// save better optimised plan
					strategic_plan.clear();
					for(size_t i=0;i<potentialPlan.size();i++)
						strategic_plan.push_back(potentialPlan[i]);
				} else {
					ROS_INFO("Duration: %f, expected %f; plan discarded", planDuration, expectedPlanDuration);
				}
			}
		}
		planfile.close();
	}

	/**
	 * Passes the problem to the Planner; the plan to post-processing.
	 */
	bool runStrategicPlanner(std::string &dataPath)
	{
		std::string popfCommand = "rosrun planning_system bin/popf ";

		// run the planner
		std::string commandString = popfCommand
			 + strategicDomain + " "
			 + dataPath + "pandora_strategic_problem.pddl > "
			 + dataPath + "plan_strategic.pddl";

		ROS_INFO("KCL: Running: %s", commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: Planning complete");

		// check the Planner solved the problem
		std::ifstream planfile;
		planfile.open((dataPath + "plan_strategic.pddl").c_str());
		std::string line;
		bool solved = false;
		while(!planfile.eof() && !solved) {
			getline(planfile, line);
			if (line.find("; Time", 0) != std::string::npos)
				solved = true;
		}
		if(!solved) {
				planfile.close();
				ROS_INFO("Plan was unsolvable! Try again?");
				return false;
		}
		planfile.close();
		
		ROS_INFO("KCL: Processing strategic plan");

		// Convert plan into message list for dispatch
		prepareStrategicPlan(dataPath);

		return true;
	}

	/*---------*/
	/* problem */
	/*---------*/

	/**
	 * Generate a PDDL problem file, saving the result in dataPath/pandora_strategic_problem.pddl.
	 * This file is later read by the planner, and the result saved in dataPath/strategic_plan.pddl.
	 */
	void generateStrategicPDDLProblemFile(std::string &dataPath)
	{
		/*--------*/
		/* header */
		/*--------*/

		ROS_INFO("KCL: Generating strategic PDDL problem file");
		std::ofstream pFile;
		pFile.open((dataPath + "pandora_strategic_problem.pddl").c_str());

		pFile << "(define (problem pandora_strategic_mission)" << std::endl;
		pFile << "(:domain pandora_domain_strategic)" << std::endl;

		/* objects */
		pFile << "(:objects" << std::endl;

		// vehicles
		pFile << "auv - vehicle" << std::endl;

		// structure waypoints
		for(int s=0;s<structures.size();s++)
			pFile << structure_wp_names[structures[s]] << " ";
		for(size_t i=0;i<auv_names.size();i++)
			pFile << auv_starting_locations[auv_names[i]] << " ";
		if(structures.size()>0 || auv_names.size() > 0)
			pFile << "- waypoint" << std::endl;

		// missions
		for (std::map<std::string,std::string>::iterator it=mission_structure_map.begin(); it!=mission_structure_map.end(); ++it)
			pFile << it->first << " ";
		if(mission_structure_map.size()>0)
			pFile << "- mission" << std::endl;

		pFile << ")" << std::endl;

		/*---------------*/
		/* initial state */
		/*---------------*/

		pFile << "(:init" << std::endl;

		pFile << "(vehicle_free auv)" << std::endl;
		pFile << "(= (mission_total) 0)" << std::endl;

		// position
		if (structures.size()>0) {
			for(size_t i=0;i<auv_names.size();i++) {
				pFile << "(at auv " << auv_starting_locations[auv_names[i]] << ") (= (charge auv) 1200)" << std::endl;
				pFile << std::endl;
			}
		}

		// recharge locations
		for(std::vector<std::string>::iterator rit=recharge_structures.begin(); rit!=recharge_structures.end(); ++rit)
			pFile << "(recharge_at " << structure_wp_names[*rit] << ")" << std::endl;
		pFile << std::endl;

		// mission active (TODO TILs)
		for (std::map<std::string,std::string>::iterator it=mission_structure_map.begin(); it!=mission_structure_map.end(); ++it)
			pFile << "(active " << it->first << ")" << std::endl;
		pFile << std::endl;

		// mission deadlines
		for (std::map<std::string,double>::iterator it=mission_deadline_map.begin(); it!=mission_deadline_map.end(); ++it)
			pFile << "(at " << it->second << " (not (active " << it->first << ")))" << std::endl;

		// strategic deadline (comment out if needed)
		for (std::map<std::string,std::string>::iterator it=mission_structure_map.begin(); it!=mission_structure_map.end(); ++it) {
			if(mission_deadline_map.find(it->first) ==  mission_deadline_map.end())
				pFile << "(at 99999 (not (active " << it->first << ")))" << std::endl;
		}

		pFile << std::endl;

		// mission locations
		for (std::map<std::string,std::string>::iterator it=mission_structure_map.begin(); it!=mission_structure_map.end(); ++it)
			pFile << "(in " << it->first << " " << structure_wp_names[it->second] << ")" << std::endl;
		pFile << std::endl;

		// mission durations
		for (std::map<std::string,double>::iterator it=mission_duration_map.begin(); it!=mission_duration_map.end(); ++it)
			pFile << "(= (mission_duration " << it->first << ") " << (it->second+10) << ")" << std::endl;
		pFile << std::endl;

		// structure distances
		for(size_t i=0;i<structures.size();i++) {
			for(size_t j=0;j<structures.size();j++) {
				if(i==j) continue;
				pFile << "(connected " << structure_wp_names[structures[i]] << " " << structure_wp_names[structures[j]] << ") ";
				pFile << "(= (distance " << structure_wp_names[structures[i]] << " " << structure_wp_names[structures[j]] << ") "
						<< computeDistance(structure_wps[structures[i]], structure_wps[structures[j]]) << ")" << std::endl;
			}
			for(size_t j=0;j<auv_names.size();j++) {
				pFile << "(connected " << structure_wp_names[structures[i]] << " " << auv_starting_locations[auv_names[j]] << ") ";
				pFile << "(= (distance " << structure_wp_names[structures[i]] << " " << auv_starting_locations[auv_names[j]] << ") "
						<< computeDistance(structure_wps[structures[i]], auv_starting_waypoints[auv_names[j]]) << ")" << std::endl;
				pFile << "(connected " << auv_starting_locations[auv_names[j]] << " " << structure_wp_names[structures[i]] << ") ";
				pFile << "(= (distance " << auv_starting_locations[auv_names[j]] << " " << structure_wp_names[structures[i]] << ") "
						<< computeDistance(structure_wps[structures[i]], auv_starting_waypoints[auv_names[j]]) << ")" << std::endl;
			}
		};
		pFile << std::endl;
		pFile << ")" << std::endl;

		/*-------*/
		/* goals */
		/*-------*/

/*
		// complete all missions!
		pFile << "(:goal (and" << std::endl;
		for (std::map<std::string,std::string>::iterator it=mission_structure_map.begin(); it!=mission_structure_map.end(); ++it)
			pFile << "(completed " << it->first << ")" << std::endl;
		pFile << ")))" << std::endl;
*/

		// try to complete some missions.
		pFile << "(:metric maximize (mission_total))" << std::endl;
		pFile << "(:goal (> (mission_total) 0))" << std::endl;
		pFile << ")" << std::endl;
	}

} // close namespace