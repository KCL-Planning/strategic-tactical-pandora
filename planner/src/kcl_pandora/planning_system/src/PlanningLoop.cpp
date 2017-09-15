#include "ros/ros.h"
 
#include "planning_msgs/CompletePlan.h"

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "knowledge_msgs/RoadmapRefresh.h"
#include "knowledge_msgs/Notification.h"
#include "knowledge_msgs/Filter.h"

#include "Utilities.h"

#include "roadmap/Roadmap.h"
#include "PlanningEnvironment.h"
#include "ActionDispatch.h"
#include "VisualiserTimer.h"

//#include "roadmap/RoadmapGenerator.cpp"
#include "StrategicProblemGenerator.cpp"
#include "PDDLProblemGenerator.cpp"
#include "PublishMarkerArray.cpp"
#include "ActionFeedback.cpp"
#include "PostProcess.cpp"
#include "SensorFeedback.cpp"
#include "VisualiserTimer.h"
//#include "util/PlanVisualisation.cpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>

namespace PandoraKCL {

	int planning_timeout = 10;

	/**
	 * listen to and process "/knowledge/ontology/notification" topic
	*/
	void notificationCallBack(const knowledge_msgs::Notification::ConstPtr& msg) {
		if("Chain" == msg->type_name) {
			ROS_INFO("KCL: Notification received; Opportunity spotted");
			opportunistic_plan_requested = true;
			opportunistic_mission = msg->data_property_value;
			mission_structure_map[opportunistic_mission] = "chain_structure";
		} else {
			ROS_INFO("KCL: Notification received; replanning");
			replanRequested = true;
		}
	}

	void publishFilter() {

		// clear the old filter
		knowledge_msgs::Filter filterMessage;
		filterMessage.function = knowledge_msgs::Filter::F_CLEAR;
		filterPublisher.publish(filterMessage);

		ros::spinOnce();
		ros::Rate loop_rate(1);
		loop_rate.sleep();

		// ask to be notified about pillars
		knowledge_msgs::Filter msg_pillar;
		msg_pillar.function = knowledge_msgs::Filter::F_INSERT;
		msg_pillar.type_name = "Pillar";
		knowledgeFilter.push_back(msg_pillar);

		// ask to be notified about chains
		knowledge_msgs::Filter msg_chain;
		msg_chain.function = knowledge_msgs::Filter::F_INSERT;
		msg_chain.type_name = "Chain";
		knowledgeFilter.push_back(msg_chain);

		// push the new filter
		ROS_INFO("KCL: Update filter in knowledge base");
		for(size_t i=0;i<knowledgeFilter.size();i++) {
			filterPublisher.publish(knowledgeFilter[i]);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	/*----------*/
	/* planning */
	/*----------*/

	/**
	 * Passes the problem to the Planner; the plan to post-processing.
	 */
	bool runPlanner(std::string &dataPath, std::string &mission, double timeLimit)
	{
		// save previous plan
		PandoraKCL::planningAttempts = PandoraKCL::planningAttempts + 1;
		if(PandoraKCL::actionList.size() > 0) {

			std::vector<planning_msgs::ActionDispatch> oldplan(PandoraKCL::actionList);
			PandoraKCL::planList.push_back(oldplan);
			PandoraKCL::planListLastAction.push_back(currentAction);
		}

		replanRequested = false;
		std::stringstream ss;
		ss << "timeout " << planning_timeout << " rosrun planning_system bin/popf ";
		std::string popfCommand = ss.str();

		// run the planner
		std::string commandString = popfCommand + "-n "
			 + missionDomain + " "
			 + dataPath + "pandora_problem_" + mission + ".pddl > "
			 + dataPath + "plan_" + mission + ".pddl";

		ROS_INFO("KCL: Running: %s", commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: Planning complete");

		// check the Planner solved the problem
		std::ifstream planfile;
		planfile.open((dataPath + "plan_" + mission + ".pddl").c_str());
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
				planning_timeout += 10;
				return false;
		} else {
				planning_timeout = 10;
		}
		planfile.close();
		
		// save file
		std::ifstream source;
		std::ofstream dest;
		source.open( (dataPath + "plan_" + mission + ".pddl").c_str() );
		dest.open( (dataPath + "output/plan_" + mission + "_" + convert(planningAttempts)).c_str() );
		dest << source.rdbuf();
		source.close();
		dest.close();

		ROS_INFO("KCL: Post processing plan");

		// trim the end of any existing plan
		while(PandoraKCL::actionList.size() > PandoraKCL::currentAction) {
			PandoraKCL::actionList.pop_back();
			PandoraKCL::orientationTarget.pop_back();
		}
		PandoraKCL::freeActionID = PandoraKCL::currentAction;

		// Convert plan into message list for dispatch
		preparePlan(dataPath, mission);
		addOrientation();

		//planStart = ros::WallTime::now().toSec();
		planStart = VisualiserTimer::getTime();
		if(timeLimit <= 0) {
			planStart += free_time_budget;
			free_time_budget = 0;
		} else {
			ROS_INFO("KCL: Plan start, Duration, Deadline: [%f, %f, %f]", planStart, PandoraKCL::totalPlanDuration, timeLimit);
			if(planStart + PandoraKCL::totalPlanDuration > timeLimit) {
				ROS_INFO("Plan was not solvable in available time remaining. Abandoning opportunity.");
				return false;
			}
		}

		// publish filter
		publishFilter();

		// publish complete plan
		planning_msgs::CompletePlan cp_msgs;
		cp_msgs.actions.insert(cp_msgs.actions.end(), PandoraKCL::actionList.begin() + PandoraKCL::currentAction, PandoraKCL::actionList.end());
		planPublisher.publish(cp_msgs);

		return true;
	}

	/*------------------*/
	/* generate problem */
	/*------------------*/

	/**
	 * Requests objects from ontology; generates the PDDL problem.
	 */
	bool generatePlanningProblem(ros::NodeHandle nh, std::string &dataPath, std::string mission, bool useDeadlines) {

		// publish state
		std_msgs::String stateMsg;
		stateMsg.data = "planning";
		statePublisher.publish(stateMsg);

		// update the environment from the ontology
		ROS_INFO("KCL: Fetching objects");

		loadConfigFile(dataPath);

		// (re)generate waypoints, if not using knowledge base
		if(!PandoraKCL::use_octomap) {
			// buildPRM(dataPath);
		} else {
			ros::ServiceClient roadmapClient = nh.serviceClient<knowledge_msgs::RoadmapRefresh>("/roadmap_server/refresh");
			knowledge_msgs::RoadmapRefresh srv;
			srv.request.missions.push_back(mission);
			if (!roadmapClient.call(srv))
				ROS_ERROR("KCL: Failed to call service roadmap_server/refresh");
		}

		if(PandoraKCL::use_octomap) updateEnvironment(nh, mission);

		// print map for debugging
		std::ofstream wpsFile, ipsFile, visFile;
		ipsFile.open((dataPath + "IPs.txt").c_str());
		wpsFile.open((dataPath + "WPs.txt").c_str());
		visFile.open((dataPath + "VIs.txt").c_str());
		for(size_t i=0;i<edges.size();i++)
			wpsFile << waypoints[edges[i].start] << std::endl << waypoints[edges[i].end]  << std::endl << std::endl << std::endl;
		for (std::map<std::string,Point3D>::iterator iit=PandoraKCL::inspectionPoints.begin(); iit!=PandoraKCL::inspectionPoints.end(); ++iit) {
			for (std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin(); wit!=PandoraKCL::waypoints.end(); ++wit) {
				double vis = computeVisibility(wit->second,iit->second,0.0,2.0,0.2);
				if(vis>0) visFile << wit->second << std::endl << iit->second  << std::endl << std::endl << std::endl;
			}
			ipsFile << iit->second << std::endl;
		};
		visFile.close();
		wpsFile.close();
		ipsFile.close();

		// generate PDDL problem
		generatePDDLProblemFile(dataPath, useDeadlines, mission);
		
		// save file
		std::ifstream source;
		std::ofstream dest;
		source.open( (dataPath + "pandora_problem_" + mission + ".pddl").c_str() );
		dest.open( (dataPath + "output/pandora_problem_" + mission + "_" + convert(planningAttempts+1)).c_str() );
		dest << source.rdbuf();
		source.close();
		dest.close();

		return true;
	}

	/*-----------*/
	/* main loop */
	/*-----------*/

	/**
	 * Sets up ROS; prepares planning; main loop.
	 */
	void runPlanningServer(std::string mission, double timeLimit)
	{
		ros::NodeHandle nh("~");

		std_msgs::String stateMsg;

		// setup environment
		std::string dataPath;
		nh.param("data_path", dataPath, std::string("data/"));

		// wait for one publisher to action_feedback
		ros::Rate loop_rate(10);
		ROS_INFO("KCL: waiting for one publisher to action_feedback");
		while(ros::ok() && feedbackSub[auv_names[0]].getNumPublishers() < 1) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		// get current location if we are building PRM locally
		if(!PandoraKCL::use_octomap) {
			while(ros::ok() && !PandoraKCL::positionInitialised) {
				ros::spinOnce();
				loop_rate.sleep();
			}
		}

		//missionStart = ros::WallTime::now().toSec();
		missionStart = VisualiserTimer::getTime();

		// generate PDDL problem and run planner
		generatePlanningProblem(nh, dataPath, mission, true);
		if(!runPlanner(dataPath, mission, timeLimit)) {
			ROS_INFO("KCL: Mission Failed");
			return;
		}

		// planning completed, publish interesting information
		PandoraKCL::publishInspectionMarkerArray(nh);
		PandoraKCL::publishWaypointMarkerArray(nh);

		for(int i=0;i<auv_names.size();i++)
			PandoraKCL::auv_busy[auv_names[i]] = false;

		// Loop through and publish planned actions
		PandoraKCL::repeatAction = false;
		PandoraKCL::currentAction--;
		while (ros::ok() && PandoraKCL::actionList.size() > PandoraKCL::currentAction+1 && !strategic_replan_requested) {

			// get ready for next action
			if(PandoraKCL::repeatAction) PandoraKCL::repeatAction = false;
			else PandoraKCL::currentAction++;
			PandoraKCL::actionReceived[PandoraKCL::currentAction] = false;
			PandoraKCL::actionCompleted[PandoraKCL::currentAction] = false;

			planning_msgs::ActionDispatch currentMessage = PandoraKCL::actionList[PandoraKCL::currentAction];
			if((unsigned int)currentMessage.action_id != PandoraKCL::currentAction)
				ROS_INFO("KCL: ERROR message action_id [%d] does not meet expected [%zu]", currentMessage.action_id, PandoraKCL::currentAction);

			// loop while waiting for dispatch time
			if(!dispatch_early[currentMessage.name]) {

				// publish state
				stateMsg.data = "waiting for scheduled dispatch";
				statePublisher.publish(stateMsg);

				double wait_period = 6.0;
				free_time_budget = 0;
				//int wait_print = (int)(currentMessage.dispatch_time + planStart - ros::WallTime::now().toSec()) / wait_period;
				int wait_print = (int)(currentMessage.dispatch_time + planStart - VisualiserTimer::getTime());
				//while (ros::ok() && ros::WallTime::now().toSec() < currentMessage.dispatch_time + planStart && !PandoraKCL::replanRequested) {
				while (ros::ok() && VisualiserTimer::getTime() < currentMessage.dispatch_time + planStart && !PandoraKCL::replanRequested) {
					ros::spinOnce();
					loop_rate.sleep();

					//double remaining = planStart + currentMessage.dispatch_time - ros::WallTime::now().toSec();
					double remaining = planStart + currentMessage.dispatch_time - VisualiserTimer::getTime();
					if(wait_print > (int)remaining / wait_period) {
						ROS_INFO("KCL: Waiting %f before dispatching action: [%i, %s, %f, %f]",
								remaining,currentMessage.action_id, currentMessage.name.c_str(),
								 (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
						wait_print--;
					}
				}
			}

			// loop while waiting for AUV to become free
			int counter = 0;
			while (ros::ok() && PandoraKCL::auv_busy[currentMessage.auv] && !PandoraKCL::replanRequested) {

				// publish state
				stateMsg.data = "executing plan";
				statePublisher.publish(stateMsg);

				ros::spinOnce();
				loop_rate.sleep();
				counter++;
				if (counter == 2000) {
					ROS_INFO("KCL: Action %i timeout now. Cancelling and replanning...", currentMessage.action_id);
					planning_msgs::ActionDispatch cancelMessage;
					cancelMessage.action_id = currentMessage.action_id;
					cancelMessage.name = "cancel_action";
					actionPublisher[currentMessage.auv].publish(cancelMessage);
					replanRequested = true;
				}

				// START OPPORTUNISTIC PLANNING
				if(opportunistic_plan_requested && "myopic" == opportunistic_mode && "goto" == currentMessage.name) {
					opportunistic_plan_requested = false;
					current_mission_ids.push_back(opportunistic_mission);
					current_plans.push_back(actionList);
					//runPlanningServer(opportunistic_mission, ros::WallTime::now().toSec() + free_time_budget);
					runPlanningServer(opportunistic_mission, VisualiserTimer::getTime() + free_time_budget);
					actionList = current_plans[current_plans.size()-1];
					current_mission_ids.pop_back();
					current_plans.pop_back();
					// republish goto
					PandoraKCL::repeatAction = true;
				}
				// END OPPORTUNISTIC PLANNING
			}

			// update action
			if ("turn_valve" == currentMessage.name) {
				// get id
				std::string valveID = "";
				for(size_t i=0; i<currentMessage.parameters.size(); i++)
					if(0 == currentMessage.parameters[i].key.compare("valve_id"))
						valveID = currentMessage.parameters[i].value;
				// determine angle
				double desired_angle = 0.0;
				for(size_t i=0; i<currentMessage.parameters.size(); i++)
					if(0 == currentMessage.parameters[i].key.compare("desired_angle"))
						desired_angle = atof(currentMessage.parameters[i].value.c_str());
				// set increment
				double desired_increment = desired_angle - valveAngles[valveID];
				for(size_t i=0; i<currentMessage.parameters.size(); i++)
					if(0 == currentMessage.parameters[i].key.compare("desired_increment"))
						currentMessage.parameters[i].value = convert(desired_increment);
			}

			// dispatch action
			if(!PandoraKCL::replanRequested) {

				// publish state
				stateMsg.data = "dispatching action";
				statePublisher.publish(stateMsg);

				ROS_INFO("KCL: Dispatching action [%i, %s, %f, %f]", currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
				actionPublisher[currentMessage.auv].publish(currentMessage);
				//double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time + planStart));
				double late_print = (VisualiserTimer::getTime() - (currentMessage.dispatch_time + planStart));
				if(late_print >  0.1) ROS_INFO("KCL: Action [%i] is %f second(s) late", currentMessage.action_id, late_print);
				if(late_print < -0.1) {
					free_time_budget = -1*late_print;
					ROS_INFO("KCL: Action [%i] is early. Free time budget is %f second(s)", currentMessage.action_id, free_time_budget);
					std_msgs::Float64 ftb_msg;
					ftb_msg.data = free_time_budget;
					free_time_publisher.publish(ftb_msg);
					ros::spinOnce();
				}
				auv_busy[currentMessage.auv] = true;
			}

			// check of we have drifted
			if(drifted) {
				ROS_INFO("KCL: We have drifted. Resetting Landmarks.");
				planning_msgs::ActionDispatch landmarkMessage;
				landmarkMessage.action_id = currentMessage.action_id;
				landmarkMessage.name = "reset_landmarks";
				actionPublisher[currentMessage.auv].publish(landmarkMessage);
				drifted = false;
			}

			// update markers in rvis
			PandoraKCL::publishInspectionMarkerArray(nh);
			PandoraKCL::publishWaypointMarkerArray(nh);

			// mission goal check (UdG valve turning)
			if(PandoraKCL::currentAction == PandoraKCL::actionList.size()-1) {

				// check if panel is examined
				for (std::map<std::string,bool>::iterator it=PandoraKCL::panelExamined.begin(); it!=PandoraKCL::panelExamined.end(); ++it)
				if(!it->second) {
					ROS_INFO("KCL: Replanning to validate valve positions on panel %s", it->first.c_str());
					replanRequested = true;
				}

				// check if valves deadlines are finished
				if(!deadlinesCompleted()) {
					ROS_INFO("KCL: Replanning to correct valves according to deadlines: %f", finalValveDeadline);
					replanRequested = true;
				}
			}

			// wait for final action
			if(PandoraKCL::currentAction == PandoraKCL::actionList.size()-1) {

				// publish state
				stateMsg.data = "executing plan";
				statePublisher.publish(stateMsg);

				ROS_INFO("KCL: Waiting for final action to complete");
				bool missionCompleted = false;
				while (ros::ok() && !missionCompleted) {
					missionCompleted = true;
					for(int i=0;i<auv_names.size();i++) {
						if(auv_busy[auv_names[i]]) missionCompleted = false;
					}
					ros::spinOnce();
					loop_rate.sleep();
				}
			}

			// generate PDDL problem and (re)run planner
			if(replanRequested) {
				generatePlanningProblem(nh, dataPath, mission, true);
				while(!runPlanner(dataPath, mission, timeLimit))
					generatePlanningProblem(nh, dataPath, mission, true);
			}
		}

		// publish state
		stateMsg.data = "mission complete";
		statePublisher.publish(stateMsg);

		planning_msgs::ActionDispatch currentStrategicAction;
		strategicActionPublisher.publish(currentStrategicAction);
		
		ROS_INFO("KCL: Mission Complete");
	}

	/*----------------*/
	/* strategic loop */
	/*----------------*/

	void setupStrategicProblem(ros::NodeHandle &nh, std::string &dataPath) {

		// get mission data
		strategic_replan_requested = false;
		ROS_INFO("KCL: Fetching structures and missions");
		updateMissions(nh);
		//strategicMissionStart = ros::WallTime::now().toSec();
		strategicMissionStart = VisualiserTimer::getTime();
		free_time_budget = 0;
	}

	void setupStrategicPlan(ros::NodeHandle &nh, std::string &dataPath) {

		// publish state
		std_msgs::String stateMsg;
		stateMsg.data = "strategic planning";
		statePublisher.publish(stateMsg);

		// plan for each mission
		for (std::map<std::string,std::string>::iterator iit=mission_structure_map.begin(); iit!=mission_structure_map.end(); ++iit) {

			// generate PDDL problem and run planner
			ROS_INFO("KCL: Planning submission");
			std::string missionID = iit->first;
			generatePlanningProblem(nh, dataPath, missionID, false);
			runPlanner(dataPath, missionID, -1);
			
			// process
			mission_duration_map[missionID] = totalPlanDuration;
		}

		// generate strategic PDDL problem and run planner
		ROS_INFO("KCL: Generate strategic problem");
		generateStrategicPDDLProblemFile(dataPath);
		runStrategicPlanner(dataPath);
	}

	/**
	 * Sets up ROS; prepares planning; main loop.
	 */
	void runStrategicPlanningServer() {
		ros::NodeHandle nh("~");
		
		VisualiserTimer* timer = new VisualiserTimer(nh);

		// setup action parameters
		free_time_budget = 0;
		dispatch_early["goto"] = true;
		dispatch_early["observe"] = true;
		dispatch_early["illuminate_pillar"] = true;
		dispatch_early["observe_pillar"] = true;
		dispatch_early["examine_panel"] = true;
		dispatch_early["turn_valve"] = false;
		dispatch_early["recalibrate_arm"] = true;
		dispatch_early["enable_chain_follow"] = true;
		dispatch_early["dock_auv"] = true;

		// setup environment
		std::string dataPath;
		nh.param("data_path", dataPath, std::string("data/"));
		nh.param("strategic_domain", strategicDomain, std::string("data/pandora_domain_strategic.pddl"));
		nh.param("mission_domain", missionDomain, std::string("data/pandora_domain_persistent.pddl"));
		nh.param("use_octomap", use_octomap, false);
		ROS_INFO("KCL: Using data path: %s; Using octomap: %u", dataPath.c_str(), PandoraKCL::use_octomap);

		nh.param("opportunistic_mode", opportunistic_mode, std::string("myopic"));
		ROS_INFO("KCL: Opportunitic planning mode: %s", opportunistic_mode.c_str());

		// current position update
		sensorSubOdometry = nh.subscribe("/pose_ekf_slam/odometry", 1, PandoraKCL::odometryCallbackUDG);

		// marker array publishers
		ipsPublisher = nh.advertise<visualization_msgs::MarkerArray>( "vis/planning_inspection_marker", 1000, true);
		wpsPublisher = nh.advertise<visualization_msgs::MarkerArray>( "vis/planning_waypoint_marker", 1000, true);
		free_time_publisher = nh.advertise<std_msgs::Float64>( "vis/free_time_budget", 1000, true);

		// complete plan publisher
		planPublisher = nh.advertise<planning_msgs::CompletePlan>("current_plan", 1000, true);
		strategicPlanPublisher = nh.advertise<planning_msgs::CompletePlan>("current_strategic_plan", 1000, true);
		statePublisher = nh.advertise<std_msgs::String>("state", 1000, true);
		strategicActionPublisher = nh.advertise<planning_msgs::ActionDispatch>("strategic_action_dispatch", 1000, true);
 
		// publishing "/ontology/filter"; listening "/ontology/notification"
		filterPublisher = nh.advertise<knowledge_msgs::Filter>("/knowledge/ontology/filter", 10, true);
		notificationSub = nh.subscribe("/knowledge/ontology/notification", 100, PandoraKCL::notificationCallBack);

		setupStrategicProblem(nh, dataPath);

		// publishing "action_dispatch"; listening "action_feedback"
		for(int i=0;i<auv_names.size();i++) {
			std::stringstream ss1; ss1 << auv_names[i] << "/action_dispatch";
			std::stringstream ss2; ss2 << auv_names[i] << "/action_feedback";
			actionPublisher[auv_names[i]] = nh.advertise<planning_msgs::ActionDispatch>(ss1.str(), 1000, true);
			feedbackSub[auv_names[i]] = nh.subscribe(ss2.str(), 1000, PandoraKCL::feedbackCallback);
		}

		setupStrategicPlan(nh, dataPath);

		// publish complete plan
		planning_msgs::CompletePlan cp_msgs;
		cp_msgs.actions.insert(cp_msgs.actions.end(), strategic_plan.begin(), strategic_plan.end());
		strategicPlanPublisher.publish(cp_msgs);

		// dispatch strategic plan
		ROS_INFO("KCL: Dispatch strategic plan");
		ros::Rate loop_rate(10);
		for(size_t a=0; a<strategic_plan.size(); a++) {

			planning_msgs::ActionDispatch currentStrategicAction = strategic_plan[a];
			strategicActionPublisher.publish(currentStrategicAction);

			if("goto_structure" == currentStrategicAction.name) {

				// publish state
				std_msgs::String stateMsg;
				stateMsg.data = "moving to another structure";
				statePublisher.publish(stateMsg);

				// publish goto_structure action
				ROS_INFO("KCL: Dispatching action [%s, %f, %f]", currentStrategicAction.name.c_str(), (currentStrategicAction.dispatch_time-missionStart), currentStrategicAction.duration);
				actionCompleted[currentStrategicAction.action_id] = false;
				free_time_budget = (VisualiserTimer::getTime() - (currentStrategicAction.dispatch_time + missionStart));
				if(free_time_budget < 0) free_time_budget = 0;
				else ROS_INFO("KCL: Free time [%f]", free_time_budget);
				for(int i=0;i<auv_names.size();i++)
					actionPublisher[auv_names[i]].publish(currentStrategicAction);

				// callback and sleep
				while (ros::ok() && !actionCompleted[currentStrategicAction.action_id] && !strategic_replan_requested) {

					ros::spinOnce();
					loop_rate.sleep();

					// START OPPORTUNISTIC PLANNING
					if(opportunistic_plan_requested && "myopic" == opportunistic_mode) {
						opportunistic_plan_requested = false;
						current_mission_ids.push_back(opportunistic_mission);
						//runPlanningServer(opportunistic_mission, ros::WallTime::now().toSec() + free_time_budget);
						runPlanningServer(opportunistic_mission, VisualiserTimer::getTime() + free_time_budget);
						current_mission_ids.pop_back();
						// republish goto
						for(int i=0;i<auv_names.size();i++)
							actionPublisher[auv_names[i]].publish(currentStrategicAction);
						ros::spinOnce();
					}
					// END OPPORTUNISTIC PLANNING
				}

				// replan between missions
				if(strategic_replan_requested) {
					for(int i=0;i<auv_names.size();i++) {
						planning_msgs::ActionDispatch cancelMessage;
						cancelMessage.action_id = -1;
						cancelMessage.name = "cancel_action";
						actionPublisher[auv_names[i]].publish(cancelMessage);
					}
					setupStrategicProblem(nh, dataPath);
					setupStrategicPlan(nh, dataPath);
					a = 0;
				}

			} else if("complete_mission" == currentStrategicAction.name) {

				for(size_t i=0; i<currentStrategicAction.parameters.size(); i++) {
					if(0 == currentStrategicAction.parameters[i].key.compare("mission")) {

						std::string missionID = currentStrategicAction.parameters[i].value;
						ROS_INFO("KCL: Beginning mission [%s]", missionID.c_str());
						current_mission_ids.push_back(missionID);
						runPlanningServer(missionID, -1);
						current_mission_ids.pop_back();
					}
				}

				// replan during mission
				if(strategic_replan_requested) {
					setupStrategicProblem(nh, dataPath);
					setupStrategicPlan(nh, dataPath);
					a = 0;
				}

			} else {

				// publish state
				std_msgs::String stateMsg;
				stateMsg.data = "recharge docking/undocking";
				statePublisher.publish(stateMsg);

				// undock_auv, dock_auv

				// loop while waiting for dispatch time
				if(!dispatch_early[currentStrategicAction.name]) {
					double wait_period = 60.0;
					free_time_budget = 0;
					//int wait_print = (int)(currentStrategicAction.dispatch_time + strategicMissionStart - ros::WallTime::now().toSec()) / wait_period;
					int wait_print = (int)(currentStrategicAction.dispatch_time + strategicMissionStart - VisualiserTimer::getTime()) / wait_period;
					//while (ros::ok() && ros::WallTime::now().toSec() < currentStrategicAction.dispatch_time + strategicMissionStart) {
					while (ros::ok() && VisualiserTimer::getTime() < currentStrategicAction.dispatch_time + strategicMissionStart) {
						ros::spinOnce();
						loop_rate.sleep();

						//double remaining = planStart + currentStrategicAction.dispatch_time - ros::WallTime::now().toSec();
						double remaining = planStart + currentStrategicAction.dispatch_time - VisualiserTimer::getTime();
						if(wait_print > (int)remaining / wait_period) {
							ROS_INFO("KCL: Waiting %f before dispatching action: [%s, %f, %f]",
									remaining, currentStrategicAction.name.c_str(),
									currentStrategicAction.dispatch_time, currentStrategicAction.duration);
							wait_print--;
						}
					}
				}

				ROS_INFO("KCL: Dispatching action [%s, %f, %f]", currentStrategicAction.name.c_str(), (currentStrategicAction.dispatch_time-missionStart), currentStrategicAction.duration);
				actionCompleted[currentStrategicAction.action_id] = false;
				for(int i=0;i<auv_names.size();i++)
					actionPublisher[auv_names[i]].publish(currentStrategicAction);

				// callback and sleep
				while (ros::ok() && !actionCompleted[currentStrategicAction.action_id] && !strategic_replan_requested) {

					ros::spinOnce();
					loop_rate.sleep();
				}

				// replan between missions
				if(strategic_replan_requested) {
					for(int i=0;i<auv_names.size();i++) {
						planning_msgs::ActionDispatch cancelMessage;
						cancelMessage.action_id = -1;
						cancelMessage.name = "cancel_action";
						actionPublisher[auv_names[i]].publish(cancelMessage);
					}
					setupStrategicProblem(nh, dataPath);
					setupStrategicPlan(nh, dataPath);
					a = 0;
				}
			}
		}
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"planning_system");
		srand (static_cast <unsigned> (time(0)));
		PandoraKCL::runStrategicPlanningServer();

		return 0;
	}
