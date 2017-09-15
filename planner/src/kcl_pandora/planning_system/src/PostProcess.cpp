#include "planning_msgs/ActionDispatch.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

namespace PandoraKCL
{

	double MIN_WAIT_TIME = 20.0;

	/**
	 * Process the parameters of a PDDL action into an ActionDispatch message.  This includes
	 * some intermediary processing, such as expanding waypoints into coordinates.
	 * Refer to the PDDL domain(s) for more information on the actions.
	 */
	void processParameters(planning_msgs::ActionDispatch &msg, std::vector<std::string> &params, double dispatchTime)
	{
		// no parameters: "recalibrate_arm"

		if("do_hover_controlled" == msg.name || "correct_position" == msg.name || "do_hover_fast" == msg.name) {

			// correct_position	parameters (?v - vehicle ?target - waypoint)
			std::string wp_id = params[0];

			// do_hover		parameters (?v - vehicle ?from ?to - waypoint)
			if("do_hover_controlled" == msg.name) wp_id = params[1];
			if("do_hover_fast" == msg.name) wp_id = params[1];

			// movement type (fast || controlled)
			diagnostic_msgs::KeyValue pair;
			pair.key = "movement_type";
			if("do_hover_fast" == msg.name)
				pair.value = "fast";
			else
				pair.value = "controlled";
			msg.parameters.push_back(pair);

			// name
			msg.name = "goto";

			// goal location
			Point3D destination = waypoints[wp_id];
			const std::string keys[] = {"north", "east", "depth", "yaw", "pitch"};
			const std::string values[] = {convert(destination.N), convert(destination.E), convert(destination.D), "0", "0"};
			for(int i=0;i<5;i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = keys[i];
				pair.value = values[i];
				msg.parameters.push_back(pair);
			}

			// wpID
			diagnostic_msgs::KeyValue wp_pair;
			wp_pair.key = "wp_ID";
			wp_pair.value = wp_id;
			msg.parameters.push_back(wp_pair);

			// TODO control points

		} else if ("turn_valve" == msg.name) {

			// name
			msg.name = "turn_valve";

			// turn_valve	parameters (?v - vehicle ?wp - waypoint ?p - panel ?a - valve)
			std::string valveID = params[2];
			double valveGoal = PandoraKCL::getCurrentValveGoal(valveID, dispatchTime + planStart - missionStart);
			double valveDeadline = PandoraKCL::getCurrentValveDeadline(valveID, dispatchTime + planStart - missionStart);

			diagnostic_msgs::KeyValue turn_valve_id;
			turn_valve_id.key = "valve_id";
			turn_valve_id.value = valveID;
			msg.parameters.push_back(turn_valve_id);

			diagnostic_msgs::KeyValue turn_valve_angle;
			turn_valve_angle.key = "desired_increment";
			//turn_valve_angle.value = convert(normalizeHalfAngle(valveGoal - valveAngles[valveID]));
			turn_valve_angle.value = convert(valveGoal - valveAngles[valveID]);
			msg.parameters.push_back(turn_valve_angle);

			diagnostic_msgs::KeyValue turn_valve_deadline;
			turn_valve_deadline.key = "valve_deadline";
			turn_valve_deadline.value = convert(valveDeadline);
			msg.parameters.push_back(turn_valve_deadline);

			diagnostic_msgs::KeyValue desired_valve_angle;
			desired_valve_angle.key = "desired_angle";
			desired_valve_angle.value = convert(valveGoal);
			msg.parameters.push_back(desired_valve_angle);

		} else if ("follow_chain" == msg.name) {

			// name
			msg.name = "enable_chain_follow";

			diagnostic_msgs::KeyValue chain_id;
			chain_id.key = "chain_id";
			chain_id.value = params[0];
			msg.parameters.push_back(chain_id);

		} else if("observe_inspection_point" == msg.name || "observe_pillar" == msg.name) {

			Point3D inspection_target;
			if("observe_inspection_point" == msg.name)
				inspection_target = inspectionPoints[params[1]];
			else if("observe_pillar" == msg.name)
				inspection_target = pillars[params[1]];

			// observe prameters (?v - vehicle ?wp - waypoint ?ip - inspectionpoint)
			// observe prameters (?v - vehicle ?wp - waypoint ?p - pillar)
			msg.name = "observe";

			Point3D pose = waypoints[params[0]];
			const std::string keys[] = {"north", "east", "depth", "yaw", "pitch", "ip_north", "ip_east", "ip_depth", "ip_id"};
			const std::string values[] = {
				convert(pose.N), convert(pose.E), convert(pose.D),
				convert(findYaw(pose,inspection_target)), "0",
				convert(inspection_target.N), convert(inspection_target.E), convert(inspection_target.D),
				params[1]};
			for(int i=0;i<9;i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = keys[i];
				pair.value = values[i];
				msg.parameters.push_back(pair);
			}

		} else if ("examine_panel" == msg.name) {

			diagnostic_msgs::KeyValue panel_id;
			panel_id.key = "panel_id";
			panel_id.value = params[1];
			msg.parameters.push_back(panel_id);
		}
	}

	/**
	 * Parses the output of popf, generating a list of action messages.
	 */
	void preparePlan(std::string &dataPath, std::string &mission)
	{
		// popf output
		std::ifstream planfile;
		planfile.open((dataPath + "plan_" + mission + ".pddl").c_str());

		int curr, next;
		std::string line;
		std::vector<planning_msgs::ActionDispatch> potentialPlan;
		std::vector<Point3D> potentialPlanOrientation;
		double planDuration;
		double expectedPlanDuration = 0;
		knowledgeFilter.clear();

		while(!planfile.eof()) {

			getline(planfile, line);

			if (line.substr(0,6).compare("; Plan") == 0) {
				expectedPlanDuration = atof(line.substr(25).c_str());
			} else if (line.substr(0,6).compare("; Time")!=0) {
				//consume useless lines
			} else {

				potentialPlan.clear();
				potentialPlanOrientation.clear();
				PandoraKCL::freeActionID = PandoraKCL::currentAction;
				planDuration = 0;

				while(!planfile.eof() && line.compare("")!=0) {

					getline(planfile, line);
					if (line.length()<2)
						break;

					planning_msgs::ActionDispatch msg;

					// action ID
					msg.action_id = PandoraKCL::freeActionID;
					PandoraKCL::freeActionID++;

					// dispatchTime
					curr=line.find(":");
					double dispatchTime = (double)atof(line.substr(0,curr).c_str());
					msg.dispatch_time = dispatchTime;

					// name
					curr=line.find("(")+1;
					next=line.find(" ",curr);
					std::string name = line.substr(curr,next-curr).c_str();
					msg.name = name;

					// auv
					curr=next+1;
					next=line.find(" ",curr);
					std::string auv = line.substr(curr,next-curr).c_str();
					msg.auv = auv;

					// parameters
					std::vector<std::string> params;
					curr=next+1;
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

					processParameters(msg, params, dispatchTime);

					// duration
					curr=line.find("[",curr)+1;
					next=line.find("]",curr);
					msg.duration = (double)atof(line.substr(curr,next-curr).c_str());

					if(0 == msg.name.compare("turn_valve")) {
						// extra examine
						planning_msgs::ActionDispatch msg_ex;
						msg_ex.action_id = PandoraKCL::freeActionID-1;
						msg_ex.auv = msg.auv;
						msg_ex.dispatch_time = msg.dispatch_time;
						msg_ex.name = "examine_panel";
						diagnostic_msgs::KeyValue panel_id;
						panel_id.key = "panel_id";
						panel_id.value = params[1];
						msg_ex.parameters.push_back(panel_id);
						potentialPlan.push_back(msg_ex);

						msg.action_id = PandoraKCL::freeActionID;
						PandoraKCL::freeActionID++;
						msg.dispatch_time += 1;
					}

					potentialPlan.push_back(msg);

					if(0 == msg.name.compare("turn_valve")) {
						// extra examine
						planning_msgs::ActionDispatch msg_ex;
						msg_ex.action_id = PandoraKCL::freeActionID;
						PandoraKCL::freeActionID++;
						msg_ex.auv = msg.auv;
						msg_ex.dispatch_time = msg.dispatch_time;
						msg.dispatch_time += 1;
						msg_ex.name = "examine_panel";
						diagnostic_msgs::KeyValue panel_id;
						panel_id.key = "panel_id";
						panel_id.value = params[1];
						msg_ex.parameters.push_back(panel_id);
						potentialPlan.push_back(msg_ex);
					}

					// store what we care about for the ontology filter
					if("do_hover_fast" == name || "do_hover_controlled" == name) {

						knowledge_msgs::Filter msg_instance;
						msg_instance.function = knowledge_msgs::Filter::F_INSERT;
						msg_instance.type_name = "Waypoint";
						msg_instance.instance_name = params[0];
						knowledgeFilter.push_back(msg_instance);

						knowledge_msgs::Filter msg_dest;
						msg_dest.function = knowledge_msgs::Filter::F_INSERT;
						msg_dest.type_name = "Waypoint";
						msg_dest.instance_name = params[1];
						knowledgeFilter.push_back(msg_dest);

						knowledge_msgs::Filter msg_conn;
						msg_conn.function = knowledge_msgs::Filter::F_INSERT_OBJ_ATTR;
						msg_conn.type_name = "Waypoint";
						msg_conn.instance_name = params[0];
						msg_conn.obj_property_name = "connectedTo";
						msg_conn.obj_property_type = "Waypoint";
						msg_conn.obj_property_value = params[1];
						knowledgeFilter.push_back(msg_conn);

					} else if("correct_position" == name) {
						
						knowledge_msgs::Filter msg_instance;
						msg_instance.function = knowledge_msgs::Filter::F_INSERT;
						msg_instance.type_name = "Waypoint";
						msg_instance.instance_name = params[0];
						knowledgeFilter.push_back(msg_instance);

					} else if("observe" == name) {

						knowledge_msgs::Filter msg_conn;
						msg_conn.function = knowledge_msgs::Filter::F_INSERT_OBJ_ATTR;
						msg_conn.type_name = "Waypoint";
						msg_conn.instance_name = params[0];
						msg_conn.obj_property_name = "canSee";
						msg_conn.obj_property_type = "InspectionPoint";
						msg_conn.obj_property_value = params[1];
						knowledgeFilter.push_back(msg_conn);

					}


					// get orientation for previous message
					Point3D orientation;
					if("do_hover_controlled" == name || "do_hover_fast" == name) {
						orientation.N = waypoints[params[1]].N;
						orientation.E = waypoints[params[1]].E;
						orientation.D = waypoints[params[1]].D;
					} else if ("correct_position" == name) {
						orientation.N = waypoints[params[0]].N;
						orientation.E = waypoints[params[0]].E;
						orientation.D = waypoints[params[0]].D;
					} else if ("observe_inspection_point" == name) {
						orientation.N = inspectionPoints[params[1]].N;
						orientation.E = inspectionPoints[params[1]].E;
						orientation.D = inspectionPoints[params[1]].D;
					} else if ("observe_pillar" == name) {
						orientation.N = pillars[params[1]].N;
						orientation.E = pillars[params[1]].E;
						orientation.D = pillars[params[1]].D;
					} else if ("illuminate_pillar" == name) {
						orientation.N = pillars[params[1]].N;
						orientation.E = pillars[params[1]].E;
						orientation.D = pillars[params[1]].D;
					} else if ("examine_panel" == name) {
						std::string wp_index = params[1];
						orientation.N = panelPositions[wp_index].N;
						orientation.E = panelPositions[wp_index].E;
						orientation.D = panelPositions[wp_index].D;
					} else if ("turn_valve" == msg.name) {
						std::string valve_index = params[2];
						orientation.N = panelPositions[valves[valve_index]].N;
						orientation.E = panelPositions[valves[valve_index]].E;
						orientation.D = panelPositions[valves[valve_index]].D;
						// extra examine orientation
						potentialPlanOrientation.push_back(orientation);
						// potentialPlanOrientation.push_back(orientation);
						potentialPlanOrientation.push_back(orientation);
					}
					potentialPlanOrientation.push_back(orientation);

					// update plan duration
					curr=line.find(":");
					planDuration = msg.duration + atof(line.substr(0,curr).c_str());
				}

				if(fabs(planDuration - expectedPlanDuration) < 0.01 || true)  {

					// trim any previously read plan
					while(PandoraKCL::actionList.size() > PandoraKCL::currentAction) {
						PandoraKCL::actionList.pop_back();
						PandoraKCL::orientationTarget.pop_back();
					}

					// save better optimised plan
					for(size_t i=0;i<potentialPlan.size();i++) {
						PandoraKCL::actionList.push_back(potentialPlan[i]);
						PandoraKCL::orientationTarget.push_back(potentialPlanOrientation[i]);
					}

					PandoraKCL::totalPlanDuration = planDuration;

				} else {
					ROS_INFO("Duration: %f, expected %f; plan discarded", planDuration, expectedPlanDuration);
				}
			}
		}
		planfile.close();
	}

	/**
	 * Iterates through the plan, adding orientation to GOTO messages, pointing towards
	 * the goal of the next waypoint, if a goto follows, or an inpection point for anything else.
	 */
	void addOrientation()
	{
		Point3D orientation;
		for(int a=PandoraKCL::actionList.size()-1;a>=0;a--) {
			if("goto" == PandoraKCL::actionList[a].name) {
				Point3D destination(
					atof(PandoraKCL::actionList[a].parameters[1].value.c_str()),
					atof(PandoraKCL::actionList[a].parameters[2].value.c_str()),
					atof(PandoraKCL::actionList[a].parameters[3].value.c_str()));
				PandoraKCL::actionList[a].parameters[4].value = convert(findYaw(destination,orientation));
			} else if ("recalibrate_arm" == PandoraKCL::actionList[a].name) {
				// nothing yet
			} else {
				orientation.N = orientationTarget[a].N;
				orientation.E = orientationTarget[a].E;
				orientation.D = orientationTarget[a].D;
			}
		}
	}
} // close namespace
