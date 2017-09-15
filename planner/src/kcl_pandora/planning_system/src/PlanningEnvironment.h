#ifndef KCL_environment
#define KCL_environment

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include "knowledge_msgs/KnowledgeInterface.h"
#include "knowledge_msgs/Filter.h"

namespace PandoraKCL
{

	/* multi-mission simulation */
	std::vector<std::string> structures;
	std::map<std::string,std::string> structure_wp_names;
	std::map<std::string,Point3D> structure_wps;
	std::map<std::string,std::string> mission_structure_map;
	std::map<std::string,double> mission_duration_map;
	std::map<std::string,double> mission_deadline_map;
	std::vector<planning_msgs::ActionDispatch> strategic_plan;

	/* simulation information */
	bool use_octomap;
	bool use_plan_visualisation;
	std::string missionDomain;
	std::string strategicDomain;

	/* vehicle information */
	Point3D vehicleSize;
	ros::Subscriber sensorSubOdometry;
	ros::Subscriber energyEstimation;
	std::vector<std::string> auv_names;
	std::map<std::string,std::string> auv_starting_locations;
	std::map<std::string,Point3D> auv_starting_waypoints;

	/* any simulation */
	double strategicMissionStart;
	double missionStart;
	double planStart;
	Point3D currentEstimatedLocation;
	Point3D minBounds;
	Point3D maxBounds;
	bool drifted;

	/* knowledge-base filter */
	ros::Publisher filterPublisher;
	ros::Subscriber notificationSub;
	std::vector<knowledge_msgs::Filter> knowledgeFilter;

	/* PDDL to Ontology naming map */
	std::map<std::string,std::string> name_map;

	/* angle to rotate map */
	double theta;

	/* waypoint roadmap */
	std::map<std::string, Point3D> waypoints;
	std::vector<Connection> edges;

	/* inspection point list */
	std::map<std::string, Point3D> inspectionPoints;
	std::map<std::string, std::vector<std::string> > inspectionPointVisibility;
	std::map<std::string,bool> inspectionPointsInspected;
	std::map<std::string,bool> inspectionPointsDiscovered;
	double inspection_distance;

	/* pillars */
	std::map<std::string, Point3D> pillars;
	std::map<std::string, bool> pillarsObserved;
	std::map<std::string, std::vector<std::string> > pillarVisibility;
	std::map<std::string, std::string> inpsectionPointPillar;

	/* valve turning */
	class ValveGoal
	{
		public:
		std::string valve_id;
		double valve_angle;
		double startTime;
		double deadline;
	};

	std::map<std::string,Point3DQuat> panelPositions;
	std::map<std::string,bool> panelExamined;
	std::map<std::string,std::string> valves;
	std::map<std::string,double> valveAngles;
	std::vector<ValveGoal> valveGoals;
	std::map<std::string, std::string> panelVisibility;
	std::map<std::string, std::string> panelReachability;
	std::map<std::string,int> valveBlocked;

	double finalValveDeadline;
	std::map<std::string,double> lastCompletedDeadline;

	/* arm recalibration */
	std::map<std::string,int> arm_calibration;

	/* chain following */
	std::map<std::string,std::string> chain_positions;
	std::map<std::string,bool> chain_examined;

	/* recharge stations */
	std::vector<std::string> recharge_structures;

	/*-----------------*/
	/* parsing strings */
	/*-----------------*/

	/**
	 * parses a Point3D from strings: "[f, f, f]"
	 */
	void parsePoint3D(Point3D &p, std::string line) {
		int curr,next;
		curr=line.find("[")+1;
		next=line.find(",",curr);
		p.N = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		p.E = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find("]",curr);
		p.D = (double)atof(line.substr(curr,next-curr).c_str());
	}

	/**
	 * parses a Point3D with yaw from strings: "[f, f, f, f]"
	 */
	void parsePoint3DwithYaw(Point3D &p, std::string line) {
		int curr,next;
		curr=line.find("[")+1;
		next=line.find(",",curr);
		p.N = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		p.E = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		p.D = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find("]",curr);
		p.ya = (double)atof(line.substr(curr,next-curr).c_str());
	}

	/**
	 * parses a Point3D with Quat. from strings: "[f, f, f, f, f, f, f]"
	 */
	void parsePoint3DwithQuaternion(Point3DQuat &position, std::string line) {

		int curr,next;
		curr=line.find("[")+1;
		next=line.find(",",curr);
		position.N = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		position.E = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		position.D = (double)atof(line.substr(curr,next-curr).c_str());

		curr=next+1; next=line.find(",",curr);
		position.x = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		position.y = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find(",",curr);
		position.z = (double)atof(line.substr(curr,next-curr).c_str());
		curr=next+1; next=line.find("]",curr);
		position.w = (double)atof(line.substr(curr,next-curr).c_str());
	}

	/*-----------------*/
	/* loading mission */
	/*-----------------*/

	/**
	 * Requests from the ontology interface all the information
	 * required to build a strategic problem instance.
	 */
	void updateMissions(ros::NodeHandle nh) {

		ros::ServiceClient knowledgeInterface = nh.serviceClient<knowledge_msgs::KnowledgeInterface>("/knowledge/ontology");

		// AUVs
		{
			// clear old data
			auv_names.clear();
			auv_starting_locations.clear();
			auv_starting_waypoints.clear();

			// create request
			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "AUV";

			if (knowledgeInterface.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.items.size();i++) {
					
					// structure ID
					std::string auv_id = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(auv_id)] = auv_id;
					auv_names.push_back(auv_id);

					// get AUV properties
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "AUV";
					instanceAttrSrv.request.instance_name = instanceSrv.response.items[i].instance_name;

					if (knowledgeInterface.call(instanceAttrSrv)) {

						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {
							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string type = instanceAttrSrv.response.items[j].obj_properties[k].type;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								if(0 == key.compare("location")) {
									name_map[toLowerCase(value)] = value;
									auv_starting_locations[auv_id] = value;
								}
							}
						}

						// get AUV starting position
						knowledge_msgs::KnowledgeInterface startLocAttrSrv;
						startLocAttrSrv.request.request_type = "get_instance_properties";
						startLocAttrSrv.request.type_name = "Waypoint";
						startLocAttrSrv.request.instance_name = auv_starting_locations[auv_id];
						Point3D wp;

						if (knowledgeInterface.call(startLocAttrSrv)) {
							for(size_t j=0;j<startLocAttrSrv.response.items.size();j++) {

								// data properties
								for(size_t k=0;k<startLocAttrSrv.response.items[j].data_properties.size();k++) {
									std::string key = startLocAttrSrv.response.items[j].data_properties[k].name;
									std::string value = startLocAttrSrv.response.items[j].data_properties[k].value;
									std::string parsed_value = value.substr(value.find(',') + 1);
									if(0 == key.compare("wpN")) wp.N = atof(parsed_value.c_str());
									if(0 == key.compare("wpE")) wp.E = atof(parsed_value.c_str());
									if(0 == key.compare("wpD")) wp.D = atof(parsed_value.c_str());
								}
							}
						} else {
							ROS_ERROR("KCL: Failed to call ontology interface for attributes");
						}
						auv_starting_waypoints[auv_id] = wp;

					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// structures and missions
		{
			// clear old data
			structures.clear();
			mission_structure_map.clear();
			structure_wp_names.clear();
			structure_wps.clear();
			recharge_structures.clear();

			// create request
			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Structure";

			if (knowledgeInterface.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// structure ID
					std::string sid = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(sid)] = sid;
					structures.push_back(sid);

					// get structure missions
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Structure";
					instanceAttrSrv.request.instance_name = instanceSrv.response.items[i].instance_name;

					if (knowledgeInterface.call(instanceAttrSrv)) {

						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("Mission")) {
									std::string mid = parsed_value.c_str();
									name_map[toLowerCase(mid)] = mid;
									mission_structure_map[mid] = sid;
								}
								std::cout << key << " " << parsed_value	<< std::endl;
								if(0 == key.compare("has_recharge")) {
									if(0 == parsed_value.compare("true")) {
										recharge_structures.push_back(sid);
										std::cout << "here " << recharge_structures.size() << std::endl;
									}
								}
							}

							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string type = instanceAttrSrv.response.items[j].obj_properties[k].type;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								if(0 == key.compare("start_point")) {
									name_map[toLowerCase(value)] = value;
									structure_wp_names[sid] = value;
								}
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}

					// get structure access point
					knowledge_msgs::KnowledgeInterface accessAttrSrv;
					accessAttrSrv.request.request_type = "get_instance_properties";
					accessAttrSrv.request.type_name = "Waypoint";
					accessAttrSrv.request.instance_name = structure_wp_names[sid];
					Point3D wp;

					if (knowledgeInterface.call(accessAttrSrv)) {
						for(size_t j=0;j<accessAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<accessAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = accessAttrSrv.response.items[j].data_properties[k].name;
								std::string value = accessAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("wpN")) wp.N = atof(parsed_value.c_str());
								if(0 == key.compare("wpE")) wp.E = atof(parsed_value.c_str());
								if(0 == key.compare("wpD")) wp.D = atof(parsed_value.c_str());
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
					structure_wps[sid] = wp;
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}
	}

	/*---------------------*/
	/* loading environment */
	/*---------------------*/

	/**
	 * Requests from the ontology interface all the information
	 * required to build a problem instance.
	 */
	void updateEnvironment(ros::NodeHandle nh, std::string mission) {

		ros::ServiceClient knowledgeInterface = nh.serviceClient<knowledge_msgs::KnowledgeInterface>("/knowledge/ontology");

		mission_deadline_map[mission] = 3600*24;

		// valves
		{
			valves.clear();
			valveAngles.clear();
			valveBlocked.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Valve";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new valve
					std::string vid, pid;
					int blocked_count;
					double angle;
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get valve attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Valve";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {

						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("ID")) vid = parsed_value;
								if(0 == key.compare("panel_ID")) pid = parsed_value.c_str();
								if(0 == key.compare("valve_angle")) angle = atof(parsed_value.c_str());
								if(0 == key.compare("valve_blocked")) blocked_count =  atoi(parsed_value.c_str());
							}

							// valve goals
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {

								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								if(0 == key.compare("goal")) {

									knowledge_msgs::KnowledgeInterface valveGoalSrv;
									valveGoalSrv.request.request_type = "get_instance_properties";
									valveGoalSrv.request.type_name = "ValveGoal";
									valveGoalSrv.request.instance_name = value;
									valveGoalSrv.request.mission_id = mission;

									if (knowledgeInterface.call(valveGoalSrv)) {
										// valve goal properties
										ValveGoal vg0;
										vg0.valve_id = vid;
										for(size_t l=0;l<valveGoalSrv.response.items[0].data_properties.size();l++) {
											std::string goal_key = valveGoalSrv.response.items[0].data_properties[l].name;
											std::string goal_value = valveGoalSrv.response.items[0].data_properties[l].value;
											std::string parsed_value = goal_value.substr(goal_value.find(',') + 1);
											if(0 == goal_key.compare("startTime")) vg0.startTime = atof(parsed_value.c_str());
											if(0 == goal_key.compare("valve_angle")) vg0.valve_angle = atof(parsed_value.c_str());
											if(0 == goal_key.compare("deadline")) {
												vg0.deadline = atof(parsed_value.c_str());
												if(vg0.deadline < mission_deadline_map[mission])
													mission_deadline_map[mission] = vg0.deadline;
											}
										}
										valveGoals.push_back(vg0);
									} else ROS_ERROR("KCL: Failed to call ontology interface for valve goal");
								}
							}
						}

						valves[vid] = pid;
						valveAngles[vid] = angle;
						valveBlocked[vid] = blocked_count;

					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// panels
		{
			panelPositions.clear();
			panelExamined.clear();
			panelVisibility.clear();
			panelReachability.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Panel";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new panel
					std::string pid;
					Point3DQuat position;
					bool examined = false;
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get panel attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Panel";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("ID")) pid = parsed_value;
								if(0 == key.compare("N")) position.N = atof(parsed_value.c_str());
								if(0 == key.compare("E")) position.E = atof(parsed_value.c_str());
								if(0 == key.compare("D")) position.D = atof(parsed_value.c_str());
								if(0 == key.compare("x")) position.x = atof(parsed_value.c_str());
								if(0 == key.compare("y")) position.y = atof(parsed_value.c_str());
								if(0 == key.compare("z")) position.z = atof(parsed_value.c_str());
								if(0 == key.compare("w")) position.w = atof(parsed_value.c_str());
								if(0 == key.compare("examined")) examined = (0 == parsed_value.compare("true"));
							}

							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string type = instanceAttrSrv.response.items[j].obj_properties[k].type;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								
								if(0 == key.compare("canExamine")) {
									panelVisibility[value] = name;
									panelReachability[value] = name;
								}
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}

					panelPositions[pid] = position;
					panelExamined[pid] = examined;
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// waypoints and edges
		{
			edges.clear();
			waypoints.clear();
			inspectionPointVisibility.clear();
			pillarVisibility.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Waypoint";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {

				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new waypoint
					Point3D wp;
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get waypoint attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Waypoint";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("wpN")) wp.N = atof(parsed_value.c_str());
								if(0 == key.compare("wpE")) wp.E = atof(parsed_value.c_str());
								if(0 == key.compare("wpD")) wp.D = atof(parsed_value.c_str());
							}

							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {

								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);

								// "connectedTo" -> "wp_instance_name"
								if(0 == key.compare("connectedTo")) {
									Connection conn;
									conn.start = name;
									conn.end = parsed_value;
									edges.push_back(conn);
								}
								// "canSee" -> "ip_instance_name"
								if(0 == key.compare("canSee")) {
									inspectionPointVisibility[name];
									inspectionPointVisibility[name].push_back(parsed_value);
								}
								// "canSeePillar" -> "pillar_instance_name"
								if(0 == key.compare("canSeePillar")) {
									pillarVisibility[name];
									pillarVisibility[name].push_back(parsed_value);
								}
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
					waypoints[name] = wp;
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// inspection points
		{
			inspectionPoints.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "InspectionPoint";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {

				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new inspection point
					Point3D ip;
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get IP attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "InspectionPoint";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("ipN")) ip.N = atof(parsed_value.c_str());
								if(0 == key.compare("ipE")) ip.E = atof(parsed_value.c_str());
								if(0 == key.compare("ipD")) ip.D = atof(parsed_value.c_str());
								if(0 == key.compare("ipY")) ip.ya = atof(parsed_value.c_str());
							}

							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {

								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);

								// "isPartOfPillar" -> "pillar_instance_name"
								if(0 == key.compare("isPartOfPillar")) {
									inpsectionPointPillar[name] = value;
								}
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
					inspectionPoints[name] = ip;
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// chain
		{
			chain_positions.clear();
			chain_examined.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Chain";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {

				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new chain
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get IP attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Chain";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("examined")) chain_examined[name] = ("true"==parsed_value);
							}

							// object properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].obj_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].obj_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].obj_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("location")) {
									chain_positions[name] = value;
									structure_wp_names["chain_structure"] = value;
									structure_wps["chain_structure"] = waypoints[value];
								}
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}

		// pillar
		{
			pillars.clear();

			knowledge_msgs::KnowledgeInterface instanceSrv;
			instanceSrv.request.request_type = "get_type_instances";
			instanceSrv.request.type_name = "Pillar";
			instanceSrv.request.mission_id = mission;

			if (knowledgeInterface.call(instanceSrv)) {

				for(size_t i=0;i<instanceSrv.response.items.size();i++) {

					// add new pillar
					Point3D p;
					std::string name = instanceSrv.response.items[i].instance_name;
					name_map[toLowerCase(name)] = name;

					// get pillar attributes
					knowledge_msgs::KnowledgeInterface instanceAttrSrv;
					instanceAttrSrv.request.request_type = "get_instance_properties";
					instanceAttrSrv.request.type_name = "Pillar";
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.mission_id = mission;

					if (knowledgeInterface.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.items.size();j++) {

							// data properties
							for(size_t k=0;k<instanceAttrSrv.response.items[j].data_properties.size();k++) {
								std::string key = instanceAttrSrv.response.items[j].data_properties[k].name;
								std::string value = instanceAttrSrv.response.items[j].data_properties[k].value;
								std::string parsed_value = value.substr(value.find(',') + 1);
								if(0 == key.compare("pN")) p.N = atof(parsed_value.c_str());
								if(0 == key.compare("pE")) p.E = atof(parsed_value.c_str());
								if(0 == key.compare("pD")) p.D = atof(parsed_value.c_str());
							}
						}
					} else {
						ROS_ERROR("KCL: Failed to call ontology interface for attributes");
					}
					pillars[name] = p;
				}
			} else {
				ROS_ERROR("KCL: Failed to call ontology interface for %s", instanceSrv.request.type_name.c_str());
			}
		}
	}

	/**
	 * Loads environment data from config file.
	 */
	void loadConfigFile(std::string &dataPath)
	{
		// clear some things
		valveGoals.clear();
		inspectionPointVisibility.clear();
		pillarVisibility.clear();
		panelReachability.clear();

		// load configuration file
		std::ifstream infile((dataPath+"config.txt").c_str());
		std::string line;
		size_t curr, next;
		{
			// vehicle size
			std::getline(infile, line);
			parsePoint3D(vehicleSize, line.substr(line.find("[")));
		
			// bounds
			std::getline(infile, line);
			parsePoint3D(minBounds, line.substr(line.find("[")));

			std::getline(infile, line);
			parsePoint3D(maxBounds, line.substr(line.find("[")));

			// theta
			std::getline(infile, line);
			curr=line.find("[")+1;
			next=line.find("]",curr);
			theta = (double)atof(line.substr(curr,next-curr).c_str());

			// inspection points
			size_t counter = 0;
			std::getline(infile, line);
			curr=line.find("[")+1;
			while(curr != std::string::npos) {
				Point3D ip;
				parsePoint3DwithYaw(ip, line.substr(curr));
				ip.rotate(theta);
				std::stringstream ss;
				ss << "ip" << counter;
				inspectionPoints[ss.str()] = ip;
				name_map[ss.str()] = ss.str();
				curr=line.find("[",curr+1);
				counter++;
			}

			// max inspection distance
			std::getline(infile, line);
			curr=line.find("[")+1;
			next=line.find("]",curr);
			inspection_distance = (double)atof(line.substr(curr,next-curr).c_str());

			// valve goals
			finalValveDeadline = 0;
/*
			lastCompletedDeadline[0] = 0;
			lastCompletedDeadline[1] = 0;
			lastCompletedDeadline[2] = 0;
			lastCompletedDeadline[3] = 0;

			std::getline(infile, line);
			curr=line.find("[")+1;
			std::string varName = line.substr(0,curr-1);
			
			while(varName == "valve goal angle ") {

				ValveGoal vg0;
				next=line.find(",",curr);
				vg0.valve_id = atoi(line.substr(curr,next-curr).c_str());
	
				curr = next+1;
				next = line.find(",",curr);
				vg0.startTime = (double)atof(line.substr(curr,next-curr).c_str());

				curr = next+1;
				next = line.find(",",curr);
				vg0.deadline = (double)atof(line.substr(curr,next-curr).c_str());
				if(finalValveDeadline < vg0.deadline) finalValveDeadline = vg0.deadline;

				curr = next+1;
				next = line.find("]",curr);
				vg0.valve_angle = (double)atof(line.substr(curr,next-curr).c_str());

				valveGoals.push_back(vg0);

				std::getline(infile, line);
				curr=line.find("[")+1;
				varName = line.substr(0,curr-1);
			}
*/
		}
	}

	double getCurrentValveGoal(const std::string& valve_id, double time) {
		
		double angle = 0;
		double closestDeadline = -1;
		for(size_t i=0;i<valveGoals.size();i++) {
			if(valve_id == valveGoals[i].valve_id && time < valveGoals[i].deadline) {
			if(closestDeadline<0 || valveGoals[i].deadline<closestDeadline) {
				closestDeadline = valveGoals[i].deadline;
				angle = valveGoals[i].valve_angle;
			}}
		}
		return angle;
	}

	double getCurrentValveDeadline(const std::string& valve_id, double time) {
		
		double closestDeadline = -1;
		for(size_t i=0;i<valveGoals.size();i++) {
			if(valve_id == valveGoals[i].valve_id && time < valveGoals[i].deadline) {
			if(closestDeadline<0 || valveGoals[i].deadline<closestDeadline) {
				closestDeadline = valveGoals[i].deadline;
			}}
		}
		return closestDeadline;
	}

	bool deadlinesCompleted() {
		for(size_t i=0;i<valveGoals.size();i++) {
			if(lastCompletedDeadline[valveGoals[i].valve_id] < valveGoals[i].deadline)
				return false;
		}
		return true;
	}
}

#endif
