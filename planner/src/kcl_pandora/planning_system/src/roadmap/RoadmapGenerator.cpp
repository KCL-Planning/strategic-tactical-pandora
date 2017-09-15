#include "ros/ros.h"

#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace PandoraKCL {


/*	void print_query_info(octomap::point3d query, octomap::OcTreeNode* node) {
		if (node != NULL) std::cout << query << ":\t " << node->getOccupancy() << std::endl;
		else std::cout << "No data" << std::endl;
	}
*/
	/*---------------------*/
	/* Collision Detection */
	/*---------------------*/

	/* Check against the octomap to see if the specified point is free. */
/*	bool isPointCollisionFree(Point3D wp, octomap::OcTree &tree) {
			octomap::point3d query(wp.N, -1*wp.E, -1*wp.D); //flip wp.E if axis are wrong
			octomap::OcTreeNode* result = tree.search (query);
			// print_query_info(query, result);
			return (result == NULL || result->getOccupancy() <= 0.4);
	}
*/
	/* Check against the octomap to see if the specified space is free for the vehicle. */
/*	bool isSpaceCollisionFree(Point3D wp, octomap::OcTree &tree) {
		Point3D check;
		for(float i=-vehicleSize.N/2;i<vehicleSize.N/2;i+=0.1) {
		for(float j=-vehicleSize.E/2;j<vehicleSize.E/2;j+=0.1) {
		for(float k=-vehicleSize.D/2;k<v#include <Eigen/Dense>ehicleSize.D/2;k+=0.1) {
			check.N = wp.N + i;
			check.E = wp.E + j;
			check.D = wp.D + k;
			if(!isPointCollisionFree(check,tree))
				return false;
		}}};
		return true;
	}
*/

	/* Check against the octomap to see if the specified connection is free for the vehicle. */
/*	bool isConnectionCollisionFree(Point3D source, Point3D dest, octomap::OcTree &tree) {

		Point3D inter(source.N,source.E,source.D);
		Point3D step(
			dest.N - source.N,
			dest.E - source.E,
			dest.D - source.D);
		float mag = step.mag();
		step.N = 0.1*step.N/mag;
		step.E = 0.1*step.E/mag;
		step.D = 0.1*step.D/mag;

		Point3D check;
		while(computeDistance(dest, inter)>0.1) {

			inter.N += step.N;
			inter.E += step.E;
			inter.D += step.D;

			float mag = step.mag2D();
			for(float i=-vehicleSize.E/2;i<vehicleSize.E/2;i+=0.1) {
			for(float j=-vehicleSize.D/2;j<vehicleSize.D/2;j+=0.1) {

				check.N = inter.N + i*step.E/mag;
				check.E = inter.E + i*-step.N/mag;
				check.D = inter.D;

				if(!isPointCollisionFree(check,tree))
					return false;
			}};
		}
		return true;
	}
*/
	/*---------------*/
	/* Build Roadmap */
	/*---------------*/

	/**
	 * Store the inspection points (possible valve positions)
	 */
/*	void buildInspectionPoints(std::string &dataPath) {
		ROS_INFO("KCL: Adding Inspection Points (from ontology)");
	}
*/
	/* Generate edges for waypoint, connecting to nearest neighbours.  Return wether it was connected. */
/*	bool connectWaypoint(std::string wpID, Point3D wp, octomap::OcTree &tree) {
		bool connected = false;
		for (std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin(); wit!=PandoraKCL::waypoints.end(); ++wit) {
			if(0 == wpID.compare(wit->first)) continue;
			if(isConnectionCollisionFree(wp,wit->second,tree)) {
				Connection conn;
				conn.start = wpID;
				conn.end = wit->first;
				edges.push_back(conn);
				connected = true;
			}
		}
		return connected;
	}
*/

/*	void connectWaypoints(octomap::OcTree &tree) {
		// TODO reimplement proper PRM
		for (std::map<std::string,Point3D>::iterator wit=PandoraKCL::waypoints.begin(); wit!=PandoraKCL::waypoints.end(); ++wit) {
			connectWaypoint(wit->first, wit->second, tree);
		}
	}
*/
	void connectWaypointsWithoutCollision() {
		for (std::map<std::string,Point3D>::iterator wit_i=PandoraKCL::waypoints.begin(); wit_i!=PandoraKCL::waypoints.end(); ++wit_i) {
		for (std::map<std::string,Point3D>::iterator wit_j=PandoraKCL::waypoints.begin(); wit_j!=PandoraKCL::waypoints.end(); ++wit_j) {
			if(0 != wit_i->first.compare(wit_j->first)) {
				Connection conn;
				conn.start = wit_i->first;
				conn.end = wit_j->first;
				edges.push_back(conn);
			}
		}};
	}

	/**
	 * Builds the Probabalistic Roadmap (PRM)
	 */
	void buildPRM(std::string &dataPath) {

		ROS_INFO("KCL: Building PRM");

		// build the roadmap
		waypoints.clear();
		double coorN,coorE,coorD;

		// generate initial WP from odometry (should be [0,0,*] at start)
		coorN = PandoraKCL::currentEstimatedLocation.N;
		coorE = PandoraKCL::currentEstimatedLocation.E;
		coorD = PandoraKCL::currentEstimatedLocation.D;
		Point3D wp(coorN, coorE, coorD);
		waypoints["wp0"] = wp;
		name_map["wp0"] = "wp0";

		// add strategic waypoints to roadmap
		ROS_INFO("KCL: Adding Strategic points");
		
		// inspection points
		for (std::map<std::string,Point3D>::iterator it=PandoraKCL::inspectionPoints.begin(); it!=PandoraKCL::inspectionPoints.end(); ++it) {
			Point3D sp(-inspection_distance, 0, 0);
			sp.rotate(it->second.ya);
			sp.N = it->second.N + sp.N;
			sp.E = it->second.E + sp.E;
			sp.D = it->second.D;
			std::stringstream ss;
			ss << "wp_strat_" << it->first;
			waypoints[ss.str()] = sp;
			name_map[ss.str()] = ss.str();
			inspectionPointVisibility[ss.str()];
			inspectionPointVisibility[ss.str()].push_back(it->first);			
		}

		// valve panel inspection locations
		for (std::map<std::string,Point3DQuat>::iterator it=PandoraKCL::panelPositions.begin(); it!=PandoraKCL::panelPositions.end(); ++it) {

			Point3DQuat pose = it->second;

			Eigen::Vector3d panel_position(pose.N,pose.E,pose.D);
			Eigen::Quaterniond panel_orientation(pose.w, pose.x, pose.y, pose.z);
			Eigen::Translation< double, 3 > translation( panel_position );
			Eigen::Transform< double, 3, Eigen::Projective > T = translation * panel_orientation;

			Eigen::Vector3d displacement(0,0,inspection_distance);
			Eigen::Vector4d dis_hom = displacement.homogeneous();
			dis_hom = T * dis_hom;

			Eigen::Vector3d dhvec = dis_hom.hnormalized();
			Point3D sp(dhvec[0], dhvec[1], dhvec[2]);

			std::stringstream ss;
			ss << "wp_strat_panel" << it->first;
			waypoints[ss.str()] = sp;
			name_map[ss.str()] = ss.str();
			panelVisibility[ss.str()] = it->first;
		}

		// valve panel inspection locations
		for (std::map<std::string,Point3DQuat>::iterator it=PandoraKCL::panelPositions.begin(); it!=PandoraKCL::panelPositions.end(); ++it) {

			Point3DQuat pose = it->second;

			Eigen::Vector3d panel_position(pose.N,pose.E,pose.D);
			Eigen::Quaterniond panel_orientation(pose.w, pose.x, pose.y, pose.z);
			Eigen::Translation< double, 3 > translation( panel_position );
			Eigen::Transform< double, 3, Eigen::Projective > T = translation * panel_orientation;

			Eigen::Vector3d displacement(0,0,2.25);
			Eigen::Vector4d dis_hom = displacement.homogeneous();
			dis_hom = T * dis_hom;

			Eigen::Vector3d dhvec = dis_hom.hnormalized();
			Point3D sp(dhvec[0], dhvec[1], dhvec[2]);

			std::stringstream ss;
			ss << "wp_reach_panel" << it->first;
			waypoints[ss.str()] = sp;
			name_map[ss.str()] = ss.str();
			panelReachability[ss.str()] = it->first;
		}

 		if(PandoraKCL::use_octomap) {
			octomap::OcTree tree((dataPath + "map.bt").c_str());
			connectWaypoints(tree);
		} else {
			connectWaypointsWithoutCollision();
		}
	}
} // close namespace
