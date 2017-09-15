#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "../Utilities.h"
#include "../roadmap/Roadmap.h"
#include "../PlanningEnvironment.h"
#include "../ActionDispatch.h"
#include "../ActionFeedback.cpp"

#include <fstream>
#include <sstream>
#include <string>
#include <ctime>

namespace PandoraKCL {

	/* marker array publishers */
	ros::Publisher trajPublisher;

	/* waypoints */
	std::vector<Point3D> wps;
	int skip = 1;

	/**
	 * output all trajectory points
	 */
	void publishTrajectoryMarkerArray(ros::NodeHandle nh, size_t start_index, size_t count)
	{
		visualization_msgs::MarkerArray marker_array;
		
		for (size_t i=start_index; i < start_index+count && i < wps.size(); i++) {

			visualization_msgs::Marker marker;

			marker.header.frame_id = "world";
			marker.header.stamp = ros::Time();
			marker.ns = "mission_trajectory_point";
			marker.id = i - start_index;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			marker.pose.position.x = wps[i].N;
			marker.pose.position.y = wps[i].E;
			marker.pose.position.z = wps[i].D;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.5;
			marker.scale.y = 0.5;
			marker.scale.z = 0.5;
			marker.color.a = 0.5;

			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;

			// HWU simulation
			marker.header.frame_id = "map";
			marker.pose.position.y = -wps[i].E;
			marker.pose.position.z = -wps[i].D;

			marker_array.markers.push_back(marker);
		}
		trajPublisher.publish( marker_array );
	}

	/**
	 * Generates a series of trajectories in the form of action dispatch messages.
	 * Populates the actionList vector.
	 */
	void makeTrajectory(ros::NodeHandle nh) {

		std::string dataPath;
		nh.param("data_path", dataPath, std::string("data/"));
		ROS_INFO("KCL: Using data path: %s", dataPath.c_str());

		// prepare file
		std::ifstream wpsFile((dataPath+"path").c_str());

		// retrieve path data
		std::string line;
		double x, y, z;

		int l=0;
		while (getline(wpsFile,line)) {
			std::istringstream s(line);
			s >> x >> y >> z;
			if(l % skip == 0) {
				Point3D pos(-10*x,10*y,1.5);
				wps.push_back(pos);
			}
			l++;		
		}
		wpsFile.close();

		size_t wpIndex = 0;
		while(wpIndex+10 < wps.size()) {

			planning_msgs::ActionDispatch msg;
			msg.name = "goto";

			// movement type (fast || controlled)
			diagnostic_msgs::KeyValue pair;
			pair.key = "movement_type";
			pair.value = "fast";
			msg.parameters.push_back(pair);

			// goal position
			Point3D destination(
				wps[wpIndex+9].N,
				wps[wpIndex+9].E,
				wps[wpIndex+9].D);
			Point3D orientation(
				wps[wpIndex+10].N,
				wps[wpIndex+10].E,
				wps[wpIndex+10].D);
			const std::string keys[] = {"north", "east", "depth", "yaw", "pitch"};
			const std::string values[] = {
				convert(destination.N), convert(destination.E), convert(destination.D),
				convert(findYaw(destination,orientation)), "0"};
			for(int i=0;i<5;i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = keys[i];
				pair.value = values[i];
				msg.parameters.push_back(pair);
			}

			// trajectory
			Point3D inter(0,0,0);
			std::vector<Point3D> traj;
			for(int i=0;i<10;i++) {

				traj.push_back(wps[wpIndex+i]);

				diagnostic_msgs::KeyValue pair;
				std::stringstream sk;
				sk << "t" << i;
				pair.key = sk.str();

				std::stringstream ss;
				ss << wps[wpIndex+i].N << "," << wps[wpIndex+i].E << "," << wps[wpIndex+i].D
						<< "," << convert(findYaw(wps[wpIndex+i],wps[wpIndex+i+1])) << ",0";
				pair.value = ss.str();

				msg.parameters.push_back(pair);
			}
			actionList.push_back(msg);

			wpIndex+=10;
		}
	}

	/*---------*/
	/* Testing */
	/*---------*/

	/**
	 * Sets up ROS; prepares trajectories; main loop.
	 */
	void runTrajectoryServer()
	{
		ros::NodeHandle nh("~");
		ROS_INFO("KCL: --- trajectoryTester ---");

		// marker array publishers
		trajPublisher = nh.advertise<visualization_msgs::MarkerArray>( "planning_trajectory_marker_array", 1000, true);
		
		// prepare paths
		makeTrajectory(nh);

		// setup environment
		std::string dataPath;
		nh.param("data_path", dataPath, std::string("data/"));
		ROS_INFO("KCL: Using data path: %s", dataPath.c_str());

		// publishing "action_dispatch"; listening "action_feedback"
		actionPublisher = nh.advertise<planning_msgs::ActionDispatch>("action_dispatch", 1000, true);
		feedbackSub = nh.subscribe("action_feedback", 1000, PandoraKCL::feedbackCallback);

		ros::Rate loop_rate(10);
		ros::Rate poll_rate(1);
		while(actionPublisher.getNumSubscribers() == 0)
		    poll_rate.sleep();

		// Loop through and publish planned actions
		ROS_INFO("KCL: Waiting before dispatch...");
	    poll_rate.sleep();
		while (ros::ok() && PandoraKCL::actionList.size() > PandoraKCL::currentAction) {

			planning_msgs::ActionDispatch currentMessage = PandoraKCL::actionList[PandoraKCL::currentAction];
			currentMessage.action_id = PandoraKCL::currentAction;

			// loop while dispatch is paused
			while (ros::ok() && PandoraKCL::dispatchPaused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// dispatch action
			ROS_INFO("KCL: Dispatching action: [%i, %s, %f]", currentMessage.action_id, currentMessage.name.c_str(), currentMessage.duration);
			for(size_t i=0;i<currentMessage.parameters.size();i++) {
				ROS_INFO("\t%s:%s", currentMessage.parameters[i].key.c_str(), currentMessage.parameters[i].value.c_str());
			}
			actionPublisher.publish(currentMessage);
			publishTrajectoryMarkerArray(nh,PandoraKCL::currentAction*10,10);

			// callback and sleep
			while (ros::ok() && !PandoraKCL::actionCompleted[PandoraKCL::currentAction]) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// get ready for next action
			PandoraKCL::currentAction++;
			PandoraKCL::actionReceived[PandoraKCL::currentAction] = false;
			PandoraKCL::actionCompleted[PandoraKCL::currentAction] = false;
		}
		ROS_INFO("KCL: Trajectories Complete");
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"planning_system");
		srand (static_cast <unsigned> (time(0)));
		PandoraKCL::runTrajectoryServer();

		return 0;
	}
