#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <iomanip> 
#include <boost/foreach.hpp>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>

#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include <planning_msgs/ActionDispatch.h>
#include <planning_msgs/ActionFeedback.h>
#include <planning_msgs/SearchForChainAction.h>
#include <planning_msgs/SearchForChainGoal.h>
#include <planning_msgs/SearchForChainFeedback.h>
#include <planning_msgs/SearchForChainResult.h>
#include "Utilities.h"

#ifndef KCL_chainsearcher
#define KCL_chainsearcher

/**
 * This file defines the PANDORA chain search class.
 */
namespace PandoraKCL {

	class ChainSearcher
	{

	private:

		ros::Publisher action_publisher;
		ros::Publisher viz_publisher;

	protected:

		/* action server */
		ros::NodeHandle n;
		actionlib::SimpleActionServer<planning_msgs::SearchForChainAction> as;
		planning_msgs::SearchForChainFeedback feedback;
		planning_msgs::SearchForChainResult result;
		std::string action_name;

		/* vehicle position */
		geometry_msgs::Pose start_position;
		bool positionInitialised;

		/* action dispatch */
		void publishMarkerArray();
		std::map<int,int> gotoReceived;
		std::map<int,int> gotoCompleted;

		/* grid search */
		int latest_link_size;
		int grid_width;
		int grid_height;
		std::vector<double> grid_utility;
		std::vector<int> path;

		void preemptCB();
		void executeCB(const planning_msgs::SearchForChainGoalConstPtr& goal);
		void getPath(int maxSteps, int x, int y);

	public:

		/* constructor */
		ChainSearcher(std::string name);

		/* listen to action_feedback, odometry, and link poses */
		void feedbackCallback(const planning_msgs::ActionFeedback::ConstPtr& msg);
		void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
		void linkPoseCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
	};
}
#endif
