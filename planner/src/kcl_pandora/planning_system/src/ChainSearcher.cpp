#include "ChainSearcher.h"

/* The implementation of ChainSearcher.h */
namespace PandoraKCL {

	/* constructor */
	ChainSearcher::ChainSearcher(std::string name)
			:as(n, "chain_searcher", boost::bind(&ChainSearcher::executeCB, this, _1), false), action_name(name)
	{
		latest_link_size = 0;
		action_publisher = n.advertise<planning_msgs::ActionDispatch>("/planning_system/action_dispatch", 1000, true);
		viz_publisher = n.advertise<visualization_msgs::MarkerArray>("/planning_system/chain_search_path", 1000, true);
		ROS_INFO("KCL: (ChainSearcher) starting action server");
		// register Callbacks. Need to use boost::bind because it is inside a class instance
		as.registerPreemptCallback(boost::bind(&ChainSearcher::preemptCB, this));
		// start the server
		as.start();
	}

	/*-------------------*/
	/* Actionlib methods */
	/*-------------------*/

	/* Callback for handling preemption. */
	void ChainSearcher::preemptCB() {
		ROS_INFO("KCL: (ChainSearcher) %s got preempted", action_name.c_str());
		as.setPreempted(result, "I got Preempted!");
	}

	/* Callback for processing a goal */
	void ChainSearcher::executeCB(const planning_msgs::SearchForChainGoalConstPtr& goal) {

		int initialLinkSize = latest_link_size;
		ros::Rate loop_rate(10);

		ROS_INFO("KCL: (ChainSearcher) is waiting for vehicle position on /pose_ekf_slam/odometry");
		positionInitialised = false;
		while (ros::ok() && as.isActive() && !as.isPreemptRequested() && !positionInitialised) {
				ros::spinOnce();
				loop_rate.sleep();
		}

		// if the server has been killed, don't process
		if(!as.isActive() || as.isPreemptRequested()) return;
   
		// initialise grid
		double total_utility = 0.0;
		grid_utility.clear();
		grid_width = goal->search_x;
		grid_height = goal->search_y;
		for(int i=0; i<(grid_width*grid_height); i++) {
			double xOff = 1.0 - fabs(i%grid_width - grid_width/2.0)/grid_width;
			double yOff = (1.0 - fabs(i/grid_width - grid_height/2.0)/grid_height)*0.5;
			xOff += yOff;
			if((abs(i%grid_width - grid_width/2) < 2) && (i/grid_width < grid_height/2))
				xOff =0.0;
			grid_utility.push_back(xOff);
			total_utility += xOff;
		}

		// normalise grid
		for(int i=0; i<(grid_width*grid_height); i++)
			grid_utility[i] = grid_utility[i]*(grid_width*grid_height)/total_utility;

		// search a route
		ROS_INFO("KCL: (ChainSearcher) publishing markers");
		publishMarkerArray();
		ROS_INFO("KCL: (ChainSearcher) searching for chain links");
		getPath(goal->max_steps, grid_width / 2, grid_height / 2);

		// dispatch route as movements
		for(size_t i = 1; i<path.size();i++) {

			// check for ros kill
			if(!ros::ok()){
				ROS_INFO("KCL: (ChainSearcher) %s Shutting Down", action_name.c_str());
				break;
			}

			// if the server has been killed/preempted, stop processing
			if(!as.isActive() || as.isPreemptRequested()){
				return;
			}

			// compute next location
			double xDisplacement = (path[i]/grid_width - grid_height/2);
			double yDisplacement = (path[i]%grid_width - grid_width/2);

			Eigen::Vector3d cell_position(start_position.position.x,start_position.position.y,start_position.position.z);
			Eigen::Quaterniond start_orientation(start_position.orientation.w, start_position.orientation.x, start_position.orientation.y, start_position.orientation.z);
			Eigen::Translation< double, 3 > translation( cell_position );
			Eigen::Transform< double, 3, Eigen::Projective > T = translation * start_orientation;

			Eigen::Vector3d displacement(xDisplacement,yDisplacement,0);
			Eigen::Vector4d dis_hom = displacement.homogeneous();
			dis_hom = T * dis_hom;

			Eigen::Vector3d dhvec = dis_hom.hnormalized();

			double xc = dhvec[0] - start_position.position.x;
			double yc = dhvec[1] - start_position.position.y;
			double yaw = atan2(yc,xc);

			// dispatch next movement
			planning_msgs::ActionDispatch goMsg;
			goMsg.action_id = i;
			goMsg.name = "goto";
			goMsg.duration = 10.0;
			const std::string keys[] = {"movement_type", "north", "east", "depth", "yaw", "pitch"};
			const std::string values[] = {"fast", convert(dhvec[0]) , convert(dhvec[1]), convert(start_position.position.z), convert(yaw), "0"};
			for(int i=0;i<5;i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = keys[i];
				pair.value = values[i];
				goMsg.parameters.push_back(pair);
			}
			ROS_INFO("KCL: (ChainSearcher) Dispatching next movement [%f, %f, %f]", dhvec[0] , dhvec[1], start_position.position.z);
			gotoCompleted[goMsg.action_id] = false;
			action_publisher.publish(goMsg);

			// callback and sleep
			while (ros::ok() && !gotoCompleted[goMsg.action_id] && as.isActive() && !as.isPreemptRequested()) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// check for new links
			if(initialLinkSize < latest_link_size) {
				ROS_INFO("KCL: (ChainSearcher) Succeeded, we have seen links"); 
				result.found = true;
				as.setSucceeded(result);
				return;
			}
		}
 
		// not found
		ROS_INFO("KCL: (ChainSearcher) Completed without finding any links"); 
		as.setAborted(result,"search failed");
	}

	/*------------------*/
	/* Feedback methods */
	/*------------------*/

	/* get position from /pose_ekf_slam/odometry topic (UdG) */
	void ChainSearcher::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {

		// set start position
		if(!positionInitialised) {
			start_position.position.x = msg->pose.pose.position.x;
			start_position.position.y = msg->pose.pose.position.y;
			start_position.position.z = msg->pose.pose.position.z;
			start_position.orientation.x = msg->pose.pose.orientation.x;
			start_position.orientation.y = msg->pose.pose.orientation.y;
			start_position.orientation.z = msg->pose.pose.orientation.z;
			start_position.orientation.w = msg->pose.pose.orientation.w;
		}
		positionInitialised = true;
	}


	void ChainSearcher::feedbackCallback(const planning_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: (ChainSearcher) Feedback received [%i,%s]", msg->action_id, msg->status.c_str());

		// action enabled
		if(!gotoReceived[msg->action_id] && (0 == msg->status.compare("action enabled")))
			gotoReceived[msg->action_id] = true;
		
		// action completed (successfuly)
		if(!gotoCompleted[msg->action_id] && 0 == msg->status.compare("action achieved"))
			gotoCompleted[msg->action_id] = true;
	}

	void ChainSearcher::linkPoseCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		latest_link_size = msg->markers.size();
	}

	/*-------------*/
	/* Pathfinding */
	/*-------------*/

	void ChainSearcher::getPath(int maxSteps, int x, int y) {

		path.clear();
		int current_x = x;
		int current_y = y;
		
		double moving_cost = 0.25;
		double sensor_error = 0.1f;
		
		std::cout << "[ChainSearcher::getPath] maxSteps=" << maxSteps << ", x=" << x << ", y=" << y << std::endl;
		
		while (path.size() != maxSteps)
		{
			// Find the best grid point around us that we have not yet visited.
			int best_grid_to_visit;
			double best_utility = -std::numeric_limits<double>::max();
			
			// Go over the whole grid and pick the cell that has the best utility minus the penalty of moving there.
			for (int y = 0; y < grid_width; ++y)
			{
				for (int x = 0; x < grid_height; ++x)
				{
					int grid_cell = x + y * grid_width;
					double distance = sqrt((current_x - x) * (current_x - x) + (current_y - y) * (current_y - y));
					double utility = grid_utility[grid_cell] - distance * moving_cost;
					if (utility > best_utility)
					{
						best_grid_to_visit = grid_cell;
						best_utility = utility;
					}
				}
			}
			current_x = best_grid_to_visit % grid_width;
			current_y = best_grid_to_visit / grid_width;
			
			// Visit this grid cell and update the probability distribution.
			std::cout << "Best grid cell: " << best_grid_to_visit << "(" << best_utility << ")" << std::	endl;
			path.push_back(best_grid_to_visit);
			
			float divider = (grid_utility[best_grid_to_visit] - grid_utility[best_grid_to_visit] * sensor_error);// / (float)(grid_utility.size() - 1);
			float utility_to_ignore = grid_utility[best_grid_to_visit];
			std::cout << "Devider: " << divider << std::endl;
			float sum = 0;
			for (int y = 0; y < grid_width; ++y)
			{
				for (int x = 0; x < grid_height; ++x)
				{
					int grid_cell = x + y * grid_width;
					if (grid_cell == best_grid_to_visit)
					{
						grid_utility[grid_cell] = grid_utility[grid_cell] * sensor_error;
					}
					else
					{
						grid_utility[grid_cell] = grid_utility[grid_cell] + (grid_utility[grid_cell] / (grid_utility.size() - utility_to_ignore)) * divider;
					}
					std::cout << std::setw(10) << std::right << std::setprecision(6) << grid_utility[grid_cell];
					sum += grid_utility[grid_cell];
				}
				std::cout << std::endl;
			}
			std::cout << "Best grid cell: (" << best_grid_to_visit % grid_width << "," << best_grid_to_visit / grid_width << ") UTILITY:" << best_utility << ". SUM:" << sum << "." << std::endl;
			
			path.push_back(best_grid_to_visit);
		}
	}

	/*---------------*/
	/* Visualization */
	/*---------------*/

	void ChainSearcher::publishMarkerArray() {
		visualization_msgs::MarkerArray marker_array;
		
		size_t counter = 0;
		for (int i=0;i<grid_width*grid_height;i++) {


			// compute location
			double xDisplacement = (i/grid_width - grid_height/2);
			double yDisplacement = (i%grid_width - grid_width/2);

			Eigen::Vector3d cell_position(start_position.position.x,start_position.position.y,start_position.position.z);
			Eigen::Quaterniond start_orientation(start_position.orientation.w, start_position.orientation.x, start_position.orientation.y, start_position.orientation.z);
			Eigen::Translation< double, 3 > translation( cell_position );
			Eigen::Transform< double, 3, Eigen::Projective > T = translation * start_orientation;

			Eigen::Vector3d displacement(xDisplacement,yDisplacement,0);
			Eigen::Vector4d dis_hom = displacement.homogeneous();
			dis_hom = T * dis_hom;

			Eigen::Vector3d dhvec = dis_hom.hnormalized();

			// create marker
			visualization_msgs::Marker marker;

			marker.header.frame_id = "world";
			marker.header.stamp = ros::Time();
			marker.ns = "chain_search_cell";
			marker.id = counter; counter++;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::MODIFY;
			marker.pose.position.x = dhvec[0];
			marker.pose.position.y = dhvec[1];
			marker.pose.position.z = dhvec[2];
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;
			marker.scale.x = 0.3;
			marker.scale.y = 0.3;
			marker.scale.z = 0.3;
			marker.color.a = 1.0;
			marker.color.r = 0.2 + grid_utility[i]/2;
			if(marker.color.r>1.0) marker.color.r = 1.0;
			marker.color.g = grid_utility[i]/2;
			if(marker.color.g>1.0) marker.color.g = 1.0;
			marker.color.b = 0.5;
			marker.text = "";

			marker_array.markers.push_back(marker);
		}

		viz_publisher.publish( marker_array );
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/


	int main(int argc, char **argv) {

		ros::init(argc, argv, "chain_searcher_server");
		ros::NodeHandle nh;

		// spawn the server
		PandoraKCL::ChainSearcher server(ros::this_node::getName());

		// subscriptions
		ros::Subscriber actionFeedback = nh.subscribe("/planning_system/action_feedback", 1000, &PandoraKCL::ChainSearcher::feedbackCallback, &server);
		ros::Subscriber sensorSubOdometry = nh.subscribe("/pose_ekf_slam/odometry", 1, &PandoraKCL::ChainSearcher::odometryCallback, &server);
		ros::Subscriber linkPose = nh.subscribe("/udg_pandora/link_waypoints", 1, &PandoraKCL::ChainSearcher::linkPoseCallback, &server);
		ROS_INFO("KCL: (ChainSearcher) Ready to receive");

		ros::spin();
		return 0;
	}
