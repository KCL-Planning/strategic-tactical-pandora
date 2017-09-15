#include "ros/ros.h"

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

	/*---------*/
	/* Testing */
	/*---------*/

	/**
	 * Sets up ROS; main loop.
	 */
	void runValveTurner()
	{
		ros::NodeHandle nh("~");
		ROS_INFO("KCL: --- valveTester ---");

		// publishing "action_dispatch"; listening "action_feedback"
		actionPublisher = nh.advertise<planning_msgs::ActionDispatch>("action_dispatch", 1000, true);
		feedbackSub = nh.subscribe("action_feedback", 1000, PandoraKCL::feedbackCallback);

		// Loop through and publish planned actions
		planning_msgs::ActionDispatch currentMessage;
		currentMessage.action_id = 0;
		currentMessage.name = "turn_valve";
		
		diagnostic_msgs::KeyValue pair;
		pair.key = "valve_id";
		pair.value = "0";
		currentMessage.parameters.push_back(pair);

		diagnostic_msgs::KeyValue pair1;
		pair1.key = "desired_increment";
		pair1.value = "1.57";
		currentMessage.parameters.push_back(pair1);

		// dispatch action
		ROS_INFO("KCL: Dispatching action: [%i, %s, %f]", currentMessage.action_id, currentMessage.name.c_str(), currentMessage.duration);
		actionPublisher.publish(currentMessage);

		// callback and sleep
		ros::Rate loop_rate(10);
		PandoraKCL::actionCompleted[PandoraKCL::currentAction] = false;
		while (ros::ok() && !PandoraKCL::actionCompleted[PandoraKCL::currentAction]) {
			ros::spinOnce();
			loop_rate.sleep();
		}

		ROS_INFO("KCL: Valve turning testing complete");
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"planning_system");
		srand (static_cast <unsigned> (time(0)));
		PandoraKCL::runValveTurner();

		return 0;
	}
