#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
//#include "auv_msgs/NavSts.h"
//#include "ntua_controlNessie/EnergyEstimation.h"

/**
 * listener to update the current AUV position and stats.
 * This is used as the initial position of the vehicle during PRM construction;
 * it may be later replaced by a call to the ontology.
 */
namespace PandoraKCL
{

	bool positionInitialised = false;

	/* get position from /pose_ekf_slam/odometry topic (UdG) */
	void odometryCallbackUDG(const nav_msgs::Odometry::ConstPtr& msg)
	{
		PandoraKCL::currentEstimatedLocation.N = msg->pose.pose.position.x;
		PandoraKCL::currentEstimatedLocation.E = msg->pose.pose.position.y;
		PandoraKCL::currentEstimatedLocation.D = msg->pose.pose.position.z;
		positionInitialised = true;
	}

	/* get position from /nav/nav_sts topic (HWU) 
	void odometryCallbackHWU(const auv_msgs::NavSts::ConstPtr& msg)
	{
		PandoraKCL::currentEstimatedLocation.N = msg->position.north;
		PandoraKCL::currentEstimatedLocation.E = msg->position.east;
		PandoraKCL::currentEstimatedLocation.D = msg->position.depth;
		positionInitialised = true;
	}*/

	/* get energy estimation from /path_planning/energyEstimation topic (NTUA/HWU) 
	void totalEnergyCallbackHWU(const ntua_controlNessie::EnergyEstimation::ConstPtr& msg)
	{
		// nothing yet
	}*/
} //close namespace
