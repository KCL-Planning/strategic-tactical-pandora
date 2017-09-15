#include "planning_msgs/ActionDispatch.h"
#include "planning_msgs/ActionFeedback.h"

#ifndef KCL_dispatcher
#define KCL_dispatcher

namespace PandoraKCL
{
	/* marker array publishers */
	ros::Publisher ipsPublisher;
	ros::Publisher wpsPublisher;

	/* first free action ID */
	size_t freeActionID;

	/* action dispatch list (current plan) */
	std::vector<std::string> current_mission_ids;
	std::vector<std::string> current_goal_locations;
	std::vector<planning_msgs::ActionDispatch> actionList;
	std::vector<std::vector<planning_msgs::ActionDispatch> > current_plans;
	double totalPlanDuration;
	size_t currentAction;
	bool repeatAction = false;

	/* plan list (for remembering previous plans) */
	std::vector< std::vector<planning_msgs::ActionDispatch> > planList;
	std::vector<size_t> planListLastAction;
	size_t planningAttempts;

	// position the AUV should be oriented towards for each action
	std::vector<Point3D> orientationTarget;

	/* dispatch state */
	std::map<int,bool> actionReceived;
	std::map<int,bool> actionCompleted;
	std::map<std::string,bool> auv_busy;
	bool replanRequested;
	bool strategic_replan_requested;
	bool opportunistic_plan_requested;
	bool dispatchPaused;

	/* opportunistic planning */
	std::string opportunistic_mode;
	std::map<std::string,bool> dispatch_early;
	double free_time_budget;
	std::string opportunistic_mission;
	
	/* action dispatch and feedback methods */
	std::map<std::string,ros::Publisher> actionPublisher;
	std::map<std::string, ros::Subscriber> feedbackSub;
	ros::Publisher free_time_publisher;
	ros::Publisher planPublisher;
	ros::Publisher strategicPlanPublisher;
	ros::Publisher statePublisher;
	ros::Publisher strategicActionPublisher;
	void publishAction(ros::NodeHandle nh);
}

#endif
