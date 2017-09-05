#ifndef PANDORA_ROS_TIMER_H
#define PANDORA_ROS_TIMER_H


#include <ros/ros.h>

/**
 * This class is to replace the ROS timer so we can speedup the simulation and the planner at the same time.
 */
class ROSTimer
{
public:
	ROSTimer(ros::NodeHandle& node);
	
	void update(float dt);
	
	float getTime() const { return time_; }
	
private:
	float time_;
	
	ros::Publisher time_publisher_;
};

#endif
