#ifndef PLANNING_SYSTEM_VISUALISER_TIMER_H
#define PLANNING_SYSTEM_VISUALISER_TIMER_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class VisualiserTimer
{
public:
	VisualiserTimer(ros::NodeHandle& node);
	
	static float getTime();
private:
	
	void receiveTime(const std_msgs::Float32::ConstPtr& msg);
	
	static float latest_received_time_;
	ros::Subscriber timer_sub_; 
};

#endif
