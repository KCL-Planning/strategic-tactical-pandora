#include "ROSTimer.h"

#include <std_msgs/Float32.h>

ROSTimer::ROSTimer(ros::NodeHandle& node)
	: time_(0)
{
	time_publisher_ = node.advertise<std_msgs::Float32>("/visualiser/timer", 1);
}

void ROSTimer::update(float dt)
{
	time_ += dt;
	
	std_msgs::Float32 time_msg;
	time_msg.data = time_;
	time_publisher_.publish(time_msg);
}
