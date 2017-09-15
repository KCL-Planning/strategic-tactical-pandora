#include "VisualiserTimer.h"

float VisualiserTimer::latest_received_time_ = 0;

VisualiserTimer::VisualiserTimer(ros::NodeHandle& node)
{
	timer_sub_ = node.subscribe("/visualiser/timer", 1, &VisualiserTimer::receiveTime, this);
}

void VisualiserTimer::receiveTime(const std_msgs::Float32::ConstPtr& msg)
{
	latest_received_time_ = msg->data;
}

float VisualiserTimer::getTime()
{
	return VisualiserTimer::latest_received_time_;
}
