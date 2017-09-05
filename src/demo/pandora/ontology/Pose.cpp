#include <iostream>

#include "Pose.h"

std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
	os << "(" << pose.x_ << ", " << pose.y_ << ", " << pose.z_ << "); Pitch: " << pose.pitch_ << "; Yaw: " << pose.yaw_;
	return os;
}
