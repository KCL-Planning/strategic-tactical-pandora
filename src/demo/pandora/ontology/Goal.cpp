#include "Goal.h"
#include <sstream>

int Goal::next_goal_id_ = 0;

Goal::Goal(Structure& structure)
	: structure_(&structure)
{
	std::stringstream ss;
	ss << "goal_" << next_goal_id_;
	id_ = ss.str();
	++next_goal_id_;
}

Goal::~Goal()
{
	
}
