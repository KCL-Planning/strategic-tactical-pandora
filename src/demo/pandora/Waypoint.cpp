#include "Waypoint.h"

std::ostream& operator<<(std::ostream& os, const Waypoint& waypoint)
{
	os << " *** <WAYPOINT> *** " << std::endl;
	os << "Waypoint[" << waypoint.id_ << "]: (" << waypoint.position_.x << ", " << waypoint.position_.y << ", " << waypoint.position_.z << ")" << std::endl;
	os << "Connectivity: " << std::endl;
	for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = waypoint.edges_.begin(); ci != waypoint.edges_.end(); ++ci)
	{
		os << "\t[" << (*ci).first->id_ << "]: (" << (*ci).first->position_.x << ", " << (*ci).first->position_.y << ", " << (*ci).first->position_.z << ") -- " << (*ci).second << std::endl;
	}
	os << "Connected to start?" << std::endl;
	for (std::vector<bool>::const_iterator ci = waypoint.is_connected_to_start_.begin(); ci != waypoint.is_connected_to_start_.end(); ++ci)
	{
		os << "\t" << *ci << std::endl;
	}
	os << " *** </WAYPOINT> *** " << std::endl;
	return os;
}
