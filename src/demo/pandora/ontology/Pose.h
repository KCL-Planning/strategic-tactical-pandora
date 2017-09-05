#ifndef DEMO_PANDORA_ONTOLOGY_POSE_H
#define DEMO_PANDORA_ONTOLOGY_POSE_H

/**
 * Struct for a pose used in the ontology.
 */
struct Pose
{
	Pose(float x, float y, float z, float pitch, float yaw)
		: x_(x), y_(y), z_(z), pitch_(pitch), yaw_(yaw)
	{
		
	}
	
	float x_, y_, z_, pitch_, yaw_;
};

std::ostream& operator<<(std::ostream& os, const Pose& pose);

#endif
