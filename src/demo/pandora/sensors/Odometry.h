/**
 * In order to visalise the robot in RVIZ we add an odometry component that tracks
 * the location of the robot relative to the global frame (/map).
 */
#ifndef DEMO_PANDORA_SENSORS_ODOMETRY_H
#define DEMO_PANDORA_SENSORS_ODOMETRY_H

#include <ros/ros.h>

namespace DreadedPE
{
	class Entity;
};

class Odometry
{
public:
	/**
	 * Create an odometry sensor for the given entity.
	 */
	Odometry(DreadedPE::Entity& entity);

	/**
	 * Send a message through ROS which contains the location of this entity.
	 */
	void update(float dt);
private:
	DreadedPE::Entity* entity_;
};

#endif

