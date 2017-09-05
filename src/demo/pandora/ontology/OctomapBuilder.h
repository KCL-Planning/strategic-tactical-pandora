#ifndef DEMO_PANDORA_ONTOLOGY_OCTOMAP_BUILDER_H
#define DEMO_PANDORA_ONTOLOGY_OCTOMAP_BUILDER_H

// For debugging we can use the ground truth location of the turtlebot.
#define _USE_GROUND_TRUTH

#include <ros/ros.h>

#include <octomap/math/Utils.h>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>

#include <glm/glm.hpp>

class SceneNode;
class Entity;
class OctomapUpdateListener;

/**
 * The octomap keep track of what the AUV has seen so far. It processes the 'sensor' data
 * and updates it accordingly.
 */
class OctomapBuilder
{
public:
	/**
	 * Construct an empty octomap.
	 * @param node The ROS node used to publish topics.
	 * @param source The entity from where the sensor is creating the octomap.
	 */
	OctomapBuilder(ros::NodeHandle& node, const Entity& source);
	
	/**
	 * Special case for two auvs.
	 * @param node The ROS node used to publish topics.
	 * @param source1 The entity from where the sensor is creating the octomap.
	 * @param source2 The entity from where the sensor is creating the octomap.
	 */
	OctomapBuilder(ros::NodeHandle& node, const Entity& source1, const Entity& source2);
	
	/**
	 * Destroy the octomap.
	 */
	~OctomapBuilder();
	
	/**
	 * DEPRICATED.
	 * If disabled, the octomap will no longer be updated.
	 */
	void setEnabled(bool enabled) { enabled_ = enabled; }
	
	/**
	 * Check if a location is occupied.
	 * @return True if it's occupied, false otherwise.
	 */
	bool isOccupied(float x, float y, float z) const;
	
	/**
	 * Check if the line segment is blocked by any known features in the octomap.
	 * @param from The beginning of the line segment.
	 * @param to The end of the line segment.
	 * @param min_distance_to_obstacle We mark a path as blocked if it passes within @ref{min_distance_to_obstacle}
	 * of any obstacle.
	 * @return True if it's blocked, false otherwise.
	 */
	bool isBlocked(const glm::vec3& from, const glm::vec3& to, float min_distance_to_obstacle) const;
	
	/**
	 * Get the resolution of this octomap.
	 */
	float getResolution() const;
	
	/**
	 * Add a listener for updates of the ontology.
	 */
	void addListener(OctomapUpdateListener& listener) { listeners_.push_back(&listener); }
	void removeListener(OctomapUpdateListener& listener);
	
private:
	
	ros::NodeHandle* node_;
	const Entity* source_;
	const Entity* source2_;
	void receivedPointCloud(const sensor_msgs::PointCloudPtr& msg);
	void receivedPointCloud2(const sensor_msgs::PointCloudPtr& msg);
	
	void updateOctomap(const glm::vec3& source_location, const sensor_msgs::PointCloudPtr& msg);
	
	ros::Subscriber sub_depth_, sub_depth2_;
	ros::Publisher octomap_pub_;
	
	octomap::OcTree* occupancy_map_;
	
	bool enabled_;
	
	std::vector<OctomapUpdateListener*> listeners_;
};

#endif
