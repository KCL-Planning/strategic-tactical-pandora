#ifndef DEMO_PANDORA_SENSORS_SLICE_SONAR_H
#define DEMO_PANDORA_SENSORS_SLICE_SONAR_H

// Ros specific.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//#include <octomap

#include "dpengine/entities/Entity.h"
#include "dpengine/renderer/FrustumCaster.h"

namespace DreadedPE
{
	class SceneManager;
	class ShadowRenderer;
};

/**
 * Special class that uses a ortho perspective matrix to create a single 'slice' 
 * that is used by Franciesco.
 */
class SliceSonar : public DreadedPE::FrustumCaster, public DreadedPE::Entity
{
public:
	SliceSonar(ros::NodeHandle& node, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transform, float min_range, float max_range, float angle);
	~SliceSonar();

	void prepare(float dt);

	float getAngle() const { return angle_; }

	glm::mat4 getPerspectiveMatrix() const;
	glm::mat4 getViewMatrix() const;
	glm::vec3 getLocation() const;
private:
	ros::NodeHandle* ros_node_;
	float min_range_, max_range_, angle_;

	DreadedPE::ShadowRenderer* shadow_renderer_;

	float* image_data_;

	// The message to send through ROS.
	sensor_msgs::Image ros_image_;
	ros::Publisher image_publisher_;
	float near_range_, far_range_, image_size_;
	
	// Send a 'slice' of the sonar image.
	sensor_msgs::Image slice_image_;
	ros::Publisher slice_publisher_;
	//int slice_height_;      // The number of pixels on the y-axis.
	//int sice_width_;        // The number of pixels on the x-axis.
	//float slice_range_;     // The depth we can see using the sonar. The cm / pixel is equal to slice_range_ / slice_height.
	//float slice_width_;     // The width we can see using the sonar.

	float time_since_last_message_;

	int shadow_map_width_, shadow_map_height_;
	
};

#endif
