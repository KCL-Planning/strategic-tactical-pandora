#ifndef DEMO_PANDORA_SENSORS_SONAR_H
#define DEMO_PANDORA_SENSORS_SONAR_H

// Ros specific.
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

//#include <octomap

#include "dpengine/entities/Entity.h"
#include "dpengine/renderer/FrustumCaster.h"

class AUV;
namespace DreadedPE
{
	class SceneManager;
	class ShadowRenderer;
};

class Sonar : public DreadedPE::FrustumCaster, public DreadedPE::Entity
{
public:
	Sonar(ros::NodeHandle& node, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, AUV& auv, const std::string& frame_id, const glm::mat4& transform, float min_range, float max_range, float angle);
	~Sonar();

	void prepare(float dt);

	float getAngle() const { return angle_; }

	glm::mat4 getPerspectiveMatrix() const;
	glm::mat4 getViewMatrix() const;
	glm::vec3 getLocation() const;
private:
	ros::NodeHandle* ros_node_;
	AUV* auv_;
	float min_range_, max_range_, angle_;
	bool pitch_moving_up_;
	//GLuint depth_texture_id_;

	DreadedPE::ShadowRenderer* shadow_renderer_;

	float* image_data_;
	float pitch_;

	// The message to send through ROS.
	sensor_msgs::Image ros_image_;
	ros::Publisher image_publisher_;

	sensor_msgs::PointCloud point_cloud_;
	ros::Publisher point_cloud_publisher_;

	float time_since_last_message_;

	int shadow_map_width_, shadow_map_height_;
};

#endif

