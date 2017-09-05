#include "Sonar.h"

#include <fstream>
#include <string.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "../../../core/renderer/ShadowRenderer.h"
#include "../../../core/entities/camera/Camera.h"
#include "../../../core/texture/Texture.h"
#include "../AUV.h"

Sonar::Sonar(ros::NodeHandle& ros_node, SceneManager& scene_manager, SceneNode* parent, AUV& auv, const std::string& frame_id, const glm::mat4& transform, float min_range, float max_range, float angle)
	: Entity(scene_manager, parent, transform, OBSTACLE, "Sonar"), ros_node_(&ros_node), auv_(&auv), min_range_(min_range), max_range_(max_range), angle_(angle), pitch_moving_up_(false), pitch_(0), time_since_last_message_(0), shadow_map_width_(128), shadow_map_height_(128)
{
	shadow_renderer_ = new ShadowRenderer(scene_manager, shadow_map_width_);
	shadow_renderer_->setCullMode(GL_BACK);
	image_data_ = new float[shadow_map_width_ * shadow_map_height_];
	memset(&image_data_[0], 0., sizeof(float) * shadow_map_width_ * shadow_map_height_);

	// Prepare the image to send to ROS.
	ros_image_.header.frame_id = frame_id;
	ros_image_.height = shadow_map_height_;
	ros_image_.width = shadow_map_width_;
	ros_image_.encoding = sensor_msgs::image_encodings::RGB8;
	ros_image_.is_bigendian = 0;
	ros_image_.step = shadow_map_width_ * sizeof(char) * 3;
	ros_image_.data.resize(shadow_map_width_ * shadow_map_height_ * 3);

	point_cloud_.header.frame_id = "world";
	point_cloud_.points.resize(shadow_map_width_ * shadow_map_height_);
	
	// Setup the publishers
	{
	std::stringstream ss;
	ss << "engine/image/" << frame_id;
	image_publisher_ = ros_node_->advertise<sensor_msgs::Image>(ss.str(), 1);
	}
	{
	std::stringstream ss;
	ss << "engine/cloud/" << frame_id;
	point_cloud_publisher_ = ros_node_->advertise<sensor_msgs::PointCloud>(ss.str(), 1);
	}
}

Sonar::~Sonar()
{
	delete shadow_renderer_;
	delete[] image_data_;
}

glm::mat4 Sonar::getPerspectiveMatrix() const
{
	return glm::perspective(angle_ * 2, 1.0f, min_range_, max_range_);
}

glm::mat4 Sonar::getViewMatrix() const
{
	glm::fquat rot = getGlobalRotation();
	glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -glm::pitch(rot), glm::vec3(1.0f, 0.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::yaw(rot), glm::vec3(0.0f, 1.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::roll(rot), glm::vec3(0.0f, 0.0f, 1.0f));
	view_matrix = glm::translate(view_matrix, -getGlobalLocation());
	return view_matrix;
}

glm::vec3 Sonar::getLocation() const
{
	return getGlobalLocation();
}

void Sonar::prepare(float dt)
{
	float max_angle = 80;
	//float pitch = glm::pitch(getLocalRotation());
	float turning_speed = dt * 5;
	float pitch_dt = 0;
	if (pitch_moving_up_)
	{
		pitch_ += turning_speed;
		if (pitch_ > max_angle)
		{
			pitch_moving_up_ = false;
			pitch_dt = pitch_ - max_angle;
			pitch_ = max_angle;
			//std::cout << "Pitch: " << pitch_ << "; Swap direction, update pitch by: " << pitch_dt << "." << std::endl;
		}
		else
		{
			pitch_dt = turning_speed;
		}
	}
	else
	{
		pitch_ -= turning_speed;
		if (pitch_ < -max_angle)
		{
			pitch_moving_up_ = true;
			pitch_dt = -(pitch_ + max_angle);
			pitch_ = -max_angle;
			//std::cout << "Pitch: " << pitch_ << "; Swap direction, update pitch by: " << pitch_dt << "." << std::endl;
		}
		else
		{
			pitch_dt = -turning_speed;
		}
	}
	
	glm::vec3 desired_direction = auv_->getDesiredDirection();
	float desired_yaw = 0;
	//if (desired_direction.x != 0)
	{
		desired_yaw = atan2(-desired_direction.x, -desired_direction.z) * 180.0f / M_PI;
	}
	//std::cout << "Desired direction: " << desired_yaw << " - (" << desired_direction.x << ", " << desired_direction.y << ", " << desired_direction.z << ")." << std::endl;
	desired_direction.y = 0;
	desired_direction = glm::normalize(desired_direction) * 1.5f;
	
	local_transformation_ = glm::translate(glm::mat4(1.0f), desired_direction);
	local_transformation_ = glm::rotate(local_transformation_, desired_yaw, glm::vec3(0, 1, 0));
	local_transformation_ = glm::rotate(local_transformation_, pitch_, glm::vec3(1, 0, 0));
	
	
	updateTransformations();
	
	Entity::prepare(dt);
	shadow_renderer_->render(*this);

	time_since_last_message_ += dt;
	if (time_since_last_message_ > 1.0f)
	{
		time_since_last_message_ -= 1.0f;
	}
	else
	{
		return;
	}

	// Safe the image.
	glBindTexture(GL_TEXTURE_2D, shadow_renderer_->getTexture().getTextureId());
	glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, image_data_);

	// Send this image through ROS.
	ros_image_.header.stamp = ros::Time::now();
	//slice_image_.header.stamp = ros::Time::now();
	point_cloud_.header.stamp = ros::Time::now();

	// Calculate the locations of the pixels on the screen to the global reference frame.
	float total_angle = 2 * angle_;

	float angle_per_pixel_w = total_angle / shadow_map_width_;
	float angle_per_pixel_h = total_angle / shadow_map_height_;

	float length_per_pixel_w = tan(angle_per_pixel_w * M_PI / 180.0f) * min_range_;
	float length_per_pixel_h = tan(angle_per_pixel_h * M_PI / 180.0f) * min_range_;

	// Calculate the ray directions.
	glm::mat4 view_matrix = getCompleteTransformation();
	glm::vec4 direction(0.0f, 0.0f, -1.0f, 0.0f);

	for (int y = 0; y < shadow_map_height_; ++y)
	{
		for (int x = 0; x < shadow_map_width_; ++x)
		{
			float z_n = 2.0 * image_data_[x + y * shadow_map_width_] - 1.0;
			float z_e = 2.0 * min_range_ * max_range_ / (max_range_ + min_range_ - z_n * (max_range_ - min_range_));

			// Scale the distance to account for the perspective.
			float rx = ((float)(x) - shadow_map_width_ / 2.0f) * length_per_pixel_w;
			float ry = ((float)(y) - shadow_map_height_ / 2.0f) * length_per_pixel_h;
			float d = sqrt(rx * rx + ry * ry);
			float s = sqrt(d * d + min_range_ * min_range_) / min_range_;

			glm::mat4 m_direction = glm::rotate(glm::mat4(1.0f), -((float)(x) - (shadow_map_width_ / 2.0f)) * angle_per_pixel_w, glm::vec3(0.0f, 1.0f, 0.0f));
			m_direction = glm::rotate(m_direction, -((float)(y) - (shadow_map_height_ / 2.0f)) * angle_per_pixel_h, glm::vec3(1.0f, 0.0f, 0.0f));
			glm::vec4 point = glm::normalize(m_direction * direction);
			point = point * z_e * s;
			
			glm::vec4 global_point(point.x, -point.y, point.z, 1.0f);
			global_point = getCompleteTransformation() * global_point;

			point_cloud_.points[y * shadow_map_width_ + x].x = global_point.x;
			point_cloud_.points[y * shadow_map_width_ + x].y = global_point.y;
			point_cloud_.points[y * shadow_map_width_ + x].z = global_point.z;
			
			char data = (char)((z_e / max_range_) * 255.0f);
			for (unsigned z = 0; z < 3; ++z)
			{
				ros_image_.data[(((shadow_map_width_ - 1) - y) * shadow_map_height_ + x) * 3 + z] = data;
			}
		}
	}
	image_publisher_.publish(ros_image_);
	point_cloud_publisher_.publish(point_cloud_);
}
