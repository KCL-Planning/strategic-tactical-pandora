#include "SliceSonar.h"

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

SliceSonar::SliceSonar(ros::NodeHandle& ros_node, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transform, float min_range, float max_range, float angle)
	: Entity(scene_manager, parent, transform, OBSTACLE, "SliceSonar"), ros_node_(&ros_node), min_range_(min_range), max_range_(max_range), angle_(angle), time_since_last_message_(0), shadow_map_width_(64), shadow_map_height_(64)
{
	near_range_ = 0.1f;
	far_range_ = 5.0f;
	image_size_ = 5.0f;
	
	shadow_renderer_ = new ShadowRenderer(scene_manager, shadow_map_width_);
	shadow_renderer_->setCullMode(GL_BACK);
	image_data_ = new float[shadow_map_width_ * shadow_map_height_];
	memset(&image_data_[0], 0., sizeof(float) * shadow_map_width_ * shadow_map_height_);

	// Prepare the image to send to ROS.
	ros_image_.header.frame_id = "AUV";
	ros_image_.height = shadow_map_height_;
	ros_image_.width = shadow_map_width_;
	ros_image_.encoding = sensor_msgs::image_encodings::RGB8;
	ros_image_.is_bigendian = 0;
	ros_image_.step = shadow_map_width_ * sizeof(char) * 3;
	ros_image_.data.resize(shadow_map_width_ * shadow_map_height_ * 3);

	// Setup the 2d slice image. This image will be taken by taking the
	// height values of the middle row of the depth image.
	//slice_height_ = shadow_map_height_;
	//slice_width_ = shadow_map_width_;
	//slice_range_ = far_range_;
	
	slice_image_.header.frame_id = "AUV";
	slice_image_.height = shadow_map_height_;
	slice_image_.width = shadow_map_width_;
	slice_image_.encoding = sensor_msgs::image_encodings::MONO16;
	slice_image_.is_bigendian = 0;
	slice_image_.step = shadow_map_width_ * sizeof(char) * 2;
 	slice_image_.data.resize(shadow_map_height_ * shadow_map_width_ * 2);
	
/**
 * sonar_msgs/Image
 * 
 * Sonar data:
 * 
 * Big indian: 0
 * Encoding: Mono16
 * 
 * hight = 675, 
 * width : 480
 * 
 * step 960
 */

	// Setup the publishers
	slice_publisher_ = ros_node_->advertise<sensor_msgs::Image>("/bvtP/XY/image", 1);
	image_publisher_ = ros_node_->advertise<sensor_msgs::Image>("engine/ortho_image", 1);
}

SliceSonar::~SliceSonar()
{
	delete shadow_renderer_;
	delete[] image_data_;
}

glm::mat4 SliceSonar::getPerspectiveMatrix() const
{
	return glm::ortho(-image_size_, image_size_, -image_size_, image_size_, near_range_, far_range_);
}

glm::mat4 SliceSonar::getViewMatrix() const
{
	glm::fquat rot = getGlobalRotation();
	glm::mat4 view_matrix = glm::rotate(glm::mat4(1.0f), -glm::pitch(rot), glm::vec3(1.0f, 0.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::yaw(rot), glm::vec3(0.0f, 1.0f, 0.0f));
	view_matrix = glm::rotate(view_matrix, -glm::roll(rot), glm::vec3(0.0f, 0.0f, 1.0f));
	view_matrix = glm::translate(view_matrix, -getGlobalLocation());
	return view_matrix;
}

glm::vec3 SliceSonar::getLocation() const
{
	return getGlobalLocation();
}

void SliceSonar::prepare(float dt)
{
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
	for (int y = 0; y < shadow_map_height_; ++y)
	{
		for (int x = 0; x < shadow_map_width_; ++x)
		{
			char data = (char)(image_data_[x + y * shadow_map_width_] * 255.0f);
			for (unsigned z = 0; z < 3; ++z)
			{
				ros_image_.data[(((shadow_map_width_ - 1) - y) * shadow_map_height_ + x) * 3 + z] = data;
			}
		}
	}
	
	slice_image_.header.stamp = ros::Time::now();
	int last_y = shadow_map_height_;
	for (int x = 0; x < shadow_map_width_; ++x)
	{
		int y = shadow_map_height_ / 2;
		float distance = image_data_[x + y * shadow_map_width_] * (far_range_ - near_range_) + near_range_;
		float depth = near_range_;
		float depth_per_pixel = (far_range_ - near_range_) / (float)shadow_map_height_;
		int current_y = shadow_map_height_;
		for (int i = 0; i < shadow_map_height_; ++i)
		{
			if (distance > depth && distance < depth + depth_per_pixel)
			{
				current_y = i;
				for (unsigned z = 0; z < 2; ++z)
				{
					//slice_image_.data[(i * shadow_map_width_ + x) * 2 + z] = (char)255;
					slice_image_.data[(((shadow_map_width_ - 1) - i) * shadow_map_height_ + x) * 2 + z] = (char)255;
				}
			}
			else
			{
				for (unsigned z = 0; z < 2; ++z)
				{
					//slice_image_.data[(i * shadow_map_width_ + x) * 2 + z] = (char)0;
					slice_image_.data[(((shadow_map_width_ - 1) - i) * shadow_map_height_ + x) * 2 + z] = (char)0;
				}
			}
			depth += depth_per_pixel;
		}
		
		if (current_y != shadow_map_height_ && last_y != shadow_map_height_)
		{
			for (int i = std::min(current_y, last_y); i < std::max(current_y, last_y); ++i)
			{
				for (unsigned z = 0; z < 2; ++z)
				{
					slice_image_.data[(((shadow_map_width_ - 1) - i) * shadow_map_height_ + x) * 2 + z] = (char)255;
				}
			}
		}
		
		last_y = current_y;
	}
	
	slice_publisher_.publish(slice_image_);
	image_publisher_.publish(ros_image_);
}
