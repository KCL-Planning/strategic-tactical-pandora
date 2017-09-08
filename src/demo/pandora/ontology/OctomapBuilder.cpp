#define _USE_MATH_DEFINES

#include <math.h>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Transform.h>
#include <octomap/math/Utils.h>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "dpengine/scene/SceneNode.h"

#include "dpengine/entities/Entity.h"

#include "OctomapBuilder.h"
#include "OctomapUpdateListener.h"

OctomapBuilder::OctomapBuilder(ros::NodeHandle& node, const DreadedPE::Entity& source)
	: node_(&node), source_(&source), occupancy_map_(new octomap::OcTree(0.25)), enabled_(true)
{
	occupancy_map_->enableChangeDetection(false);
	occupancy_map_->setOccupancyThres(0.5);
	
	std::stringstream ss;
	ss << "engine/cloud/" << source.getName();
	sub_depth_ = node_->subscribe(ss.str(), 1, &OctomapBuilder::receivedPointCloud, this);
	octomap_pub_ = node_->advertise<sensor_msgs::PointCloud>("/engine3d/octomap/cloud", 1);
}

OctomapBuilder::OctomapBuilder(ros::NodeHandle& node, const DreadedPE::Entity& source1, const DreadedPE::Entity& source2)
	: node_(&node), source_(&source1), source2_(&source2), occupancy_map_(new octomap::OcTree(0.25)), enabled_(true)
{
	occupancy_map_->enableChangeDetection(false);
	occupancy_map_->setOccupancyThres(0.5);
	std::stringstream ss;
	ss << "engine/cloud/" << source1.getName();
	std::stringstream ss2;
	ss2 << "engine/cloud/" << source2.getName();
	
	std::cout << "Source: " << ss.str() << std::endl;
	std::cout << "Source: " << ss2.str() << std::endl;
	
	sub_depth_ = node_->subscribe(ss.str(), 1, &OctomapBuilder::receivedPointCloud, this);
	sub_depth2_ = node_->subscribe(ss2.str(), 1, &OctomapBuilder::receivedPointCloud2, this);
	octomap_pub_ = node_->advertise<sensor_msgs::PointCloud>("/engine3d/octomap/cloud", 1);
}

OctomapBuilder::~OctomapBuilder()
{
	delete occupancy_map_;
}

bool OctomapBuilder::isOccupied(float x, float y, float z) const
{
	octomap::OcTreeNode* node = occupancy_map_->search(x, y, z);
	
	if (node != NULL && occupancy_map_->isNodeOccupied(*node))//node->getOccupancy() > 0.75f)
//	if (node != NULL && node->getOccupancy() > 0.75f)
	{
		return true;
	}
	return false;
}

bool OctomapBuilder::isBlocked(const glm::vec3& from, const glm::vec3& to, float min_distance_to_obstacle) const
{
	if (from == to)
	{
		return false;
	}
	//std::cout << "Is blocked: (" << from.x << ", " << from.y << ", " << from.z << ") - (" << to.x << ", " << to.y << ", " << to.z << ")" << std::endl;	
	
	octomap::point3d start(from.x, from.y, from.z);
	octomap::point3d direction(to.x - from.x, to.y - from.y, to.z - from.z);
	octomap::point3d intersection;
	
	glm::vec3 axis_y;
	
	// Use the largest parameter to devide by.
	if (fabs(direction.x()) > fabs(direction.y()) && fabs(direction.x()) > fabs(direction.z()))
	{
		if (fabs(direction.y()) < 0.001f && fabs(direction.z()) < 0.001f)
		{
			axis_y = glm::vec3(0, 1, 0);
		}
		
		// Check which one is the second bigest.
		if (fabs(direction.y()) > fabs(direction.z()))
		{
			axis_y = glm::vec3(-direction.y() / direction.x(), 1, 0);
		}
		else 
		{
			axis_y = glm::vec3(-direction.z() / direction.x(), 0, 1);
		}
	}
	else if (fabs(direction.y()) > fabs(direction.x()) && fabs(direction.y()) > fabs(direction.z()))
	{
		if (fabs(direction.x()) < 0.001f && fabs(direction.z()) < 0.001f)
		{
			axis_y = glm::vec3(1, 0, 0);
		}
		
		// Check which one is the second bigest.
		if (fabs(direction.x()) > fabs(direction.z()))
		{
			axis_y = glm::vec3(1, -direction.x() / direction.y(), 0);
		}
		else 
		{
			axis_y = glm::vec3(0, -direction.z() / direction.y(), 1);
		}
	}
	else
	{
		if (fabs(direction.x()) < 0.001f && fabs(direction.y()) < 0.001f)
		{
			axis_y = glm::vec3(1, 0, 0);
		}
		
		// Check which one is the second bigest.
		if (fabs(direction.x()) > fabs(direction.y()))
		{
			axis_y = glm::vec3(1, 0, -direction.x() / direction.z());
		}
		else 
		{
			axis_y = glm::vec3(0, 1, -direction.y() / direction.z());
		}
	}
	//= glm::normalize(glm::vec3(-direction.y() / direction.x(), 1.0f, 0.0f));// * min_distance_to_obstacle;
	glm::vec3 axis_z = glm::normalize(glm::cross(to - from, axis_y));// * min_distance_to_obstacle;
	
	octomap::point3d octomap_axis_y(axis_y.x, axis_y.y, axis_y.z);
	octomap::point3d octomap_axis_z(axis_z.x, axis_z.y, axis_z.z);
	
	float max_distance = glm::distance(from, to);
	
	/*
	glm::vec3 small_axis_y = glm::normalize(glm::vec3(-direction.y() / direction.x(), 1.0f, 0.0f)) * 0.25f;
	glm::vec3 small_axis_z = glm::normalize(glm::cross(to - from, axis_y)) * 0.25f;
	
	octomap::point3d small_octomap_axis_y(small_axis_y.x, small_axis_y.y, small_axis_y.z);
	octomap::point3d small_octomap_axis_z(small_axis_z.x, small_axis_z.y, small_axis_z.z);
	
	if (occupancy_map_->castRay(start, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start + small_octomap_axis_y + small_octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start + small_octomap_axis_y - small_octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start - small_octomap_axis_y + small_octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start - small_octomap_axis_y - small_octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start + octomap_axis_y + octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start + octomap_axis_y - octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start - octomap_axis_y + octomap_axis_z, direction, intersection, true, max_distance) ||
	    occupancy_map_->castRay(start - octomap_axis_y - octomap_axis_z, direction, intersection, true, max_distance))
	{
		return true;
	}
	*/
	
	float resolution = 0.25f;
	
	for (float x = -min_distance_to_obstacle; x < min_distance_to_obstacle; x += resolution)
	{
		for (float y = -min_distance_to_obstacle; y < min_distance_to_obstacle; y += resolution)
		{
			//std::cout << (from - axis_y * x + axis_z * y).x << ", " << (from - axis_y * x + axis_z * y).y << ", " << (from - axis_y * x + axis_z * y).z << ") -> (" << (to - from).x << ", " << (to - from).y << ", " << (to - from).z << ")" << std::endl;
			if (occupancy_map_->castRay(start - octomap_axis_y * x + octomap_axis_z * y, direction, intersection, true, max_distance))
			{
				return true;
			}
		}
	}
	return false;
}

float OctomapBuilder::getResolution() const
{
	return occupancy_map_->getResolution();
}

void OctomapBuilder::removeListener(OctomapUpdateListener& listener)
{
	for (std::vector<OctomapUpdateListener*>::iterator i = listeners_.begin(); i != listeners_.end(); ++i)
	{
		if (&listener == *i)
		{
			listeners_.erase(i);
			break;
		}
	}
}

void OctomapBuilder::receivedPointCloud(const sensor_msgs::PointCloudPtr& msg)
{
	if (!enabled_)
	{
		return;
	}

	// Transform the points.
	updateOctomap(source_->getGlobalLocation(), msg);
}

void OctomapBuilder::receivedPointCloud2(const sensor_msgs::PointCloudPtr& msg)
{
	if (!enabled_)
	{
		return;
	}

	// Transform the points.
	updateOctomap(source2_->getGlobalLocation(), msg);
}

void OctomapBuilder::updateOctomap(const glm::vec3& source_location, const sensor_msgs::PointCloudPtr& msg)
{
	octomap::Pointcloud octo_point_cloud;
	for (std::vector<geometry_msgs::Point32>::const_iterator ci = msg->points.begin(); ci != msg->points.end(); ++ci)
	{
		const geometry_msgs::Point32& p = *ci;
		glm::vec4 point(p.x, p.y, p.z, 1.0f);
		
		if (glm::length(glm::vec3(point) - source_location) > 15.0f)
		{
			continue;
		}
		
		//point = source_location * point;
		
		// Test to check if the float is a NaN.
		if (p.x == p.x)
		{
			// Need to translate them back again, since we use the point cloud to finally send the points to ROS.
			//octo_point_cloud.push_back(p.x + source_location.x, p.y + source_location.y, p.z + source_location.z);
			octo_point_cloud.push_back(point.x, point.y, point.z);
		}
	}

	// We've already performed all the transitions above. No need to do them again.
	occupancy_map_->insertPointCloud(octo_point_cloud, octomap::point3d(0.0f, 0.0f, 0.0f));
	
	sensor_msgs::PointCloud total_point_cloud_msg;
	total_point_cloud_msg.header.stamp = ros::Time::now();
	total_point_cloud_msg.header.frame_id = "world";

	for (octomap::OcTree::leaf_iterator li = occupancy_map_->begin(); li != occupancy_map_->end(); ++li)
	{
		if (occupancy_map_->isNodeOccupied(*li))
		{
			geometry_msgs::Point32 point;
			point.x = -li.getX();
			point.y = li.getZ();
			point.z = li.getY();
			total_point_cloud_msg.points.push_back(point);
		}
	}

	octomap_pub_.publish(total_point_cloud_msg);
	
	
	// Update the listeners.
	for (std::vector<OctomapUpdateListener*>::const_iterator ci = listeners_.begin(); ci != listeners_.end(); ++ci)
	{
		(*ci)->octomapUpdated();
	}
	//std::cout << "[OctomapBuilder::receivedPointCloud] Everything is OK!" << total_point_cloud_msg.points.size() << std::endl;
}
