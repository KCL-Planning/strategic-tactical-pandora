#include <stdlib.h>

#include "HWUOntology.h"

HWUOntology::HWUOntology(ros::NodeHandle& ros_node, OctomapBuilder& octomap_builder)
	: OntologyInterface(ros_node, octomap_builder)
{
	ontology_client_ = ros_node.serviceClient<knowledge_msgs::KnowledgeInterface>("/knowledge/ontology");
	
	ROS_INFO("Ontology services up and running...");
	
	/// Debug data.
	//inspection_points_.push_back(new InspectionPoint(Pose(-3, 15, -2.0, 0, 180)));
	//inspection_points_.push_back(new InspectionPoint(Pose(-5, 7, -3.0, 0, 90)));
	//inspection_points_.push_back(new InspectionPoint(Pose(5, 7, 3.0, 0, -90)));
	InspectionPoint* ip1 = new InspectionPoint(Pose(3, 9, -5, 0, 0));
	InspectionPoint* ip2 = new InspectionPoint(Pose(-9, 9, -5, 0, 0));
	InspectionPoint* ip3 = new InspectionPoint(Pose(3, 15, -5, 0, 0));
	InspectionPoint* ip4 = new InspectionPoint(Pose(-9, 15, -5, 0, 0));
	InspectionPoint* ip5 = new InspectionPoint(Pose(-9, 6, -5, 0, 0));
	InspectionPoint* ip6 = new InspectionPoint(Pose(3, 6, -5, 0, 0));
	
	InspectionPoint* ip7 = new InspectionPoint(Pose(3, 12, 0, 0, 180));
	InspectionPoint* ip8 = new InspectionPoint(Pose(-9, 12, 0, 0, 180));
	InspectionPoint* ip9 = new InspectionPoint(Pose(3, 17, 0, 0, 180));
	InspectionPoint* ip10 = new InspectionPoint(Pose(-9, 17, 0, 0, 180));
	
	
	addInspectionPoint(*ip1);
	addInspectionPoint(*ip2);
	addInspectionPoint(*ip3);
	addInspectionPoint(*ip4);
	addInspectionPoint(*ip5);
	addInspectionPoint(*ip6);
	addInspectionPoint(*ip7);
	addInspectionPoint(*ip8);
	addInspectionPoint(*ip9);
	addInspectionPoint(*ip10);
	//inspection_points_.push_back(new InspectionPoint(Pose(3, 9, -5, 0, 0)));
	//inspection_points_.push_back(new InspectionPoint(Pose(-9, 9, -5, 0, 0)));
	//inspection_points_.push_back(new InspectionPoint(Pose(3, 15, -5, 0, 0)));
	//inspection_points_.push_back(new InspectionPoint(Pose(-9, 15, -5, 0, 0)));
}

bool HWUOntology::getInspectionPoints(std::vector<InspectionPoint>& inspection_points)
{
	std::cout << "Get all inspection points" << std::endl;
	knowledge_msgs::KnowledgeInterface::Request waypoint_instances_req;
	knowledge_msgs::KnowledgeInterface::Response waypoint_instances_res;
	
	waypoint_instances_req.request_type = "get_type_instances";
	waypoint_instances_req.type_name = "InspectionPoint";
	bool success = ontology_client_.call(waypoint_instances_req, waypoint_instances_res);
	if (!success)
	{
		return false;
	}
	
	knowledge_msgs::KnowledgeInterface::Request waypoint_properties_req;
	knowledge_msgs::KnowledgeInterface::Response waypoint_properties_res;
	for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = waypoint_instances_res.items.begin(); ci != waypoint_instances_res.items.end(); ++ci)
	{
		std::cout << "Get all the properties of the inspection point: " << (*ci).instance_name << std::endl;
		waypoint_properties_req.request_type = "get_instance_properties";
		waypoint_properties_req.type_name = "InspectionPoint";
		waypoint_properties_req.instance_name = (*ci).instance_name;
		waypoint_properties_res.items.clear();
		
		success = ontology_client_.call(waypoint_properties_req, waypoint_properties_res);
		
		if (!success)
		{
			return false;
		}
		
		float x = 0; float y = 0; float z = 0; float yaw = 0;
		for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = waypoint_properties_res.items.begin(); ci != waypoint_properties_res.items.end(); ++ci)
		{
			const knowledge_msgs::KnowledgeObject& ko = *ci;
			for (std::vector<knowledge_msgs::DataProperty>::const_iterator ci = ko.data_properties.begin(); ci != ko.data_properties.end(); ++ci)
			{
				const knowledge_msgs::DataProperty& data_p = *ci;
				std::string value = data_p.value.substr(data_p.value.find(',') + 1);
				if ("ipN" == data_p.name)
				{
					z = ::atof(value.c_str());
				}
				else if ("ipE" == data_p.name)
				{
					x = ::atof(value.c_str());
				}
				else if ("ipD" == data_p.name)
				{
					y = ::atof(value.c_str());
				}
				else if ("ipY" == data_p.name)
				{
					yaw = ::atof(value.c_str());
				}
				std::cout << data_p.name << " -> " << data_p.value << "(" << value << ")" << std::endl;
			}
			std::cout << "Found the inspection point: " << x << ", " << y << ", " << z << ", " << yaw << std::endl;
		}
		inspection_points.push_back(InspectionPoint(Pose(x, y, z, 0, yaw)));
	}
	
	return true;
}

InspectionPoint& HWUOntology::getInspectionPoint(const std::string& inspection_point_name)
{
	std::cout << "Get inspection point: " << inspection_point_name << std::endl;
	knowledge_msgs::KnowledgeInterface::Request waypoint_properties_req;
	knowledge_msgs::KnowledgeInterface::Response waypoint_properties_res;
	
	waypoint_properties_req.request_type = "get_instance_properties";
	waypoint_properties_req.type_name = "InspectionPoint";
	waypoint_properties_req.instance_name = inspection_point_name;
	
	bool success = ontology_client_.call(waypoint_properties_req, waypoint_properties_res);
	
	if (!success)
	{
		std::cerr << "Could not find an inspection point in the ontology with name: " << inspection_point_name << std::endl;
		assert (false);
		exit(1);
	}
	
	float x = 0; float y = 0; float z = 0; float yaw = 0;
	for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = waypoint_properties_res.items.begin(); ci != waypoint_properties_res.items.end(); ++ci)
	{
		const knowledge_msgs::KnowledgeObject& ko = *ci;
		for (std::vector<knowledge_msgs::DataProperty>::const_iterator ci = ko.data_properties.begin(); ci != ko.data_properties.end(); ++ci)
		{
			const knowledge_msgs::DataProperty& data_p = *ci;
			
 			std::string value = data_p.value.substr(data_p.value.find(',') + 1);
			
			if ("ipN" == data_p.name)
			{
				z = ::atof(value.c_str());
			}
			else if ("ipE" == data_p.name)
			{
				x = ::atof(value.c_str());
			}
			else if ("ipD" == data_p.name)
			{
				y = ::atof(value.c_str());
			}
			else if ("ipY" == data_p.name)
			{
				yaw = ::atof(value.c_str());
			}
			
			std::cout << data_p.name << " -> " << data_p.value << "(" << value << ")" << std::endl;
		}
	}
	std::cout << "Return the inspection point: " << x << ", " << y << ", " << z << ", " << yaw << std::endl;
	return InspectionPoint(Pose(x, y, z, 0.0f, yaw));
}

void HWUOntology::rrtUpdated()
{
	std::cout << "[HWUOntology] RRT is updated!" << std::endl;
	// Delete all the waypoints from the ontology.
	knowledge_msgs::KnowledgeInterface waypoint_service;
	waypoint_service.request.request_type = "get_type_instances";
	waypoint_service.request.type_name = "Waypoint";
	
	if (!ontology_client_.call(waypoint_service))
	{
		std::cerr << "Failed to get all the waypoints stored in the ontology." << std::endl;
		return;
	}
	
	for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = waypoint_service.response.items.begin(); ci != waypoint_service.response.items.end(); ++ci)
	{
		knowledge_msgs::KnowledgeInterface remove_waypoint_service;
		remove_waypoint_service.request.request_type = "remove_type_instance";
		remove_waypoint_service.request.type_name = "Waypoint";
		remove_waypoint_service.request.instance_name = (*ci).instance_name;
		
		if (!ontology_client_.call(remove_waypoint_service))
		{
			std::cerr << "Failed to remove the waypoint: " << (*ci).instance_name << " from the ontology!" << std::endl;
			return;
		}
	}
	
	// Add the new waypoints to the ontology.
	unsigned int i = 0;
	for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
	{
		Waypoint* waypoint = *ci;
		std::stringstream ss;
		ss << "W" << i;
		
		std::cout << "Process the waypoint: W" << i << "..." << std::endl;
		
		// Update the ontology and store all the waypoints into the ontology.
		knowledge_msgs::KnowledgeInterface add_waypoint_srv;
		add_waypoint_srv.request.request_type = "add_type_instance";
		add_waypoint_srv.request.type_name = "Waypoint";
		add_waypoint_srv.request.instance_name = ss.str();
		
		if (!ontology_client_.call(add_waypoint_srv))
		{
			std::cout << "Failed to add the waypoint: " << ss.str() << " to the ontology." << std::endl;
			return;
		}
		
		{
		std::ostringstream o;
		o << waypoint->position_.z;
		
		add_waypoint_srv.request.request_type = "add_instance_data_property";
		add_waypoint_srv.request.data_property_name = "wpN";
		add_waypoint_srv.request.data_property_value = o.str();
		
		if (!ontology_client_.call(add_waypoint_srv))
		{
			std::cout << "Failed to add the waypoint: " << ss.str() << " to the ontology." << std::endl;
			return;
		}
		}
		
		{
		std::ostringstream o;
		o << waypoint->position_.x;
		
		add_waypoint_srv.request.data_property_name = "wpE";
		add_waypoint_srv.request.data_property_value = o.str();
		
		if (!ontology_client_.call(add_waypoint_srv))
		{
			std::cout << "Failed to add the waypoint: " << ss.str() << " to the ontology." << std::endl;
			return;
		}
		}
		
		{
		std::ostringstream o;
		o << waypoint->position_.y;
		add_waypoint_srv.request.data_property_name = "wpD";
		add_waypoint_srv.request.data_property_value = o.str();
		
		if (!ontology_client_.call(add_waypoint_srv))
		{
			std::cout << "Failed to add the waypoint: " << ss.str() << " to the ontology." << std::endl;
			return;
		}
		}
		
		++i;
	}
	
	// Add the connectivity to the ontology.
	i = 0;
	for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
	{
		Waypoint* waypoint = *ci;
		std::stringstream ss;
		ss << "W" << i;
		
		std::cout << "Process the waypoint: W" << i << "..." << std::endl;
		
		// Add the connections between these waypoints.
		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = waypoint->edges_.begin(); ci != waypoint->edges_.end(); ++ci)
		{
			unsigned int j = 0;
			for ( ; j < rrt_->getPoints().size(); ++j)
			{
				if (rrt_->getPoints()[j] == (*ci).first)
				{
					break;
				}
			}
			std::stringstream ss2;
			ss2 << "W" << j;
			
			std::cout << "-> connect to: W" << j << "..." << std::endl;
			
			// Update the ontology and store all the waypoints into the ontology.
			knowledge_msgs::KnowledgeInterface add_edge_srv;
			add_edge_srv.request.request_type = "add_instance_obj_property";
			add_edge_srv.request.type_name = "Waypoint";
			add_edge_srv.request.instance_name = ss.str();
			
			add_edge_srv.request.obj_property_name = "connectedTo";
			add_edge_srv.request.obj_property_type = "Waypoint";
			add_edge_srv.request.obj_property_value = ss2.str();
			
			if (!ontology_client_.call(add_edge_srv))
			{
				std::cout << "Failed to add the connection between waypoint: " << ss.str() << " and " << ss2.str() << " to the ontology." << std::endl;
				return;
			}
		}
		
		// Check which inspection points are visible from this waypoint.
		int index = 0;
		for (std::vector<InspectionPoint*>::const_iterator ci = inspection_points_.begin(); ci != inspection_points_.end(); ++ci)
		{
			const InspectionPoint* inspection_point = *ci;
			
			std::cout << "Compare: (" << waypoint->position_.x << ", " << waypoint->position_.y << ", " << waypoint->position_.z << ") to (" << inspection_point->getVisiblePoint().x << ", " << inspection_point->getVisiblePoint().y << ", " << inspection_point->getVisiblePoint().z << std::endl;
			
			if (waypoint->position_ == inspection_point->getVisiblePoint())
			{
				std::stringstream ss3;
				ss3 << "I" << index;
				
				knowledge_msgs::KnowledgeInterface add_visible_srv;
				add_visible_srv.request.request_type = "add_instance_obj_property";
				add_visible_srv.request.type_name = "Waypoint";
				add_visible_srv.request.instance_name = ss.str();
				
				add_visible_srv.request.obj_property_name = "canSee";
				add_visible_srv.request.obj_property_type = "InspectionPoint";
				add_visible_srv.request.obj_property_value = ss3.str();
				
				if (!ontology_client_.call(add_visible_srv))
				{
					std::cout << "Failed to add the visibility between waypoint: " << ss.str() << " and inspection point " << ss3.str() << " to the ontology." << std::endl;
					return;
				}
				std::cout << "Inspection point: " << ss3.str() << " is visible from " << ss.str() << std::endl;
			}
			++index;
		}
		++i;
	}
}

void HWUOntology::addInspectionPoint(InspectionPoint& inspection_point)
{
	std::cout << "Add an inspection point to the ontology: (" << inspection_point.getPose().x_ << ", " << inspection_point.getPose().y_ << ", " << inspection_point.getPose().z_ << "; yaw=" << inspection_point.getPose().yaw_ << ")" << std::endl;
	std::stringstream ss;
	ss << "I" << inspection_points_.size();

	inspection_points_.push_back(&inspection_point);
	
	
	// Update the ontology and store all the waypoints into the ontology.
	knowledge_msgs::KnowledgeInterface add_inspection_point_srv;
	add_inspection_point_srv.request.request_type = "add_type_instance";
	add_inspection_point_srv.request.type_name = "InspectionPoint";
	add_inspection_point_srv.request.instance_name = ss.str();
		
	if (!ontology_client_.call(add_inspection_point_srv))
	{
		std::cout << "Failed to add the inspection point: " << ss.str() << " to the ontology." << std::endl;
		return;
	}
	
	{
	std::ostringstream o;
	o << inspection_point.getPose().z_;
	add_inspection_point_srv.request.request_type = "add_instance_data_property";
	add_inspection_point_srv.request.data_property_name = "ipN";
	add_inspection_point_srv.request.data_property_value = o.str();
	
	if (!ontology_client_.call(add_inspection_point_srv))
	{
		std::cout << "Failed to add the inspection point: " << ss.str() << " to the ontology." << std::endl;
		return;
	}
	}
	
	{
	std::ostringstream o;
	o << inspection_point.getPose().x_;
	add_inspection_point_srv.request.data_property_name = "ipE";
	add_inspection_point_srv.request.data_property_value = o.str();
	
	if (!ontology_client_.call(add_inspection_point_srv))
	{
		std::cout << "Failed to add the inspection point: " << ss.str() << " to the ontology." << std::endl;
		return;
	}
	}
	
	{
	std::ostringstream o;
	o << inspection_point.getPose().y_;
	add_inspection_point_srv.request.data_property_name = "ipD";
	add_inspection_point_srv.request.data_property_value = o.str();
	
	if (!ontology_client_.call(add_inspection_point_srv))
	{
		std::cout << "Failed to add the inspection point: " << ss.str() << " to the ontology." << std::endl;
		return;
	}
	}
	
	{
	std::ostringstream o;
	o << inspection_point.getPose().yaw_;
	add_inspection_point_srv.request.data_property_name = "ipY";
	add_inspection_point_srv.request.data_property_value = o.str();
	
	if (!ontology_client_.call(add_inspection_point_srv))
	{
		std::cout << "Failed to add the inspection point: " << ss.str() << " to the ontology." << std::endl;
		return;
	}
	}
	
	std::cout << "Succes!" << std::endl;
}
