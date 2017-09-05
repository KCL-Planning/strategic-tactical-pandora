#include <diagnostic_msgs/KeyValue.h>
#include <knowledge_msgs/DataProperty.h>
#include <knowledge_msgs/ObjectProperty.h>
#include <knowledge_msgs/Notification.h>

#include "OntologyInterface.h"
#include "Pose.h"
#include "OctomapBuilder.h"
#include "InspectionPoint.h"
#include "InspectionGoal.h"
#include "ChainGoal.h"
#include "Goal.h"
#include "ValveGoal.h"
#include "../structures/Valve.h"
#include "../level/MissionSite.h"
#include "../level/Mission.h"
#include "../structures/ValvePanel.h"
#include "../structures/Pillar.h"
#include "../structures/Chain.h"
#include "../Waypoint.h"
#include "../RRT.h"

#include "filter/ChainFilter.h"
#include "filter/PillarFilter.h"
#include "filter/ConnectionFilter.h"
#include "InspectionGoal.h"

//#define ONTOLOGY_DEBUG_ENABLED

OntologyInterface::OntologyInterface(ros::NodeHandle& ros_node, OctomapBuilder& octomap_builder)
	: octomap_(&octomap_builder), last_time_filters_checked_(0)
{
	notification_server_ = ros_node.advertise<knowledge_msgs::Notification>("/knowledge/ontology/notification", 10);
	filter_server_ = ros_node.subscribe("/knowledge/ontology/filter", 100, &OntologyInterface::updateFilter, this);
	
	// Register the ontology as a listener of the octomap.
	octomap_builder.addListener(*this);
}

void OntologyInterface::rrtUpdated()
{
	std::cout << "[OntologyInterface] RRT is updated!" << std::endl;
}

void OntologyInterface::initialise(RRT& rrt)
{
	rrt_ = &rrt;
	rrt_->addListener(*this);
}

Waypoint* OntologyInterface::getWaypoint(const std::string& waypoint_name) const
{
	for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
	{
		Waypoint* waypoint = *ci;
		if (waypoint->id_ == waypoint_name)
		{
			return waypoint;
		}
	}
	return NULL;
	/*
	int i = ::atoi(waypoint_name.substr(1).c_str());
	const std::vector<Waypoint*>& points = rrt_->getPoints();
	
	if (i < 0 || i >= points.size())
	{
		ROS_ERROR("Waypoint index out of range.");
		return NULL;
	}
	
	return points[i];
	*/
}

void OntologyInterface::octomapUpdated()
{
	clock_t current_time = clock() / CLOCKS_PER_SEC;
	if (current_time - last_time_filters_checked_ < 30)
	{
		//return;
	}
	last_time_filters_checked_ = current_time;
	// Check if any of the filters are violated.
	for (std::vector<Filter*>::const_iterator ci = filters_.begin(); ci != filters_.end(); ++ci)
	{
		if (!(*ci)->checkFilter())
		{
			std::cout << "\t *** Triggered filter ***" << std::endl;
			std::cout << **ci << std::endl;
			
			// A filter has been violated! Next we need to notify the planner.
			knowledge_msgs::Notification notification = (*ci)->prepareNotification();
			notification_server_.publish(notification);
		}
	}
}

void OntologyInterface::updateFilter(const knowledge_msgs::Filter::ConstPtr& msg)
{
	switch (msg->function)
	{
		case knowledge_msgs::Filter::F_CLEAR:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Clear the filter." << std::endl;
#endif
			std::cout << "Clear the filter." << std::endl;
			for (std::vector<Filter*>::const_iterator ci = filters_.begin(); ci != filters_.end(); ++ci)
			{
				delete *ci;
			}
			filters_.clear();
			break;
		case knowledge_msgs::Filter::F_INSERT:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Insert something to the filter. " << msg->type_name << std::endl;
#endif
			if ("Pillar" == msg->type_name)
			{
				for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
				{
					MissionSite* mission_site = *ci;
					for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
					{
						Pillar* pillar = *ci;
						// Add a new pillar filter.
						filters_.push_back(new PillarFilter(*msg, *pillar));
						std::cout << "Insert new pillar filter!" << std::endl;
					}
				}
			}
			else if ("Chain" == msg->type_name)
			{
				for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
				{
					MissionSite* mission_site = *ci;
					for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
					{
						Chain* chain = *ci;
						// Add a new pillar filter.
						filters_.push_back(new ChainFilter(*msg, *chain));
						std::cout << "Insert new chain filter!" << std::endl;
					}
				}
			}
			break;
		case knowledge_msgs::Filter::F_REMOVE:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Remove something to the filter." << std::endl;
#endif
			break;
		case knowledge_msgs::Filter::F_INSERT_DATA_ATTR:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Insert data attribute to the filter." << std::endl;
#endif
			break;
		case knowledge_msgs::Filter::F_REMOVE_DATA_ATTR:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Remove data attribute to the filter." << std::endl;
#endif
			break;
		case knowledge_msgs::Filter::F_INSERT_OBJ_ATTR:
			
			if ("connectedTo" == msg->obj_property_name)
			{
				// Get the waypoints associated with this request.
				Waypoint* w1 = getWaypoint(msg->instance_name);
				Waypoint* w2 = getWaypoint(msg->obj_property_value);
				
				if (w1 == NULL || w2 == NULL)
				{
					std::cerr << "Could not intall the filter for the waypoints " << msg->instance_name << "(" << (w1 == NULL ? "NULL" : "FOUND") << ") " << msg->obj_property_value << "(" << (w2 == NULL ? "NULL" : "FOUND") << ")" << std::endl;
					return;
				}
				ConnectionFilter* filter = new ConnectionFilter(*msg, w1->position_, w2->position_, *octomap_, 1.0f);
				filters_.push_back(filter);
//#ifdef ONTOLOGY_DEBUG_ENABLED
				std::cout << "Inserted a filter for the connectivity of (" << w1->position_.x << ", " << w1->position_.y << ", " << w1->position_.z << "), " << " and (" << w2->position_.x << ", " << w2->position_.y << ", " << w2->position_.z << ")" << std::endl;
//#endif
			}
			else if ("canSee" == msg->obj_property_name)
			{
				// Get the waypoints associated with this request.
				Waypoint* w = getWaypoint(msg->instance_name);
				//const InspectionPoint* inspection_point = getInspectionPoint(msg->obj_property_value);
				
				//if (w == NULL || inspection_point == NULL)
				
				InspectionPoint inspection_point = getInspectionPoint(msg->obj_property_value);
				
				if (w == NULL)
				{
					std::cerr << "Could not intall the filter for the waypoints " << msg->instance_name << "(" << (w == NULL ? "NULL" : "FOUND") << ") " << msg->obj_property_value << ")" << std::endl;
					return;
				}
				/// DEBUG, turned off because it's too sensitive...
				//ConnectionFilter* filter = new ConnectionFilter(*msg, w->position_, glm::vec3(inspection_point->getPose().x_, inspection_point->getPose().y_, inspection_point->getPose().z_), *octomap_, 0.01f);
				//filters_.push_back(filter);
#ifdef ONTOLOGY_DEBUG_ENABLED
				std::cout << "Inserted a filter for the visibility of (" << w->position_.x << ", " << w->position_.y << ", " << w->position_.z << ") and (" << inspection_point.getVisiblePoint().x << ", " << inspection_point.getVisiblePoint().y << ", " << inspection_point.getVisiblePoint().z << ")" << std::endl;
#endif
			}
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Insert object attribute to the filter." << std::endl;
#endif
			break;
		case knowledge_msgs::Filter::F_REMOVE_OBJ_ATTR:
#ifdef ONTOLOGY_DEBUG_ENABLED
			std::cout << "Remove object attribute to the filter." << std::endl;
#endif
			break;
	}
	
#ifdef ONTOLOGY_DEBUG_ENABLED
	std::cout << "Update filter: " << std::endl;
	std::cout << "\t* type_name: "<< msg->type_name << std::endl;
	std::cout << "\t* instance_name: "<< msg->instance_name << std::endl;
	
	std::cout << "\t* data_property_name: "<< msg->data_property_name << std::endl;
	std::cout << "\t* data_property_value: "<< msg->data_property_value << std::endl;
	
	std::cout << "\t* obj_property_name: "<< msg->obj_property_name << std::endl;
	std::cout << "\t* obj_property_type: "<< msg->obj_property_type << std::endl;
	std::cout << "\t* obj_property_value: "<< msg->obj_property_value << std::endl;
	/*
	for (std::vector<knowledge_msgs::KnowledgeItem>::const_iterator ci = msg->knowledge_items.begin(); ci != msg->knowledge_items.end(); ++ci)
	{
		std::cout << "\t\tknowledge_type: " << (*ci).knowledge_type << std::endl;
		std::cout << "\t\tinstance_type: " << (*ci).instance_type << std::endl;
		std::cout << "\t\tnstance_name: " << (*ci).instance_name << std::endl;
		std::cout << "\t\tattribute_name: " << (*ci).attribute_name << std::endl;
		std::cout << "\t\tvalue: " << (*ci).value.key << "->" << (*ci).value.value << std::endl;
	}
	*/
#endif
}

void OntologyInterface::addMissionSite(MissionSite& mission_site)
{
	// Split the missions up.
	for (std::vector<Mission*>::const_iterator ci = mission_site.getMissions().begin(); ci != mission_site.getMissions().end(); ++ci)
	{
		missions_[(*ci)->getId()] = *ci;
	}
	mission_sites_.push_back(&mission_site);
}

void OntologyInterface::removeMissionSite(const MissionSite& mission_site)
{
	for (std::vector<Mission*>::const_iterator ci = mission_site.getMissions().begin(); ci != mission_site.getMissions().end(); ++ci)
	{
		missions_.erase((*ci)->getId());
	}
	for (std::vector<MissionSite*>::iterator i = mission_sites_.begin(); i != mission_sites_.end(); ++i)
	{
		if (*i == &mission_site)
		{
			mission_sites_.erase(i);
			break;
		}
	}
}

MissionSite* OntologyInterface::getMissionSite(const std::string& mission_site_id) const
{
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		MissionSite* ms = *ci;
		if (ms->getId() == mission_site_id)
		{
			return ms;
		}
	}
	std::cerr << "Could not find the mission site with the id: " << mission_site_id << " this should NEVER happen!" << std::endl;
	assert(false);
	exit(1);
	return NULL;
}

void OntologyInterface::addAUV(const AUV& auv)
{
	auvs_.push_back(&auv);
}
