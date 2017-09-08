#include "Ontology.h"

#include <sstream>

#include <diagnostic_msgs/KeyValue.h>
#include <knowledge_msgs/DataProperty.h>
#include <knowledge_msgs/ObjectProperty.h>
#include <knowledge_msgs/Notification.h>

#include "../structures/Pillar.h"
#include "../structures/Valve.h"
#include "../structures/Chain.h"
#include "../structures/Structure.h"
#include "../RRT.h"
#include "../AUV.h"
#include "OctomapBuilder.h"
#include "InspectionPoint.h"
#include "Goal.h"
#include "InspectionGoal.h"
#include "InspectionPoint.h"
#include "ValveGoal.h"
#include "ChainGoal.h"
#include "../level/MissionSite.h"
#include "../level/Mission.h"

#define ONTOLOGY_DEBUG_ENABLED

Ontology::Ontology(ros::NodeHandle& ros_node, OctomapBuilder& octomap_builder)
	: OntologyInterface(ros_node, octomap_builder)
{
	
	ontology_server_ = ros_node.advertiseService("/knowledge/ontology", &Ontology::getKnowledge, this);
	
	complete_plan_listener_ = ros_node.subscribe("/planning_system/current_plan", 1, &Ontology::setCurrentPlan, this);
	
	ROS_INFO("Ontology services up and running...");
}

bool Ontology::getKnowledge(knowledge_msgs::KnowledgeInterface::Request& req, knowledge_msgs::KnowledgeInterface::Response& res)
{
	if ("get_type_instances" == req.request_type)
	{
		return GetInstancesOfType(req, res);
	}
	else if ("get_instance_properties" == req.request_type)
	{
		return GetAttributesOfInstance(req, res);
	}
	return false;
}

bool Ontology::getInspectionPoints(std::vector<InspectionPoint*>& inspection_points)
{
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
		{
			Structure* structure = *ci;
			for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
			{
				inspection_points.push_back(*ci);
			}
		}
	}
	return true;
}

void Ontology::observedInspectionPoint(const glm::vec3& inspection_point_location)
{
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
		{
			Structure* structure = *ci;
			for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
			{
				InspectionPoint* inspection_point = *ci;
				if (inspection_point->getPose().x_ == inspection_point_location.x &&
					inspection_point->getPose().y_ == inspection_point_location.y &&
					inspection_point->getPose().z_ == inspection_point_location.z)
				{
					if (inspection_point->getPillar() != NULL)
					{
						inspection_point->getPillar()->setObserved();
					}
				}
			}
		}
	}
}

Waypoint* Ontology::getWaypoint(const std::string& waypoint_name) const
{
	Waypoint* waypoint = OntologyInterface::getWaypoint(waypoint_name);
	if (waypoint != NULL)
	{
		return waypoint;
	}
	
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		if ((*ci)->getStartWaypoint().id_ == waypoint_name)
		{
			return &(*ci)->getStartWaypoint();
		}
	}
	
	for (std::vector<const AUV*>::const_iterator ci = auvs_.begin(); ci != auvs_.end(); ++ci)
	{
		if ((*ci)->getWaypoint().id_ == waypoint_name)
		{
			return &(*ci)->getWaypoint();
		}
	}
	/*
	if (waypoint_name.at(0) == 'W')
	{
		return OntologyInterface::getWaypoint(waypoint_name);
	}
	
	// We assume it starts with an L.
	int i = ::atoi(waypoint_name.substr(1).c_str());
	
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		if ((*ci)->getId() == i)
		{
			return &(*ci)->getStartWaypoint();
		}
	}
	*/
	std::cerr << "The waypoint " << waypoint_name << " does not exist!" << std::endl;
	assert(false);
	exit(1);
	return NULL;
}

bool Ontology::GetInstancesOfType(knowledge_msgs::KnowledgeInterface::Request& req, knowledge_msgs::KnowledgeInterface::Response& res)
{
#ifdef ONTOLOGY_DEBUG_ENABLED
	std::cout << "GetInstancesOfType: " << req.type_name << "; Mission ID: " << req.mission_id << std::endl;
#endif
	Mission* mission = missions_[req.mission_id];
	
	if (mission == NULL)
	{
		for (std::map<std::string, Mission*>::const_iterator ci = missions_.begin(); ci != missions_.end(); ++ci)
		{
			std::cout << (*ci).first << std::endl;
		}
		std::cout << "Mission " << req.mission_id << " does not exists!" << std::endl;
	}
	
	res.items.clear();
	if ("AUV" == req.type_name)
	{
		for (std::vector<const AUV*>::const_iterator ci = auvs_.begin(); ci != auvs_.end(); ++ci)
		{
			knowledge_msgs::KnowledgeObject knowledge_object;
			knowledge_object.type_name = "AUV";
			knowledge_object.instance_name = (*ci)->getName();
			res.items.push_back(knowledge_object);
		}
	}
	else if ("Chain" == req.type_name)
	{
		//Goal* goal = goal_mappings_[req.mission_id];
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			ChainGoal* chain_goal = dynamic_cast<ChainGoal*>(*ci);
			if (chain_goal != NULL)
			{
				if (chain_goal->getChain().hasBeenObserved())
				{
					knowledge_msgs::KnowledgeObject knowledge_object;
					knowledge_object.type_name = "Chain";
					knowledge_object.instance_name = chain_goal->getChain().getId();
					res.items.push_back(knowledge_object);
				}
			}
		}
		/*
		else
		{
			std::cerr << "There is no goal with mission id: " << req.mission_id << " (or it is not a chain goal)." << std::endl;
		}
		*/
	}
	else if ("Structure" == req.type_name)
	{
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			
			bool has_goal = false;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* structure = *ci;
				if (structure->getInspectionGoals().size() > 0)
				{
					has_goal = true;
					break;
				}
				
				for (std::vector<Valve*>::const_iterator ci = structure->getValves().begin(); ci != structure->getValves().end(); ++ci)
				{
					Valve* valve = *ci;
					for (std::vector<ValveGoal*>::const_iterator ci = valve->getValveGoals().begin(); ci != valve->getValveGoals().end(); ++ci)
					{
						has_goal = true;
					}
				}
			}
			
			for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
			{
				if (!(*ci)->hasBeenObserved())
				{
					continue;
				}
				has_goal = true;
			}
			
			if (has_goal || mission_site->canRecharge())
			{
				knowledge_msgs::KnowledgeObject knowledge_object;
				knowledge_object.type_name = "Structure";
				knowledge_object.instance_name = mission_site->getId();
				
				res.items.push_back(knowledge_object);
			}
			else
			{
				std::cerr << "Omit the structure " << mission_site->getId() << " as it has no goals!" << std::endl;
			}
		}
	}
	else if ("Waypoint" == req.type_name)
	{
		unsigned int i = 0;
		const std::vector<Waypoint*>& points = rrt_->getPoints();
		for (std::vector<Waypoint*>::const_iterator ci = points.begin(); ci != points.end(); ++ci)
		{
			knowledge_msgs::KnowledgeObject knowledge_object;
			knowledge_object.type_name = "Waypoint";
			
			knowledge_object.instance_name = (*ci)->id_;
			
			res.items.push_back(knowledge_object);
			++i;
		}
	}
	else if ("InspectionPoint" == req.type_name)
	{
		//Goal* goal = goal_mappings_[req.mission_id];
		
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			InspectionGoal* inspection_goal = dynamic_cast<InspectionGoal*>(*ci);
			if (inspection_goal == NULL)
			{
				std::cout << "Requested inspection points for a goal that is NOT an inspection goal. You get nothing!" << std::endl;
			}
			else
			{
				for (std::vector<InspectionPoint*>::const_iterator ci = inspection_goal->getInspectionPoints().begin(); ci != inspection_goal->getInspectionPoints().end(); ++ci)
				{
					if ((*ci)->getPillar() != NULL && (*ci)->getPillar()->hasBeenObserved())
					{
						std::cout << "Ommiting: " << (*ci)->getId() << " because its pillar has been observed!" << std::endl;
						continue;
					}
					knowledge_msgs::KnowledgeObject knowledge_object;
					knowledge_object.type_name = "InspectionPoint";
					knowledge_object.instance_name = (*ci)->getId();
					res.items.push_back(knowledge_object);
				}
			}
		}
	}
	else if ("Pillar" == req.type_name)
	{
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			Goal* goal = *ci;//goal_mappings_[req.mission_id];
			if (goal == NULL)
			{
				//std::cerr << "There is no goal with mission id: " << req.mission_id << "." << std::endl;
			}
			else
			{
				for (std::vector<Pillar*>::const_iterator ci = goal->getStructure().getMissionSite().getPillars().begin(); ci != goal->getStructure().getMissionSite().getPillars().end(); ++ci)
				{
					if (!(*ci)->hasBeenObserved())
					{
						continue;
					}
					
					knowledge_msgs::KnowledgeObject knowledge_object;
					knowledge_object.type_name = "Pillar";
					
					//std::stringstream ss;
					//ss << "Pillar" << (*ci)->getId();
					knowledge_object.instance_name = (*ci)->getName();//.str();
					
					res.items.push_back(knowledge_object);
				}
			}
		}
	}
	else if ("Panel" == req.type_name)
	{
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			Goal* goal = *ci;
			//Goal* goal = goal_mappings_[req.mission_id];
			
			ValveGoal* valve_goal = dynamic_cast<ValveGoal*>(goal);
			if (valve_goal != NULL)
			{
				Structure& valve_panel = valve_goal->getValve().getStructure();
				knowledge_msgs::KnowledgeObject knowledge_object;
				knowledge_object.type_name = "Panel";
				knowledge_object.instance_name = valve_panel.getName();
				
				res.items.push_back(knowledge_object);
			}
			else
			{
				//std::cerr << "There is no goal with mission id: " << req.mission_id << " (or it is not a valve goal)." << std::endl;
			}
		}
	}
	else if ("Valve" == req.type_name)
	{
		for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
		{
			Goal* goal = *ci;
			//Goal* goal = goal_mappings_[req.mission_id];
			
			ValveGoal* valve_goal = dynamic_cast<ValveGoal*>(goal);
			if (valve_goal != NULL)
			{
				knowledge_msgs::KnowledgeObject knowledge_object;
				knowledge_object.type_name = "Valve";
				//knowledge_object.instance_name = valve_goal->getValve().getName();
				knowledge_object.instance_name = valve_goal->getValve().getName();
				
				res.items.push_back(knowledge_object);
			}
			else
			{
				//std::cerr << "There is no goal with mission id: " << req.mission_id << " (or it is not a valve goal)." << std::endl;
			}
		}
	}
#ifdef ONTOLOGY_DEBUG_ENABLED
	for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = res.items.begin(); ci != res.items.end(); ++ci)
	{
		std::cout << "- " << (*ci).type_name << ": " << (*ci).instance_name << std::endl;
	}
#endif
	res.result = true;
	//res.response_type = knowledge_msgs::KnowledgeInterface::REQ_GET_INST;
	res.response_type = "get_type_instances";
	return true;
}

bool Ontology::GetAttributesOfInstance(knowledge_msgs::KnowledgeInterface::Request& req, knowledge_msgs::KnowledgeInterface::Response& res)
{
#ifdef ONTOLOGY_DEBUG_ENABLED
	std::cout << "GetAttributesOfInstance: " << req.instance_name << "; Type: " << req.type_name << std::endl;
#endif
	// Prepare return value.
	knowledge_msgs::KnowledgeObject knowledge_object;
	knowledge_object.type_name = req.type_name;
	knowledge_object.instance_name = req.instance_name;
	if ("AUV" == req.type_name)
	{
		for (std::vector<const AUV*>::const_iterator ci = auvs_.begin(); ci != auvs_.end(); ++ci)
		{
			if ((*ci)->getName() == req.instance_name)
			{
				knowledge_msgs::ObjectProperty kop;
				kop.name = "location";
				kop.type = "Waypoint";
				std::stringstream ss;
				ss << "wp_" << (*ci)->getName();
				kop.value = ss.str();
				knowledge_object.obj_properties.push_back(kop);
			}
		}
		res.items.push_back(knowledge_object);
	}
	else if ("Chain" == req.type_name)
	{
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Chain*>::const_iterator ci = mission_site->getChains().begin(); ci != mission_site->getChains().end(); ++ci)
			{
				Chain* chain = *ci;
				if ((*ci)->getId() == req.instance_name)
				{
					knowledge_msgs::ObjectProperty kop;
					kop.name = "location";
					kop.type = "Waypoint";
					//std::stringstream ss;
					//ss << "L" << chain->getMissionSite().getId();
					//kop.value = ss.str();
					kop.value = mission_site->getStartWaypoint().id_;
					knowledge_object.obj_properties.push_back(kop);
					
					knowledge_msgs::DataProperty kdp;
					/*
					{
						kdp.name = "Mission";
						{
						std::ostringstream ss;
						ss << "Mission,M" << chain->getGoalId();
						kdp.value = ss.str();
						}
						knowledge_object.data_properties.push_back(kdp);
					}
					*/
					{
						kdp.name = "examined";
						kdp.value = chain->isExamined() ? "true" : "false";
						knowledge_object.data_properties.push_back(kdp);
					}
				}
			}
		}
		res.items.push_back(knowledge_object);	
	}
	else if ("Structure" == req.type_name)
	{
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			//bool has_goal = false;
			
			std::cout << "Consider mission: " << mission_site->getId() << std::endl;
			
			if (mission_site->getId() == req.instance_name)
			{
				knowledge_msgs::DataProperty kdp;
				std::cout << "Found a connection: " << mission_site->getId() << std::endl;
				
				for (std::vector<Mission*>::const_iterator ci = mission_site->getMissions().begin(); ci != mission_site->getMissions().end(); ++ci)
				{
					const Mission* mission = *ci;
					
					std::cout << "Process the mission: " << mission->getId() << std::endl;
					
					bool mission_has_active_goal = false;
					for (std::vector<Goal*>::const_iterator ci = mission->getGoals().begin(); ci != mission->getGoals().end(); ++ci)
					{
						const Goal* goal = *ci;
						std::cout << "Process the goal: " << goal->getId() << " - is enabled? " << goal->isEnabled() << std::endl;
						if (goal->isEnabled())
						{
							//has_goal = true;
							mission_has_active_goal = true;
							break;
						}
					}
					
					if (mission_has_active_goal)
					{
						kdp.name = "Mission";
						{
						std::ostringstream ss;
						ss << "Mission," << mission->getId();
						kdp.value = ss.str();
						}
						knowledge_object.data_properties.push_back(kdp);
					}
				}
				
				knowledge_msgs::ObjectProperty kop;
				kop.name = "start_point";
				kop.type = "Waypoint";
				kop.value = mission_site->getStartWaypoint().id_;
				knowledge_object.obj_properties.push_back(kop);
				
				knowledge_msgs::DataProperty dop;
				dop.name = "has_recharge";
				dop.value = mission_site->canRecharge() ? "Boolean,true" : "Boolean,false";
				knowledge_object.data_properties.push_back(dop);
			}
		}
		res.items.push_back(knowledge_object);
	}
	else if ("Waypoint" == req.type_name)
	{
		const std::vector<Waypoint*>& points = rrt_->getPoints();
		Waypoint* w = getWaypoint(req.instance_name);
		if (w == NULL)
		{
			ROS_ERROR("Waypoint index out of range.");
			return false;
		}
		
		knowledge_msgs::DataProperty kdp;
		kdp.name = "wpN";
		{
		std::ostringstream ss;
		ss << "float," << w->position_.z;
		kdp.value = ss.str();
		}
		knowledge_object.data_properties.push_back(kdp);
		
		kdp.name = "wpE";
		{
		std::ostringstream ss;
		ss << "float," << w->position_.x;
		kdp.value = ss.str();
		}
		knowledge_object.data_properties.push_back(kdp);
		
		kdp.name = "wpD";
		{
		std::ostringstream ss;
		ss << "float," << w->position_.y;
		kdp.value = ss.str();
		}
		knowledge_object.data_properties.push_back(kdp);
		
		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = w->edges_.begin(); ci != w->edges_.end(); ++ci)
		{
			Waypoint* w2 = (*ci).first;
			knowledge_msgs::ObjectProperty object_property;
			object_property.name = "connectedTo";
			object_property.value = w2->id_;
			object_property.type = "Waypoint";
			knowledge_object.obj_properties.push_back(object_property);
			/*
			for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
			{
				MissionSite* site = *ci;
				if (w2->position_ == site->getStartWaypoint().position_)
				{
					knowledge_msgs::ObjectProperty object_property;
					object_property.name = "connectedTo";
					object_property.value = site->getStartWaypoint().id_;
					object_property.type = "Waypoint";
					knowledge_object.obj_properties.push_back(object_property);
					break;
				}
			}
			
			// Find its index.
			int index = 0;
			for (std::vector<Waypoint*>::const_iterator ci = points.begin(); ci != points.end(); ++ci)
			{
				// Check if these waypoints are connected.
				if (*ci == w2 && !octomap_->isBlocked(w->position_, w2->position_, 2.0f))
				{
					knowledge_msgs::ObjectProperty object_property;
					object_property.name = "connectedTo";
					object_property.value = (*ci)->id_;
					object_property.type = "Waypoint";
					knowledge_object.obj_properties.push_back(object_property);
					break;
				}
			}
			*/
		}
		
		// Check if this waypoint is designated to look from to a inspection point.
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* structure = *ci;
				for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
				{
					const InspectionPoint* inspection_point = *ci;
					
					if (w->position_ == inspection_point->getVisiblePoint())
					{
						knowledge_msgs::ObjectProperty object_property;
						object_property.name = "canSee";
						std::stringstream ss;
						ss << "InspectionPoint," << inspection_point->getId();
						object_property.value = ss.str();
						object_property.type = "InspectionPoint";
						knowledge_object.obj_properties.push_back(object_property);
						
						if (inspection_point->getPillar() != NULL)
						{
							knowledge_msgs::ObjectProperty object_property;
							object_property.name = "canSeePillar";
							std::stringstream ss;
							ss << "Pillar," << inspection_point->getPillar()->getName();
							
							object_property.value = ss.str();
							object_property.type = "Pillar";
							knowledge_object.obj_properties.push_back(object_property);
						}
					}
				}
			}
		}
		
		res.items.push_back(knowledge_object);
	}
	else if ("InspectionPoint" == req.type_name)
	{
		bool found_inspection_point = false;
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* structure = *ci;
				for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
				{
					const InspectionPoint* inspection_point = *ci;
				
					if (inspection_point->getId() == req.instance_name)
					{
						knowledge_msgs::DataProperty kdp;
						kdp.name = "ipN";
						{
						std::ostringstream ss;
						ss << "float," << inspection_point->getPose().z_;
						kdp.value = ss.str();
						}
						knowledge_object.data_properties.push_back(kdp);
						
						kdp.name = "ipE";
						{
						std::ostringstream ss;
						ss << "float," << inspection_point->getPose().x_;
						kdp.value = ss.str();
						}
						knowledge_object.data_properties.push_back(kdp);
						
						kdp.name = "ipD";
						{
						std::ostringstream ss;
						ss << "float," << inspection_point->getPose().y_;
						kdp.value = ss.str();
						}
						knowledge_object.data_properties.push_back(kdp);
						
						kdp.name = "ipY";
						kdp.value = "float,0";
						knowledge_object.data_properties.push_back(kdp);
						
						// Check which pillar is part of this inspection point.
						if (inspection_point->getPillar() != NULL)
						{
							knowledge_msgs::ObjectProperty object_property;
							object_property.name = "isPartOfPillar";
							std::stringstream ss;
							ss << "Pillar," << inspection_point->getPillar()->getName();
							
							object_property.value = ss.str();
							object_property.type = "Pillar";
							knowledge_object.obj_properties.push_back(object_property);
						}
						
						res.items.push_back(knowledge_object);
						found_inspection_point = true;
						break;
					}
				}
			}
			if (found_inspection_point) break;
		}
		res.items.push_back(knowledge_object);
	}
	
	else if ("Pillar" == req.type_name)
	{
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Pillar*>::const_iterator ci = mission_site->getPillars().begin(); ci != mission_site->getPillars().end(); ++ci)
			{
				Pillar* pillar = *ci;
				if (pillar->getName() == req.instance_name)
				{
					knowledge_msgs::DataProperty kdp;
					kdp.name = "pN";
					{
					std::ostringstream ss;
					ss << "float," << pillar->getGlobalLocation().z;
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					kdp.name = "pE";
					{
					std::ostringstream ss;
					ss << "float," << pillar->getGlobalLocation().x;
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					kdp.name = "pD";
					{
					std::ostringstream ss;
					ss << "float," << pillar->getGlobalLocation().y;
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
				}
			}
		}
		
		res.items.push_back(knowledge_object);
	}
	
	else if ("Panel" == req.type_name)
	{
		bool found_panel = false;
		
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				const Structure* structure = *ci;
				if (structure->getValves().empty())
				{
					continue;
				}
				
				if (structure->getName() != req.instance_name)
				{
					continue;
				}
				found_panel = true;
				knowledge_msgs::DataProperty kdp;
	
				kdp.name = "ID";
				{
				kdp.value = structure->getName();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "N";
				{
				std::ostringstream ss;
				ss << structure->getGlobalLocation().z;
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "E";
				{
				std::ostringstream ss;
				ss << structure->getGlobalLocation().x;
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "D";
				{
				std::ostringstream ss;
				ss << structure->getGlobalLocation().y;
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "Y";
				kdp.value = "0";
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "examined";
				kdp.value = structure->isExamined() ? "true" : "false";
				knowledge_object.data_properties.push_back(kdp);
				
				// Check if this waypoint is designated to interact from for this Valvee.
				int index = 0;
				for (std::vector<Waypoint*>::const_iterator ci = rrt_->getPoints().begin(); ci != rrt_->getPoints().end(); ++ci)
				{
					const Waypoint* waypoint = *ci;
					
					//std::cout << "Compare: (" << waypoint->position_.x << ", " << waypoint->position_.y << ", "  << waypoint->position_.z << ") -> Panel: (" << structure->getInteractLocation().x << ", " << structure->getInteractLocation().y << ", " << structure->getInteractLocation().z << ")" << std::endl;
					
					if (structure->getInteractLocation() == waypoint->position_)
					{
						knowledge_msgs::ObjectProperty object_property;
						object_property.name = "canExamine";
						object_property.value = waypoint->id_;
						object_property.type = "Waypoint";
						knowledge_object.obj_properties.push_back(object_property);
					}
				}
				
				for (std::vector<Valve*>::const_iterator ci = structure->getValves().begin(); ci != structure->getValves().end(); ++ci)
				{
					const Valve* valve = *ci;
					knowledge_msgs::ObjectProperty object_property;
					object_property.name = "valve";
					object_property.value = valve->getName();
					object_property.type = "Valve";
					knowledge_object.obj_properties.push_back(object_property);
				}
				res.items.push_back(knowledge_object);
				break;
			}
			if (found_panel) break;
		}
		res.items.push_back(knowledge_object);
	}
	else if ("Valve" == req.type_name)
	{
		bool found_valve = false;
		
		for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
		{
			MissionSite* mission_site = *ci;
			for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
			{
				Structure* valve_panel = *ci;
				for (std::vector<Valve*>::const_iterator ci = valve_panel->getValves().begin(); ci != valve_panel->getValves().end(); ++ci)
				{
					Valve* valve = *ci;
					if (valve->getName() != req.instance_name)
					{
						continue;
					}
					found_valve = true;
					
					knowledge_msgs::DataProperty kdp;
		
					kdp.name = "ID";
					{
					kdp.value = valve->getName();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					kdp.name = "panel_ID";
					{
					std::ostringstream ss;
					ss << valve->getStructure().getName();
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					kdp.name = "valve_angle";
					{
					std::ostringstream ss;
					ss << (glm::roll(valve->getLocalRotation()) / 180.0f) * M_PI;
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					kdp.name = "valve_blocked";
					{
					std::ostringstream ss;
					ss << valve->getTimesBlocked();
					kdp.value = ss.str();
					}
					knowledge_object.data_properties.push_back(kdp);
					
					// Get the goals.
					for (std::vector<ValveGoal*>::const_iterator ci = valve->getValveGoals().begin(); ci != valve->getValveGoals().end(); ++ci)
					{
						const ValveGoal* valve_goal = *ci;
						
						Mission* missions = missions_[req.mission_id];
						if (std::find(missions->getGoals().begin(), missions->getGoals().end(), valve_goal) != missions->getGoals().end())
						{
							knowledge_msgs::ObjectProperty object_property;
							object_property.name = "goal";
							object_property.value = valve_goal->getId();
							object_property.type = "ValveGoal";
							knowledge_object.obj_properties.push_back(object_property);
						}
					}
					
					break;
				}
				if (found_valve) break;
			}
			if (found_valve) break;
		}
		res.items.push_back(knowledge_object);
	}
	else if ("ValveGoal" == req.type_name)
	{
		Mission* misision = missions_[req.mission_id];
		//std::cout << "Found " << misision->getGoals().size() << " goals matching this type." << std::endl;
		for (std::vector<Goal*>::const_iterator ci = misision->getGoals().begin(); ci != misision->getGoals().end(); ++ci)
		{
			Goal* goal = *ci;
				
			ValveGoal* valve_goal = dynamic_cast<ValveGoal*>(goal);
			if (valve_goal != NULL && valve_goal->getId() == req.instance_name)
			{
				knowledge_msgs::DataProperty kdp;

				kdp.name = "ID";
				{
				kdp.value = valve_goal->getId();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "startTime";
				{
				std::ostringstream ss;
				ss << valve_goal->getStartTime();
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "deadline";
				{
				std::ostringstream ss;
				ss << valve_goal->getDeadline();
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
				
				kdp.name = "valve_angle";
				{
				std::ostringstream ss;
				ss << valve_goal->getValveAngle();
				kdp.value = ss.str();
				}
				knowledge_object.data_properties.push_back(kdp);
			}
			else
			{
				//std::cout << "This was not a valve goal!" << std::endl;
			}
		}
		res.items.push_back(knowledge_object);
	}
#ifdef ONTOLOGY_DEBUG_ENABLED
	std::cout << "GetAttributesOfInstance: " << req.instance_name << "; Type: " << req.type_name << std::endl;
	
	std::cout << "Result: " << std::endl;
	for (std::vector<knowledge_msgs::KnowledgeObject>::const_iterator ci = res.items.begin(); ci != res.items.end(); ++ci)
	{
		const knowledge_msgs::KnowledgeObject& knowledge_object = *ci;
 		std::cout << "- " << knowledge_object.type_name << ": " << knowledge_object.instance_name << std::endl;
		for (std::vector<knowledge_msgs::DataProperty>::const_iterator ci = knowledge_object.data_properties.begin(); ci != knowledge_object.data_properties.end(); ++ci)
		{
			std::cout << "DATA: " << (*ci).name << " - " << (*ci).value << std::endl;
		}
		for (std::vector<knowledge_msgs::ObjectProperty>::const_iterator ci = knowledge_object.obj_properties.begin(); ci != knowledge_object.obj_properties.end(); ++ci)
		{
			std::cout << "OBJECT: " << (*ci).name << " - " << (*ci).value << " - " << (*ci).type << std::endl;
		}
	}
#endif
	res.result = true;
	//res.response_type = knowledge_msgs::KnowledgeInterface::REQ_GET_INST_PROP;
	res.response_type = "get_instance_properties";
	return true;
}

void Ontology::setCurrentPlan(const planning_msgs::CompletePlan::ConstPtr& msg)
{
#ifdef ONTOLOGY_DEBUG_ENABLED
	for (std::vector<planning_msgs::ActionDispatch>::const_iterator ci = msg->actions.begin(); ci != msg->actions.end(); ++ci)
	{
		const planning_msgs::ActionDispatch& action = *ci;
		std::cout << "(d=" << (*ci).duration << ") " << (*ci).action_id << ": " << (*ci).name << std::endl;
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = action.parameters.begin(); ci != action.parameters.end(); ++ci)
		{
			std::cout << "\t*" << (*ci).key << " - " << (*ci).value << std::endl;
		}
	}
#endif
}

InspectionPoint& Ontology::getInspectionPoint(const std::string& inspection_point_name)
{
	for (std::vector<MissionSite*>::const_iterator ci = mission_sites_.begin(); ci != mission_sites_.end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		for (std::vector<Structure*>::const_iterator ci = mission_site->getStructures().begin(); ci != mission_site->getStructures().end(); ++ci)
		{
			Structure* structure = *ci;
			for (std::vector<InspectionPoint*>::const_iterator ci = structure->getInspectionPoints().begin(); ci != structure->getInspectionPoints().end(); ++ci)
			{
				InspectionPoint* inspection_point = *ci;
				if (inspection_point->getId() == inspection_point_name)
				{
					return *inspection_point;
				}
			}
		}
	}
	std::cerr << "Could not get the inspection point: " << inspection_point_name << "." << std::endl;
	assert(false);
	exit(1);
}
