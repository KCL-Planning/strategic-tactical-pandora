#ifdef _WIN32
#include <Windows.h>
#endif

#include <set>
#include <queue>
#include <time.h>
#include <algorithm>
#include <vector>

#include <GL/glew.h>
#include <string.h>
#include "RRT.h"

#include <glm/gtx/quaternion.hpp>

#include "../../core/shaders/LineShader.h"
#include "../../core/scene/SceneManager.h"
#include "../../core/scene/SceneNode.h"
#include "../../core/collision/CollisionInfo.h"
#include "../../core/entities/Entity.h"

#include "ontology/OntologyInterface.h"
#include "ontology/OctomapBuilder.h"
#include "ontology/InspectionPoint.h"
#include "ontology/InspectionGoal.h"
#include "ontology/ChainGoal.h"
#include "structures/ValvePanel.h"
#include "structures/Valve.h"
#include "structures/Chain.h"
#include "level/MissionSite.h"
#include "level/Mission.h"

RRT::RRT(ros::NodeHandle& ros_node, SceneManager& scene_manager, Entity& entity, SceneNode& parent, OctomapBuilder& octomap, OntologyInterface& ontology, Material& material)
	: SceneLeafModel(parent, NULL, *(new Line(false)), material, LineShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW),  ros_node_(&ros_node), scene_manager_(&scene_manager), octomap_(&octomap), ontology_(&ontology)
{
	entities_.push_back(&entity);
	// Launch the ROS services.
	roadmap_refresh_service_ = ros_node.advertiseService("/roadmap_server/refresh", &RRT::refreshRoadmaps, this);
}

RRT::RRT(ros::NodeHandle& ros_node, SceneManager& scene_manager, std::vector<Entity*>& entities, SceneNode& parent, OctomapBuilder& octomap, OntologyInterface& ontology, Material& material)
	: SceneLeafModel(parent, NULL, *(new Line(false)), material, LineShader::getShader(), false, false, OBJECT, ShadowRenderer::NO_SHADOW),  ros_node_(&ros_node), scene_manager_(&scene_manager), octomap_(&octomap), ontology_(&ontology)
{
	entities_ = entities;
	// Launch the ROS services.
	roadmap_refresh_service_ = ros_node.advertiseService("/roadmap_server/refresh", &RRT::refreshRoadmaps, this);
}

RRT::~RRT()
{
	for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		delete *ci;
	}
}

bool RRT::refreshRoadmaps(knowledge_msgs::RoadmapRefresh::Request& req, knowledge_msgs::RoadmapRefresh::Response& res)
{
	// Update the listeners.
	for (std::vector<RRTUpdateListener*>::const_iterator ci = update_listeners_.begin(); ci != update_listeners_.end(); ++ci)
	{
		(*ci)->rrtInvalidated();
	}
	
	std::set<std::string> mission_ids;
	for (std::vector<std::string>::const_iterator ci = req.missions.begin(); ci != req.missions.end(); ++ci)
	{
		mission_ids.insert(*ci);
		//std::cout << "Mission id: " << *ci << std::endl;
	}
	
	std::set<Waypoint*> strategic_locations;
	std::vector<glm::vec3> strategic_points;
	std::vector<bool> goals_not_reached(entities_.size(), false);
	
	for (std::vector<MissionSite*>::const_iterator ci = ontology_->getMissionSites().begin(); ci != ontology_->getMissionSites().end(); ++ci)
	{
		MissionSite* mission_site = *ci;
		bool include_mission_site = false;
		
		for (std::vector<Mission*>::const_iterator ci = mission_site->getMissions().begin(); ci != mission_site->getMissions().end(); ++ci)
		{
			Mission* mission = *ci;
			if (mission_ids.count(mission->getId()) == 1)
			{
				include_mission_site = true;
				mission->getStrageticPoints(strategic_points);	
			}
		}
		
		if (include_mission_site)
		{
			Waypoint* mission_site_waypoint = new Waypoint(mission_site->getStartWaypoint().id_, mission_site->getStartWaypoint().position_, goals_not_reached);
			strategic_locations.insert(mission_site_waypoint);
		}
	}
	
	for (std::vector<glm::vec3>::const_iterator ci = strategic_points.begin(); ci != strategic_points.end(); ++ci)
	{
		std::stringstream ss;
		ss << "strategic_location_" << strategic_locations.size();
		strategic_locations.insert(new Waypoint(ss.str(), *ci, goals_not_reached));
	}
	
	//ROS_INFO("refreshRoadmaps");
	//createRRT(entity_->getGlobalLocation(), ontology_->getInspectionPoints(), -15, 15, 0, 15, -5, 15);
	std::vector<glm::vec3> auv_locations;
	for (std::vector<Entity*>::const_iterator ci = entities_.begin(); ci != entities_.end(); ++ci)
	{
		auv_locations.push_back((*ci)->getGlobalLocation());
		std::cout << "(" << (*ci)->getGlobalLocation().x << ", " << (*ci)->getGlobalLocation().y << ", "  << (*ci)->getGlobalLocation().z << ")" << std::endl;
	}
	
	// Create a perimiter around the start / end points.
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = 0.5f;
	float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max();
	float max_z = -std::numeric_limits<float>::max();
	
	for (std::vector<glm::vec3>::const_iterator ci = auv_locations.begin(); ci != auv_locations.end(); ++ci)
	{
		if (min_x > (*ci).x) min_x = (*ci).x;
		if (max_x < (*ci).x) max_x = (*ci).x;
		if (min_y > (*ci).y) min_y = (*ci).y;
		if (max_y < (*ci).y) max_y = (*ci).y;
		if (min_z > (*ci).z) min_z = (*ci).z;
		if (max_z < (*ci).z) max_z = (*ci).z;
	}
	
	for (std::set<Waypoint*>::const_iterator ci = strategic_locations.begin(); ci != strategic_locations.end(); ++ci)
	{
		const Waypoint* waypoint = *ci;
		if (min_x > waypoint->position_.x) min_x = waypoint->position_.x;
		if (max_x < waypoint->position_.x) max_x = waypoint->position_.x;
		if (min_y > waypoint->position_.y) min_y = waypoint->position_.y;
		if (max_y < waypoint->position_.y) max_y = waypoint->position_.y;
		if (min_z > waypoint->position_.z) min_z = waypoint->position_.z;
		if (max_z < waypoint->position_.z) max_z = waypoint->position_.z;
	}
	
	//createRRT(auv_locations, strategic_locations, -100, 100, 1, 100, -100, 100);
	createRRT(auv_locations, strategic_locations, min_x - 15, max_x + 15, min_y, max_y + 15, min_z - 15, max_z + 15);
	
	//ROS_INFO("Done! refreshRoadmaps");
	/*
	for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		const Waypoint* waypoint = *ci;
		std::cout << "[" << waypoint->id_ << "] (" << waypoint->position_.x << ", " << waypoint->position_.y << ", " << waypoint->position_.z << "). Is connected to start? ";
		for (std::vector<bool>::const_iterator ci = waypoint->is_connected_to_start_.begin(); ci != waypoint->is_connected_to_start_.end(); ++ci)
		{
			std::cout << (*ci) << ", " << std::endl;
		}
		std::cout << std::endl;
		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = waypoint->edges_.begin(); ci != waypoint->edges_.end(); ++ci)
		{
			const Waypoint* waypoint2 = (*ci).first;
			std::cout << "\t[" << waypoint2->id_ << "] (" << waypoint2->position_.x << ", " << waypoint2->position_.y << ", " << waypoint2->position_.z << "). Is connected to start? " << "Distance = " << (*ci).second;
			for (std::vector<bool>::const_iterator ci = waypoint2->is_connected_to_start_.begin(); ci != waypoint2->is_connected_to_start_.end(); ++ci)
			{
				std::cout << (*ci) << ", " << std::endl;
			}
			std::cout << std::endl;
		}
	}
	*/
	for (std::vector<RRTUpdateListener*>::const_iterator ci = update_listeners_.begin(); ci != update_listeners_.end(); ++ci)
	{
		(*ci)->rrtUpdated();
	}
	return true;
}

bool RRT::canConnect(const glm::vec3& w1, const glm::vec3& w2, float min_width, bool perfect_data) const
{
	if (perfect_data)
	{
		return !scene_manager_->getRoot().doesCollide(entities_[0], w1, w2, min_width);
	}
	return !octomap_->isBlocked(w1, w2, min_width);
}

void RRT::createRRT(const std::vector<glm::vec3>& begin, std::set<Waypoint*>& goals, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z)
{
	float min_d = 1.0f;
	float max_d = 20.0f;
	float bias_towards_goal = 0.4f;
	/*
	std::cout << "[RRT::createRRT] " << std::endl;
	for (std::vector<glm::vec3>::const_iterator ci = begin.begin(); ci != begin.end(); ++ci)
	{
		std::cout << "\t- (" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
	}
	for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
	{
		std::cout << "\t+ (" << (*ci)->position_.x << ", " << (*ci)->position_.y << ", " << (*ci)->position_.z << ")" << std::endl;
	}
	*/
	for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		/*
		// Don't delete waypoints that are part of mission sites.
		if ((*ci)->id_[0] != 'L')
		{
			delete *ci;
		}
		else*/
		{
			(*ci)->edges_.clear();
			(*ci)->is_connected_to_start_ = std::vector<bool>(begin.size(), false);
		}
	}
	points_.clear();
	path_buffer_.clear();
	path_.clear();
	
	std::vector<Waypoint*> starting_waypoints;

	unsigned int max_connections = 10;
	CollisionInfo collision;

	for (unsigned int i = 0; i < begin.size(); ++i)
	{
		std::vector<bool> connected_to_begin(begin.size(), false);
		connected_to_begin[i] = true;
		std::stringstream ss;
		ss << "wp_AUV" << i;
		Waypoint* start = new Waypoint(ss.str(), begin[i], connected_to_begin);
		starting_waypoints.push_back(start);
		points_.push_back(start);
	
		// Add the waypoint that is on top of the mission site.
		for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
		{
			Waypoint* goal = *ci;
			goal->edges_.clear();
			goal->is_connected_to_start_ = std::vector<bool>(begin.size(), false);
			for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
			{
				Waypoint* other_goal = *ci;
				if (canConnect(goal->position_, other_goal->position_, 1.5f, false))
				{
					goal->edges_.push_back(std::make_pair(other_goal, glm::distance(goal->position_, other_goal->position_)));
					other_goal->edges_.push_back(std::make_pair(goal, glm::distance(goal->position_, other_goal->position_)));
				}
			}
			
			//std::cout << "Can connect " << *start << " TO " << *goal << "?" << std::endl;
			if (canConnect(start->position_, goal->position_, 1.5f, false))
			{
				start->edges_.push_back(std::make_pair(goal, glm::distance(start->position_, goal->position_)));
				goal->edges_.push_back(std::make_pair(start, glm::distance(start->position_, goal->position_)));
				goal->is_connected_to_start_[i] = true;
			}
		}
	}
	/*
	std::cout << "Start waypoints after initialisation: " << std::endl;
	for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		std::cout << **ci << std::endl;
	}
	*/
	
	float x, y, z;
	while (true)
	{
		//std::cout << "Update starting waypoints." << std::endl;
		for (unsigned int i = 0; i < starting_waypoints.size(); ++i)
		{
			std::set<Waypoint*> processed_waypoints;
			starting_waypoints[i]->updateConnectivity(processed_waypoints, i);
			//std::cout << *starting_waypoints[i] << std::endl;
		}
		
		//std::cout << "Check all the goals: " << std::endl;
		// Check if we have connected all the goals.
		bool all_goals_connected = true;
		for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
		{
			Waypoint* goal_waypoint = *ci;
			//std::cout << *goal_waypoint << std::endl;
			for (std::vector<bool>::const_iterator ci = goal_waypoint->is_connected_to_start_.begin(); ci != goal_waypoint->is_connected_to_start_.end(); ++ci)
			{
				if (*ci == false)
				{
					//std::cout << "Cannot connect: " << goal_waypoint->id_ << " (" << goal_waypoint->position_.x << ", " << goal_waypoint->position_.y << ", " << goal_waypoint->position_.z << ")" << std::endl;
					all_goals_connected = false;
					break;
				}
			}
			if (!all_goals_connected)
			{
				break;
			}
		}
		if (all_goals_connected) break;

		// Check if we are going to create a point towards the goal or if we generate a random point.
		if (static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x - min_x))) < bias_towards_goal)
		{
			const Waypoint* w = points_[rand() % points_.size()];
			
			// Pick the closest goal.
			float minimal_distance_to_goal = std::numeric_limits<float>::max();
			glm::vec3 closest_goal;
			for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
			{
				float d = glm::distance((*ci)->position_, w->position_);
				if (d < minimal_distance_to_goal)
				{
					minimal_distance_to_goal = d;
					closest_goal = (*ci)->position_;
				}
			}
			
			glm::vec3 v = closest_goal - w->position_;
			v = glm::normalize(v) * static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)) * max_d;

			x = w->position_.x + v.x;
			y = w->position_.y + v.y;
			z = w->position_.z + v.z;
		}
		else
		{
			x = min_x + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * (max_x - min_x);
			y = min_y + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * (max_y - min_y);
			z = min_z + (static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) * (max_z - min_z);
		}

		glm::vec3 point(x, y, z);

		std::vector<bool> connectivity(entities_.size(), false);
		Waypoint* wp = new Waypoint(points_.size(), point, connectivity);

		// Sort waypoints such that the waypoints closer to this new point are accessed first.
		CloseWaypointCompare close_compare(*wp);
		std::sort(points_.begin(), points_.end(), close_compare);

		// Search for the max_connections closest waypoints it can connect with.
		std::vector<Waypoint*> points_too_close;
		for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
		{
			Waypoint* other_wp = *ci;
			
			// Ignore waypoints that are goals.
			/*if (goals.count(other_wp) != 0)
			{
				continue;
			}*/

			if (!canConnect(point, other_wp->position_, 1.5f, false))
			{
				continue;
			}

			float d = glm::distance(other_wp->position_, point);
			if (d > max_d) break; // Due to the ordering, if the point is too far away we know that the subsequent nodes will be too.
			// Ignore points that are too close.
			if (d < min_d)
			{
				points_too_close.push_back(other_wp);
				continue;
			}

			wp->edges_.push_back(std::make_pair(other_wp, d));
			other_wp->edges_.push_back(std::make_pair(wp, d));
			
			for (unsigned int i = 0; i < other_wp->is_connected_to_start_.size(); ++i)
			{
				if (other_wp->is_connected_to_start_[i])
				{
					wp->is_connected_to_start_[i] = true;
				}
			}
			/*
			if (wp->edges_.size() >= max_connections)
			{
				break;
			}
			*/
		}
		
		// If the point has no edges we can delete it.
		if (wp->edges_.empty())
		{
			delete wp;
			continue;
		}
		
		// Connect with waypoints that were previously thought to be too close.
		for (std::vector<Waypoint*>::const_iterator ci = points_too_close.begin(); ci != points_too_close.end(); ++ci)
		{
			Waypoint* other_wp = *ci;
			float d = glm::distance(other_wp->position_, point);
			wp->edges_.push_back(std::make_pair(other_wp, d));
			other_wp->edges_.push_back(std::make_pair(wp, d));
		}
		
		// Try to connect the waypoint to the goal(s).
		for (std::set<Waypoint*>::const_iterator ci = goals.begin(); ci != goals.end(); ++ci)
		{
			Waypoint* goal_waypoint = *ci;
			if (canConnect(point, goal_waypoint->position_, 1.5f, false))
			{
				wp->edges_.push_back(std::make_pair(goal_waypoint, glm::distance(point, goal_waypoint->position_)));
				goal_waypoint->edges_.push_back(std::make_pair(wp, glm::distance(point, goal_waypoint->position_)));
			}
		}

		//std::cout << "NEW WAYPOIN!" << *wp << std::endl;
		points_.push_back(wp);
		if (points_.size() % 50 == 0)
		{
#ifdef _WIN32
			std::stringstream ss;
			ss << "Points: " << points_.size() << std::endl;
			OutputDebugString(ss.str().c_str());
#endif
			//std::cout << "Points: " << points_.size() << std::endl;
		}
	}

	// In order for the planner to recognise the initial location of the AUV it needs to be the first waypoint.
	for (std::vector<Waypoint*>::iterator i = points_.begin(); i != points_.end(); ++i)
	{
		if (std::find(starting_waypoints.begin(), starting_waypoints.end(), *i) != starting_waypoints.end())
		//if (*i == start)
		{
			points_.erase(i);
			break;
		}
	}
	//points_.insert(points_.begin(), start);
	points_.insert(points_.begin(), starting_waypoints.begin(), starting_waypoints.end());

	points_.insert(points_.end(), goals.begin(), goals.end());
	std::vector<glm::vec3> line_points;
	for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		Waypoint* w = *ci;
		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = w->edges_.begin(); ci != w->edges_.end(); ++ci)
		{
			Waypoint* w2 = (*ci).first;
			line_points.push_back(w->position_);
			line_points.push_back(w2->position_);
		}
	}
	shape_->setVertexBuffer(line_points);
	
#ifdef _WIN32
		float begin_prepare = float(clock()) / CLOCKS_PER_SEC;
#endif
	//findPath(*start, *finish);
#ifdef _WIN32
		std::stringstream ss;
		ss << "Find path took: " << float(clock()) / CLOCKS_PER_SEC - begin_prepare << " seconds!" << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
}

void RRT::findPath(Waypoint& start, Waypoint& end)
{
	/*for (std::vector<Waypoint*>::const_iterator ci = points_.begin(); ci != points_.end(); ++ci)
	{
		(*ci)->total_distance_to_start_ = (*ci == &start ? 0 : std::numeric_limits<float>::max());
		(*ci)->parent_ = NULL;
	}*/

	SearchNode* start_node = new SearchNode(start);
	SearchNode* end_node = NULL;

	std::set<const Waypoint*> closed_set;
	std::priority_queue<SearchNode*, std::vector<SearchNode*>, SearchNode> open_queue;

	open_queue.push(start_node);
	while (open_queue.size() > 0)
	{
		SearchNode* w = open_queue.top();
		open_queue.pop();

		if (closed_set.count(w->waypoint_) != 0)
		{
			delete w;
			continue;
		}
		closed_set.insert(w->waypoint_);
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Process: (" << w->waypoint_->position_.x << ", " << w->waypoint_->position_.y << ", " << w->waypoint_->position_.z << ")[" << w->total_distance_to_start_ << "]" << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
		if (w->waypoint_ == &end)
		{
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Found the end!" << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
			end_node = w;
			break;
		}

		for (std::vector<std::pair<Waypoint*, float> >::const_iterator ci = w->waypoint_->edges_.begin(); ci != w->waypoint_->edges_.end(); ++ci)
		{
			Waypoint* next_w = (*ci).first;
			SearchNode* next_sn = new SearchNode(*next_w, w, w->total_distance_to_start_ + (*ci).second);
/*
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "Possible new child? (" << next_w->position_.x << ", " << next_w->position_.y << ", " << next_w->position_.z << ")[" << w->total_distance_to_start_ << " + " << (*ci).second << "]" << std::endl;
			OutputDebugString(ss.str().c_str());
			}
			{
			std::stringstream ss;
			ss << open_queue.size() << " | New child: (" << next_w->position_.x << ", " << next_w->position_.y << ", " << next_w->position_.z << ") " << next_sn->total_distance_to_start_ << " - " << w->total_distance_to_start_ + (*ci).second << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#endif
*/
			open_queue.push(next_sn);
		}
	}
/*
#ifdef _WIN32
	std::stringstream ss;
	ss << "Create path: " << std::endl;
#endif
*/
	while (end_node != NULL)
	{
/*
#ifdef _WIN32
		ss << "(" << end_node->waypoint_->position_.x << ", " << end_node->waypoint_->position_.y << ", " << end_node->waypoint_->position_.z << ") -- ";
		if (end_node->parent_ != NULL)
		{
			ss << "(" << end_node->parent_->waypoint_->position_.x << ", " << end_node->parent_->waypoint_->position_.y << ", " << end_node->parent_->waypoint_->position_.z << ") -- ";
		}
#endif
*/
		path_buffer_.insert(path_buffer_.end(), end_node->waypoint_->position_);
		path_.insert(path_.begin(), end_node->waypoint_);
		end_node = end_node->parent_;
	}
/*
	std::vector<glm::vec3> test_line_poinst;
	test_line_poinst.push_back(glm::vec3(0, 0, 0));
	test_line_poinst.push_back(glm::vec3(0, 10, 0));
	test_line_poinst.push_back(glm::vec3(0, 10, 10));
	test_line_poinst.push_back(glm::vec3(10, 10, 0));
	test_line_poinst.push_back(glm::vec3(0, 10, 10));
	test_line_poinst.push_back(glm::vec3(10, 10, 10));
	shape_->setVertexBuffer(test_line_poinst);
*/
	shape_->setVertexBuffer(path_buffer_);
	
	if (path_buffer_.empty())
	{
#ifdef _WIN32
	OutputDebugString("NO Path was found!?");
#else
	std::cerr << "No path was found!?" << std::endl;
#endif
	}
}

void RRT::draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader) const
{
	return;
	//void initialise(const glm::mat4& view_matrix, const glm::mat4& model_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights);
	shader_->initialise(*this, view_matrix,  glm::mat4(1.0f), projection_matrix, lights);
	shape_->render();
}

void RRT::clear()
{
	points_.clear();
	path_buffer_.clear();
	path_.clear();
}
