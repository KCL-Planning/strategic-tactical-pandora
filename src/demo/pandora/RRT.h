#ifndef DEMO_PANDORA_RRT_H
#define DEMO_PANDORA_RRT_H

#include <ros/ros.h>
#include <knowledge_msgs/RoadmapRefresh.h>

#include <vector>
#include <glm/glm.hpp>

#include "../../core/scene/SceneLeafModel.h"
#include "../../shapes/Line.h"

#include "RRTUpdateListener.h"

#include "Waypoint.h"

class SceneManager;
class Entity;
class OctomapBuilder;
class OntologyInterface;
class Material;

struct CloseWaypointCompare
{
	CloseWaypointCompare(const Waypoint& wp)
		: wp_(&wp)
	{

	}

	bool operator()(Waypoint* lhs, Waypoint* rhs)
	{
		return glm::distance(wp_->position_, lhs->position_) < glm::distance(rhs->position_, wp_->position_);
	}

	const Waypoint* wp_;
};

struct SearchNode
{
	SearchNode()
		: waypoint_(NULL), parent_(NULL), total_distance_to_start_(0)
	{

	}

	SearchNode(Waypoint& w)
		: waypoint_(&w), parent_(NULL), total_distance_to_start_(0)
	{

	}

	SearchNode(Waypoint& w, SearchNode* parent, float distance_to_start)
		: waypoint_(&w), parent_(parent), total_distance_to_start_(distance_to_start)
	{

	}

	bool operator()(const SearchNode* lhs, const SearchNode* rhs) const
	{
		return lhs->total_distance_to_start_ > rhs->total_distance_to_start_;
	}

	Waypoint* waypoint_;
	SearchNode* parent_;
	float total_distance_to_start_;
};

/**
 * Class that creates and publishes waypoints for the AUV to follow.
 */
class RRT : public SceneLeafModel
{
public:
	RRT(ros::NodeHandle& ros_node, SceneManager& scene_manager, Entity& entity, SceneNode& parent, OctomapBuilder& octomap, OntologyInterface& ontology, Material& material);
	
	RRT(ros::NodeHandle& ros_node, SceneManager& scene_manager, std::vector<Entity*>& entities, SceneNode& parent, OctomapBuilder& octomap, OntologyInterface& ontology, Material& material);

	~RRT();

	//void createRRT(const glm::vec3& begin, const std::vector<glm::vec3>& end, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);
	void createRRT(const std::vector<glm::vec3>& begin, std::set<Waypoint*>& goals, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

	const std::vector<Waypoint*>& getPoints() const { return points_; }
	const std::vector<Waypoint*>& getPath() const { return path_; }
	const std::vector<glm::vec3>& getPathLocations() const { return path_buffer_; }

	void draw(const glm::mat4& view_matrix, const glm::mat4& projection_matrix, const std::vector<const SceneLeafLight*>& lights, ShaderInterface* shader = NULL) const;

	void clear();
	
	void addListener(RRTUpdateListener& listener) { update_listeners_.push_back(&listener); }

private:
	/**
	 * Check if two waypoints can be connected without colliding with any known scenery. The line should not
	 * come closer than @ref{min_width} than any known obstacle.
	 * @param w1 The first waypoint.
	 * @param w2 The second waypoint.
	 * @param min_width The minimal distance the line should be from any known obstacle.
	 * @param perfect_data Use perfect data (not the data stored in the octomap / ontology).
	 * @return True if the waypoints can be connected, false otherwise.
	 */
	bool canConnect(const glm::vec3& w1, const glm::vec3& w2, float min_width, bool perfect_data) const;

	void findPath(Waypoint& start, Waypoint& end);
	
	/**
	 * Callback function to refresh the roadmaps.
	 */
	bool refreshRoadmaps(knowledge_msgs::RoadmapRefresh::Request& req, knowledge_msgs::RoadmapRefresh::Response& res); 

	// Ros specific stuff.
	ros::NodeHandle* ros_node_;    /// The handle for ros functions.
	
	// Servies.
	ros::ServiceServer roadmap_refresh_service_;
	
	SceneManager* scene_manager_;
	std::vector<Entity*> entities_;

	std::vector<Waypoint*> points_;
	std::vector<Waypoint*> path_;
	std::vector<glm::vec3> path_buffer_;
	
	OctomapBuilder* octomap_;
	OntologyInterface* ontology_;
	
	std::vector<RRTUpdateListener*> update_listeners_;
};

#endif
