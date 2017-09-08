#ifndef DEMO_PANDORA_STRUCTURES_PIPE_NETWORK_H
#define DEMO_PANDORA_STRUCTURES_PIPE_NETWORK_H

#include <memory>
#include <vector>

#include "dpengine/entities/Entity.h"

class MissionSite;

namespace DreadedPE
{
	class SceneManager;
	class Material;
	class Shape;
};

/**
 * The data regarding the grid.
 */
struct PipeNetworkGrid
{
	PipeNetworkGrid(std::vector<int>& grid, int min_x, int min_y, int max_x, int max_y, float grid_size)
		: grid_(grid), min_x_(min_x), min_y_(min_y), max_x_(max_x), max_y_(max_y), grid_size_(grid_size)
	{
		grid_width_ = max_x - min_x;
		grid_height_ = max_y - min_y;
	}
	
	int getGridId(float x, float y) const
	{
		int grid_x = (x - min_x_) / grid_size_;
		int grid_y = (y - min_y_) / grid_size_;
		
		if (grid_x < 0 || grid_x >= grid_width_) return -1;
		if (grid_y < 0 || grid_y >= grid_height_) return -1;
		
		return grid_y * grid_width_ + grid_x;
	}
	
	glm::vec2 getCoordinates(int grid_id) const
	{
		return glm::vec2((grid_id % grid_width_) * grid_size_ + min_x_, (grid_id / grid_width_) * grid_size_ + min_y_);
	}
	
	std::vector<int> grid_;
	int min_x_;
	int min_y_;
	int max_x_;
	int max_y_;
	int grid_width_;
	int grid_height_;
	float grid_size_;
};

struct PipeNetworkNode: public std::binary_function<PipeNetworkNode*, PipeNetworkNode*, bool>
{
	enum PIPE_NETWORK_DIRECTION { N = 0, NE, E, SE, S, SW, W, NW };
	
	PipeNetworkNode()
	{
		
	}
	
	PipeNetworkNode(const PipeNetworkGrid& grid, PipeNetworkNode* parent, int grid_id, int grid_goal_id, const std::vector<PipeNetworkNode*>& path, PipeNetworkNode::PIPE_NETWORK_DIRECTION direction)
		: grid_(&grid), grid_id_(grid_id), grid_goal_id_(grid_goal_id), path_(path), direction_(direction)
	{
		path_.push_back(this);
		
		// Determine whether we went 'straight' or whether we took a turn.
		cost_ = pipe_direction_cost[direction];
		
		// Check whether this section of pipe has already been constructed. If not we incurr a penalty.
		if (grid.grid_[grid_id_] == 0)
		{
			cost_ += 5;
		}
		
		if (parent != NULL)
		{
			cost_ += parent->cost_;
			if (parent->direction_ != direction)
			{
				cost_ += 1;
			}
		}
		
		// Estimate the cost to the goal as a straight line.
		glm::vec2 start = grid.getCoordinates(grid_id);
		glm::vec2 goal = grid.getCoordinates(grid_goal_id);
		
		estimated_cost_to_goal_ = glm::distance(start, goal) * sqrt(2.0f);
	}
	
	/**
	 * Generate children for this node.
	 */
	void getChildren(std::vector<PipeNetworkNode*>& children)
	{
		//int i = direction_ - 2;
		int i = direction_ - 1;
		if (i < 0) i = 8 + i;
		//for (int j = 0; j < 5; ++j, ++i)
		for (int j = 0; j < 3; ++j, ++i)
		{
			const glm::vec2& direction = pipe_direction[i % 8];
			int grid_x = grid_id_ % grid_->grid_width_;
			int grid_y = grid_id_ / grid_->grid_width_;
			
			grid_x += direction.x;
			grid_y += direction.y;
			
			if (grid_x < 0 || grid_x >= grid_->grid_width_ ||
			    grid_y < 0 || grid_y >= grid_->grid_height_)
			{
				continue;
			}
			
			if (grid_->grid_[grid_x + grid_y * grid_->grid_width_] == -1)
			{
				continue;
			}
			
			children.push_back(new PipeNetworkNode(*grid_, this, grid_x + grid_y * grid_->grid_width_, grid_goal_id_, path_, static_cast<PIPE_NETWORK_DIRECTION>(i % 8)));
		}
	}
	
	void getAllChildren(std::vector<PipeNetworkNode*>& children)
	{
		for (int j = 0; j < 8; ++j)
		{
			const glm::vec2& direction = pipe_direction[j];
			int grid_x = grid_id_ % grid_->grid_width_;
			int grid_y = grid_id_ / grid_->grid_width_;
			
			grid_x += direction.x;
			grid_y += direction.y;
			
			if (grid_x < 0 || grid_x >= grid_->grid_width_ ||
			    grid_y < 0 || grid_y >= grid_->grid_height_)
			{
				continue;
			}
			
			if (grid_->grid_[grid_x + grid_y * grid_->grid_width_] == -1)
			{
				continue;
			}
			
			children.push_back(new PipeNetworkNode(*grid_, this, grid_x + grid_y * grid_->grid_width_, grid_goal_id_, path_, static_cast<PIPE_NETWORK_DIRECTION>(j)));
		}
	}
	
	bool operator()(const PipeNetworkNode* lhs, const PipeNetworkNode* rhs) const
	{
		return lhs->cost_ + lhs->estimated_cost_to_goal_ > rhs->cost_ + rhs->estimated_cost_to_goal_;
	}
	
	const PipeNetworkGrid* grid_;
	float cost_;
	float estimated_cost_to_goal_;
	int grid_id_;
	int grid_goal_id_;
	std::vector<PipeNetworkNode*> path_;
	PipeNetworkNode::PIPE_NETWORK_DIRECTION direction_;   // The direction the pipe came from.
	
	static float pipe_direction_cost[];
	static glm::vec2 pipe_direction[];
};

std::ostream& operator<<(std::ostream& os, const PipeNetworkNode& node);

/**
 * Create a pipe network given a set of mission sites.
 */
class PipeNetwork : public DreadedPE::Entity
{
public:
	PipeNetwork(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const std::vector<MissionSite*>& mission_sites, float grid_size);
	
	void findPath(const glm::vec3& from, const glm::vec3& to, PipeNetworkNode::PIPE_NETWORK_DIRECTION direction);
private:
	std::shared_ptr<DreadedPE::Shape> pipe_shape_;
	PipeNetworkGrid* grid_;
	std::shared_ptr<DreadedPE::Material> pipe_material_;
};

#endif
