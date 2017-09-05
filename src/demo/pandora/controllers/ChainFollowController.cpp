#include "ChainFollowController.h"
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../AUV.h"
#include "../structures/Chain.h"
#include "../gui/AUVStatusIcon.h"
#include "../../../core/math/Math.h"
#include "../../../core/shaders/LineShader.h"
#include "../../../core/scene/Material.h"
#include "../../../core/scene/SceneManager.h"
#include "../../../core/scene/SceneLeafModel.h"
#include "../../../core/collision/CollisionInfo.h"
#include "../../../core/entities/HeightMap.h"

#include "../../../shapes/Line.h"


ChainFollowController::ChainFollowController(SceneManager& scene_manager, AUV& auv, OntologyInterface& ontology, ros::Publisher& action_feedback_pub, HeightMap& height_map)
	: scene_manager_(&scene_manager), auv_(&auv), chain_(NULL), time_(0), max_time_(0), path_(NULL), height_map_(&height_map), turn_left_(true), turning_time_(0)
{
	//rrt_->addListener(*this);
	
	MaterialLightProperty* ambient = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive = new MaterialLightProperty(1, 1, 0, 0.8f);
	Material* material = new Material(*ambient, *diffuse, *specular, *emmisive);
	
	MaterialLightProperty* ambient2 = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* diffuse2 = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* specular2 = new MaterialLightProperty(0, 0, 0, 0);
	MaterialLightProperty* emmisive2 = new MaterialLightProperty(1, 0, 0, 0.8f);
	Material* material2 = new Material(*ambient2, *diffuse2, *specular2, *emmisive2);
	
	MaterialLightProperty* emmisive3 = new MaterialLightProperty(0, 0, 1, 0.8f);
	Material* material3 = new Material(*ambient2, *diffuse2, *specular2, *emmisive3);
	
	line_ = new Line(false);
	colliding_line_ = new Line(false);
	grid_line_ = new Line(true);
	path_ = new SceneLeafModel(scene_manager.getRoot(), NULL, *line_, *material, LineShader::getShader(), true, true);
	SceneLeafModel* colliding_path = new SceneLeafModel(scene_manager.getRoot(), NULL, *colliding_line_, *material2, LineShader::getShader(), true, true);
	SceneLeafModel* grid_path = new SceneLeafModel(scene_manager.getRoot(), NULL, *grid_line_, *material3, LineShader::getShader(), true, true);
}

void ChainFollowController::followChain(Chain& chain, float max_time)
{
	std::cout << "Follow a chain!" << std::endl;
	time_ = 0;
	max_time_ = max_time;
	chain_ = &chain;
	observed_chain_links_.clear();
	latest_discovered_chain_loc_ = glm::vec3(0, 0, 0);
	general_heading_ = glm::vec3(auv_->getAUVModel().getCompleteTransformation() * glm::vec4(0, 0, -1, 0));
	turn_left_ = true;
	turning_time_ = 0;
	rotating_time_ = 0;
	auv_->unsetDesiredOrientation();
}


void ChainFollowController::amendFeedback(planning_msgs::ActionFeedback& feedback, PLANNER_ACTION_STATUS status)
{
	std::cout << "Chain follow feedback!" << std::endl;
	if (status == SUCCEEDED)
	{
		diagnostic_msgs::KeyValue kv;
		kv.key = "examined";
		kv.value = "true";
		feedback.information.push_back(kv);
		kv.key = "location";
		kv.value = "Chain_Location";
		feedback.information.push_back(kv);
	}
	else
	{
		std::cout << "No feedback amended!" << std::endl;
	}
}

void ChainFollowController::update(float dt)
{
	time_ += dt;
	turning_time_ += dt;
	std::vector<glm::vec3> line_segments;
	std::vector<glm::vec3> colliding_line_segments;
	std::vector<glm::vec3> grid_line_segments;
	
	bool found_chain_link = false;
	
	// Check which bits of the chain we see and follow it best we can, we perform some random ray casting.
	for (unsigned int i = 0; i < 10; ++i)
	{
		float x = 4.0f * ((float)rand() / (float)RAND_MAX - 0.5f);
		float y = 20.0f * (-0.25f - (float)rand() / (float)RAND_MAX);
		glm::vec4 random_ray(x, -5, y, 1.0f);
		
		glm::vec4 start_point(0.0f, 0.0f, -0.5f, 1.0f);
		
		random_ray = auv_->getAUVModel().getCompleteTransformation() * random_ray;
		start_point = auv_->getAUVModel().getCompleteTransformation() * start_point;
		
		// Perform a collision detection with the chain using this link.
		std::vector<CollisionInfo> collisions;
		bool collides_with_chain = false;
		if (chain_->getCollisions(*auv_, glm::vec3(start_point), glm::vec3(random_ray), collisions))
		{
			for (std::vector<CollisionInfo>::const_iterator ci = collisions.begin(); ci != collisions.end(); ++ci)
			{
				const CollisionInfo& collision_info = *ci;
				if (observed_chain_links_.count(collision_info.colliding_entity_) == 0)
				{
					// This should always be true.
					if (collision_info.collision_loc_.size() > 0)
					{
						glm::vec3 heighest_collision(0, -10, 0);
						for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
						{
							if ((*ci).y > heighest_collision.y)
							{
								heighest_collision = *ci;
							}
						}
						
						float height = height_map_->getHeight(heighest_collision.x, heighest_collision.z);
						if (height < heighest_collision.y)
						{
							// Check if this link is further away than the one we found so far.
							if (latest_discovered_chain_loc_ == glm::vec3(0, 0, 0) ||
							    glm::distance(auv_->getAUVModel().getGlobalLocation(), heighest_collision) > glm::distance(auv_->getAUVModel().getGlobalLocation(), latest_discovered_chain_loc_))
							{
								latest_discovered_chain_loc_ = heighest_collision;
							}
							
							colliding_line_segments.push_back(glm::vec3(start_point));
							colliding_line_segments.push_back(glm::vec3(random_ray));
							collides_with_chain = true;
							//std::cout << "Highest collision: (" << latest_discovered_chain_loc_.x << ", " << latest_discovered_chain_loc_.y << ", " << latest_discovered_chain_loc_.z << ") -- ";
							observed_chain_links_.insert(collision_info.colliding_entity_);
							collision_info.colliding_entity_->activate(*auv_);
							found_chain_link = true;
						}
					}
					else
					{
						std::cout << "NO COLLISION INFO!?" << std::endl;
					}
					
				}
			}
		}
		
		if (!collides_with_chain)
		{
			line_segments.push_back(glm::vec3(start_point));
			line_segments.push_back(glm::vec3(random_ray));
		}
	}
	
	// Move the AUV to the latest discovered chain.
	if (latest_discovered_chain_loc_ != glm::vec3(0, 0, 0) && (found_chain_link || glm::distance(auv_->getAUVModel().getGlobalLocation(), latest_discovered_chain_loc_ + glm::vec3(0, 2, 0)) > 0.01f))
	{
		general_heading_ = (latest_discovered_chain_loc_ + glm::vec3(0, 2, 0)) - auv_->getAUVModel().getGlobalLocation();
		auv_->setDirection((latest_discovered_chain_loc_ + glm::vec3(0, 2, 0)) - auv_->getAUVModel().getGlobalLocation());
		auv_->setVelocity(0.5f);
		
		//std::cout << "Set the genearl heading: (" << general_heading_.x << ", " << general_heading_.y << ", " << general_heading_.z << ")" << std::endl;
		
		//std::cout << "Chain: (" << latest_discovered_chain_loc_.x << ", " << latest_discovered_chain_loc_.y << ", " << latest_discovered_chain_loc_.z << ") -- ";
		//std::cout << "AUV: (" << auv_->getAUVModel().getGlobalLocation().x << ", " << auv_->getAUVModel().getGlobalLocation().y << ", " << auv_->getAUVModel().getGlobalLocation().z << ");";
		//std::cout << " Distance: (" << glm::distance(auv_->getAUVModel().getGlobalLocation(), latest_discovered_chain_loc_ + glm::vec3(0, 2, 0)) << ")" << std::endl;
		
		turning_time_ = 0;
		rotating_time_ = 0;
		search_pattern_.clear();
	}
	else if (rotating_time_ < 15.0f)
	{
		rotating_time_ += dt;
		auv_->setVelocity(0.0f);
		if (turn_left_)
		{
			auv_->setDirection(glm::vec3(-general_heading_.z, general_heading_.y, general_heading_.x));
			//std::cout << "Turn left: (" << -general_heading_.z << ", " << general_heading_.y << ", " << general_heading_.x << ")" << std::endl;
		}
		else
		{
			auv_->setDirection(glm::vec3(general_heading_.z, general_heading_.y, -general_heading_.x));
			//std::cout << "Turn right: (" << general_heading_.z << ", " << general_heading_.y << ", " << -general_heading_.x << ")" << std::endl;
		}
		
		if (turning_time_ > 3.5f)
		{
			turn_left_ = !turn_left_;
			turning_time_ = -3.5f;
		}
		latest_discovered_chain_loc_ = glm::vec3(0, 0, 0);
	}
	// If we have rotated a few times and not found a chain we do a grid-based search!
	else
	{
		if (!search_pattern_.empty() && glm::distance(auv_->getAUVModel().getGlobalLocation(), search_pattern_[0]) < 0.1f)
		{
			search_pattern_.erase(search_pattern_.begin());
		}
		
		if (search_pattern_.empty())
		{
			std::vector<glm::vec3> directions;
			directions.push_back(glm::vec3(0, 0, -1));
			directions.push_back(glm::vec3(1, 0, 0));
			directions.push_back(glm::vec3(0, 0, 1));
			directions.push_back(glm::vec3(-1, 0, 0));
			
			glm::vec3 auv_location = auv_->getGlobalLocation();
			unsigned int current_direction_id = 0;
			float distance = 1.0f;
			for (unsigned int i = 1; i < 20; ++i)
			{
				auv_location += directions[current_direction_id] * distance;
				search_pattern_.push_back(auv_location);
				
				if (i % 2 == 0)
				{
					current_direction_id = (current_direction_id + 1) % directions.size();
					distance += 1.0f;
				}
			}
		}
		
		grid_line_segments = search_pattern_;
		
		auv_->setDirection(search_pattern_[0] - auv_->getAUVModel().getGlobalLocation());
		auv_->setVelocity(0.5f);
	}
	line_->setVertexBuffer(line_segments);
	colliding_line_->setVertexBuffer(colliding_line_segments);
	grid_line_->setVertexBuffer(grid_line_segments);
}

PlannerAction::PLANNER_ACTION_STATUS ChainFollowController::getStatus()
{
	if (latest_discovered_chain_loc_ != glm::vec3(0, 0, 0))
	{
		glm::vec3 direction = (latest_discovered_chain_loc_ + glm::vec3(0, 2, 0)) - auv_->getGlobalLocation();
		float length = glm::length(direction);
		glm::vec3 end_point = latest_discovered_chain_loc_ + glm::vec3(0, 2, 0);
		
		if (length > 5.0f)
		{
			end_point = auv_->getGlobalLocation() + glm::normalize(direction) * 5.0f;
		}
		
		// Check if the AUV is about to collide, if this is the case then we do an emergency stop.
		if (scene_manager_->getRoot().doesCollide(auv_, auv_->getGlobalLocation(), end_point, 1.0f))
		{
			auv_->setVelocity(0.0f);
			std::cout << "ACTION FAILED!!!!!!!!!!" << std::endl;
			
			AUVStatusIcon& auv_status_icon = AUVStatusIcon::getInstance();
			const std::vector<glm::vec2>& uv_mapping = auv_status_icon.getCollisionIcon();
			/*
			uv_mapping.push_back(glm::vec2(0.5f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.75f, 1.0f));
			uv_mapping.push_back(glm::vec2(0.5f, 0.75f));
			uv_mapping.push_back(glm::vec2(0.75f, 0.75f));
			*/
			auv_->setBillBoardUVs(uv_mapping);
			return FAILED;
		}
	}
	
	
	if (time_ < max_time_)
	{
		return EXECUTING;
	}
	auv_->setVelocity(0);
	std::cout << time_ << " << " << max_time_ << "." << std::endl;
	return SUCCEEDED;
}
