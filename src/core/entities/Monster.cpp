#include "Monster.h"
#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"
#include "GL/glfw.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "../entities/camera/Camera.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneManager.h"
#include "../../shapes/terrain.h"
#include "../../shapes/Line.h"
#include "../collision/BoxCollision.h"
#include "../collision/CollisionInfo.h"
#include "../scene/portal/Region.h"
#include "../scene/SceneLeafModel.h"
#include "../shaders/LineShader.h"

Monster::Monster(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager)
	: Entity(scene_manager, parent, transformation, OBSTACLE, "monster")
{
	//camera_ = new Camera();
	y_velocity_ = 0.0f;
	height_ = 2.1f;
	//BoxCollision* bc = new BoxCollision(*this, 1.0f, height_, 1.0f);
	//BoxCollision* bc = new BoxCollision(*this);
	//addCollision(*bc, material, shader);
}

void Monster::init(Material& material, ShaderInterface& shader)
{
	//return;
	// TODO: Automate this process.
	//BoxCollision* bc = new BoxCollision(*this);
	//addCollision(*bc, material, shader);

	/*
	std::stringstream ss;
	ss << "Bounded Box: ";
	for (unsigned int i = 0; i < 8; ++i)
	{
		ss << "Points: (" << bc->getPoints()[i].x << ", " << bc->getPoints()[i].y << ", " << bc->getPoints()[i].z << ")" << std::endl;
	}
#ifdef _WIN32
	MessageBox(NULL, ss.str().c_str(), "An error occurred", MB_ICONERROR | MB_OK);
#endif
	*/
	float size = 0.4f;
	float width = 0.6f;
	float height = 2.2f;
	float offset = 0.01f;
	glm::vec3 bottom_left_away(-width, offset, -size);
	glm::vec3 bottom_right_away(width, offset, -size);
	glm::vec3 top_left_away(-width, height, -size);
	glm::vec3 top_right_away(width, height, -size);
	glm::vec3 bottom_left_close(-width, offset, size); 
	glm::vec3 bottom_right_close(width, offset, size);
	glm::vec3 top_left_close(-width, height, size);
	glm::vec3 top_right_close(width, height, size);
	BoxCollision* bc = new BoxCollision(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	addCollision(*bc, material, shader);
	BoundedBox* bc2 = new BoundedBox(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	bounded_collision_box_ = bc2;

	frustum_checker_ = new BoundedBox(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	/*
	delete frustum_checker_;
	std::vector<const SceneNode*> excluded_nodes;
	frustum_checker_ = new BoundedBox(*this, true, excluded_nodes);
	*/
	
	collision_line_ = new Line();
	std::vector<glm::vec3> line_vec;
	line_vec.push_back(glm::vec3(0.0f, height_ / 2.0f, 0.0f));
	line_vec.push_back(glm::vec3(2.0f, height_ / 2.0f, 2.0f));
	line_vec.push_back(glm::vec3(-2.0f, height_ / 2.0f, 2.0f));
	line_vec.push_back(glm::vec3(0.0f, height_ / 2.0f, 0.0f));
	collision_line_->setVertexBuffer(line_vec);
	SceneLeafModel* lines = new SceneLeafModel(scene_manager_->getRoot(), NULL, *collision_line_, *SceneNode::bright_material_, LineShader::getShader(), true, true);
}

void Monster::prepare(float dt)
{
	/**
	 * Make the monster move towards a waypoint (if any).
	 */
	if (waypoints_.empty())
	{
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "No waypoints left." << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
		Entity::prepare(dt);
		return;
	}

#ifdef _WIN32
		{
		std::stringstream ss;
		ss << waypoints_.size() << " waypoints left." << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif

	/**
	 * Check if the monster needs to turn or whether it can simply move :).
	 */
	const glm::vec3& next_waypoint = waypoints_[0];
	glm::vec3 to_waypoint = next_waypoint - getGlobalLocation();
	glm::vec2 to_angle(to_waypoint.x, to_waypoint.z);

	glm::vec3 facing_direction = glm::rotate(getGlobalRotation(), glm::vec3(0.0f, 0.0f, 1.0f));
	glm::vec2 facing_direction_angle(facing_direction.x, facing_direction.z);
	float angle = glm::orientedAngle(glm::normalize(to_angle), glm::normalize(facing_direction_angle));
	
	float angle_to_rotate;
	if (angle > 0.0f)
	{
 		angle_to_rotate = 500.0f * dt > angle ? angle : 500.0f * dt ;
	}
	else
	{
		angle_to_rotate = -500.0f * dt < angle ? angle : -500.0f * dt;
	}
	
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "To waypoint: (" << to_waypoint.x << ", " << to_waypoint.y << ", " << to_waypoint.z << "). Facing: (" << facing_direction.x << ", " << facing_direction.y << ", " << facing_direction.z << "). Rotate: [" << 10.0f * dt << "]" << angle  << "(" << angle_to_rotate << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	}
	{
	glm::vec2 t2d(next_waypoint.x, next_waypoint.z);
	glm::vec2 m2d(getGlobalLocation().x, getGlobalLocation().z);
	float distance = glm::distance(t2d, m2d);
	std::stringstream ss;
	ss << "Waypoint: (" << next_waypoint.x << ", " << next_waypoint.y << ", " << next_waypoint.z << "). Monster: (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << "). Distance: " << distance << "." << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#endif
	
	local_transformation_ = glm::rotate(local_transformation_, angle_to_rotate, glm::vec3(0.0f, 1.0f, 0.0f));
	if (std::abs(angle) > 5.0f)
	{
		//Entity::prepare(dt);
		//return;
	}

	// If it is going to move, check if it will not move too close to a wall. If it does then we need to adjust the orientation of the monster.
	glm::vec4 centre(0.0f, height_ / 2.0f, 0.0f, 1.0f);
	glm::vec4 right_ray(2.0f, height_ / 2.0f, 2.0f, 1.0f);
	glm::vec4 left_ray(-2.0f, height_ / 2.0f, 2.0f, 1.0f);

	centre = getCompleteTransformation() * centre;
	right_ray = getCompleteTransformation() * right_ray;
	left_ray = getCompleteTransformation() * left_ray;
	
	std::vector<glm::vec3> line_vec;
	line_vec.push_back(static_cast<glm::vec3>(centre));
	line_vec.push_back(static_cast<glm::vec3>(left_ray));
	line_vec.push_back(static_cast<glm::vec3>(right_ray));
	line_vec.push_back(static_cast<glm::vec3>(centre));
	collision_line_->setVertexBuffer(line_vec);

	// Check the collisions of these rays.
	std::vector<CollisionInfo> l_collisions;
	std::vector<CollisionInfo> r_collisions;
	if (current_region_ != NULL)
	{
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(left_ray), l_collisions);
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(right_ray), r_collisions);
	}
	else
	{
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(left_ray), l_collisions);
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(right_ray), r_collisions);
	}

	float ld = std::numeric_limits<float>::max();
	float rd = std::numeric_limits<float>::max();

	for (std::vector<CollisionInfo>::const_iterator ci = l_collisions.begin(); ci != l_collisions.end(); ++ci)
	{
		const CollisionInfo& info = *ci;
		for (std::vector<glm::vec3>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(*ci, getGlobalLocation());
			if (d < ld) ld = d;
		}
	}

	for (std::vector<CollisionInfo>::const_iterator ci = r_collisions.begin(); ci != r_collisions.end(); ++ci)
	{
		const CollisionInfo& info = *ci;
		for (std::vector<glm::vec3>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(*ci, getGlobalLocation());
			if (d < rd) rd = d;
		}
	}

	if (ld != std::numeric_limits<float>::max() || rd != std::numeric_limits<float>::max())
	{
#ifdef _WIN32
		std::stringstream ss;
		ss << "Done rotating! :D (" << to_waypoint.x << ", " << to_waypoint.y << ", " << to_waypoint.z << "). Facing: (" << facing_direction.x << ", " << facing_direction.y << ", " << facing_direction.z << "). Rotate: [" << 10.0f * dt << "]" << angle  << "(" << angle_to_rotate << ")" << std::endl;
		ss << "L: (" << left_ray.x << ", " << left_ray.y << ", " << left_ray.z << ") = " << ld << "; (" << right_ray.x << ", " << right_ray.y << ", " << right_ray.z << ") = " << rd << std::endl;
		OutputDebugString(ss.str().c_str());
#endif
	}
	
	/*
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Done rotating! :D (" << to_waypoint.x << ", " << to_waypoint.y << ", " << to_waypoint.z << "). Facing: (" << facing_direction.x << ", " << facing_direction.y << ", " << facing_direction.z << "). Rotate: [" << 10.0f * dt << "]" << angle  << "(" << angle_to_rotate << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#endif
	*/
	glm::vec3 old_local_position(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]);

	// Update the heading of the monster so it does not collide with any wall.
	glm::vec3 heading(0.0f, 0.0f, 5.0f * dt);
	if (ld != std::numeric_limits<float>::max())
	{
		//glm::vec3 l(dt * (2.0f) / (1.0f + ld), 0.0f, 0.0f);
		glm::vec3 l(dt * (1.0f) / std::max(0.2f, ld * 0.1f), 0.0f, 0.0f);
		heading += l;
		//std::cout << "L:" << ld;
	}
	if (rd != std::numeric_limits<float>::max())
	{
		//glm::vec3 r(-dt * (2.0f) / (1.0f + rd), 0.0f, 0.0f);
		glm::vec3 r(-dt * (1.0f) / std::max(0.2f, rd * 0.1f), 0.0f, 0.0f);
		heading += r;
		//std::cout << "R:" << rd;
	}
/*	
	if (ld != std::numeric_limits<float>::max() || rd != std::numeric_limits<float>::max())
	{
		std::cout << std::endl;
		std::flush(std::cout);
	}
*/	
	y_velocity_ += 9.81f * dt;
	heading.y -= y_velocity_ * dt;

	local_transformation_ = glm::translate(local_transformation_, heading);
	//local_transformation_ = glm::translate(local_transformation_, glm::vec3(0.0f, 0.0f, heading.z));
	updateTransformations();
	glm::vec3 local_position(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]);

	// Hard coded (for now).
	if (local_position.y < 0.01f)
	{
		local_position.y = 0.01f;
		y_velocity_ = 0;
	}
	
	//std::cout << "Heading: (" << heading.x << ", " << heading.y << ", " << heading.z << ")" << std::endl;

	/**
	 * NEW:
	 */
	bool collided_on_top = false;

	std::vector<CollisionInfo> collision_infos;
	glm::fquat parent_rotation = parent_->getGlobalRotation();

	nr_collisions_checked_ = 0;
/*
	// Localise the monster.
	if (current_region_ == NULL)
	{
		current_region_ = Region::findRegionGlobal(getGlobalLocation());
	}
*/
	bool found_collision = false;

	if (current_region_ != NULL)
	{
		found_collision = current_region_->getCollisions(*this, collision_infos);
	}
	else
	{
		found_collision = scene_manager_->getRoot().getCollisions(*this, collision_infos);
	}
	
	if (found_collision)
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "Collision detected. Entities checked: " << nr_entities_checked << "|";
#endif

		CollisionInfo highest_found_collision;
		
		{
			float highest_collision = -std::numeric_limits<float>::max();
			glm::vec3 collision_point;

			for (std::vector<CollisionInfo>::const_iterator ci = collision_infos.begin(); ci != collision_infos.end(); ++ci)
			{
				const CollisionInfo& collision_info = *ci;
				for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
				{
					Entity* colliding_entity = collision_info.colliding_entity_;
					if (colliding_entity == this)
					{
						colliding_entity = collision_info.other_colliding_entity_;
					}

					// Transform the collision location so it corresponds to the local location.
					glm::vec3 local_collision = *ci;

					local_collision = glm::rotate(glm::inverse(glm::quat_cast(colliding_entity->getCompleteTransformation())), local_collision);
				
					if (local_collision.y > highest_collision)
					{
						highest_collision = local_collision.y;
						collision_point = *ci;
						highest_found_collision = collision_info;
					}
				}
#ifdef HORROR_GAME_ENABLE_DEBUG
				ss_ << (*ci).colliding_entity_->getName() << " collided with: " << (*ci).other_colliding_entity_->getName() << " High point: " << highest_collision << ". Point: (" << collision_point.x << ", " << collision_point.y << ", " << collision_point.z << ")|";
#endif
			}
		}

		const CollisionInfo& collision_info = highest_found_collision;//collision_infos[collision_infos.size() - 1];
		Entity* colliding_entity = collision_info.colliding_entity_;
		if (colliding_entity == this)
		{
			colliding_entity = collision_info.other_colliding_entity_;
		}

		float highest_collision = -std::numeric_limits<float>::max();
		glm::vec3 collision_point;

		for (std::vector<glm::vec3>::const_iterator ci = collision_info.collision_loc_.begin(); ci != collision_info.collision_loc_.end(); ++ci)
		{
			// Transform the collision location so it corresponds to the local location.
			glm::vec3 local_collision = *ci - colliding_entity->getGlobalLocation();
			local_collision = glm::rotate(glm::inverse(glm::quat_cast(colliding_entity->getCompleteTransformation())), local_collision);
				
			if (local_collision.y > highest_collision)
			{
				highest_collision = local_collision.y;
				collision_point = *ci;
			}
		}

		//local_position.y = highest_collision + height_ / 2.0f - colliding_entity->getCompleteTransformation()[3][1];
		//local_position.y = highest_collision + height_ / 2.0f;

		float height_difference = std::abs(complete_transformation_[3][1] - (collision_point.y));

		// Check if we collided because we 'fall' through a surface we stood on.
		float prev_y = complete_transformation_[3][1];
		
		//complete_transformation_[3][1] = highest_collision + height_ / 2.0f;
		complete_transformation_[3][1] = collision_point.y;// + height_ / 2.0f + 0.1f;
		//ss_ << "Updated height: " << complete_transformation_[3][1] << "|";
		
		CollisionInfo higher_collision_info;

		Region* tmp_region = NULL;
		if (current_region_ != NULL)
		{
			tmp_region = current_region_->findRegion(getGlobalLocation());
		}

		//if (height_difference > 0.35f || 
		if (height_difference > 2.75f || 
		   (tmp_region == NULL && scene_manager_->getRoot().doesCollide(*this, higher_collision_info)) ||
		   (tmp_region != NULL && tmp_region->doesCollide(*this, higher_collision_info)))
		{
			local_position = old_local_position;
			complete_transformation_[3][1] = prev_y;
			transition(NULL, local_position);
		}
		else
		{
			// If the colliding entity is not the one we are colliding with at the moment then we need to change it!
			transition(colliding_entity, local_position);
			local_position.y = highest_collision;// + height_ / 2.0f;
			collided_on_top = true;
		}
		y_velocity_ = 0.0f;
	}
	else
	{
		transition(NULL, local_position);
	}

	//local_transformation_ = glm::translate(glm::mat4(1.0f), local_position);
	local_transformation_[3][0] = local_position.x;
	local_transformation_[3][1] = local_position.y;
	local_transformation_[3][2] = local_position.z;
	Entity::prepare(dt);
/*
	if (region_ != NULL)
	{
		region_ = region_->findRegion(getGlobalLocation());
	}
	else
	{
		region_ = Region::findRegionGlobal(getGlobalLocation());
	}
*/
	// Are we at the waypoint yet?
	//updateTransformations();
	glm::vec2 t2d(next_waypoint.x, next_waypoint.z);
	glm::vec2 m2d(getGlobalLocation().x, getGlobalLocation().z);
	float distance = glm::distance(t2d, m2d);
	/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "DISTANCE: " << distance << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
	*/
	if (distance < 0.5f)
	{
	/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Reached the waypoint!" << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
	*/
		waypoints_.erase(waypoints_.begin());
	}

    return;
}

void Monster::transition(SceneNode* scene_node, glm::vec3& local_location)
{
	return;
	if (scene_node == NULL)
	{
		return;
		//scene_node = default_entity_;
	}

	// If nothing changes then we are done!
	if (parent_ == scene_node)
	{
		return;
	}

	// Update the local location relative to the current scene node.
	glm::mat4 transition_matrix = scene_node->getCompleteTransformation() - parent_->getCompleteTransformation();
	glm::vec3 translation = glm::vec3(transition_matrix[3][0], transition_matrix[3][1], transition_matrix[3][2]);
	
	// Before placing the player local to the new scene node, rotate the location player on the current scene node
	// so we get the correct global rotation.
	glm::fquat pre_rotate = glm::quat_cast(parent_->getCompleteTransformation());
	local_location = glm::rotate(pre_rotate, local_location);
	// Move the player to the new scene node.
	local_location -= translation;

	// Counter rotate the local position to find the actual position on the rotated surface.
	glm::fquat post_rotate = glm::quat_cast(scene_node->getCompleteTransformation());
	post_rotate = glm::conjugate(post_rotate);
	local_location = glm::rotate(post_rotate, local_location);

	local_transformation_ = glm::translate(local_transformation_, local_location);
	//setTransformation(local_transformation_);

	// Store the scene node we are on at the moment.
	parent_ = scene_node;
}

void Monster::onCollision(const CollisionInfo& collision_info)
{
	
}
