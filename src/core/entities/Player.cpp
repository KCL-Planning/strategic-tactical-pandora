#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"
#include "GL/glfw.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "Player.h"

#include "../entities/camera/Camera.h"
#include "../scene/SceneNode.h"
#include "../scene/SceneManager.h"
#include "../../shapes/terrain.h"
#include "../collision/BoxCollision.h"
#include "../collision/CollisionInfo.h"
#include "../scene/portal/Region.h"

#define HORROR_GAME_ENABLE_DEBUG

Player::Player(SceneNode* parent, const glm::mat4& transformation, float height, SceneNode& scene_node, const Terrain& terrain, SceneManager& scene_manager)
	: Entity(scene_manager, parent, transformation, PLAYER, "player"), height_(height), terrain_(&terrain), pitch_(0.0f), yaw_(0.0f), roll_(0.0f), parent_pitch_(0.0f), parent_yaw_(0.0f), parent_roll_(0.0f), default_entity_(&scene_node), score_(0), deaths_(0)
{
//	camera_ = new Camera();
	y_velocity_ = 0.0f;
	BoxCollision* bc = new BoxCollision(*this, 0.4f, height_, 0.4f);
	addCollision(*bc);
}

Player::~Player()
{
	//delete camera_;
}

void Player::prepare(float dt)
{
#ifdef HORROR_GAME_ENABLE_DEBUG
	ss_.str(std::string());
#endif

#ifdef _WIN32
	float begin_prepare = float(GetTickCount()) / 1000.0f;
#endif
#ifdef __linux__
	float begin_prepare = float(clock()) / 1000.0f;
#endif
	//float begin_prepare = float(clock()) / 1000.0f;
	glm::vec3 old_local_position(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]);
	glm::vec3 local_position = old_local_position;

	int width, height;
	glfwGetWindowSize(&width, &height);

	// Handle input.
	int mouseX, mouseY;
	glfwGetMousePos(&mouseX, &mouseY);

	glm::vec2 current_direction(0.0f, 0.0f);
	if (glfwGetKey('W') == GLFW_PRESS)
	{
		current_direction.x = 1.0f;
	}
	else if (glfwGetKey('S') == GLFW_PRESS)
	{
		current_direction.x = -1.0f;
	}
	if (glfwGetKey('A') == GLFW_PRESS)
	{
		current_direction.y = 1.0f;
	}
	else if (glfwGetKey('D') == GLFW_PRESS)
	{
		current_direction.y = -1.0f;
	}

	// Jump!
	if (glfwGetKey(GLFW_KEY_SPACE) == GLFW_PRESS)
	{
		if (y_velocity_ == 0.0f)
		{
			y_velocity_ = -5.0f;
		}
	}

	pitch_ += ((height / 2.0f) - mouseY) * 0.5f;
	yaw_ += ((width / 2.0f) - mouseX) * 0.27f;
	Camera::checkLimits(pitch_, yaw_, roll_);
	
	float updated_pitch = pitch_ - parent_pitch_;
	float updated_yaw = yaw_ - parent_yaw_;
	float updated_roll = roll_ - parent_roll_;

	Camera::checkLimits(updated_pitch, updated_yaw, updated_roll);

	float speed = 3.0f;

	if (glfwGetKey(GLFW_KEY_LSHIFT) == GLFW_PRESS || glfwGetKey(GLFW_KEY_RSHIFT) == GLFW_PRESS)
	{
		speed = 8.0f;
	}

	if (current_direction.x != 0.0f || current_direction.y != 0.0f)
	{
		current_direction = glm::rotate(current_direction, updated_yaw);
		current_direction = glm::normalize(current_direction);
		current_direction *= dt * speed;
	}

	// We only move straight forward, so we ignore the pitch.
	float angle = atan2(current_direction.y, current_direction.x);
	local_position.x -= sin(angle) * glm::length(current_direction);
	local_position.z -= cos(angle) * glm::length(current_direction);

	// Lets subject the player to some gravity if we are not on a surface.
	y_velocity_ += 9.81f * dt;
	local_position.y -= y_velocity_ * dt;

	local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(local_position.x, local_position.y, local_position.z));
	local_transformation_ = glm::rotate(local_transformation_, updated_roll, glm::vec3(0, 0, 1));
	local_transformation_ = glm::rotate(local_transformation_, updated_yaw, glm::vec3(0, 1, 0));
	local_transformation_ = glm::rotate(local_transformation_, updated_pitch, glm::vec3(1, 0, 0));
	updateTransformations();


	glm::vec3 global_location = getGlobalLocation();
	glm::vec3 parent_location = parent_->getGlobalLocation();
/*
	glm::fquat global_rotation = getGlobalRotation();
	float global_pitch = glm::pitch(global_rotation);
	float global_yaw = glm::yaw(global_rotation );
	float global_roll = glm::roll(global_rotation);
*/
	
	if (parent_ == default_entity_)
	{
		float terrain_height = terrain_->getHeight(local_position.x + terrain_->getWidth() / 2, local_position.z + terrain_->getWidth() / 2) + parent_location.y;
#ifdef HORROR_GAME_ENABLE_DEBUG
		ss_ << "Collide with the terrain!";
#endif
		if (terrain_height + height_ / 2.0f > global_location.y)
		{
			//global_location.y = terrain_height + height_ / 2.0f;
			y_velocity_ = 0.0f;
			local_position.y = terrain_height + height_ / 2.0f - parent_location.y;
			//ss_ << local_position.y << "|";
		}
	}
	

	// Check if we have bumped into anything. If so then we do not allow the player to move. We always allow the player to rotate.
	// Mimick the move and check if the player will collide with any entity. We ignore any rotations.
	local_transformation_ = glm::translate(glm::mat4(1.0f), local_position);
	updateTransformations();

	bool collided_on_top = false;

	std::vector<CollisionInfo> collision_infos;
	glm::fquat parent_rotation = parent_->getGlobalRotation();

	nr_collisions_checked_ = 0;
/*
	// Localise the player.
	if (current_region_ == NULL)
	{
		current_region_ = Region::findRegionGlobal(getGlobalLocation());
	}
*/
#ifdef _WIN32
	float begin_collision = float(GetTickCount()) / 1000.0f;
#endif
#ifdef __linux__
	float begin_collision = float(clock()) / 1000.0f;
#endif
	//float begin_collision = float(clock()) / 1000.0f;
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
		//ss_ << "Collision detected. Entities checked: " << nr_entities_checked << "|";
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

		float height_difference = std::abs(complete_transformation_[3][1] - (collision_point.y + height_ / 2.0f));

		// Check if we collided because we 'fall' through a surface we stood on.
		float prev_y = complete_transformation_[3][1];
		
		//complete_transformation_[3][1] = highest_collision + height_ / 2.0f;
		complete_transformation_[3][1] = collision_point.y + height_ / 2.0f + 0.1f;
		//ss_ << "Updated height: " << complete_transformation_[3][1] << "|";
		
		CollisionInfo higher_collision_info;

		Region* tmp_region = NULL;
		if (current_region_ != NULL)
		{
			tmp_region = current_region_->findRegion(getGlobalLocation());
		}

		if (height_difference > 0.35f || 
		//if (height_difference > 1.75f || 
		   (tmp_region == NULL && scene_manager_->getRoot().doesCollide(*this, higher_collision_info)) ||
		   (tmp_region != NULL && tmp_region->doesCollide(*this, higher_collision_info)))
		{
#ifdef HORROR_GAME_ENABLE_DEBUG
			ss_ << "Collided with something above! Height difference: " << height_difference << "|";
			ss_ << (higher_collision_info.colliding_entity_ != NULL ? higher_collision_info.colliding_entity_->getName() : "UNKNOWN") << " collides with " << (higher_collision_info.other_colliding_entity_ != NULL ? higher_collision_info.other_colliding_entity_->getName() : "UNKNOWN") << "|";
#endif
			local_position = old_local_position;
//			complete_transformation_[3][1] -= height_ / 2.0f;
			complete_transformation_[3][1] = prev_y;
			transition(NULL, local_position);
#ifdef HORROR_GAME_ENABLE_DEBUG
			ss_ << "RESET! Height:" << height_difference << ". " << prev_y << " v.s. " << (collision_point.y + height_ / 2.0f) << "|";
			for (std::vector<glm::vec3>::const_iterator ci = higher_collision_info.collision_loc_.begin(); ci != higher_collision_info.collision_loc_.end(); ++ci)
			{
				ss_ << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << "|";
			}
#endif
		}
		else
		{
#ifdef HORROR_GAME_ENABLE_DEBUG
			ss_ << "Height difference: " << height_difference << " -> " << complete_transformation_[3][1] << "|";
			ss_ << "Local: " << local_position.x << ", " << local_position.y << ", " << local_position.z << "|";
			ss_ << "Parent: " << parent_->getGlobalLocation().x << ", " << parent_->getGlobalLocation().y << ", " << parent_->getGlobalLocation().z << "|";
			ss_ << "Transition to: " << colliding_entity->getGlobalLocation().x << ", " << colliding_entity->getGlobalLocation().y << ", " << colliding_entity->getGlobalLocation().z << "|";
#endif
			// If the colliding entity is not the one we are colliding with at the moment then we need to change it!
			transition(colliding_entity, local_position);
			local_position.y = highest_collision + height_ / 2.0f;
#ifdef HORROR_GAME_ENABLE_DEBUG
			ss_ << "POST: Local: " << local_position.x << ", " << local_position.y << ", " << local_position.z << "|";
			ss_ << "POST: Parent: " << parent_->getGlobalLocation().x << ", " << parent_->getGlobalLocation().y << ", " << parent_->getGlobalLocation().z << "|";
#endif
			collided_on_top = true;
		}
		y_velocity_ = 0.0f;
	}
	else
	{
#ifdef HORROR_GAME_ENABLE_DEBUG
		//ss_ << "No collisions! Entities checked: " << nr_entities_checked << "|";
#endif
		transition(NULL, local_position);
	}
#ifdef _WIN32
	collision_detection_time_ += float(GetTickCount()) / 1000.0f - begin_collision;
#endif
#ifdef __linux__
	collision_detection_time_ += float(clock()) / 1000.0f - begin_collision;
#endif

	//collision_detection_time_ += float(clock()) / 1000.0f - begin_collision;

	// Integrate the local transitions and rotations with the global ones.
	//camera_->setRotation(global_pitch, global_yaw, global_roll);

	// Reset mouse pointer.
	//glfwSetMousePos(width / 2, height / 2);

	// Update the local transformation.
	updated_pitch = pitch_ - parent_pitch_;
	updated_yaw = yaw_ - parent_yaw_;
	updated_roll = roll_ - parent_roll_;

	local_transformation_ = glm::translate(glm::mat4(1.0f), local_position);
	local_transformation_ = glm::rotate(local_transformation_, updated_roll, glm::vec3(0.0f, 0.0f, 1.0f));
	local_transformation_ = glm::rotate(local_transformation_, updated_yaw, glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, updated_pitch, glm::vec3(1.0f, 0.0f, 0.0f));
	updateTransformations();
	SceneNode::prepare(dt);
	//camera_->setPosition(complete_transformation_[3][0], complete_transformation_[3][1] + (height_ / 3.0f), complete_transformation_[3][2]);
/*
	if (current_region_ != NULL)
	{
		current_region_ = current_region_->findRegion(getGlobalLocation());
	}
*/
	if (glfwGetKey('E') == GLFW_PRESS)
	{
		glm::vec3 direction(0.0f, 0.0f, -2.0f);
		
		// Check if the player touches anything.
		// TODO: Update to the values in the global reference frame.
		glm::vec3 begin_point(0.0f, height_ / 2.0f, 0.0f);
		glm::vec3 end_point(begin_point.x + direction.x, begin_point.y + direction.y, begin_point.z + direction.z);

		// Transform these points to the global frame of reference.
		begin_point = glm::vec3(getCompleteTransformation() * glm::vec4(begin_point, 1.0f));
		end_point = glm::vec3(getCompleteTransformation() * glm::vec4(end_point, 1.0f));

		std::vector<CollisionInfo> info;
		scene_manager_->getRoot().getCollisions(*this, begin_point, end_point, info);

		//ss_.str(std::string());
		//ss_ << "(" << getGlobalLocation().x + begin_point.x << ", " << getGlobalLocation().y + begin_point.y << ", " << getGlobalLocation().z + begin_point.z << ") -> " << "(" << getGlobalLocation().x + end_point.x << ", " << getGlobalLocation().y + end_point.y << ", " << getGlobalLocation().z + end_point.z << ") " << info.size() << "|";

		for (std::vector<CollisionInfo>::const_iterator ci = info.begin(); ci != info.end(); ++ci)
		{
			const CollisionInfo& collision_info = *ci;
			collision_info.colliding_entity_->activate(*this);
			//ss_ << "Activate: " << collision_info.colliding_entity_->getName() << " - " << collision_info.other_colliding_entity_->getName() << "|";
		}
	}
#ifdef _WIN32
	prepare_time_ += float(GetTickCount()) / 1000.0f - begin_prepare;
#endif
#ifdef __linux__
	prepare_time_ += float(clock()) / 1000.0f - begin_prepare;
#endif
	//prepare_time_ += float(clock()) / 1000.0f - begin_prepare;
}

void Player::transition(SceneNode* scene_node, glm::vec3& local_location)
{
	if (scene_node == NULL)
	{
		scene_node = default_entity_;
	}

	// If nothing changes then we are done!
	if (parent_ == scene_node)
	{
		return;
	}
	
	//local_transformation_ = (local_transformation_ * glm::inverse(parent_->getCompleteTransformation())) * scene_node->getCompleteTransformation();
	//parent_ = scene_node;
	
	ss_.str(std::string());
	bool to_box = parent_ == default_entity_;
/*
	if (to_box)
	{
		ss_.str(std::string());
		ss_ << "Global location: (" << getCompleteTransformation()[3][0] << ", " << getCompleteTransformation()[3][1] << ", " << getCompleteTransformation()[3][2] << ")|";
		ss_ << "Transition from: " << (parent_ == default_entity_ ? "TERRAIN" : "BOX") << " to " << (scene_node == default_entity_ ? "TERRAIN" : "BOX") << "|";
		ss_ << "Local location: (" << local_location.x << ", " << local_location.y << ", " << local_location.z << ")|";
		ss_ << "Parent location: (" << scene_node->getGlobalLocation().x << ", " << scene_node->getGlobalLocation().y << ", " << scene_node->getGlobalLocation().z << ")|";
	}*/

	// Update the local location relative to the current scene node.
	glm::vec3 translation = scene_node->getGlobalLocation() - parent_->getGlobalLocation();
	glm::quat prev_rotation = glm::quat_cast(scene_node->getCompleteTransformation());
	glm::quat current_rotation = glm::quat_cast(parent_->getCompleteTransformation());

	//ss_ << "Rotation current. Pitch=" << glm::pitch(prev_rotation) << "; Yaw=" << glm::yaw(prev_rotation) << "; Roll=" << glm::roll(prev_rotation) << ")|";
	//ss_ << "Rotation next. Pitch=" << glm::pitch(current_rotation) << "; Yaw=" << glm::yaw(current_rotation) << "; Roll=" << glm::roll(current_rotation) << ")|";

	float pitch_diff = glm::pitch(prev_rotation) - glm::pitch(current_rotation);
	float yaw_diff = glm::yaw(prev_rotation) - glm::yaw(current_rotation);
	float roll_diff = glm::roll(prev_rotation) - glm::roll(current_rotation);

	ss_ << "Rotation difference (actual). Pitch=" << pitch_diff << "; Yaw=" << yaw_diff << "; Roll=" << roll_diff << ")|";
	if (std::abs(roll_diff) == 180)
	{
		if (yaw_diff > 0) yaw_diff = 180 - yaw_diff;
		else yaw_diff = -180 - yaw_diff;
		pitch_diff = 0;
		roll_diff = 0;
	}
	//ss_ << "After correction: Pitch=" << pitch_diff << "; Yaw=" << yaw_diff << "; Roll=" << roll_diff << ")|";

	parent_pitch_ += pitch_diff;
	parent_yaw_ += yaw_diff;
	parent_roll_ += roll_diff;

	//ss_ << "Before culling. Pitch=" << parent_pitch_ << "; Yaw=" << parent_yaw_ << "; Roll=" << parent_roll_ << ")|";

	if (parent_pitch_ >= 360) parent_pitch_ -= 360;
	else if (parent_pitch_ < 0) parent_pitch_ = 360 + parent_pitch_;
	if (parent_roll_ >= 360) parent_roll_ -= 360;
	else if (parent_roll_ < 0) parent_roll_ = 360 + parent_roll_;
	if (parent_yaw_ >= 360) parent_yaw_ -= 360;
	else if (parent_yaw_ < 0) parent_yaw_ = 360 + parent_yaw_;

	if (parent_pitch_ == 180 && parent_roll_ == 180)
	{
		if (parent_yaw_ > 0) parent_yaw_ = 180 - parent_yaw_;
		else parent_yaw_ = -180 - parent_yaw_;
		parent_pitch_ = 0;
		parent_roll_ = 0;
		//ss_ << "Rotate! Pitch=" << parent_pitch_ << "; Yaw=" << parent_yaw_ << "; Roll=" << parent_roll_ << ")|";
	}
	if (parent_yaw_ >= 360) parent_yaw_ += 360;
	else if (parent_yaw_ < 0) parent_yaw_ = 360 + parent_yaw_;

	//parent_rotation_ = parent_rotation_ * rotation_difference;
	//ss_ << "BIAS. Pitch=" << parent_pitch_ << "; Yaw=" << parent_yaw_ << "; Roll=" <<parent_roll_ << ")|";

	// Before placing the player local to the new scene node, rotate the location player on the current scene node
	// so we get the correct global rotation.
	glm::fquat pre_rotate = glm::quat_cast(parent_->getCompleteTransformation());
	local_location = glm::rotate(pre_rotate, local_location);
	//ss_ << "Pre rotation. Pitch=" << glm::pitch(pre_rotate) << "; Yaw=" << glm::yaw(pre_rotate) << "; Roll=" << glm::roll(pre_rotate) << ")|";
	
	if (to_box)
	{
		//ss_ << "Pre rotation. Local location: (" << local_location.x << ", " << local_location.y << ", " << local_location.z << ")|";
	}
	
	// Move the player to the new scene node.
	local_location -= translation;
	
	if (to_box)
	{
		//ss_ << "Transition: " << translation.x << ", " << translation.y << ", " << translation.z << ")|";
		//ss_ << "Transition. Local location: (" << local_location.x << ", " << local_location.y << ", " << local_location.z << ")|";
	}
	
	// Counter rotate the local position to find the actual position on the rotated surface.
	glm::fquat post_rotate = glm::quat_cast(scene_node->getCompleteTransformation());
	post_rotate = glm::conjugate(post_rotate);
	local_location = glm::rotate(post_rotate, local_location);
	//ss_ << "Post rotation. Pitch=" << glm::pitch(post_rotate) << "; Yaw=" << glm::yaw(post_rotate) << "; Roll=" << glm::roll(post_rotate) << ")|";
	
	if (to_box)
	{
		//ss_ << "Transition. Local location: (" << local_location.x << ", " << local_location.y << ", " << local_location.z << ")|";
		//ss_ << "Global location: (" << local_location.x + scene_node->getCompleteTransformation()[3][0] << ", " << local_location.y + scene_node->getCompleteTransformation()[3][1] << ", " << local_location.z + scene_node->getCompleteTransformation()[3][2] << ")|";
	}
	
	local_transformation_ = glm::translate(local_transformation_, local_location);
	//setTransformation(local_transformation_);

	// Store the scene node we are on at the moment.
	parent_ = scene_node;
}

void Player::onCollision(const CollisionInfo& collision_info)
{
	Entity* other = collision_info.colliding_entity_ == this ? collision_info.other_colliding_entity_ : collision_info.colliding_entity_;
	if (other->isAlive())
	{
		switch (other->getType())
		{
		// In case of the coin, we pick it up.
		case COIN:
			other->destroy();
			++score_;
			break;

		// In case of the goal, we have won the game!
		case GOAL:
			break;
		}
	}
}
