#include "Shark.h"
#define GLX_GLXEXT_LEGACY //Must be declared so that our local glxext.h is picked up, rather than the system one
#define GLFW_NO_GLU // Do not allow GL/glfw to include the gl.h header

#include "GL/glew.h"

#define _USE_MATH_DEFINES
#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

#include <glm/glm.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "dpengine/entities/camera/Camera.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/shapes/terrain.h"
#include "dpengine/shapes/Line.h"
#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/collision/CollisionInfo.h"
#include "dpengine/collision/CollisionPoint.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/shaders/LineShader.h"
#include "dpengine/shaders/BasicShadowShader.h"

Shark::Shark(DreadedPE::SceneNode* parent, const glm::mat4& transformation, DreadedPE::SceneManager& scene_manager)
	: DreadedPE::Entity(scene_manager, parent, transformation, DreadedPE::OBSTACLE, "shark")
{
	height_ = 2.1f;

	current_waypoint_ = 0;
	
	velocity_ = 0;
	desired_velocity_ = 0;
	max_velocity_ = 0.05;
	rotation_speed_ = 10.0f;
	desired_direction_ = glm::vec3(0, 0, 0);
	pitch_ = 0;
	yaw_ = 0;
	roll_ = 0;
	desired_pitch_ = 0;
	desired_yaw_ = 0;
}

void Shark::init(DreadedPE::Material& material, DreadedPE::ShaderInterface& shader)
{
	float width = 4.449f * 0.4f;
	float height = 3.602f * 0.4f;
	float size = 9.587f * 0.4f;
	
	DreadedPE::ConvexPolygon* bc = new DreadedPE::ConvexPolygon(*this, width, height, size);
	addCollision(*bc);
	
	
	/*
	//float offset = 0.01f;
	glm::vec3 bottom_left_away(-width, 0, -size);
	glm::vec3 bottom_right_away(width, 0, -size);
	glm::vec3 top_left_away(-width, height, -size);
	glm::vec3 top_right_away(width, height, -size);
	glm::vec3 bottom_left_close(-width, 0, size); 
	glm::vec3 bottom_right_close(width, 0, size);
	glm::vec3 top_left_close(-width, height, size);
	glm::vec3 top_right_close(width, height, size);
	ConvexPolygon* bc = new ConvexPolygon(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	addCollision(*bc, material, shader);
	
	BoundedBox* bc2 = new BoundedBox(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	bounded_collision_box_ = bc2;

	frustum_checker_ = new BoundedBox(*this, bottom_left_away, bottom_right_away, top_left_away, top_right_away, bottom_left_close, bottom_right_close, top_left_close, top_right_close);
	*/
	
	/*
	collision_line_ = new Line();
	std::vector<glm::vec3> line_vec;
	line_vec.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	line_vec.push_back(glm::vec3(1.0f, 0.0f, -3.0f));
	line_vec.push_back(glm::vec3(-1.0f, 0.0f, -3.0f));
	line_vec.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	line_vec.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	line_vec.push_back(glm::vec3(0.0f, 1.0f, -3.0f));
	line_vec.push_back(glm::vec3(0.0f, -1.0f, -3.0f));
	line_vec.push_back(glm::vec3(0.0f, 0.0f, 0.0f));
	collision_line_->setVertexBuffer(line_vec);
	SceneLeafModel* lines = new SceneLeafModel(*this, NULL, *collision_line_, *SceneNode::bright_material_, LineShader::getShader(), true, true);
	*/
}

void Shark::prepare(float dt)
{
	if (waypoints_.empty())
	{
		return;
	}
	
	// If there are no waypoint we do not need to do anything! :)
	if (current_waypoint_ >= waypoints_.size())
	{
		current_waypoint_ = 0;
	}
	
	// Before we start, make sure we haven't all ready accomplished our task!
	float distance = glm::distance(getGlobalLocation(), waypoints_[current_waypoint_]);
	
	// If we are very close, then we move on to the next waypoint.
	if (distance < 0.1f)
	{
		++current_waypoint_;
		// If there are no waypoint we do not need to do anything! :)
		if (current_waypoint_ >= waypoints_.size())
		{
			current_waypoint_ = 0;
		}
		//std::cout << "Shark: Move on to the next waypoint! " << waypoints_.size() << " v.s " << current_waypoint_ << std::endl;
	}
	
	// Move the AUV forward. The speed we set is relative to the distance to the point.
	desired_velocity_ = -std::min(0.5f, distance / 2.0f);
	
#ifdef _WIN32
	float begin_prepare = float(GetTickCount()) / 1000.0f;
#endif
#ifdef __linux__
	float begin_prepare = float(clock()) / CLOCKS_PER_SEC;
#endif
	//float begin_prepare = float(clock()) / 1000.0f;
	glm::vec3 old_local_position(local_transformation_[3][0], local_transformation_[3][1], local_transformation_[3][2]);
	glm::vec3 local_position = old_local_position;
	
	float friction = 0.0001f;
	glm::vec3 current_direction(0.0f, 0.0f, 0.0f);
	if (velocity_ > 0 && friction * dt < velocity_)
		velocity_ -= friction * dt; // Friction.
	else if (velocity_ < 0 && -friction * dt > velocity_)
		velocity_ += friction * dt; // Friction.
	else
		velocity_ = 0.0f;
	bool received_input = true;
	if (velocity_ < desired_velocity_) velocity_ += 0.003f * dt;
	else if (velocity_ > desired_velocity_) velocity_ -= 0.003f * dt;
	
	if (velocity_ > max_velocity_) velocity_ = max_velocity_;
	if (velocity_ < -max_velocity_) velocity_ = -max_velocity_;
	
	if (desired_velocity_ == 0)
	{
		velocity_ *= 0.1;
	}
	
	if (velocity_ > max_velocity_)
	{
		velocity_ = max_velocity_;
	}
	else if (velocity_ < -max_velocity_)
	{
		velocity_ = -max_velocity_;
	}

	current_direction.z = velocity_;

	// If it is going to move, check if it will not move too close to a wall. If it does then we need to adjust the orientation of the shark.
	glm::vec4 centre(0.0f, 0.0f, 0.0f, 1.0f);
	glm::vec4 right_ray(1.0f, 0.0f, -3.0f, 1.0f);
	glm::vec4 left_ray(-1.0f, 0.0f, -3.0f, 1.0f);
	glm::vec4 up_ray(0.0f, 1.0f, -3.0f, 1.0f);
	glm::vec4 down_ray(0.0f, -1.0f, -3.0f, 1.0f);
	
	centre = getCompleteTransformation() * centre;
	right_ray = getCompleteTransformation() * right_ray;
	left_ray = getCompleteTransformation() * left_ray;
	
	up_ray = getCompleteTransformation() * up_ray;
	down_ray = getCompleteTransformation() * down_ray;
	
	// Check the collisions of these rays.
	std::vector<DreadedPE::CollisionInfo> l_collisions;
	std::vector<DreadedPE::CollisionInfo> r_collisions;
	std::vector<DreadedPE::CollisionInfo> u_collisions;
	std::vector<DreadedPE::CollisionInfo> d_collisions;
	//std::cout << "*** (" << getGlobalLocation().x << ", " << getGlobalLocation().y << ", " << getGlobalLocation().z << ")" << std::endl;
	if (current_region_ != NULL)
	{
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(left_ray), l_collisions);
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(right_ray), r_collisions);
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(up_ray), u_collisions);
		current_region_->getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(down_ray), d_collisions);
	}
	else
	{
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(left_ray), l_collisions);
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(right_ray), r_collisions);
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(up_ray), u_collisions);
		scene_manager_->getRoot().getCollisions(*this, static_cast<glm::vec3>(centre), static_cast<glm::vec3>(down_ray), d_collisions);
	}
	/*
	if (l_collisions.size() > 0 || r_collisions.size() > 0)
	{
		std::cout << l_collisions.size() << " " << r_collisions.size() << std::endl;
	}
	*/
	float ld = std::numeric_limits<float>::max();
	float rd = std::numeric_limits<float>::max();
	float ud = std::numeric_limits<float>::max();
	float dd = std::numeric_limits<float>::max();

	for (std::vector<DreadedPE::CollisionInfo>::const_iterator ci = l_collisions.begin(); ci != l_collisions.end(); ++ci)
	{
		const DreadedPE::CollisionInfo& info = *ci;
		for (std::vector<DreadedPE::CollisionPoint>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(ci->intersection_point_, getGlobalLocation());
			if (d < ld) ld = d;
		}
	}

	for (std::vector<DreadedPE::CollisionInfo>::const_iterator ci = r_collisions.begin(); ci != r_collisions.end(); ++ci)
	{
		const DreadedPE::CollisionInfo& info = *ci;
		for (std::vector<DreadedPE::CollisionPoint>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(ci->intersection_point_, getGlobalLocation());
			if (d < rd) rd = d;
		}
	}

	for (std::vector<DreadedPE::CollisionInfo>::const_iterator ci = u_collisions.begin(); ci != u_collisions.end(); ++ci)
	{
		const DreadedPE::CollisionInfo& info = *ci;
		for (std::vector<DreadedPE::CollisionPoint>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(ci->intersection_point_, getGlobalLocation());
			if (d < ud) ud = d;
		}
	}
	
	for (std::vector<DreadedPE::CollisionInfo>::const_iterator ci = d_collisions.begin(); ci != d_collisions.end(); ++ci)
	{
		const DreadedPE::CollisionInfo& info = *ci;
		for (std::vector<DreadedPE::CollisionPoint>::const_iterator ci = info.collision_loc_.begin(); ci != info.collision_loc_.end(); ++ci)
		{
			float d = glm::distance(ci->intersection_point_, getGlobalLocation());
			if (d < dd) dd = d;
		}
	}
	
	if (ld != std::numeric_limits<float>::max() || rd != std::numeric_limits<float>::max())
	{

	}
	
	// Update the heading of the monster so it does not collide with any wall.
	if (ld != std::numeric_limits<float>::max())
	{
		//glm::vec3 l(dt * (2.0f) / (1.0f + ld), 0.0f, 0.0f);
		glm::vec3 l(dt * (1.0f) / std::max(0.2f, ld * 0.1f), 0.0f, 0.0f);
		current_direction += l;
		//std::cout << "L:" << ld;
	}
	if (rd != std::numeric_limits<float>::max())
	{
		//glm::vec3 r(-dt * (2.0f) / (1.0f + rd), 0.0f, 0.0f);
		glm::vec3 r(-dt * (1.0f) / std::max(0.2f, rd * 0.1f), 0.0f, 0.0f);
		current_direction += r;
		//std::cout << "R:" << rd;
	}
	if (ud != std::numeric_limits<float>::max())
	{
		//glm::vec3 r(-dt * (2.0f) / (1.0f + rd), 0.0f, 0.0f);
		glm::vec3 u(0.0f, dt * (1.0f) / std::max(0.2f, rd * 0.1f), 0.0f);
		current_direction += u;
		//std::cout << "U:" << ud;
	}
	if (dd != std::numeric_limits<float>::max())
	{
		//glm::vec3 r(-dt * (2.0f) / (1.0f + rd), 0.0f, 0.0f);
		glm::vec3 d(0.0f, -dt * (1.0f) / std::max(0.2f, rd * 0.1f), 0.0f);
		current_direction += d;
		//std::cout << "D:" << dd;
	}
	
	// Check the rotation.
	glm::vec4 auv_facing4 = glm::vec4(0, 0, -1, 1);
	auv_facing4 = getCompleteTransformation() * auv_facing4;
	glm::vec3 auv_facing(auv_facing4.x, auv_facing4.y, auv_facing4.z);
	auv_facing -= getGlobalLocation();

	float d_yaw = 0;
	float d_pitch = 0;
	
	// Check if we are (roughly) pointing in the direction of the next waypoint.
	desired_direction_ = waypoints_[current_waypoint_] - getGlobalLocation();
	desired_direction_ = glm::normalize(desired_direction_);

	//std::cout << "Shark desired direction: (" << desired_direction_.x << ", " << desired_direction_.y << ", " << desired_direction_.z << ")" << std::endl;
	
	d_yaw = atan2(auv_facing.z, auv_facing.x) - atan2(desired_direction_.z, desired_direction_.x);
	d_pitch = atan2(auv_facing.y, sqrt(auv_facing.z * auv_facing.z + auv_facing.x * auv_facing.x)) - atan2(desired_direction_.y, sqrt(desired_direction_.z * desired_direction_.z + desired_direction_.x * desired_direction_.x));
	
	d_yaw *= 180.0f / M_PI;
	d_pitch *= 180.0f / M_PI;

	if (d_yaw > 180) d_yaw -= 360;
	if (d_yaw < -180) d_yaw += 360;

	if (d_pitch > 0)
	{
		d_pitch = std::min(d_pitch, dt * rotation_speed_);
	}
	else
	{
		d_pitch = std::max(d_pitch, -dt * rotation_speed_);
	}

	if (d_yaw > 0)
	{
		d_yaw = std::min(d_yaw, dt * rotation_speed_);
	}
	else
	{
		d_yaw = std::max(d_yaw, -dt * rotation_speed_);
	}

	pitch_ -= d_pitch;
	yaw_ += d_yaw;
	
	if (current_direction.x != 0.0f || current_direction.z != 0.0f)
	{
		current_direction = glm::rotate(current_direction, glm::radians(pitch_), glm::vec3(1.0f, 0.0f, 0.0f));
		current_direction = glm::rotate(current_direction, glm::radians(yaw_), glm::vec3(0.0f, 1.0f, 0.0f));
		current_direction = glm::rotate(current_direction, glm::radians(roll_), glm::vec3(0.0f, 0.0f, 1.0f));
	}
	
	//std::cout << "Shark velocity: " << velocity_ << ". Direction: (" << current_direction.x << ", " << current_direction.y << ", " << current_direction.z << ")" << std::endl;

	local_position += current_direction;
	local_transformation_ = glm::translate(glm::mat4(1.0f), glm::vec3(local_position.x, local_position.y, local_position.z));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(roll_), glm::vec3(0, 0, 1));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(yaw_), glm::vec3(0, 1, 0));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(pitch_), glm::vec3(1, 0, 0));
	//local_transformation_ = glm::scale(local_transformation_, glm::vec3(0.4f, 0.4f, 0.4f));
	updateTransformations();
	
	std::vector<DreadedPE::CollisionInfo> collision_infos;
	//glm::fquat parent_rotation = parent_->getGlobalRotation();

	// Localise the player.
	if (current_region_ == NULL)
	{
		current_region_ = DreadedPE::Region::findRegionGlobal(getGlobalLocation());
	}

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

		std::cout << " *** Collision found *** " << std::endl;
		for (std::vector<CollisionInfo>::const_iterator ci = collision_infos.begin(); ci != collision_infos.end(); ++ci)
		{
			const std::vector<glm::vec3>& collision_locations = (*ci).collision_loc_;
			std::cout << " *** Location *** " << std::endl;
			for (std::vector<glm::vec3>::const_iterator ci = collision_locations.begin(); ci != collision_locations.end(); ++ci)
			{
				std::cout << "(" << (*ci).x << ", " << (*ci).y << ", " << (*ci).z << ")" << std::endl;
			}
		}
#endif
		velocity_ = 0.0f;
		local_position = old_local_position;
	}

	local_transformation_ = glm::translate(glm::mat4(1.0f), local_position);
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(roll_), glm::vec3(0.0f, 0.0f, 1.0f));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(yaw_), glm::vec3(0.0f, 1.0f, 0.0f));
	local_transformation_ = glm::rotate(local_transformation_, glm::radians(pitch_), glm::vec3(1.0f, 0.0f, 0.0f));
	//local_transformation_ = glm::scale(local_transformation_, glm::vec3(0.4f, 0.4f, 0.4f));
	
	
	DreadedPE::Entity::prepare(dt);
/*
	if (region_ != NULL)
	{
		region_ = region_->findRegion(getGlobalLocation());
	}
*/
}

void Shark::transition(DreadedPE::SceneNode* scene_node, glm::vec3& local_location)
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

void Shark::onCollision(const DreadedPE::CollisionInfo& collision_info)
{
	
}
