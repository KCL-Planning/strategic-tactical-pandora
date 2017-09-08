#include "GL/glew.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include <iostream>

#include <glm/gtx/quaternion.hpp>

#include "BoxCollision.h"

#include "../entities/Entity.h"
#include "../scene/SceneManager.h"
#include "../math/Plane.h"
#include "CollisionInfo.h"

std::pair<unsigned int, unsigned int> BoxCollision::collision_lines_[16] = {
	std::make_pair(0, 1),
	std::make_pair(0, 2),
	std::make_pair(2, 3),
	std::make_pair(3, 1),
	std::make_pair(2, 6),
	std::make_pair(3, 7),
	std::make_pair(0, 4),
	std::make_pair(1, 5),
	std::make_pair(4, 5),
	std::make_pair(4, 6),
	std::make_pair(6, 7),
	std::make_pair(5, 7)
};


BoxCollision::BoxCollision(Entity& entity)
	: BoundedBox(entity, true, std::vector<const SceneNode*>()), entity_(&entity)
{
	centre_point_ = glm::vec3(0.0f, 0.0f, 0.0f);
	for (unsigned int i = 0; i < 8; ++i)
	{
		centre_point_ += points_[i];
	}
	centre_point_ /= 8.0f;

	max_distance_from_centre_ = 0.0f;
	for (unsigned int i = 0; i < 8; ++i)
	{
		max_distance_from_centre_ = std::max(max_distance_from_centre_, glm::distance(points_[i], centre_point_));
	}
}

BoxCollision::BoxCollision(Entity& entity, float width, float height, float depth)
	: BoundedBox(entity, width, height, depth), entity_(&entity)
{
/*
	points_[0] = glm::vec3(-width / 2.0f, -height / 2.0f, -depth / 2.0f);
	points_[1] = glm::vec3(width / 2.0f, -height / 2.0f, -depth / 2.0f);
	points_[2] = glm::vec3(-width / 2.0f, height / 2.0f, -depth / 2.0f);
	points_[3] = glm::vec3(width / 2.0f, height / 2.0f, -depth / 2.0f);
	points_[4] = glm::vec3(-width / 2.0f, -height / 2.0f, depth / 2.0f);
	points_[5] = glm::vec3(width / 2.0f, -height / 2.0f, depth / 2.0f);
	points_[6] = glm::vec3(-width / 2.0f, height / 2.0f, depth / 2.0f);
	points_[7] = glm::vec3(width / 2.0f, height / 2.0f, depth / 2.0f);

	Plane* p0 = new Plane(points_[0], points_[1], points_[2]);
	Plane* p1 = new Plane(points_[4], points_[5], points_[6]);
	Plane* p2 = new Plane(points_[6], points_[7], points_[2]);
	Plane* p3 = new Plane(points_[4], points_[5], points_[0]);
	Plane* p4 = new Plane(points_[4], points_[0], points_[6]);
	Plane* p5 = new Plane(points_[5], points_[1], points_[7]);
	
	sides_[0] = p0;
	sides_[1] = p1;
	sides_[2] = p2;
	sides_[3] = p3;
	sides_[4] = p4;
	sides_[5] = p5;

	centre_point_ = glm::vec3(0.0f, 0.0f, 0.0f);
*/
	max_distance_from_centre_ = std::sqrt((width / 2.0f) * (width / 2.0f) + (height / 2.0f) * (height / 2.0f) + (depth / 2.0f) * (depth / 2.0f));
}

BoxCollision::BoxCollision(Entity& entity, 
                           const glm::vec3& bottom_left_away, 
                           const glm::vec3& bottom_right_away, 
                           const glm::vec3& top_left_away, 
                           const glm::vec3& top_right_away, 
                           const glm::vec3& bottom_left_close, 
                           const glm::vec3& bottom_right_close, 
                           const glm::vec3& top_left_close, 
                           const glm::vec3& top_right_close)
	: BoundedBox(entity, 0, 0, 0), entity_(&entity)
{
	points_[0] = bottom_left_away;
	points_[1] = bottom_right_away;
	points_[2] = top_left_away;
	points_[3] = top_right_away;
	points_[4] = bottom_left_close;
	points_[5] = bottom_right_close;
	points_[6] = top_left_close;
	points_[7] = top_right_close;

	Plane* p0 = new Plane(points_[0], points_[1], points_[2]);
	Plane* p1 = new Plane(points_[4], points_[5], points_[6]);
	Plane* p2 = new Plane(points_[6], points_[7], points_[2]);
	Plane* p3 = new Plane(points_[4], points_[5], points_[0]);
	Plane* p4 = new Plane(points_[4], points_[0], points_[6]);
	Plane* p5 = new Plane(points_[5], points_[1], points_[7]);
	
	sides_[0] = p0;
	sides_[1] = p1;
	sides_[2] = p2;
	sides_[3] = p3;
	sides_[4] = p4;
	sides_[5] = p5;

	centre_point_ = glm::vec3(0.0f, 0.0f, 0.0f);
	for (unsigned int i = 0; i < 8; ++i)
	{
		centre_point_ += points_[i];
	}
	centre_point_ /= 8.0f;

	max_distance_from_centre_ = 0.0f;
	for (unsigned int i = 0; i < 8; ++i)
	{
		max_distance_from_centre_ = std::max(max_distance_from_centre_, glm::distance(points_[i], centre_point_));
	}
}

bool BoxCollision::doesCollide(const BoxCollision& bc, CollisionInfo& info) const
{
	if (glm::distance(entity_->getGlobalLocation() + glm::rotate(entity_->getGlobalRotation(), centre_point_), bc.entity_->getGlobalLocation() + glm::rotate(bc.entity_->getGlobalRotation(), bc.centre_point_)) > max_distance_from_centre_ + bc.max_distance_from_centre_)
	{
		return false;
	}

	glm::vec3 bc_translation = bc.entity_->getGlobalLocation();
	glm::vec3 translation = entity_->getGlobalLocation();

	// Transform the lines such that the box we compare against is aligned.
	glm::vec3 other_translated_points[8];
	for (unsigned int i = 0; i < 8; ++i)
	{
		other_translated_points[i] = bc.points_[i];
		other_translated_points[i] = glm::rotate(bc.entity_->getGlobalRotation(), other_translated_points[i]);
		other_translated_points[i] += bc_translation - translation;
		other_translated_points[i] = glm::rotate(glm::inverse(entity_->getGlobalRotation()), other_translated_points[i]);
	}

	// Now check if both boxes intersect. We cast lines instead of calculating 
	// intersections between planes -- because it is easier :).
	glm::vec3 intersection_point;

	bool found_collision = false;
	for (unsigned int i = 0; i < 6; ++i)
	{
		for (unsigned int j = 0; j < 16; ++j)
		{
			if (sides_[i]->intersectsWith(other_translated_points[collision_lines_[j].first], other_translated_points[collision_lines_[j].second], intersection_point))
			{
				found_collision = true;

				info.colliding_entity_ = entity_;
				info.other_colliding_entity_ = bc.entity_;

				// Inverse the translations and rotations to get the collision coordinate in world space.
				glm::vec3 location = intersection_point;
				
				location = glm::rotate(entity_->getGlobalRotation(), location);
				location += translation;
				info.collision_loc_.push_back(location);
				/*
				std::cout << "[" << entity_->getName() << "] Plane: " << sides_[i]->getNormal().x << ", " << sides_[i]->getNormal().y << ", " << sides_[i]->getNormal().z << ", " << sides_[i]->getD() << std::endl;
				std::cout << "[" << bc.entity_->getName() << "] Line: (" << other_translated_points[collision_lines_[j].first].x << ", " << other_translated_points[collision_lines_[j].first].y << ", " << other_translated_points[collision_lines_[j].first].z << ") - ";
				std::cout << "(" << other_translated_points[collision_lines_[j].second].x << ", " << other_translated_points[collision_lines_[j].second].y << ", " << other_translated_points[collision_lines_[j].second].z << ")" << std::endl;
				std::cout << "Collision: " << location.x << ", " << location.y << ", " << location.z << std::endl;
				*/
//				ss << "Updated collision: " << info.collision_loc_.x << ", " << info.collision_loc_.y << ", " << info.collision_loc_.z << "|";
//				ss << "Global location this: " << entity_->getGlobalLocation().x << ", " << entity_->getGlobalLocation().y << ", " << entity_->getGlobalLocation().z << "|";
//				ss << "Global location other: " << bc.entity_->getGlobalLocation().x << ", " << bc.entity_->getGlobalLocation().y << ", " << bc.entity_->getGlobalLocation().z;
			}
		}
	}

	return found_collision;
}

bool BoxCollision::doesCollide(Entity& other, const glm::vec3& begin_point, const glm::vec3& end_point, CollisionInfo& info) const
{
/*
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Box part of: " << entity_->getName() << "(other=" << other.getName() << ")" << std::endl;
	ss << "doesCollide: (" << begin_point.x << ", " << begin_point.y << ", " << begin_point.z << ") -> (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#endif
*/
	/*
	glm::vec3 updated_begin_point = begin_point;// + other.getGlobalLocation() - entity_->getGlobalLocation();
	updated_begin_point = glm::rotate(other.getGlobalRotation(), updated_begin_point);
	updated_begin_point += other.getGlobalLocation() - entity_->getGlobalLocation();
	updated_begin_point = glm::rotate(glm::inverse(entity_->getGlobalRotation()), updated_begin_point);

	glm::vec3 updated_end_point = end_point;// + other.getGlobalLocation() - entity_->getGlobalLocation();
	updated_end_point = glm::rotate(other.getGlobalRotation(), updated_end_point);
	updated_end_point += other.getGlobalLocation() - entity_->getGlobalLocation();
	updated_end_point = glm::rotate(glm::inverse(entity_->getGlobalRotation()), updated_end_point);
	*/

	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point = glm::vec3(glm::inverse(entity_->getCompleteTransformation()) * glm::vec4(updated_begin_point, 1.0f));

	glm::vec3 updated_end_point = end_point;
	updated_end_point = glm::vec3(glm::inverse(entity_->getCompleteTransformation()) * glm::vec4(updated_end_point, 1.0f));
/*
#ifdef _WIN32
	{
	std::stringstream ss;
	ss << "Updated points: (" << updated_begin_point.x << ", " << updated_begin_point.y << ", " << updated_begin_point.z << ") -> (" << updated_end_point.x << ", " << updated_end_point.y << ", " << updated_end_point.z << ")" << std::endl;
	OutputDebugString(ss.str().c_str());
	}
#endif
*/
/*
	if (other.getName() == "shark")
	{
		std::cout << "(" << other.getGlobalLocation().x << ", " << other.getGlobalLocation().y << ", " << other.getGlobalLocation().z << ")" << std::endl;
		std::cout << "[" << entity_->getName() << "] BoxCollision::doesCollide: (" << begin_point.x << ", " << begin_point.y << ", " << begin_point.z << ") -> (" << end_point.x << ", " << end_point.y << ", " << end_point.z << ")" << std::endl;
		std::cout << "[" << entity_->getName() << "] Updated points: (" << updated_begin_point.x << ", " << updated_begin_point.y << ", " << updated_begin_point.z << ") -> (" << updated_end_point.x << ", " << updated_end_point.y << ", " << updated_end_point.z << ")" << std::endl;
	}
*/
	/*
	// Absolute values.
	glm::fquat inverse_matrix = glm::inverse(entity_->getGlobalRotation());
	glm::vec3 updated_begin_point = begin_point;
	updated_begin_point -= entity_->getGlobalLocation();
	updated_begin_point = glm::rotate(inverse_matrix, updated_begin_point);

	glm::vec3 updated_end_point = end_point;
	updated_end_point -= entity_->getGlobalLocation();
	updated_end_point = glm::rotate(inverse_matrix, updated_end_point);
	*/

	// Transform the lines such that the box we compare against is aligned.
	glm::vec3 intersection_point;
	bool found_collision = false;

	for (unsigned int i = 0; i < 6; ++i)
	{
/*
#ifdef _WIN32
		{
		std::stringstream ss;
		ss << "Compare against plane: (" << sides_[i]->getNormal().x << ", " << sides_[i]->getNormal().y << ", " << sides_[i]->getNormal().z << ", " << sides_[i]->getD() << ")" << std::endl;
		OutputDebugString(ss.str().c_str());
		}
#endif
*/
		if (sides_[i]->intersectsWith(updated_begin_point, updated_end_point, intersection_point))
		{
/*
#ifdef _WIN32
			{
			std::stringstream ss;
			ss << "Collides at: (" << intersection_point.x << ", " << intersection_point.y << ", " << intersection_point.z << ")" << std::endl;
			OutputDebugString(ss.str().c_str());
			}
#endif
*/
			found_collision = true;

			info.colliding_entity_ = entity_;
			info.other_colliding_entity_ = &other;

			glm::vec3 location = intersection_point;
			location = glm::rotate(entity_->getGlobalRotation(), location);
			location += entity_->getGlobalLocation();
			info.collision_loc_.push_back(location);
		}
	}
	return found_collision;
}
