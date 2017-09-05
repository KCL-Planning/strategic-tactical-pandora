#ifndef CORE_AGENTS_BOX_COLLISION_H
#define CORE_AGENTS_BOX_COLLISION_H

#include <glm/glm.hpp>
#include <vector>
#include <sstream>

#include "../math/BoundedBox.h"

class Entity;
class CollisionInfo;
class Plane;
class Region;

/**
 * Bounded box collision detection. This class allows us to detect collisions between two bounded boxes or a
 * bounded box and a line segment. The bounded boxes can be rotated in any way. In order to calculate whether 
 * there is a collision we allign one of the bounded boxes such that it aligns with the X, Y, and Z axis. This 
 * makes the calculations easier to perform.
 *
 * Collision is detected by separating the non aligned box into 12 separate lines segments. These line segments 
 * are compared against the 6 planes that make up the box. If the line segment collides with the plane we calculate 
 * if it colides with the plane segment. In order to do so we make sure that the direction of the four vectors 
 * which form the circumference of the plane face the same direction (e.g. clockwise). We can check if the point 
 * of intersection with the line and plane falls within the plane segment by comparing the sign of the cross 
 * products between each vector of the plane and the start position of each vector with the intersecting point.
 * Iff the sign of each cross product is the same then the point is within the plane segment and we have a collision.
 */
class BoxCollision : public BoundedBox
{
public:
	BoxCollision(Entity& entity);
	BoxCollision(Entity& entity, float width, float height, float depth);
	BoxCollision(Entity& entity, 
                           const glm::vec3& bottom_left_away, 
                           const glm::vec3& bottom_right_away, 
                           const glm::vec3& top_left_away, 
                           const glm::vec3& top_right_away, 
                           const glm::vec3& bottom_left_close, 
                           const glm::vec3& bottom_right_close, 
                           const glm::vec3& top_left_close, 
                           const glm::vec3& top_right_close);

	bool doesCollide(const BoxCollision& bc, CollisionInfo& info) const;

	/**
	 * Check if there is a collisiong with the line segment (begin_point -> end_point). These points are 
	 * in the global reference frame.
	 * @param other The entity that is associated with the line segment.
	 * @param begin_point The beginning of the line segment in the global reference frame.
	 * @param end_point The end of the line in the global reference frame.
	 * @return True if an collision is found, false otherwise.
	 */
	bool doesCollide(Entity& other, const glm::vec3& begin_point, const glm::vec3& end_point, CollisionInfo& info) const;
	
	bool doesCollide(const glm::vec3& begin_point, const glm::vec3& end_point, float effective_width) const
	{
		return BoundedBox::doesCollide(begin_point, end_point, effective_width);
	}

	/**
	 * Get collision between this (static) bounded box and another box
	 * that is moving.
	 * @param other The moving box.
	 * @param velocity The speed at which the other box is moving.
	 * @param t The max time which we want to simulate.
	 * @return The time it takes for a collision to occur or -1 if no collision occurs.
	 * @Note: Not working yet (or at least untested...).
	 */
	float getCollision(const BoundedBox& other, const glm::vec3& velocity, float t) const;

	//void addToCollisionGroup(const Region& region);
	//void removeFromCollisionGroup(const Region& region);
	//void getPoints(std::vector<glm::vec3>& points) const;
	//const glm::vec3* getPoints() const { return &points_[0]; }

private:
	Entity* entity_;
	//float width_, height_, depth_;
	//glm::vec3 points_[8];
	//Plane* sides_[6];
	static std::pair<unsigned int, unsigned int> collision_lines_[16];
	
	float max_distance_from_centre_;
	//glm::vec3 centre_point_;

	// We divide all collision boxes into collision groups. Collision can only occur between
	// objects that are in the same group.
	//std::vector<const Region*> collision_groups_;
};

#endif
