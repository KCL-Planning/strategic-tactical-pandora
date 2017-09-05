#ifndef CORE_COLLISION_COLLISION_INFO_H
#define CORE_COLLISION_COLLISION_INFO_H

#include <glm/glm.hpp>
#include <vector>

class Entity;

/**
 * Data structure to capture the collision between two entities.
 */
class CollisionInfo
{
public:
	CollisionInfo();

	Entity* colliding_entity_;
	Entity* other_colliding_entity_;

	// We can have up to 12 collision points, because there are 12 lines. We store all of them.
	//char nr_collisions_detected_;
	//glm::vec3 collision_loc_[12];
	std::vector<glm::vec3> collision_loc_;
};

#endif
