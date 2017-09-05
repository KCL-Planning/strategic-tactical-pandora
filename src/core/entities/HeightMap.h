#ifndef CORE_ENTITIES_HIGHT_MAP_H
#define CORE_ENTITIES_HIGHT_MAP_H

/**
 * A heightmap is an entity that maintains an height map. The main difference between this 
 * and a regular entity is the collision detection. The height map is expected to be centred
 * at (0, 0, 0) in local coordinate space, so width and height must be an even number.
 */
#include "Entity.h"
#include <vector>
#include <glm/glm.hpp>

class HeightMap : public Entity
{
public:
	HeightMap(unsigned int width, unsigned int height, float cell_size, const std::vector<float>& height_map, SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children = true);

	/**
	 * Overwritten from Entity. An entity calculates the bounding box based on the 
	 * collision boxes of its children. An height map is special in that it does 
	 * not detects collisions based on bounding boxes, but uses an height map.
	 */
	virtual void getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const;

	virtual bool getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const;
	virtual bool getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const;
	
	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	virtual bool doesCollide(Entity& entity, CollisionInfo& info) const;
	virtual bool doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& point) const;
	
	
	/**
	 * Get the height of the *local* x and y coordinates. So we have to translate the 
	 * coordinates to the heightmap's local coordinate frame.
	 */
	float getHeight(float x, float z) const;
private:

	unsigned int width_;
	unsigned int height_;
	float cell_size_;
	const std::vector<float>* height_map_;
};

#endif
