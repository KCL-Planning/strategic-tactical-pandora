#ifndef CORE_ENTITIES_HIGHT_MAP_H
#define CORE_ENTITIES_HIGHT_MAP_H

/**
 * A heightmap is an entity that maintains an height map. The main difference between this 
 * and a regular entity is the collision detection. The height map is expected to be centred
 * at (0, 0, 0) in local coordinate space, so width and height must be an even number.
 */
#include "dpengine/entities/Entity.h"
#include <vector>
#include <glm/glm.hpp>

class HeightMap : public DreadedPE::Entity
{
public:
	HeightMap(unsigned int width, unsigned int height, float cell_size, const std::vector<float>& height_map, DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation, DreadedPE::ENTITY_TYPE type, const std::string& name, bool init_children = true);

	/**
	 * Overwritten from Entity. An entity calculates the bounding box based on the 
	 * collision boxes of its children. An height map is special in that it does 
	 * not detects collisions based on bounding boxes, but uses an height map.
	 */
	virtual void getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const;

	virtual bool getCollisions(DreadedPE::Entity& entity, std::vector<DreadedPE::CollisionInfo>& info) const;
	virtual bool getCollisions(DreadedPE::Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<DreadedPE::CollisionInfo>& info) const;
	
	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	virtual bool doesCollide(DreadedPE::Entity& entity, DreadedPE::CollisionInfo& info) const;
	virtual bool doesCollide(DreadedPE::Entity& entity, const glm::vec3& begin, const glm::vec3& end, DreadedPE::CollisionInfo& info) const;
	virtual bool doesCollide(DreadedPE::Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const;
	virtual bool doesCollide(DreadedPE::Entity* entity, const glm::vec3& point) const;
	
	
	/**
	 * Get the height of the *local* x and y coordinates. So we have to translate the 
	 * coordinates to the heightmap's local coordinate frame.
	 */
	float getHeight(float x, float z) const;
protected:

	unsigned int width_;
	unsigned int height_;
	float cell_size_;
	const std::vector<float>* height_map_;
};

#endif
