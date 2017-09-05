#ifndef ENTITY_H
#define ENTITY_H

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <sstream>

#include "../scene/SceneNode.h"

//class ActivateListener;
class BoxCollision;
class CollisionInfo;
class SceneNode;
class Material;
class ShaderInterface;
class Behaviour;
class SceneManager;

enum ENTITY_TYPE { COIN, GOAL, OBSTACLE, PLAYER, PASSABLE, DEBUG };

class Entity : public SceneNode
{
public:
	Entity(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children = true);

	// Disable copy and assign operator.
	Entity(const Entity& entity);
	void operator=(const Entity& entity);

	virtual ~Entity();

	virtual void getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const;

	virtual void prepare(float dt);

	// Collision detecting.
	void addCollision(BoxCollision& collision);

	// Collision detecting with visual clues.
	void addCollision(BoxCollision& collision, Material& material, ShaderInterface& shader);

	/**
	 * Called whenever an entity tries to activate this entity. The default behaviour is to do
	 * notify all the behaviours.
	 * @param activator The entity that activated this entity.
	 * @return True if this entity was activated, false if not.
	 */
	virtual bool activate(Entity& activator);

	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	//virtual bool doesCollide(Entity& entity, CollisionInfo& info, std::stringstream& ss) const;

	virtual bool getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const;
	virtual bool getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const;
	
	/**
	 * Same as getCollisions, but we stop once we found the first collision.
	 */
	virtual bool doesCollide(Entity& entity, CollisionInfo& info) const;
	virtual bool doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const;
	virtual bool doesCollide(Entity* entity, const glm::vec3& point) const;


	//virtual bool checkCollision(const Entity& other, CollisionInfo& info, std::stringstream& ss) const;
	virtual void onCollision(const std::vector<CollisionInfo>& collision_info);

	const std::vector<BoxCollision*>& getCollisions() const { return collisions_; }

	ENTITY_TYPE getType() const { return type_; }

	/**
	 * Kill this entity.
	 */
	virtual void destroy();

	bool isAlive() const { return is_alive_; }

	const std::string& getName() const { return name_; }

	//Region* getRegion() const { return region_; }

	//const std::vector<Region*>& getRegionsVisibleFrom() const { return regions_visible_from_; }
protected:
	bool is_alive_;
	ENTITY_TYPE type_;

	std::vector<BoxCollision*> collisions_;

	//std::vector<Behaviour*> behaviours_;

	std::string name_;

	// The region where this entity's centre point is in.
	//Region* region_;

	/**
	 * Keep track of all the regions this entity is visible from. We also keep the scene node
	 * that is added to that region to offset this entity.
	 */
	//std::vector<Region*> regions_visible_from_;

	//glm::mat4 previous_transform_;
};

#endif
