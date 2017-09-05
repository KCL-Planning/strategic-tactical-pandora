#ifndef CORE_ENTITIES_MONSTER_H
#define CORE_ENTITIES_MONSTER_H

#include "Entity.h"

class Camera;
class SceneNode;
class Terrain;
class SceneManager;
class Material;
class ShaderInterface;
class Line;

class Monster : public Entity
{
public:
	Monster(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager);
	
	void init(Material& material, ShaderInterface& shader);
	
	virtual void prepare(float dt);
	void onCollision(const CollisionInfo& collision_info);

	void setWaypoints(const std::vector<glm::vec3>& waypoints) { waypoints_ = waypoints; }

private:
	void transition(SceneNode* new_node, glm::vec3& local_location);

	float height_;

	//Camera* camera_;
	
	float y_velocity_;

	std::stringstream ss_;

	unsigned int nr_collisions_checked_;

	std::vector<glm::vec3> waypoints_;

	Line* collision_line_;
};

#endif
