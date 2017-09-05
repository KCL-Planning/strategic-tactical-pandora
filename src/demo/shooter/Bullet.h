#ifndef DEMO_SHOOTER_BULLET_H
#define DEMO_SHOOTER_BULLET_H

#include "../../core/entities/Entity.h"

class Bullet : public Entity
{
public:
	Bullet(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children = true);
	virtual void onCollision(const std::vector<CollisionInfo>& collision_info);

	virtual void prepare(float dt);
private:
	float life_time_;
};

#endif
