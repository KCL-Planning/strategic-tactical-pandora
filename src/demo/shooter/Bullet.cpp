#include "Bullet.h"

#include "../../core/collision/ConvexPolygon.h"
#include "../../core/scene/SceneManager.h"
#include "../../core/collision/CollisionInfo.h"

Bullet::Bullet(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, bool init_children)
	: Entity(scene_manager, parent, transformation, type, name, init_children), life_time_(0)
{
	ConvexPolygon* bc = new ConvexPolygon(*this, 0.2f, 0.2f, 0.2f);
	addCollision(*bc);
}

void Bullet::prepare(float dt)
{
	std::vector<CollisionInfo> collision_infos;
	std::stringstream ss_;
	unsigned int collisions_checked = 0;
	//bool found_collision = scene_manager_->getRoot().getCollisions(*this, collision_infos, ss_, collisions_checked);

	Entity::prepare(dt);
	life_time_ += dt;
	if (/*found_collision || */life_time_ > 10)
	{
		destroy();
	}
}

void Bullet::onCollision(const std::vector<CollisionInfo>& collision_info)
{
	if (life_time_ > 2.0f)
	{
		destroy();
		//is_alive_ = false;
	}
}
