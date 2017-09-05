#ifndef CORE_ENTITIES_BRIDGE_H
#define CORE_ENTITIES_BRIDGE_H

#include <glm/glm.hpp>

#include "Entity.h"

class Lever;
class SceneNode;
class Shape;
class SceneManager;

class Bridge : public Entity
{
public:
	Bridge(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, Lever& lever, ENTITY_TYPE type, const glm::vec3& begin_position, const glm::vec3& end_position);

	void onActivate(const Entity& activated_entity);

	void prepare(float dt);

	const glm::vec3& getTarget() const { return target_position_; }

private:
	Lever* lever_;
	glm::vec3 begin_position_, end_position_, target_position_;
};

#endif
