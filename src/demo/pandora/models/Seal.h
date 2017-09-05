#ifndef DEMO_PANDORA_MODELS_SEAL_H
#define DEMO_PANDORA_MODELS_SEAL_H

#include "../../../core/entities/Entity.h"

class Seal : public Entity
{
public:
	Seal(SceneNode* parent, const glm::mat4& transformation, SceneManager& scene_manager);
	
	void init(Material& material, ShaderInterface& shader);
	
	void prepare(float dt);
	void onCollision(const CollisionInfo& collision_info);

private:
};

#endif
