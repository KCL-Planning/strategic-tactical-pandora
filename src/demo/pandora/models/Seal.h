#ifndef DEMO_PANDORA_MODELS_SEAL_H
#define DEMO_PANDORA_MODELS_SEAL_H

#include "dpengine/entities/Entity.h"

class Seal : public DreadedPE::Entity
{
public:
	Seal(DreadedPE::SceneNode* parent, const glm::mat4& transformation, DreadedPE::SceneManager& scene_manager);
	
	void init(DreadedPE::Material& material, DreadedPE::ShaderInterface& shader);
	
	void prepare(float dt);
	void onCollision(const DreadedPE::CollisionInfo& collision_info);

private:
};

#endif
