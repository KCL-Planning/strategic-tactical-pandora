#ifndef CORE_ENTITIES_WAVING_WATER_H
#define CORE_ENTITIES_WAVING_WATER_H

#include <glm/glm.hpp>

#include "Entity.h"

class SceneManager;
class Water;

class WavingWater : public Entity
{
public:
	WavingWater(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transition, Water& water);

	virtual void prepare(float dt);

private:
	Water* water_;
};

#endif
