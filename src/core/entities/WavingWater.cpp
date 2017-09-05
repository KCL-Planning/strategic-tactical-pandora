#include "WavingWater.h"

#include "../../shapes/Water.h"

WavingWater::WavingWater(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transition, Water& water)
	: Entity(scene_manager, parent, transition, PASSABLE, "water"), water_(&water)
{

}

void WavingWater::prepare(float dt)
{
	water_->progressWaves(dt);
	Entity::prepare(dt);
}
