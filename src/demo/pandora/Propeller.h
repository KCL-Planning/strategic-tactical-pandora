#ifndef DEMO_PANDORA_PROPELLER_H
#define DEMO_PANDORA_PROPELLER_H

#include "../../core/scene/SceneNode.h"

class Propeller : public SceneNode
{
public:
	Propeller(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation);
	void prepare(float dt);

	float getRotationSpeed() const { return rotation_speed_; }
	void setRotationSpeed(float rotation_speed) { rotation_speed_ = rotation_speed; }
private:
	float rotation_speed_;
};

#endif
