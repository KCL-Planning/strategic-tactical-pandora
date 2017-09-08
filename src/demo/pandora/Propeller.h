#ifndef DEMO_PANDORA_PROPELLER_H
#define DEMO_PANDORA_PROPELLER_H

#include <glm/glm.hpp>
#include <dpengine/scene/SceneNode.h>

namespace DreadedPE
{
	class SceneManager;
};

class Propeller : public DreadedPE::SceneNode
{
public:
	Propeller(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation);
	void prepare(float dt);

	float getRotationSpeed() const { return rotation_speed_; }
	void setRotationSpeed(float rotation_speed) { rotation_speed_ = rotation_speed; }
private:
	float rotation_speed_;
};

#endif
