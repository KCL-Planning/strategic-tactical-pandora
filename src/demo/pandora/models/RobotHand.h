#ifndef PANDORA_MODELS_ROBOT_HAND_H
#define PANDORA_MODELS_ROBOT_HAND_H

#include "glm/glm.hpp"

#include "dpengine/entities/Entity.h"

namespace DreadedPE
{
	class SceneManager;
	class SceneNode;
};

/**
 * Mimic a robot hand that can turn a valve.
 */
class RobotHand : public DreadedPE::Entity
{
public:
	RobotHand(DreadedPE::SceneManager& scene_manager, DreadedPE::SceneNode* parent, const glm::mat4& transformation);
	
	void prepare(float dt);
	
	void unfoldArm(bool fold);
	
	void rotateHand(float angle) { hand_angle_ = angle; }
	
	bool isArmUnfolded() { return arm_is_unfolded_; }
private:
	DreadedPE::SceneNode* first_half_;
	DreadedPE::SceneNode* second_half_;
	DreadedPE::SceneNode* hand_;
	
	float fold_;
	bool unfold_arm_;
	float hand_angle_;
	bool arm_is_unfolded_;
};

#endif
