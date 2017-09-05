#ifndef PANDORA_MODELS_ROBOT_HAND_H
#define PANDORA_MODELS_ROBOT_HAND_H

#include "glm/glm.hpp"

#include "../../../core/entities/Entity.h"

class SceneManager;
class SceneNode;

/**
 * Mimic a robot hand that can turn a valve.
 */
class RobotHand : public Entity
{
public:
	RobotHand(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation);
	
	void prepare(float dt);
	
	void unfoldArm(bool fold);
	
	void rotateHand(float angle) { hand_angle_ = angle; }
	
	bool isArmUnfolded() { return arm_is_unfolded_; }
private:
	SceneNode* first_half_;
	SceneNode* second_half_;
	SceneNode* hand_;
	
	float fold_;
	float hand_angle_;
	bool unfold_arm_;
	bool arm_is_unfolded_;
};

#endif
