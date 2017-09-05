#ifndef CORE_MODELS_ANIMATION_H
#define CORE_MODELS_ANIMATION_H

#include <vector>
#include <string>
#include <glm/glm.hpp>
//#include <glm/ext.hpp>

class AnimationNode;

class Animation
{
public:
	Animation(const std::string& name, float duration, float ticks_per_second);

	void addAnimationNode(AnimationNode& node);

	float getDuration() const { return duration_; }
	float getTicksPerSecond() const { return ticks_per_second_; }

	void setAsActiveAnimation();

	const std::vector<AnimationNode*>& getAnimationNodes() const { return animation_nodes_; }

	const std::string& getName() const { return name_; }

private:
	std::string name_;
	float duration_;
	float ticks_per_second_;

	std::vector<AnimationNode*> animation_nodes_;
};

#endif
