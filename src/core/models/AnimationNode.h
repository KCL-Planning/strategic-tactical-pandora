#ifndef CORE_MODELS_ANIMATION_NODE_H
#define CORE_MODELS_ANIMATION_NODE_H

#include <vector>
#include <glm/glm.hpp>
#include <glm/ext.hpp>

class Animation;
class BoneNode;

class AnimationNode
{
public:
	AnimationNode(Animation& animation, BoneNode& node, const std::vector<std::pair<glm::vec3, float> >& position, const std::vector<std::pair<glm::fquat, float> >& rotation, const std::vector<std::pair<glm::vec3, float> >& scaling);

	Animation& getAnimation() const { return *animation_; }

	BoneNode& getNode() const { return *node_; }

	void updateBoneInfo();

	const std::vector<std::pair<glm::vec3, float> >& getPositions() const { return position_; }
	const std::vector<std::pair<glm::fquat, float> >& getRotations() const { return rotation_; }
	const std::vector<std::pair<glm::vec3, float> >& getScalings() const { return scaling_; }

private:

	Animation* animation_;

	BoneNode* node_;

	std::vector<std::pair<glm::vec3, float> > position_;
	std::vector<std::pair<glm::fquat, float> > rotation_;
	std::vector<std::pair<glm::vec3, float> > scaling_;
};

#endif
