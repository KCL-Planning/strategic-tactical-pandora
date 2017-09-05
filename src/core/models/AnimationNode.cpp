#include "AnimationNode.h"

#include "Bone.h"
#include "BoneNode.h"

AnimationNode::AnimationNode(Animation& animation, BoneNode& node, const std::vector<std::pair<glm::vec3, float> >& position, const std::vector<std::pair<glm::fquat, float> >& rotation, const std::vector<std::pair<glm::vec3, float> >& scaling)
	: animation_(&animation), node_(&node), position_(position), rotation_(rotation), scaling_(scaling)
{
//	node.setAnimationNode(*this);
}

void AnimationNode::updateBoneInfo()
{
	node_->setActiveAnimationNode(this);
}