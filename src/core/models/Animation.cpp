#include "dpengine/models/Animation.h"
#include "dpengine/models/AnimationNode.h"

namespace DreadedPE
{

Animation::Animation(const std::string& name, float duration, float ticks_per_second)
	: name_(name), duration_(duration), ticks_per_second_(ticks_per_second)
{

}

void Animation::addAnimationNode(AnimationNode& node)
{
	animation_nodes_.push_back(&node);
}

void Animation::setAsActiveAnimation()
{
	for (std::vector<AnimationNode*>::const_iterator ci = animation_nodes_.begin(); ci != animation_nodes_.end(); ++ci)
	{
		(*ci)->updateBoneInfo();
	}
}

};

