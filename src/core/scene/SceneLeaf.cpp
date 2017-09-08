#include "dpengine/scene/SceneLeaf.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/frustum/InFrustumCheck.h"

namespace DreadedPE
{

SceneLeaf::SceneLeaf(SceneNode& scene_node, InFrustumCheck* frustum_checker)
	: parent_(&scene_node), frustum_checker_(frustum_checker), is_visible_(true)
{
	parent_->addLeaf(*this);
}

SceneLeaf::~SceneLeaf()
{
	//delete frustum_checker_;
}

void SceneLeaf::updateInterpolationMatrix(float p)
{

}

bool SceneLeaf::isInFrustum(const Frustum& frustum) const
{
	if (frustum_checker_ == NULL)
	{
		return true;
	}

	return frustum_checker_->isInsideFrustum(frustum);
}

void SceneLeaf::setParent(SceneNode* parent)
{
	if (parent == parent_)
	{
		return;
	}
	else if (parent_ != NULL)
	{
		parent_->removeLeaf(*this);
	}
	parent_ = parent;
}
/*
void SceneLeaf::destroy()
{
	parent_->removeLeaf(*this);
}
*/
};
