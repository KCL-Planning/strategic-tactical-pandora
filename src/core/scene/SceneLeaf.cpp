#include "SceneLeaf.h"
#include "SceneNode.h"
#include "frustum/InFrustumCheck.h"

SceneLeaf::SceneLeaf(SceneNode& scene_node, InFrustumCheck* frustum_checker)
	: parent_(&scene_node), frustum_checker_(frustum_checker), is_visible_(true)
{
	parent_->addLeaf(*this);
}

bool SceneLeaf::isInFrustum(const Frustum& frustum) const
{
	if (frustum_checker_ == NULL)
	{
		return true;
	}

	return frustum_checker_->isInsideFrustum(frustum);
}

void SceneLeaf::remove()
{
	parent_->removeLeaf(*this);
}
