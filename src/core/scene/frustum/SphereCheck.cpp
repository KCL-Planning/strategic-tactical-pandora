#include "SphereCheck.h"
#include "../SceneLeaf.h"
#include "../SceneNode.h"
#include "Frustum.h"

SphereCheck::SphereCheck(const SceneNode& node, float radius)
	: scene_node_(&node), radius_(radius)
{

}

bool SphereCheck::isInsideFrustum(const Frustum& frustum) const
{
	const glm::vec3& loc = scene_node_->getGlobalLocation();
	
	for (std::vector<glm::vec4>::const_iterator ci = frustum.getPlanes().begin(); ci != frustum.getPlanes().end(); ++ci)
	{
		const glm::vec4& plane = *ci;
		if (plane.x * loc.x + plane.y * loc.y + plane.z * loc.z + plane.w <= -radius_)
		{
			return false;
		}
	}

	return true;
}
