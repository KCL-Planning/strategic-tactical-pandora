#ifndef CORE_SCENE_FRUSTRUM_SPHERE_CHECK_H
#define CORE_SCENE_FRUSTRUM_SPHERE_CHECK_H

//#include <glm/glm.hpp>

#include "InFrustumCheck.h"
class Frustum;

class SphereCheck : public InFrustumCheck
{
public:
	SphereCheck(const SceneNode& node, float radius);

	bool isInsideFrustum(const Frustum& frustum) const;

	void getDebug(const SceneNode& leaf, const Frustum& frustrum, std::stringstream& ss) const { }

private:
	const SceneNode* scene_node_;
	float radius_;
};

#endif
