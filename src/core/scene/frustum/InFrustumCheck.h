#ifndef CORE_SCENE_FRUSTRUM_IN_FRUSTRUM_CHECK_H
#define CORE_SCENE_FRUSTRUM_IN_FRUSTRUM_CHECK_H

#include <sstream>

class Frustum;
class SceneNode;

class InFrustumCheck
{
public:
	//virtual bool isInsideFrustum(const SceneNode& leaf, const Frustum& frustrum) const = 0;
	virtual bool isInsideFrustum(const Frustum& frustrum) const = 0;

	//virtual void getDebug(const SceneNode& leaf, const Frustum& frustrum, std::stringstream& ss) const = 0;
private:

};

#endif
