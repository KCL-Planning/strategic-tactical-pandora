#include "dpengine/scene/SceneLeafLight.h"

#include "dpengine/light/Light.h"
#include "dpengine/renderer/Renderer.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/collision/ConvexPolygon.h"

namespace DreadedPE
{

SceneLeafLight::SceneLeafLight(SceneNode& parent, InFrustumCheck* frustum_checker, Light& light)
	: SceneLeaf(parent, frustum_checker), light_(&light), total_time_(0.0f)
{

}

SceneLeafLight::~SceneLeafLight()
{
	delete light_;
}

void SceneLeafLight::prepare(float dt)
{

}

void SceneLeafLight::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights)
{
	if (!process_lights) return;
	light_->preRender(parent_->getInterpolatedMatrix());
	renderer.visit(*this);
}

void SceneLeafLight::accept(SceneVisitor& visitor) const
{
	visitor.visit(*this);
}

void SceneLeafLight::initialiseFrustrumChecker()
{
	if (frustum_checker_ != NULL)
	{
		delete frustum_checker_;
	}
	float light_length = light_->getFarPlane();
	float width = light_length * tan(light_->getAngle()) * 2;

	//frustum_checker_ = new ConvexPolygon(*parent_, width, width, light_length);
	frustum_checker_ = new ConvexPolygon(*parent_, 100, 100, 100);
}

};
