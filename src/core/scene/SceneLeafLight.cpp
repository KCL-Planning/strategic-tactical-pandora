#include "SceneLeafLight.h"

#include "../light/Light.h"
#include "../renderer/Renderer.h"
#include "SceneNode.h"

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
	light_->preRender(parent_->getCompleteTransformation());
	renderer.visit(*this);
}

void SceneLeafLight::accept(SceneVisitor& visitor) const
{
	visitor.visit(*this);
}
