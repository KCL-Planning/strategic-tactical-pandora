#include "Renderer.h"
#include "../scene/portal/Region.h"
#include "../entities/camera/Camera.h"
#include "../scene/SceneManager.h"
#include "../scene/SceneNode.h"

Renderer::Renderer()
	: last_region_(NULL)
{

}

Region* Renderer::getRegionToRenderFrom(const glm::vec3& render_location, const SceneManager& scene_manager)
{
	last_region_ = NULL;
	if (last_region_ != NULL)
	{
		last_region_ = last_region_->findRegion(render_location);
	}
	else
	{
		last_region_ = Region::findRegionGlobal(render_location);
	}
	return last_region_;
}
