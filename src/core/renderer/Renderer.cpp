#include "dpengine/renderer/Renderer.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/entities/camera/Camera.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneNode.h"

namespace DreadedPE
{

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

};
