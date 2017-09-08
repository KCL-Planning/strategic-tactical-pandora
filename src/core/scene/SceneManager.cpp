#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <algorithm>

#include <glm/gtx/quaternion.hpp>

#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/SceneLeaf.h"
#include "dpengine/scene/SceneNode.h"
#include "dpengine/renderer/Renderer.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/collision/CollisionInfo.h"
#include "dpengine/entities/camera/Camera.h"
#include "dpengine/scene/Material.h"
#include "dpengine/scene/portal/Region.h"

namespace DreadedPE
{

SceneManager::SceneManager()
{
	glm::mat4 local_transformation(1.0f);
	root_ = new SceneNode(*this, NULL, local_transformation, glm::vec3(1, 1, 1), false);
}

SceneManager::~SceneManager()
{
	root_->destroy();
	root_->cleanup();
}

void SceneManager::destroy()
{
	for (SceneNode* node : root_->getChildren())
	{
		node->destroy();
	}
/*
	for (SceneLeaf* leaf : root_->getLeafs())
	{
		leaf->destroy();
	}
*/
	// Delete all regions.
	Region::deleteAllRegions();
	root_->cleanup();
}

void SceneManager::addUpdateableEntity(SceneNode& node)
{
	entities_to_add_or_remove_.push_back(std::make_pair(&node, true));
}

void SceneManager::removeUpdateableEntity(SceneNode& node)
{
	entities_to_add_or_remove_.push_back(std::make_pair(&node, false));
}

void SceneManager::tick(float dt)
{
	// Add / remove entities.
	for (std::vector<std::pair<SceneNode*, bool> >::const_iterator ci = entities_to_add_or_remove_.begin(); ci != entities_to_add_or_remove_.end(); ++ci)
	{
		SceneNode* entity = (*ci).first;
		std::vector<SceneNode*>::iterator entity_i = std::find(entities_to_update_.begin(), entities_to_update_.end(), entity);

		// Add an entity.
		if ((*ci).second)
		{
			if (entity_i == entities_to_update_.end())
			{
				entities_to_update_.push_back(entity);
			}
		}

		// Remove an entity.
		else
		{
			if (entity_i != entities_to_update_.end())
			{
				entities_to_update_.erase(entity_i);
			}
		}
	}
	entities_to_add_or_remove_.clear();

	for (std::vector<SceneNode*>::iterator i = entities_to_update_.begin(); i != entities_to_update_.end(); ++i)
	{
		(*i)->mark(true, true);
	}

	// Delete any entities.
	SceneNode::cleanup();
	root_->storeAndPrepare(dt);
}

void SceneManager::prepareForRendering(float p)
{
	for (std::vector<SceneNode*>::iterator i = entities_to_update_.begin(); i != entities_to_update_.end(); ++i)
	{
		(*i)->mark(true, true);
	}

	root_->updateInterpolationMatrix(p);
}

};
