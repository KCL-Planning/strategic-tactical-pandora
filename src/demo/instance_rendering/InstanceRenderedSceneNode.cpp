#include <iostream>

#include "InstanceRenderedSceneNode.h"
#include "SceneLeafInstanced.h"
#include "InstanceRenderedShape.h"
#include "../../core/math/BoundedBox.h"

InstanceRenderedSceneNode::InstanceRenderedSceneNode(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& local_transformation)
	: SceneNode(scene_manager, parent, local_transformation), instanced_scene_leaf_(NULL), update_matrixes_(false)
{
	
}
	
void InstanceRenderedSceneNode::prepare(float dt)
{
	SceneNode::prepare(dt);
	if (update_matrixes_)
	{
		updateMatrixes();
		update_matrixes_ = false;
	}
}

void InstanceRenderedSceneNode::addChild(SceneNode& child)
{
	SceneNode::addChild(child);
	update_matrixes_ = true;
}

void InstanceRenderedSceneNode::removeChild(SceneNode& child)
{
	SceneNode::removeChild(child);
	update_matrixes_ = true;
}

void InstanceRenderedSceneNode::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls)
{
	if (instanced_scene_leaf_ == NULL)
	{
		return;
	}

	++nr_calls;
	if (frustum_checker_ != NULL && !frustum_checker_->isInsideFrustum(frustum))
	{
		return;
	}	
	renderer.visit(*instanced_scene_leaf_);
}

void InstanceRenderedSceneNode::updateBoundingBoxesBoundaries(bool recursive)
{
	updateMatrixes();
}

void InstanceRenderedSceneNode::updateMatrixes()
{
	if (instanced_scene_leaf_ == NULL)
	{
		return;
	}
	SceneNode::updateBoundingBoxesBoundaries(true);
	InstanceRenderedShape& instance_shape = instanced_scene_leaf_->getInstancedShape();
	std::vector<glm::mat4> model_matrixes;
	
	// Update the model matrixes of the leaf.
	std::vector<SceneNode*> to_update(children_);
	for (unsigned int i = 0; i < to_update.size(); ++i)
	{
		SceneNode* node = to_update[i];
		model_matrixes.push_back(node->getCompleteTransformation());
		to_update.insert(to_update.end(), node->getChildren().begin(), node->getChildren().end());
	}
	
	instance_shape.setModelMatrixes(model_matrixes);
}
