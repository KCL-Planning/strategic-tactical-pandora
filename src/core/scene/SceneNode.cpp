#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_interpolation.hpp>

#include "dpengine/scene/SceneNode.h"
#include "dpengine/scene/SceneLeaf.h"
#include "dpengine/entities/Entity.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/events/SceneNodeDestroyedListener.h"

// DEBUG.
#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/shapes/Cube.h"
#include "dpengine/scene/SceneLeafModel.h"
#include "dpengine/scene/Material.h"
#include "dpengine/shaders/BasicShadowShader.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/scene/portal/Portal.h"
#include "dpengine/texture/Texture.h"
#include "dpengine/texture/TargaTexture.h"

#include "dpengine/math/Plane.h"

namespace DreadedPE
{

std::vector<SceneNode*> SceneNode::to_delete_list_;

SceneNode::SceneNode(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, const glm::vec3& scaling, bool init_children)
	: is_alive_(true), is_marked_for_removal_(false), scene_manager_(&scene_manager), local_transformation_(transformation), complete_transformation_(1.0), complete_transformation_no_scaling_(1.0), local_scaling_(scaling), scaling_(scaling), parent_(parent), bounded_collision_box_(NULL), frustum_checker_(NULL), current_region_(NULL), marked_for_update_(true), marked_for_visit_(true), previous_transform_(local_transformation_), previous_transform_no_scaling_(local_transformation_), previous_scaling_(scaling), interpolated_transform_(1.0f), ignore_rotations_(false)
{
	if (parent != NULL)
	{
		if (init_children)
		{
			parent->children_.push_back(this);
		}
		else
		{
			parent->addChild(*this);
		}
	}
	updateTransformations();
	
	// Initialise the interpolated and previous transforms / scaling.
	interpolated_transform_ = complete_transformation_;
	previous_transform_ = complete_transformation_;
	previous_scaling_ = scaling_;
	previous_transform_no_scaling_ = complete_transformation_no_scaling_;
}

SceneNode::~SceneNode()
{
	/**
	 * Remove this node from the parent's children list.
	 * NOTE: This is already done in the destroy() method.
	 */
	if (parent_ != NULL)
	{
		std::vector<SceneNode*>::iterator i = std::find(parent_->children_.begin(), parent_->children_.end(), this);
		if (i != parent_->children_.end())
		{
			parent_->children_.erase(i);
		}
	}

	/**
	 * Delete all our children.
	 */
	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		SceneNode* node = *ci;
		if (Entity::isToBeDeleted(*node))
		{
			continue;
		}
		delete *ci;
	}

	/**
	 * Delete all our leaf nodes.
	 */
	for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
	{
		delete *ci;
	}

	/**
	 * Delete the frustrum and collision convex shapes.
	 */
	delete frustum_checker_;
	delete bounded_collision_box_;
}

void SceneNode::updateChildren()
{
	// Delete and insert children of this node.
	for (std::vector<const SceneNode*>::const_iterator ci = children_to_remove_.begin(); ci != children_to_remove_.end(); ++ci)
	{
		std::vector<SceneNode*>::iterator i = std::find(children_.begin(), children_.end(), *ci);
		if (i != children_.end())
		{
			children_.erase(i);
		}
	}
	children_to_remove_.clear();

	for (std::vector<SceneNode*>::const_iterator ci = children_to_add_.begin(); ci != children_to_add_.end(); ++ci)
	{
		children_.push_back(*ci);
	}
	children_to_add_.clear();
	
	for (std::vector<const SceneLeaf*>::const_iterator ci = leafs_to_remove_.begin(); ci != leafs_to_remove_.end(); ++ci)
	{
		const SceneLeaf* leaf = *ci;
		
		std::vector<SceneLeaf*>::iterator i = std::find(leafs_.begin(), leafs_.end(), leaf);
		if (i != leafs_.end())
		{
			leafs_.erase(i);
		}
	}
	leafs_to_remove_.clear();
}

void SceneNode::prepare(float dt)
{
	if (!is_alive_)
	{
		return;
	}
	if (marked_for_update_)
	{
		marked_for_update_ = false;
		updateChildren();
		updateTransformations();
	}
	if (transformationIsUpdated())
	{
		if (current_region_ != NULL)
		{
			current_region_ = current_region_->findRegion(getGlobalLocation());
		}
		else
		{
			current_region_ = Region::findRegionGlobal(getGlobalLocation());
		}
		Region::updateVisibility(*this);
	}
	
	if (marked_for_visit_)
	{
		marked_for_visit_ = false;
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			(*ci)->storeAndPrepare(dt);
		}
		for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
		{
			(*ci)->prepare(dt);
		}
	}
}

void SceneNode::storeAndPrepare(float dt)
{
	previous_transform_ = getCompleteTransformation();
	previous_transform_no_scaling_ = complete_transformation_no_scaling_;
	previous_scaling_ = scaling_;
	prepare(dt);
}

void SceneNode::mark(bool update, bool visit)
{
	if (update)
	{
		marked_for_update_ = true;

		// Update the children as well.
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			SceneNode* child = *ci;
			if (child->is_alive_)
			{
				(*ci)->mark(true, true);
			}
		}
	}
	if (visit)
	{
		marked_for_visit_ = true;
	}

	if (parent_ != NULL)
	{
		parent_->mark(false, true);
	}
}

void SceneNode::updateTransformations()
{
	// Cache the transformation of the local transformation and all the transformation done above.
	if (parent_ != NULL)
	{
		scaling_ = parent_->scaling_ * local_scaling_;
		if (ignore_rotations_)
		{
			complete_transformation_no_scaling_ = glm::translate(local_transformation_, parent_->getGlobalLocation());
		}
		else
		{
			complete_transformation_no_scaling_ = parent_->complete_transformation_ * local_transformation_;
		}
	}
	else
	{
		scaling_ = local_scaling_;
		complete_transformation_no_scaling_ = local_transformation_;
	}
	complete_transformation_ = glm::scale(complete_transformation_no_scaling_, scaling_);
}

void SceneNode::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls)
{
	if (!isAlive())
	{
		return;
	}

	++nr_calls;
	if (frustum_checker_ != NULL && !frustum_checker_->isInsideFrustum(frustum))
	{
		return;
	}
	
	// Prepare all the children and entities which are about to be rendered.

	// relevent nodes will be rendered and prerendered.
	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		(*ci)->preRender(frustum, camera_position, renderer, process_lights, nr_calls);
	}
	for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
	{
		if ((*ci)->isVisible())
		{
			(*ci)->preRender(frustum, camera_position, renderer, process_lights);
		}
	}
}

void SceneNode::setTransformation(const glm::mat4& local_transformation, bool updateBoundaries)
{
	local_transformation_ = local_transformation;
	if (updateBoundaries)
	{
		updateBoundingBoxesBoundaries(false);
	}
	mark(true, true);
}

void SceneNode::updateInterpolationMatrix(float p)
{
	if (marked_for_update_ || marked_for_visit_)
	{
		interpolated_transform_ = glm::scale(glm::interpolate(previous_transform_no_scaling_, complete_transformation_no_scaling_, p), previous_scaling_ * (1 - p) + scaling_ * p);

		for (SceneNode* child : children_)
		{
			child->updateInterpolationMatrix(p);
		}
		for (SceneLeaf* leaf : leafs_)
		{
			leaf->updateInterpolationMatrix(p);
		}
	}
}

void SceneNode::destroy()
{
	if (is_marked_for_removal_) return;
	scene_manager_->removeUpdateableEntity(*this);
	if (parent_ != NULL)
	{
		parent_->removeChild(*this);
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		(*ci)->destroy();
	}

	Region::destroy(*this);
	is_alive_ = false;
	is_marked_for_removal_ = true;
	mark(true, true);

	to_delete_list_.push_back(this);

	for (SceneNodeDestroyedListener* listener : listeners_)
	{
		listener->sceneNodeDestroyed(*this);
	}
}

void SceneNode::cleanup()
{
	for (std::vector<SceneNode*>::const_iterator ci = to_delete_list_.begin(); ci != to_delete_list_.end(); ++ci)
	{
		SceneNode* scene_node = *ci;
		if (scene_node->getParent() != NULL)
		{
			scene_node->getParent()->updateChildren();
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = to_delete_list_.begin(); ci != to_delete_list_.end(); ++ci)
	{
		SceneNode* scene_node = *ci;
		delete scene_node;
	}
	to_delete_list_.clear();
}

void SceneNode::initialiseBoundedBoxes(const std::vector<const SceneNode*>& excluded_nodes)
{
	updateTransformations();
	previous_transform_ = complete_transformation_;
	delete frustum_checker_;
	delete bounded_collision_box_;
	
	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		(*ci)->initialiseBoundedBoxes(excluded_nodes);
	}

	for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
	{
		(*ci)->initialiseFrustrumChecker();
	}

	frustum_checker_ = new ConvexPolygon(*this, true, excluded_nodes);
	bounded_collision_box_ = new ConvexPolygon(*this, false, excluded_nodes);

#ifdef HORROR_GAME_ENABLE_DEBUG
	if (std::find(excluded_nodes.begin(), excluded_nodes.end(), this) == excluded_nodes.end())// && dynamic_cast<Region*>(this) != NULL)
	{
		// To visualise the bounding box, we add new leafs to this node.
		const glm::vec3& centre_point = frustum_checker_->getCentrePoint();

		SceneNode* parent = this;
		if (centre_point != glm::vec3(0.0f, 0.0f, 0.0f))
		{
			parent = new SceneNode(*scene_manager_, this, glm::translate(glm::mat4(1.0), centre_point));
		}
		
		float box_width = frustum_checker_->getPoints()[0].x - frustum_checker_->getPoints()[1].x;
		float box_height = frustum_checker_->getPoints()[0].y - frustum_checker_->getPoints()[2].y;
		float box_depth = frustum_checker_->getPoints()[0].z - frustum_checker_->getPoints()[4].z;
		Cube* collision_cube = new Cube(std::abs(box_width), std::abs(box_height), std::abs(box_depth));

		SceneLeafModel* collision_box_leaf = new SceneLeafModel(*parent, NULL, *collision_cube, *bright_material_, BasicShadowShader::getShader(), false, true, MODEL_TYPE::COLLISION);
	}
#endif
}

bool SceneNode::getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	//ss << "Process..." << (process_children ? " Children!" : "No child...") << "|";
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			const ConvexPolygon* collision_box = *ci;

			//ss << "Collision box: (" << bounded_collision_box_->getPoints()[0].x << ", " << bounded_collision_box_->getPoints()[0].y << ", " << bounded_collision_box_->getPoints()[0].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[2].x << ", " << bounded_collision_box_->getPoints()[2].y << ", " << bounded_collision_box_->getPoints()[2].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[4].x << ", " << bounded_collision_box_->getPoints()[4].y << ", " << bounded_collision_box_->getPoints()[4].z << ")|";

			if (collision_box->isInside(*bounded_collision_box_))
			{
				//ss << "Is inside!|";
				process_children = true;
				break;
			}
		}
	}
	
	bool collision_detected = false;
	if (process_children)
	{
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->getCollisions(entity, info))
			{
				collision_detected = true;
			}
		}
	}
	else
	{
		//ss << "Is not inside!|";
	}
	return collision_detected;
}

bool SceneNode::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		if (bounded_collision_box_->isInside(begin) || bounded_collision_box_->isInside(end) || bounded_collision_box_->doesCollide(begin, end))
		{
			process_children = true;
		}
	}
	
	bool collision_detected = false;
	if (process_children)
	{
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->getCollisions(entity, begin, end, info))
			{
				collision_detected = true;
			}
		}
	}
	return collision_detected;
}

bool SceneNode::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, float effective_width, std::vector<CollisionInfo>& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		if (bounded_collision_box_->isInside(begin) || bounded_collision_box_->isInside(end) || bounded_collision_box_->doesCollide(begin, end, effective_width))
		{
			process_children = true;
		}
	}

	if (process_children)
	{
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->getCollisions(entity, begin, end, effective_width, info))
			{
				return true;
			}
		}
	}
	return false;
}

bool SceneNode::doesCollide(Entity& entity, CollisionInfo& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			const ConvexPolygon* collision_box = *ci;

			if (bounded_collision_box_->isInside(*collision_box))
			{
				process_children = true;
				break;
			}
		}
	}
	
	if (process_children)
	{	
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, info))
			{
				return true;
			}
		}
	}
	return false;
}

bool SceneNode::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		if (bounded_collision_box_->isInside(begin) || bounded_collision_box_->isInside(end) || bounded_collision_box_->doesCollide(begin, end))
		{
			process_children = true;
		}
	}
	
	if (process_children)
	{	
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				return true;
			}
		}
	}
	return false;
}

bool SceneNode::doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		if (bounded_collision_box_->isInside(begin) || bounded_collision_box_->isInside(end) || bounded_collision_box_->doesCollide(begin, end, effective_width))
		{
			process_children = true;
		}
	}
	
	if (process_children)
	{	
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, begin, end, effective_width))
			{
				return true;
			}
		}
	}
	return false;
}

void SceneNode::setParent(SceneNode* new_parent)
{
	if (parent_ != NULL)
	{
		parent_->removeChild(*this);
	}
	
	parent_ = new_parent;
	if (parent_ != NULL)
	{
		parent_->addChild(*this);
	}
	mark(true, true);
}

bool SceneNode::doesCollide(Entity* entity, const glm::vec3& point) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		if (bounded_collision_box_->isInside(point))
		{
			process_children = true;
		}
	}
	
	if (process_children)
	{	
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, point))
			{
				return true;
			}
		}
	}
	return false;
}

void SceneNode::removeChild(const SceneNode& scene_node)
{
	mark(true, true);
	children_to_remove_.push_back(&scene_node);
}

void SceneNode::addChild(SceneNode& child)
{
	mark(true, true);
	child.parent_ = this;
	children_to_add_.push_back(&child);
}

void SceneNode::addLeaf(SceneLeaf& leaf)
{
	mark(true, true);
	leafs_.push_back(&leaf);
	leaf.setParent(this);
}

void SceneNode::removeLeaf(SceneLeaf& leaf)
{
	mark(true, true);
	leafs_to_remove_.push_back(&leaf);
}

void SceneNode::updateBoundingBoxesBoundaries(bool recursive)
{
	bool update_parent = false;
	if (frustum_checker_ != NULL)
	{
		if (frustum_checker_->update(recursive, true))
		{
			update_parent = true;
		}
	}

	if (bounded_collision_box_ != NULL)
	{
		if (frustum_checker_->update(recursive, false))
		{
			update_parent = true;
		}
	}

	if (update_parent && parent_ != NULL)
	{
		parent_->updateBoundingBoxesBoundaries(false);
	}
}

void SceneNode::addListener(SceneNodeDestroyedListener& listener)
{
	listeners_.push_back(&listener);
}

void SceneNode::removeListener(SceneNodeDestroyedListener& listener)
{
	for (int i = listeners_.size() - 1; i >= 0; --i)
	{
		if (listeners_[i] == &listener)
		{
			listeners_.erase(listeners_.begin() + i);
		}
	}
}

};
