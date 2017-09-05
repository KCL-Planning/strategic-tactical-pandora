#ifdef _WIN32
#include <windows.h>
#endif

#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "SceneNode.h"
#include "SceneLeaf.h"
#include "../math/BoundedBox.h"
#include "../entities/Entity.h"
#include "SceneManager.h"
#include "../entities/behaviours/Behaviour.h"

// DEBUG.
#include "../collision/BoxCollision.h"
#include "../../shapes/Cube.h"
#include "SceneLeafModel.h"
#include "Material.h"
#include "../shaders/BasicShadowShader.h"
#include "portal/Region.h"
#include "portal/Portal.h"
#include "../texture/Texture.h"
#include "../texture/TargaTexture.h"

Material* SceneNode::bright_material_ = NULL;

SceneNode::SceneNode(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, bool init_children)
	: scene_manager_(&scene_manager), local_transformation_(transformation), complete_transformation_(1.0), bounded_collision_box_(NULL), frustum_checker_(NULL), current_region_(NULL), ignore_rotations_(false)
{
	parent_ = parent;
	if (init_children)
	{
		if (parent != NULL)
		{
			parent->children_.push_back(this);
			parent_ = parent;
		}
		else
		{
			scene_manager.getRoot().children_.push_back(this);
			parent_ = &scene_manager.getRoot();
		}
	}

	if (bright_material_ == NULL)
	{
		Texture* texture = TargaTexture::loadTexture("data/textures/gold.tga");
		MaterialLightProperty* bright_ambient = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
		MaterialLightProperty* bright_diffuse = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
		MaterialLightProperty* bright_specular = new MaterialLightProperty(0.0, 0.0, 0.0, 1.0);
		MaterialLightProperty* bright_emmisive = new MaterialLightProperty(1.0, 1.0, 1.0, 1.0);

		bright_material_ = new Material(*bright_ambient, *bright_diffuse, *bright_specular, *bright_emmisive);
		bright_material_->add2DTexture(*texture);
	}
	updateTransformations();
}

SceneNode::~SceneNode()
{
	if (parent_ != NULL)
	{
		parent_->removeChild(*this);
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		delete *ci;
	}
	for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
	{
		delete *ci;
	}
}
/*
void SceneNode::updateVisibility()
{
	// If we have moved, check if we are in a separate region.
	bool update_visibility = false;
	if (current_region_ == NULL)
	{
		update_visibility = true;
		current_region_ = Region::findRegionGlobal(getGlobalLocation());
	}
	else if (getCompleteTransformation() != previous_transform_)
	{
		update_visibility = true;
		
		//Region* new_region = scene_manager_->getRoot().findRegion(getGlobalLocation());
		Region* new_region = current_region_->findRegion(getGlobalLocation());
		current_region_ = new_region;
	}
	
	if (update_visibility)
	{
		// Check in what regions -- others than the one it is in -- the entity is visible.
		std::vector<Region*> new_regions_visible_from;
		if (current_region_ != NULL)
		{
			std::set<Region*> closed_list, open_list;
			open_list.insert(current_region_);
			
			while (open_list.size() > 0)
			{
				Region* region = *open_list.begin();
				open_list.erase(open_list.begin());
				closed_list.insert(region);
				
				if (region->isVisibleIn(*this))
				{
					new_regions_visible_from.push_back(region);
				}
				
				for (std::vector<Portal*>::const_iterator ci = region->getPortals().begin(); ci != region->getPortals().end(); ++ci)
				{
					const Portal* portal = *ci;
					if (closed_list.count(&portal->getFromRegion()) != 1)
					{
						open_list.insert(&portal->getFromRegion());
					}
					
					if (closed_list.count(&portal->getToRegion()) != 1)
					{
						open_list.insert(&portal->getToRegion());
					}
				}
			}
		}

		// Update the existing regions the entity was visible from.
		for (int i = regions_visible_from_.size() - 1; i > -1; --i)
		{
			Region* region = regions_visible_from_[i];

			// The entity is no longer visible from the region!
			std::vector<Region*>::iterator i_find = std::find(new_regions_visible_from.begin(), new_regions_visible_from.end(), region);
			if (i_find == new_regions_visible_from.end())
			{
				region->removeVisibleEntityFromOtherRegion(*this);
				regions_visible_from_.erase(regions_visible_from_.begin() + i);
				continue;
			}

			new_regions_visible_from.erase(i_find);
		}

		// Now add all the regions where it was not visible from before.
		for (std::vector<Region*>::const_iterator ci = new_regions_visible_from.begin(); ci != new_regions_visible_from.end(); ++ci)
		{
			Region* new_region = *ci;
			new_region->addVisibleEntityFromOtherRegion(*this);
			regions_visible_from_.push_back(new_region);
		}
	}
}
*/
void SceneNode::updateChildren()
{
	// Delete and insert children of this node.
	for (std::vector<SceneNode*>::const_iterator ci = children_to_add_.begin(); ci != children_to_add_.end(); ++ci)
	{
		children_.push_back(*ci);
	}
	children_to_add_.clear();

	for (std::vector<const SceneNode*>::const_iterator ci = children_to_remove_.begin(); ci != children_to_remove_.end(); ++ci)
	{
		std::vector<SceneNode*>::iterator i = std::find(children_.begin(), children_.end(), *ci);
		if (i != children_.end())
		{
			children_.erase(i);
		}
	}
	children_to_remove_.clear();
	
	for (std::vector<const SceneLeaf*>::const_iterator ci = leafs_to_remove_.begin(); ci != leafs_to_remove_.end(); ++ci)
	{
		const SceneLeaf* leaf = *ci;
		
		std::vector<SceneLeaf*>::iterator i = std::find(leafs_.begin(), leafs_.end(), *ci);
		if (i != leafs_.end())
		{
			leafs_.erase(i);
			delete leaf;
		}
	}
	leafs_to_remove_.clear();
}

void SceneNode::prepare(float dt)
{
	if (marked_for_update_)
	{
		for (std::vector<Behaviour*>::const_iterator ci = behaviours_.begin(); ci != behaviours_.end(); ++ci)
		{
			(*ci)->prepare(dt);
		}
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
	//updateVisibility();
	updateChildren();
	
	if (marked_for_visit_)
	{
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			(*ci)->prepare(dt);
		}
		for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
		{
			(*ci)->prepare(dt);
		}
	}

	marked_for_update_ = false;
	marked_for_visit_ = false;
	previous_transform_ = getCompleteTransformation();
}

void SceneNode::mark(bool update, bool visit)
{
	if (update)
	{
		marked_for_update_ = true;

		// Update the children as well.
		for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
		{
			(*ci)->mark(true, true);
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
		complete_transformation_ = parent_->complete_transformation_ * local_transformation_;
		
		if (ignore_rotations_)
		{
			complete_transformation_ = glm::translate(local_transformation_, parent_->getGlobalLocation());
		}
	}
	else
	{
		complete_transformation_ = local_transformation_;
	}
}

void SceneNode::addBehaviour(Behaviour& behaviour)
{
	behaviours_.push_back(&behaviour);
}

void SceneNode::deleteBehaviour(const Behaviour& behaviour)
{
	for (std::vector<Behaviour*>::iterator ci = behaviours_.begin(); ci != behaviours_.end(); ++ci)
	{
		if (*ci == &behaviour)
		{
			behaviours_.erase(ci);
			return;
		}
	}
}

void SceneNode::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls)
{
	++nr_calls;
	if (frustum_checker_ != NULL && !frustum_checker_->isInsideFrustum(frustum))
	{
		return;
	}
	
	// Prepare all the children and entities which are about to be rendered.
	// TODO: We should not presome that all entities should be prepared, this will be fixed later when
	// we use a visitor's pattern or something similar such that the tree can be traversed and only the 
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
	//	std::cout << "SceneNode::setTransformation: (" << getLocalLocation().x << ", " << getLocalLocation().y << ", " << getLocalLocation().z << ")" << std::endl;
}

void SceneNode::destroy()
{
	if (parent_ != NULL)
	{
		parent_->removeChild(*this);
	}
	delete this;
}

void SceneNode::initialiseBoundedBoxes(const std::vector<const SceneNode*>& excluded_nodes)
{
	updateTransformations();
	if (frustum_checker_ != NULL)
	{
		delete frustum_checker_;
	}
	
	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		(*ci)->initialiseBoundedBoxes(excluded_nodes);
	}

	for (std::vector<SceneLeaf*>::const_iterator ci = leafs_.begin(); ci != leafs_.end(); ++ci)
	{
		(*ci)->initialiseFrustrumChecker();
	}

	frustum_checker_ = new BoundedBox(*this, true, excluded_nodes);
	bounded_collision_box_ = new BoundedBox(*this, false, excluded_nodes);

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

void SceneNode::compress()
{

}

void SceneNode::setCollisionBoundedBox(BoundedBox& box)
{
	bounded_collision_box_ = &box;
}

bool SceneNode::getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	//ss << "Process..." << (process_children ? " Children!" : "No child...") << "|";
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<BoxCollision*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			BoxCollision* collision_box = *ci;

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

bool SceneNode::doesCollide(Entity& entity, CollisionInfo& info) const
{
	bool process_children = bounded_collision_box_ == NULL;

	// Check if the entity is within this node.
	if (!process_children)
	{
		for (std::vector<BoxCollision*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			BoxCollision* collision_box = *ci;

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
	mark(false, true);
	children_to_remove_.push_back(&scene_node);
}

void SceneNode::addChild(SceneNode& child)
{
	mark(false, true);
	child.parent_ = this;
	children_to_add_.push_back(&child);
}

void SceneNode::addLeaf(SceneLeaf& leaf)
{
	mark(false, true);
	leafs_.push_back(&leaf);
	leaf.setParent(this);
}

void SceneNode::removeLeaf(SceneLeaf& leaf)
{
	mark(false, true);
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
