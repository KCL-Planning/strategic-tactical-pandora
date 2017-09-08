#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

#include "dpengine/entities/Entity.h"
#include "dpengine/scene/events/SceneNodeDestroyedListener.h"
#include "dpengine/collision/ConvexPolygon.h"
#include "dpengine/math/Plane.h"
#include "dpengine/scene/SceneManager.h"
#include "dpengine/scene/portal/Region.h"
#include "dpengine/collision/CollisionInfo.h"

namespace DreadedPE
{

	Entity::Entity(SceneManager& scene_manager, SceneNode* parent, const glm::mat4& transformation, ENTITY_TYPE type, const std::string& name, const glm::vec3& scaling, bool init_children)
	: SceneNode(scene_manager, parent, transformation, scaling, init_children), type_(type), name_(name)//, region_(NULL)
{
	
}

Entity::~Entity()
{
	for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
	{
		delete *ci;
	}
}

void Entity::getBoundedCollisionBox(const glm::mat4& transform_matrix, float& min_x, float& max_x, float& min_y, float& max_y, float& min_z, float& max_z) const
{
	for (std::vector<const ConvexPolygon*>::const_iterator ci = getCollisions().begin(); ci != getCollisions().end(); ++ci)
	{
		const ConvexPolygon* polygon = *ci;
		for (std::vector<const Plane*>::const_iterator ci = polygon->getPlanes().begin(); ci != polygon->getPlanes().end(); ++ci)
		{
			const Plane* plane = *ci;
			for (std::vector<glm::vec3>::const_iterator ci = plane->getPoints().begin(); ci != plane->getPoints().end(); ++ci)
			{
				glm::vec4 point = transform_matrix * glm::vec4(*ci, 1.0);
				min_x = std::min(min_x, point.x);
				max_x = std::max(max_x, point.x);
				min_y = std::min(min_y, point.y);
				max_y = std::max(max_y, point.y);
				min_z = std::min(min_z, point.z);
				max_z = std::max(max_z, point.z);
			}
		}
	}
}

void Entity::prepare(float dt)
{
	if (!is_alive_)
	{
		return;
	}

	// Update the collision detection in regions.
	updateTransformations();
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
		Region::updateCollidableEntity(*this);
	}

	SceneNode::prepare(dt);
}

void Entity::addCollision(const ConvexPolygon& collision)
{
	collisions_.push_back(&collision);
}

bool Entity::getCollisions(Entity& entity, std::vector<CollisionInfo>& infos) const
{
	if (!entity.is_alive_ || !is_alive_)
	{
		return false;
	}

	//ss << name_ << "-";
	bool process_children = bounded_collision_box_ == NULL;
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			const ConvexPolygon* collision_box = *ci;

			//ss << "Collision box: (" << bounded_collision_box_->getPoints()[0].x << ", " << bounded_collision_box_->getPoints()[0].y << ", " << bounded_collision_box_->getPoints()[0].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[2].x << ", " << bounded_collision_box_->getPoints()[2].y << ", " << bounded_collision_box_->getPoints()[2].z << ")";
			//ss << " - (" << bounded_collision_box_->getPoints()[4].x << ", " << bounded_collision_box_->getPoints()[4].y << ", " << bounded_collision_box_->getPoints()[4].z << ")|";

			if (bounded_collision_box_->isInside(*collision_box))
			{
				//ss << "Is inside!|";
				process_children = true;
				break;
			}
		}
	}
	if (!process_children)
	{
		return false;
	}

	bool collision_detected = false;
	if (this != &entity)
	{
		CollisionInfo info;
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			const ConvexPolygon* lhs_c = *ci;
			for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.collisions_.begin(); ci != entity.collisions_.end(); ++ci)
			{
				const ConvexPolygon* rhs_c = *ci;

				if (lhs_c->doesCollide(*rhs_c, info))
				{
					infos.push_back(info);
					collision_detected = true;
				}

				else if (rhs_c->doesCollide(*lhs_c, info))
				{
					infos.push_back(info);
					collision_detected = true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->getCollisions(entity, infos))
		{
			collision_detected = true;
		}
	}

	return collision_detected;
}

bool Entity::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& infos) const
{
	if (!entity.is_alive_ || !is_alive_)
	{
		return false;
	}

	bool collision_detected = false;
	if (this != &entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			CollisionInfo info;
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				infos.push_back(info);
				collision_detected = true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		SceneNode* scene_node = *ci;
		if (scene_node->getCollisions(entity, begin, end, infos))
		{
			collision_detected = true;
		}
	}
	return collision_detected;
}

bool Entity::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, float effective_width, std::vector<CollisionInfo>& infos) const
{
	if (!entity.is_alive_ || !is_alive_)
	{
		return false;
	}

	bool collision_detected = false;
	if (this != &entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			CollisionInfo info;
			if ((*ci)->doesCollide(entity, begin, end, effective_width, info))
			{
				infos.push_back(info);
				collision_detected = true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		SceneNode* scene_node = *ci;
		if (scene_node->getCollisions(entity, begin, end, effective_width, infos))
		{
			collision_detected = true;
		}
	}
	return collision_detected;
}

bool Entity::doesCollide(Entity& entity, CollisionInfo& info) const
{
	if (!entity.is_alive_ || entity.type_ == PASSABLE_DOES_COLLIDE ||
	    !is_alive_ || type_ == PASSABLE_DOES_COLLIDE)
	{
		return false;
	}

	bool process_children = bounded_collision_box_ == NULL;
	if (!process_children)
	{
		// Check if the entity is within this node.
		for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.getCollisions().begin(); ci != entity.getCollisions().end(); ++ci)
		{
			const ConvexPolygon* collision_box = *ci;

			if (bounded_collision_box_->isInside(*collision_box))
			{
				//ss << "Is inside!|";
				process_children = true;
				break;
			}
		}
	}
	if (!process_children)
	{
		return false;
	}

	//bool does_collide = false;

	if (this != &entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			const ConvexPolygon* lhs_c = *ci;
			for (std::vector<const ConvexPolygon*>::const_iterator ci = entity.collisions_.begin(); ci != entity.collisions_.end(); ++ci)
			{
				const ConvexPolygon* rhs_c = *ci;
				
				if (lhs_c->doesCollide(*rhs_c, info))
				{
					return true;
				}
				
				if (rhs_c->doesCollide(*lhs_c, info))
				{
					return true;
				}
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, info))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const
{
	if (!entity.is_alive_ || entity.type_ == PASSABLE_DOES_COLLIDE ||
		!is_alive_ || type_ == PASSABLE_DOES_COLLIDE)
	{
		return false;
	}

	if (this != &entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, info))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const
{
	if (entity != NULL && 
	   (!entity->is_alive_ || entity->type_ == PASSABLE_DOES_COLLIDE ||
		!is_alive_ || type_ == PASSABLE_DOES_COLLIDE))
	{
		return false;
	}

	if (this != entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->doesCollide(begin, end, effective_width))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, begin, end, effective_width))
		{
			return true;
		}
	}

	return false;
}

bool Entity::doesCollide(Entity* entity, const glm::vec3& point) const
{
	if (entity != NULL && 
	   (!entity->is_alive_ || entity->type_ == PASSABLE_DOES_COLLIDE ||
		!is_alive_ || type_ == PASSABLE_DOES_COLLIDE))
	{
		return false;
	}

	if (this != entity)
	{
		for (std::vector<const ConvexPolygon*>::const_iterator ci = collisions_.begin(); ci != collisions_.end(); ++ci)
		{
			if ((*ci)->isInside(point))
			{
				return true;
			}
		}
	}

	for (std::vector<SceneNode*>::const_iterator ci = children_.begin(); ci != children_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, point))
		{
			return true;
		}
	}

	return false;
}

void Entity::destroy()
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

bool Entity::activate(Entity& activator)
{
	return false;
}

/*
void Entity::addListener(EntityDestroyedListener& listener)
{
	listeners_.push_back(&listener);
}

void Entity::removeListener(EntityDestroyedListener& listener)
{
	for (int i = listeners_.size() - 1; i >= 0; --i)
	{
		if (listeners_[i] == &listener)
		{
			listeners_.erase(listeners_.begin() + i);
		}
	}
}
*/
};
