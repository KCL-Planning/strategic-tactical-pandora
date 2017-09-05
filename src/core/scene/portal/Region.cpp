#include <algorithm>

#include <glm/glm.hpp>
#include "../../../shapes/Cube.h"

#include "../../shaders/BasicShadowShader.h"
#include "Region.h"
#include "Portal.h"
#include "../SceneNode.h"
#include "../SceneLeafModel.h"
#include "../SceneManager.h"

#include "../../math/BoundedBox.h"
#include "../../math/Plane.h"
#include "../../entities/Entity.h"

std::vector<Region*> Region::all_regions_;

std::map<SceneNode*, std::vector<Region*>* > Region::scene_node_regions_;
std::map<Entity*, std::vector<Region*>* > Region::entity_collision_regions_;

Region::Region(SceneNode& scene_node)
	: scene_node_(&scene_node), name_(std::string())
{
	all_regions_.push_back(this);
}

Region::Region(SceneNode& scene_node, const std::string& name)
	:  scene_node_(&scene_node), name_(name)
{
	all_regions_.push_back(this);
}

Region::~Region()
{
	all_regions_.erase(std::find(all_regions_.begin(), all_regions_.end(), this));
}

const std::vector<Region*>& Region::getAllRegions() 
{
	return all_regions_;
}

void Region::debugCreateBoundedBox(SceneManager& scene_manager)
{/*
	// To visualise the bounding box, we add new leafs to this node.
	const glm::vec3& centre_point = scene_node_->getFrustumChecker().getCentrePoint();

	SceneNode* parent = scene_node_;
	if (centre_point != glm::vec3(0.0f, 0.0f, 0.0f))
	{
		parent = new SceneNode(scene_manager, scene_node_, glm::translate(glm::mat4(1.0), centre_point));
	}
		
	float box_width = scene_node_->getFrustumChecker().getPoints()[0].x - scene_node_->getFrustumChecker().getPoints()[1].x;
	float box_height = scene_node_->getFrustumChecker().getPoints()[0].y - scene_node_->getFrustumChecker().getPoints()[2].y;
	float box_depth = scene_node_->getFrustumChecker().getPoints()[0].z - scene_node_->getFrustumChecker().getPoints()[4].z;
	Cube* collision_cube = new Cube(std::abs(box_width), std::abs(box_height), std::abs(box_depth));

	//SceneLeafModel* collision_box_leaf = new SceneLeafModel(*parent, NULL, *collision_cube, *SceneNode::bright_material_, BasicShadowShader::getShader(), true, true, MODEL_TYPE::COLLISION);

	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		const Portal* portal = *ci;

		float minx = std::numeric_limits<float>::max();
		float maxx = -std::numeric_limits<float>::max();
		float miny = std::numeric_limits<float>::max();
		float maxy = -std::numeric_limits<float>::max();
		float minz = std::numeric_limits<float>::max();
		float maxz = -std::numeric_limits<float>::max();
		glm::vec3 centre_point(0,0,0);
		for (std::vector<glm::vec3>::const_iterator ci = portal->getPoints().begin(); ci != portal->getPoints().end(); ++ci)
		{
			const glm::vec3 p = *ci;
			centre_point += p;

			if (p.x < minx) minx = p.x;
			if (p.x > maxx) maxx = p.x;
			if (p.y < miny) miny = p.y;
			if (p.y > maxy) maxy = p.y;
			if (p.z < minz) minz = p.z;
			if (p.z > maxz) maxz = p.z;
		}
		
		parent = scene_node_;
		if (centre_point != glm::vec3(0.0f, 0.0f, 0.0f))
		{
			parent = new SceneNode(scene_manager, scene_node_, glm::translate(glm::mat4(1.0), centre_point));
		}
		
		centre_point = glm::vec3(centre_point.x / portal->getPoints().size(), centre_point.y / portal->getPoints().size(), centre_point.z / portal->getPoints().size());
		//centre_point /= portal->getPoints().size();
		Cube* collision_cube = new Cube(std::min(0.1f, std::abs(maxx - minx)), std::min(0.1f, std::abs(maxy - miny)), std::min(0.1f, std::abs(maxz - maxz)));
		
		SceneLeafModel* collision_box_leaf = new SceneLeafModel(*parent, NULL, *collision_cube, *SceneNode::bright_material_, BasicShadowShader::getShader(), false, true, MODEL_TYPE::COLLISION);
	}*/
}

Portal& Region::addPortalToOtherRegion(Region& other, const std::vector<glm::vec3>& points)
{
	Portal* portal = new Portal(points, *this, other);
	portals_.push_back(portal);
	return *portal;
}

void Region::preRender(const Frustum& frustum, const glm::vec3& camera_position, Renderer& renderer, bool process_lights, unsigned int& nr_calls, unsigned int portal_depth, std::vector<const Portal*>& processed_portals, std::stringstream& ss)
{
	//std::cout << "$preRender:" << name_ << " -- ";
	ss << " $preRender (" << entities_visible_from_different_regions_.size() << "):" << name_ << " ";

	// Render this region.
	scene_node_->preRender(frustum, camera_position, renderer, process_lights, nr_calls);

	// Render any scene nodes that are visible from this region.
	// TODO: Fix double rendering issue.
	for (std::vector<SceneNode*>::const_iterator ci = entities_visible_from_different_regions_.begin(); ci != entities_visible_from_different_regions_.end(); ++ci)
	{
		(*ci)->preRender(frustum, camera_position, renderer, process_lights, nr_calls);
	}

	//std::vector<const Portal*> new_processed_portals(processed_portals);

	// Check for any portals that must also be rendered.
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		//std::cout << "???" << (*ci)->getToRegion().getName();
		(*ci)->preRender(frustum, camera_position, renderer, process_lights, nr_calls, portal_depth, processed_portals, ss);
		//std::cout << "!!!" << (*ci)->getToRegion().getName();
	}
}

void Region::getRenderingPortals(const Frustum& frustum, const glm::vec3& camera_position, std::vector<glm::vec3>& portals, std::vector<const Portal*>& processed_portals) const
{
	// Check for any portals that must also be rendered.
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		//std::cout << "???" << (*ci)->getToRegion().getName();
		(*ci)->getRenderingPortals(frustum, camera_position, portals, processed_portals);
		//std::cout << "!!!" << (*ci)->getToRegion().getName();
	}
}
/*
void Region::getRegionsToRender(const Frustum& frustum, const glm::vec3& camera_position, unsigned int portal_depth, const Portal* accessed_from, std::vector<const Portal*>& processed_portals, std::vector<RegionPortalDebug>& processed) const
{
	processed.push_back(RegionPortalDebug(this, accessed_from, ""));

	std::vector<const Portal*> new_processed_portals(processed_portals);

	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		(*ci)->getRegionsToRender(frustum, camera_position, portal_depth, new_processed_portals, processed);
	}
}
*/
bool Region::doesCollide(Entity& entity, CollisionInfo& info) const
{
	std::vector<Region*>* collision_regions = entity_collision_regions_[&entity];
	if (collision_regions == NULL)
	{
		return false;
	}
	
	for (std::vector<Region*>::const_iterator ci = collision_regions->begin(); ci != collision_regions->end(); ++ci)
	{
		Region* region = *ci;
		for (std::vector<Entity*>::const_iterator ci = region->collidable_entities_.begin(); ci != region->collidable_entities_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, info))
			{
				return true;

			}
		}
	}
	return false;
	/*
	std::vector<const Region*> closed_list;
	return doesCollide(entity, info, closed_list);
	*/
}

bool Region::getCollisions(Entity& entity, std::vector<CollisionInfo>& info) const
{
	bool found_collision = false;
	std::vector<Region*>* collision_regions = entity_collision_regions_[&entity];
	if (collision_regions == NULL)
	{
		return false;
	}

	for (std::vector<Region*>::const_iterator ci = collision_regions->begin(); ci != collision_regions->end(); ++ci)
	{
		Region* region = *ci;
		for (std::vector<Entity*>::const_iterator ci = region->collidable_entities_.begin(); ci != region->collidable_entities_.end(); ++ci)
		{
			if ((*ci)->getCollisions(entity, info))
			{
				found_collision = true;
			}
		}
	}
	return found_collision;
	/*
	std::vector<const Region*> closed_list;
	return getCollisions(entity, info, closed_list);
	*/
}

bool Region::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info) const
{
	bool found_collision = false;
	std::vector<Region*>* collision_regions = entity_collision_regions_[&entity];
	if (collision_regions == NULL)
	{
		return false;
	}
	
	for (std::vector<Region*>::const_iterator ci = collision_regions->begin(); ci != collision_regions->end(); ++ci)
	{
		Region* region = *ci;
		for (std::vector<Entity*>::const_iterator ci = region->collidable_entities_.begin(); ci != region->collidable_entities_.end(); ++ci)
		{
			if ((*ci)->getCollisions(entity, begin, end, info))
			{
				found_collision = true;
			}
		}
	}
	return found_collision;
	/*
	std::vector<const Region*> closed_list;
	return getCollisions(entity, begin, end, info, closed_list);
	*/
}

bool Region::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info) const
{
	std::vector<Region*>* collision_regions = entity_collision_regions_[&entity];
	if (collision_regions == NULL)
	{
		return false;
	}

	for (std::vector<Region*>::const_iterator ci = collision_regions->begin(); ci != collision_regions->end(); ++ci)
	{
		Region* region = *ci;
		for (std::vector<Entity*>::const_iterator ci = region->collidable_entities_.begin(); ci != region->collidable_entities_.end(); ++ci)
		{
			if ((*ci)->doesCollide(entity, begin, end, info))
			{
				return true;
			}
		}
	}
	return false;
	/*
	std::vector<const Region*> closed_list;
	return doesCollide(entity, begin, end, info, closed_list);
	*/
}

bool Region::doesCollide(Entity* entity, const glm::vec3& begin, const glm::vec3& end, float effective_width) const
{
	// Find all the portals this line intersects with.
	std::set<const Region*> intersecting_regions;
	intersecting_regions.insert(this);

	std::vector<const Region*> open_list;
	open_list.push_back(this);

	while (open_list.size() > 0)
	{
		const Region* region = open_list[open_list.size() - 1];
		open_list.erase(open_list.begin() + open_list.size() - 1);

		if (intersecting_regions.count(region) != 0)
		{
			continue;
		}

		intersecting_regions.insert(region);

		for (std::vector<Portal*>::const_iterator ci = region->getPortals().begin(); ci != region->getPortals().end(); ++ci)
		{
			const Portal* portal = *ci;
			if (portal->intersectsWith(begin, end))
			{
				open_list.push_back(&portal->getToRegion());
			}
		}
	}

	for (std::set<const Region*>::const_iterator ci = intersecting_regions.begin(); ci != intersecting_regions.end(); ++ci)
	{
		const Region* region = *ci;
		for (std::vector<Entity*>::const_iterator ci = region->collidable_entities_.begin(); ci != region->collidable_entities_.end(); ++ci)
		{
			SceneNode* scene_node = *ci;
			if (scene_node->doesCollide(entity, begin, end, effective_width))
			{
				return true;
			}
		}
	}
	return false;
	/*
	std::vector<const Region*> closed_list;
	return doesCollide(begin, end, effective_width, closed_list);
	*/
}

bool Region::doesCollide(Entity* entity, const glm::vec3& point) const
{
	Region* non_const_region = const_cast<Region*>(this);
	Region* point_in_region = non_const_region->findRegion(point);
	if (point_in_region == NULL)
	{
		return false;
	}

	for (std::vector<Entity*>::const_iterator ci = point_in_region->collidable_entities_.begin(); ci != point_in_region->collidable_entities_.end(); ++ci)
	{
		if ((*ci)->doesCollide(entity, point))
		{
			return true;
		}
	}
	return false;
	/*
	std::vector<const Region*> closed_list;
	return doesCollide(point, closed_list);
	*/
}

void Region::removeVisibleEntityFromOtherRegion(SceneNode& node)
{
	std::vector<SceneNode*>::iterator i_find = std::find(entities_visible_from_different_regions_.begin(), entities_visible_from_different_regions_.end(), &node);
	if (i_find != entities_visible_from_different_regions_.end())
	{
		entities_visible_from_different_regions_.erase(i_find);
		
		// Update the cache.
		std::vector<Region*>* visible_regions = scene_node_regions_[&node];
		std::vector<Region*>::iterator r_find = std::find(visible_regions->begin(), visible_regions->end(), this);
		visible_regions->erase(r_find);
	}
	else
	{
		assert (false);
	}
}

void Region::addVisibleEntityFromOtherRegion(SceneNode& node)
{
	assert (std::find(entities_visible_from_different_regions_.begin(), entities_visible_from_different_regions_.end(), &node) == entities_visible_from_different_regions_.end());
	entities_visible_from_different_regions_.push_back(&node);

	// Update the cache.
	std::vector<Region*>* visible_regions;
	std::map<SceneNode*, std::vector<Region*>*>::const_iterator map_i = scene_node_regions_.find(&node);
	if (map_i == scene_node_regions_.end())
	{
		visible_regions = new std::vector<Region*>();
		scene_node_regions_[&node] = visible_regions;
	}
	else
	{
		visible_regions = (*map_i).second;
	}
	visible_regions->push_back(this);
}

void Region::removeCollidableEntity(Entity& entity)
{
	std::vector<Entity*>::iterator i_find = std::find(collidable_entities_.begin(), collidable_entities_.end(), &entity);
	if (i_find != collidable_entities_.end())
	{
		collidable_entities_.erase(i_find);

		// Update the cache.
		std::vector<Region*>* collidable_regions = (*entity_collision_regions_.find(&entity)).second;
		std::vector<Region*>::iterator r_find = std::find(collidable_regions->begin(), collidable_regions->end(), this);
		collidable_regions->erase(r_find);
	}
	else
	{
		assert (false);
	}
}

void Region::addCollidableEntity(Entity& entity)
{
	assert (std::find(collidable_entities_.begin(), collidable_entities_.end(), &entity) == collidable_entities_.end());
	collidable_entities_.push_back(&entity);

	// Update cache.
	std::vector<Region*>* collidable_regions;
	std::map<Entity*, std::vector<Region*>*>::const_iterator map_i = entity_collision_regions_.find(&entity);
	if (map_i == entity_collision_regions_.end())
	{
		collidable_regions = new std::vector<Region*>();
		entity_collision_regions_[&entity] = collidable_regions;
	}
	else
	{
		collidable_regions = (*map_i).second;
	}
	collidable_regions->push_back(this);
}

void Region::updateCollidableEntity(Entity& entity)
{
	// Check which region the entity is still part of.
	std::vector<Region*>* collidable_entities = entity_collision_regions_[&entity];

	if (collidable_entities == NULL)
	{
		collidable_entities = new std::vector<Region*>();
		entity_collision_regions_[&entity] = collidable_entities;
	}

	for (std::vector<Region*>::const_iterator ci = collidable_entities->begin(); ci != collidable_entities->end(); ++ci)
	{
		(*ci)->removeCollidableEntity(entity);
	}

	collidable_entities->clear();

	// Rebuild the entities this entity is collidable in.
	if (entity.getRegion() == NULL)
	{
		// If the entity is not part of a region we are done.
		return;
	}

	std::set<Region*> closed_set;
	std::vector<Region*> open_list;

	open_list.push_back(entity.getRegion());

	while (open_list.size() != 0)
	{
		Region* region = open_list[open_list.size() - 1];
		open_list.erase(open_list.begin() + open_list.size() - 1);
		
		if (closed_set.count(region) == 1)
		{
			continue;
		}
		closed_set.insert(region);

		if (entity.getCollisionChecker().isInside(region->getSceneNode().getFrustumChecker()) || region->getSceneNode().getFrustumChecker().isInside(entity.getCollisionChecker()))
		{
			collidable_entities->push_back(region);
			region->addCollidableEntity(entity);

			// Add all nearby regions to the open list.
			for (std::vector<Portal*>::const_iterator ci = region->getPortals().begin(); ci != region->getPortals().end(); ++ci)
			{
				const Portal* portal = *ci;
				if (closed_set.count(&portal->getToRegion()) == 1)
				{
					continue;
				}
				open_list.push_back(&portal->getToRegion());
			}
		}
	}
}

std::vector<Region*>* Region::getRegionsCollidableIn(Entity& entity)
{
	return entity_collision_regions_[&entity];
}

void Region::updateVisibility(SceneNode& scene_node)
{
	// Check which region the entity is still part of.
	std::vector<Region*>* visible_entities = scene_node_regions_[&scene_node];

	if (visible_entities == NULL)
	{
		visible_entities = new std::vector<Region*>();
		scene_node_regions_[&scene_node	] = visible_entities;
	}

	for (std::vector<Region*>::const_iterator ci = visible_entities->begin(); ci != visible_entities->end(); ++ci)
	{
		(*ci)->removeVisibleEntityFromOtherRegion(scene_node);
	}
	visible_entities->clear();

	// Rebuild the entities this entity is collidable in.
	if (scene_node.getRegion() == NULL)
	{
		// If the entity is not part of a region we are done.
		return;
	}

	std::set<Region*> closed_set;
	std::vector<Region*> open_list;

	open_list.push_back(scene_node.getRegion());

	while (open_list.size() != 0)
	{
		Region* region = open_list[open_list.size() - 1];
		open_list.erase(open_list.begin() + open_list.size() - 1);
		
		if (closed_set.count(region) == 1)
		{
			continue;
		}
		closed_set.insert(region);
		
		if (region->isVisibleIn(scene_node))
		{
			visible_entities->push_back(region);
			region->addVisibleEntityFromOtherRegion(scene_node);

			// Add all nearby regions to the open list.
			for (std::vector<Portal*>::const_iterator ci = region->getPortals().begin(); ci != region->getPortals().end(); ++ci)
			{
				const Portal* portal = *ci;
				if (closed_set.count(&portal->getToRegion()) == 1)
				{
					continue;
				}
				open_list.push_back(&portal->getToRegion());
			}
		}
	}
}

std::vector<Region*>* Region::getRegionsVisibleIn(SceneNode& scene_node)
{
	return scene_node_regions_[&scene_node];
}
/*
bool Region::doesCollide(Entity& entity, CollisionInfo& info, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	if (scene_node_->doesCollide(entity, info))
	{
		return true;
	}

	// Check if the entity is part of anothe region.
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
//	for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			glm::vec3 point = entity.getCollisionChecker().getPoints()[i] + entity.getCollisionChecker().getCentrePoint() + entity.getGlobalLocation();
			if (portal->getToRegion().getSceneNode().getFrustumChecker().isInside(point))
			{
				if (portal->getToRegion().doesCollide(entity, info, closed_list))
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool Region::getCollisions(Entity& entity, std::vector<CollisionInfo>& info, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	bool found_collision = false;
	if (scene_node_->getCollisions(entity, info))
	{
		found_collision = true;
	}

	// Check if the entity is part of another region.
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	//for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			glm::vec3 point = entity.getCollisionChecker().getPoints()[i] + entity.getCollisionChecker().getCentrePoint() + entity.getGlobalLocation();
			if (portal->getToRegion().getSceneNode().getFrustumChecker().isInside(point))
			{
				if (portal->getToRegion().getCollisions(entity, info, closed_list))
				{
					found_collision = true;
				}
			}
		}
	}
	return found_collision;
}

bool Region::getCollisions(Entity& entity, const glm::vec3& begin, const glm::vec3& end, std::vector<CollisionInfo>& info, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	bool found_collision = false;
	if (scene_node_->getCollisions(entity, begin, end, info))
	{
		found_collision = true;
	}

	// Check if the entity is part of another region.
	glm::vec3 intersection;
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	//for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			Plane p(portal->getPoints());
			if (p.intersectsWith(begin, end, intersection))
			{
				if (portal->getToRegion().getCollisions(entity, begin, end, info, closed_list))
				{
					found_collision = true;
				}
			}
		}
	}
	return found_collision;
}

bool Region::doesCollide(Entity& entity, const glm::vec3& begin, const glm::vec3& end, CollisionInfo& info, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	if (scene_node_->doesCollide(entity, begin, end, info))
	{
		return true;
	}

	// Check if the entity is part of another region.
	glm::vec3 intersection;
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	//for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			Plane p(portal->getPoints());
			if (p.intersectsWith(begin, end, intersection))
			{
				if (portal->getToRegion().doesCollide(entity, begin, end, info, closed_list))
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool Region::doesCollide(const glm::vec3& begin, const glm::vec3& end, float effective_width, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	if (scene_node_->doesCollide(begin, end, effective_width))
	{
		return true;
	}

	// Check if the entity is part of another region.
	glm::vec3 intersection;
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	//for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		for (unsigned int i = 0; i < 8; ++i)
		{
			Plane p(portal->getPoints());
			if (p.getDistance(begin, end) < effective_width)
			{
				if (portal->getToRegion().doesCollide(begin, end, effective_width, closed_list))
				{
					return true;
				}
			}
		}
	}
	return false;
}

bool Region::doesCollide(const glm::vec3& point, std::vector<const Region*>& closed_list) const
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return false;
	}
	closed_list.push_back(this);
	
	if (scene_node_->doesCollide(point))
	{
		return true;
	}

	// Check if the entity is part of another region.
	// NOTE: This seems very inefficient, but then again there is not much to go on with just a point :P.
	glm::vec3 intersection;
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	{
		Portal* portal = *ci;
		if (portal->getToRegion().doesCollide(point))
		{
			return true;
		}
	}
	return false;
}
*/
bool Region::isInRegion(const glm::vec3& location) const
{
	return scene_node_->getFrustumChecker().isInside(location);
}

bool Region::isVisibleIn(const SceneNode& node) const
{
	return scene_node_->getFrustumChecker().isInside(node.getFrustumChecker());
}

Region* Region::findRegion(const glm::vec3& location) 
{
	std::vector<const Region*> dummy;
	return findRegion(location, dummy);
}

Region* Region::findRegionGlobal(const glm::vec3& location)
{
	for (std::vector<Region*>::const_iterator ci = all_regions_.begin(); ci != all_regions_.end(); ++ci)
	{
		if ((*ci)->isInRegion(location))
		{
			return *ci;
		}
	}
	return NULL;
}

Region* Region::findRegion(const glm::vec3& location, std::vector<const Region*>& closed_list) 
{
	if (std::find(closed_list.begin(), closed_list.end(), this) != closed_list.end())
	{
		return NULL;
	}

	closed_list.push_back(this);
	if (scene_node_->getFrustumChecker().isInside(location))
	{
		return this;
	}

	Region* found_region = NULL;
	for (std::vector<Portal*>::const_iterator ci = portals_.begin(); ci != portals_.end(); ++ci)
	//for (Portal* portal : portals_)
	{
		Portal* portal = *ci;
		if ((found_region = portal->getToRegion().findRegion(location, closed_list)) != NULL)
		{
			break;
		}
	}
	return found_region;
}
